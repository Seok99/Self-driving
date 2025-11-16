from vehicle import Driver
from controller import Camera, Lider, GPS
import math

#GPS Index값 설정 (가독성을 위해) / GPS[0]보단 GPS[X]가 더 가독성 높음
X, Y, Z = 0, 1, 2
#TIME_STEP 센서 업데이트 주기
TIME_STEP = 50
#센서 인식 실패시 사용, 연산에 None에 비해 유용함.
UNKNOWN = 999999.99

#PID제어 알고리즘
KP = 0.25 #비례게인 / 높으면 진동,낮으면 반응 느림, 중간값으로
KI = 0.006 #적분게인 / 적분은 누적되므로 작은값에서부터 시작
KD = 2.0 #미분게인 / 급격한 변화를 억제하고 안정성 확보를 위해 큰값으로
PID_need_reset = False #차선을 잃었다가 다시 찾으면 PID초기화 시키기 위한 값

#카메라로 색을 인식했을때 다양한 환경으로 인식률이 다르기에 노이즈 제거용
FILTER_SIZE = 3 #작으면 노이즈제거 효과 부족, 높으면 지연발생

"플래그시스템"
enable_collision_avoidance = False #충돌회피기능
enable_display = False #화면 송출
has_gps = False #GPS
has_camera = False #카메라

"센서 변수/객체 선언"
"None으로 객체 선언 후 값은 -1로 초기화"
"fov : 시야각"
#Camera
camera = None
camera_width = -1
camera_height = -1
camera_fov = -1.0

#Lidar - Sick LMS291로 lidar의 한 종류
sick = None
sick_width = -1
sick_height = -1
sick_fov = -1.0

#webots에서 송출되는 운전자 시점 화면
display = None
display_width = 0 #0으로 설정한 이유는 -1은 "이미지없음"을 의미 이미지가 있어도 노출시키지않음
display_height = 0
speedmeter_image = None

#GPS
gps = None
gps_coords = [0.0 , 0.0, 0.0]
gps_speed = 0.0 #현재 차량 속도

#수동 시작

#수동 조종시 필요
speed = 0.0
steering_angle = 0.0
manual_steering = 0
autodrive = True

#와이퍼 모드
WIPER_OFF, WIPER_SLOW, WIPER_FAST = 0,1,2

#도움말
def print_help():
    print("You can drive this car!")
    print("Select the 3D window and then use the cursor keys to:")
    print("[LEFT]/[RIGHT] - steer")
    print("[UP]/[DOWN] - accelerate/slow down")
#카메라가 없으면 자동운전 불가능을 알려주기 위한 함수
def set_autodrive(onoff: bool):
    global autodrive
    if autodrive == onoff:
        return
    autodrive = onoff
    if not autodrive:
        print("switching to manual drive...\nhit [A] to return to auto-drive.")
    else:
        if has_camera:
            print("switching to auto-drive...")
        else:
            print("impossible to switch auto-drive on without camera...")

# target 속도 설정
#입력값은 kmh이고 kmh의 형태는 float
def set_speed(kmh: float):
    global speed #전역변수로 선언한 speed를 불러오기 위해 global사용
    if kmh > 250.0:
        kmh = 250.0
    speed = kmh
    print(f"setting speed to {kmh:g} km/h") #{}안에 g는 불필요한 소수점제거하는 역할
    driver.setCruisingSpeed(kmh) #차량에 바로 속도 입력

#차량 바퀴 각도
def set_steering_angle(wheel_angle : float):
    global steering_angle
    #변화율 제한(급격한 조향 방지) / 일반적인 차량 최대 조향각 28.6도 == 0.5rad
    if wheel_angle - steering_angle > 0.1:
        wheel_angle = steering_angle
    if wheel_angle - steering_angle < 0.1:
        wheel_angle = steering_angle - 0.1
    steering_angle = wheel_angle
    #최대 조향각 제한
    if wheel_angle > 0.5:
        wheel_angle = 0.5
    elif wheel_angle < -0.5:
        wheel_angle = -0.5
    driver.setSteeringAngle(wheel_angle)

def chage_manual_steer_angle(inc: int):
    global manual_steering #키보드 누를때 증가/감소하는 누적 값
    set_autodrive(False) #자율주행 정지
    new_manual_steering = manual_steering + inc
    #+-25로 할 경우 누를 수 있는 경우는 +-25씩으로 각이 더 세분화 되고 +-10으로 해도 됨
    if -25.0 <= new_manual_steering <= 25.0:
        manual_steering = new_manual_steering
        set_steering_angle(manual_steering * 0.02) #최댓값 25에  *0.02을 할 경우 차량의 최대 조향각 0.5rad에 맞춰짐. 하지만 10 * 0.05로 해도 무방하다.
        # 0.0x - 최대 조향각 0.5rad에만 맞추면 무슨값이든 상관없지만 x는 한번 누를때의 angle을 결정한다.
    if manual_steering == 0:
        #각이 0이면 직진을 출력
        print("going straight")
    else:
        #좌/우 방향을 출력한다.
        print(f"turng {steering_angle:.2f} rad ({'left' if steering_angle <0 else 'right'})")

def check_keyboard(kb: Keyboard):
    key = kb.getKey() #내가 누른키를 코드로 반환해서 key에 저장
    if key == Keyboard.UP:
        set_speed(speed + 5.0)
    elif key == Keyboard.DOWN:
        set_speed(speed + 5.0)
    elif key == Keyboard.LEFT:
        chage_manual_steer_angle(-1)
    elif key == Keyboard.RIGHT:
        chage_manual_steer_angle(+1)
    elif key in (ord('A'), ord('a')): #ord : 문자를 유니코드 정수값으로 바꿔주는 역할 / in : 묶어서 여러값을 확인하기 위함
        set_autodrive(True) #A,a 누르면 auto모드로 전환

def color_diff(a, b):
    diff = 0
    for i in range(3):
        d = a[i] - b[i]
        diff += d if d > 0 else -d
        """
        if d>0:
            diff += d
        else:
            diff += -d
        라는 문법
        """
    return diff

def process_camera_imgae(cam: Camera):
    REF = (95, 187, 203) #Wbots 도로의 중앙선 색의 픽셀값을 미리 파악해서 넣은값. 시작전 픽셀값 미리 파악하기
    num_pixels = camera_height * camera_width
    sumx = 0 #노란 픽셀들의 x좌표 합계
    pixel_count = 0 #노란 픽 수
    #x만 구하는 이유는 수평 중심을 구하기 위함.(왼쪽-오른쪽) 수직(위쪽-아래쪽)은 불필요
    #avg_x로 평균을 구해서 중심에 맞춤
    image = cam.getImage() #카메라의 원시 이미지 데이터를 가져옴
    for y in range(camera_height): #가져온 image의 크기 height와 width를 중첩for문을 이용해 전부 검사
        for x in range(camera_width):
            #RGB가 아닌 BRG 순서로 읽는다 why? 내부 포맷을 위해
            b = Camera.imageGetBlue(image, camera_width, x, y)
            g = Camera.imageGetGreen(image, camera_width, x, y)
            r = Camera.imageGetRed(image, camera_width, x, y)
            if color_diff((b,g,r), REF) <30: #color_diff함수를 통해서 오차/절대 차 계산하고 노란색 판단 / 30미만이면 노란색과 "충분히 비슷"으로 간주
                sumx += x #조건에 맞는 x좌표 더하기
                pixel_count += 1 #조건에 맞으면 픽셀 수 증가
    if pixel_count == 0:
        return UNKNOWN #노란색 인식 안되면 "값없음" 전달해 PID에서 적분 리셋
    avg_x = sumx / pixel_count #노란색으로 인식된 모든 x좌표의 중심 구하기
    return ((avg_x / camera_width) - 0.5) * camera_fov
"""
    avg_x / camera_width의 기본 값이 0 : 왼쪽 끝, 0.5 : 중앙, 1.0 : 오른쪽 끝 이기에 중앙 값을 0으로 맞추기 위한 계산방법
    (avg_x / camera_width) - 0.5 값이 -이면 왼쪽에 노란색이 많고 +면 오른쪽에 많다는 의미를 가짐
    픽셀 좌표 비율을 실제 카메라 시야각 상의 각도로 변환하기 위해서 카메라 시야각 변수인 camera_fov를 곱한다.
"""

#센서 안정화
_filter_initialized = False
_filter_buffer = [0.0] * FILTER_SIZE

def filter_anlge(new_value: float):
    global _filter_initialized, _filter_buffer
    if (not _filter_buffer) or new_value == UNKNOWN:
        #초기 평균 계산시 불균형이 생기지 않게하기 위해 값을 미리 채워 넣음
        _filter_buffer = True
        _filter_buffer = [0.0] * FILTER_SIZE
    else:
        for i in range(FILTER_SIZE-1):
            #초기가 아니면 필터사이즈(3)만큼 반복으로 새로운값을 넣는다.
            _filter_buffer[i] = _filter_buffer[i+1]

    #함수 입력값이 없으면 UNKNOWN을 return시킴
    if new_value == UNKNOWN:
        return UNKNOWN
    _filter_buffer[FILTER_SIZE -1] = new_value
    return sum(_filter_buffer) / FILTER_SIZE #평균값을 냄

#충동장애물을 찾고 장애물의 평균각도와 평균 거리 반환하는 함수
def process_sick_data(sick_dev: Lider):
    global sick_width, sick_fov
    HALF_AREA = 20 #차량의 중앙(전방영역)만 검사하려는 목적
    image = sick_dev.getRangeImage()
    if not image or sick_width <=0:
        return UNKNOWN, 0.0

    sumx = 0 #장애물이 인식된 인덱스들의 합
    collision_count = 0 #발견된 장애물 포인트 수
    obstacle_dist = 0.0 #장애물의 평균거리

    #중앙으로 부터 +-HALF_AREA(좌우) 인덱스만 검사하기 위함
    start = int(sick_width / 2 - HALF_AREA)
    end = int(sick_width / 2 + HALF_AREA)
    #검사 범위 지정
    if start < 0:
        start = 0
    if end > len(image):
        end =len(image)

    for x in range(start, end):
        r = image[x]
        # 20.0m 미만이면 "충동 가능객체"로 간주
        if r < 20.0:
            sumx += x #평균인덱스를 구하기 위해 더함
            collision_count += 1
            obstacle_dist += r

    #장애물이 없는 경우
    if collision_count == 0:
        return UNKNOWN, 0.0

    #평균 거리 계산
    obstacle_dist /= collision_count
    #평균 인덱스 -> 각도로 변환
    angle = ((sumx / collision_count) / sick_width - 0.5) * sick_fov
    return angle, obstacle_dist

def update_display():
    global speedometer_image
    NEEDLE_LENGTH = 50.0

    display.imagePaste(speedometer_image, 0, 0, False)

    #속도계 바늘
    current_speed = driver.getCurrentSpeed()
    if math.isnan(current_speed):
        current_speed = 0.0
    alpha = current_speed / 260.0 * 3.72 -0.27
    x = int (-NEEDLE_LENGTH * math.cos(alpha))
    y = int (-NEEDLE_LENGTH * math.sin(alpha))
    display.drawLine(100, 95, 100+ x, 95+y)

    display.drawText(f"GPS coords: {gps_coords[X]:.1f} {gps_coords[Z]:.1f}", 10, 130)
    display.drawText(f"GPS speed:  {gps_speed:.1f}", 10, 140)

    def compute_gps_speed():
        global gps_speed, gps_coords
        coords = gps.getValues() #현재 GPS좌표 반환
        speed_ms = gps.getSpeed() #거리변화율/시간 을 계산해서 속도를 반환
        gps_speed = speed_ms * 3.6 #반환되는 값은 m/s이므로 km/h로 변환 필요
        #1초에 1미터 -> 1시간(3600초)에 3600m = 3.6km ==> m/s에 3.6 곱하면 km/h로 변환
        gps_coords = list(coords) #coords는 get.value로 튜플값을 가져옴. list로 수정가능한 값으로 변경
        #튜플은 값 변경못함

    def applyPID(yellow_line_angle: float):
        global PID_need_reset
        #C언어 Static변수처럼 만듦 -> Python에서 static변수 만드는 방법 if, hasattr을 사용
        #ahsattr : 변수가 있는지 확인하는 함수
        if not hasattr(applyPID, "oldValue"): applyPID.oldValue = 0.0 #applyPID함수에 oldValue가 없으면 초기값 0.0으로 지정
        if not hasattr(applyPID, "integral"): applyPID.integral = 0.0
        
        #PID_need_reset = true면 PID초기화하기 위함 / 차선을 끊겼다가 다시 인식했을 경우를 위해서
        if PID_need_reset:
            applyPID.oldValue = yellow_line_angle #reset시 이전 오차가 존재하지 않아서 강제로 0을 맞추기 위함 (미분항 튐 현상/Derivative kick)방지 위함
            applyPID.integral = 0.0
            PID_need_reset = False
            