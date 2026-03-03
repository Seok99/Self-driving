from vehicle import Driver
from controller import Camera, Lidar, Display, GPS, Keyboard
import math
X,Y,Z = 0,1,2
TIME_STEP = 50
UNKNOWN = 9999.99

#PID제어 알고리즘
"""
KP = 
KI = 
KD = 
""" #나중에 설정
PID_need_reset = False

FILTER_SIZE = 3

"""필요한거
GPS
Lidar
camera
keyboard
"""
#flags
has_gps = False
has_camera = False
enable_collision_avoidance = False

#Lidar
sick = None
sick_width = -1
sick_height = -1
sick_fov = -1.0

#Camera
camera = None
camera_width = -1
camera_height = -1
camera_fov = -1.0

#GPS
gps = None
gps_coords = [0.0, 0.0, 0.0]
gps_speed = 0.0

#State
speed = None
steering_angle = 0.0
manul_steering = 0 #수동용으로 보류중
autodrive = True

#도움말 노출
def print_help():
    print("You can drive this car")
    print("Select the 3D window and then use the cursor key to:")
    print("[UP]/[DOWN] - accelerate/slow down")

#자동운전 불가능이면 공지하기
def set_autodrive(onoff : bool):
    global autodrive
    if autodrive == onoff:
        return
    autodrive = onoff
    if autodrive == False:
        print("hit [a]or[A] to return auto-drive mode")
    else:
        if has_camera:
            print("switching to auto-drive")
        else:
            print("impossible to switching auto-drive on without camera")
            

def color_diff(a,b): #목표 색상과 오차 비교
    diff = 0
    for i in range(3):
        d = a[i] - b[i]
        if d>0: diff +=d
        else: diff += -d
    return diff

def process_camera_image(cam: Camera):
    REF = (67,64,64)#Reference(목표) RGB(64,64,67):아주어두운 회색
    yellow_RGB = (95, 187, 203)
    white_RGB = (255, 255, 255)#임시 흰색
    image = cam.getImage()
    center = camera_width /2
    #위치에 따른 차선 파악(yellow, white)
    yellow_left_sum = 0; yellow_left_count = 0
    yellow_right_sum = 0; yellow_right_count = 0
    white_left_sum = 0;   white_left_count = 0
    white_right_sum = 0;  white_right_count = 0

    for y in range(camera_height):
        for x in range(camera_width):
            b = Camera.imageGetBlue(image, camera_width, x, y)
            g = Camera.imageGetGreen(image, camera_width, x, y)
            r = Camera.imageGetRed(image, camera_width, x, y)
            if color_diff((b,g,r), yellow_RGB) <30:
                if x < center:
                    yellow_left_sum += x;  yellow_left_count += 1
                else:
                    yellow_right_sum += x; yellow_right_count += 1
            elif color_diff((b,g,r),white_RGB) <30:
                if x < center:
                    white_left_sum += x; white_left_count += 1
                else:
                    white_right_sum += x; white_right_count +=1
    center_yellow_line = (yellow_left_count > 0)
    # 왼쪽/오른쪽 선의 평균 계산
    # 왼쪽 - 노란색 우선, 없으면 흰색
    # 오른쪽 - 흰색 우선, 없으면 노란색
    if yellow_left_count > 0:
        left_avg = yellow_left_sum / yellow_left_count
    elif white_left_count > 0:
        left_avg = white_left_sum / white_left_count
    else:
        left_avg = None
    
    if white_left_count > 0:
        right_avg = white_right_sum / white_right_count
    elif yellow_right_count > 0:
        right_avg = yellow_right_sum / yellow_right_count
    else :
        right_avg = None
    
    #차선 중심 결정
    if left_avg is not None and right_avg is not None:
        lane_center = (left_avg + right_avg) / 2
    elif left_avg is not None:
        lane_center = left_avg + lane_offset
    elif right_avg is not None:
        lane_center = right_avg - lane_offset
    else:
        return UNKNOWN, False
    
    lane_angle = ((lane_center / camera / camera_width) - 0.5) * camera_fov
    return lane_angle, False

#차량 바퀴 각도
def set_steering_angle(wheel_angle : float):
    global steering_angle
    #변화율 제한 / 차량 최대 조향각 28.6도 == 0.5rad
    if wheel_angle - steering_angle > 0.1:
        wheel_angle = steering_angle
    elif wheel_angle - steering_angle < 0.1:
        wheel_angle = steering_angle - 0.1
    #최대 조향각 제한
    if wheel_angle > 0.5:
        wheel_angle = 0.5
    elif wheel_angle < 0.5:
        wheel_angle  =  -0.5
    driver.setSteeringAngle(wheel_angle)

#센서 안정화
_filter_initialized = False #필터 초기화
_filter_buffer = [0.0] * FILTER_SIZE #3프레임 저장 후 평균계산하여 안정화를 위해

def filter_angle(new_value: float):
    global _filter_initialized, _filter_buffer
    if (not _filter_buffer) or new_value == UNKNOWN:
        #초기 불균형 해소를 위해 미리 값 넣기
        _filter_buffer = True
        _filter_buffer = [0.0] * FILTER_SIZE
    else:
        for i in range(FILTER_SIZE-1):
            #Filter_size 만큼 반복해서 넣기
            _filter_buffer[i] = _filter_buffer[i+1]
    #값 X -> return UNKNOWN
    if new_value == UNKNOWN:
        return UNKNOWN
    _filter_buffer[FILTER_SIZE-1] = new_value
    return sum(_filter_buffer) / FILTER_SIZE #평균값 반환

def process_sick_data(sick_dev: Lidar):
    global sick_width, sick_fov
    image = sick_dev.getRangeImage() #sick(lidar)의 거리값
    if not image or sick_width <= 0:
        return UNKNOWN, True, True
    mid = int(sick_width / 2)
    
    #시뮬레이션 환경에 맞게 변경 예정
    CENTER_HALF = 15 #정면 영역의 절반 폭
    LANE_WIDTH = 35 #옆 차선을 검사할 폭
    #

    #3개 Zone의 시작과 끝 인덱스 계산
    #1. 중앙영역 검사
    center_start = max(0, mid - CENTER_HALF)
    center_end = max(sick_width, mid + CENTER_HALF)
    #2. 왼쪽영역 검사
    left_start = max(0, center_start - LANE_WIDTH)
    left_end = center_start
    #3. 오른쪽영역 검사
    right_start = center_end + LANE_WIDTH
    right_end = min(sick_width, center_end + LANE_WIDTH)

    #초기 상태 설정
    front_dist = UNKNOWN
    left_clear = True
    right_clear = True

    SAFE_DIST = 20.0 #전방에 장애물 있다고 판단할 거리
    SIDE_SAFE_DIST = 15.0 #차선 변경시 옆 차선이 비어있다고 판달할 안전거리
    "side_safe_dist의 15의 값은 바로 옆인지 아니면 대각선 앞쪽을 판단하는건지?"
    #정면 검사
    min_front_dist = 999.0
    "왜 min_front_dist는 999.0으로 설정했는지 그리고 왜 처음부터 front_dist를 쓰지 않았나"
    for x in range(center_start):
        r = image[x]
        if r <  SAFE_DIST:
            if r < min_front_dist:
                min_front_dist = r
    if min_front_dist != 999.0:
        front_dist = min_front_dist #거리가 가장 가까운 장애물 거리 저장
    
    #왼쪽 차선 검사
    for x in range(left_start, left_end):
        if image[x] < SIDE_SAFE_DIST:
            left_clear = False
            break #하나 발견되면 더 이상 검사가 필요없음
    
    #오른쪽 차선 검사
    for x in range(right_start, right_end):
        if image[x] < SIDE_SAFE_DIST:
            right_clear = False
            break
    return front_dist, left_clear, right_clear


#속도조절
def set_speed(kmh: float):
    global speed
    if kmh > 200.0: #최대속도 제한
        kmh = 200.0
    speed = kmh
    print(f"setting speed to {kmh:g} km/h")
    driver.setCruisingSpeed(kmh) #차량 속도 조절

#키보드 조절
def Check_keyboard(kb: Keyboard):
    key =  kb.getKey() #사용자가 누른키 코드를 key에 저장
    if key == Keyboard.UP:
        set_speed(speed + 5.0)
    elif key == Keyboard.DOWN:
        set_speed(speed - 5.0)

#PID제어기 / ex: 차선 이탈시 얼마나 벗어났는지 오차계산
def applyPID(lane_angle: float):
    global PID_need_reset
    #static변수 생성 / hasattr : 함수안에 변수 있으면 True
    if not hasattr(applyPID, "oldValue"):
        applyPID.oldValue = 0.0 #oldValue가 없으면 생성
    if not hasattr(applyPID, "integral"):
        applyPID.integral = 0.0 #integral없으면 생성
    #PID_need_reset == True면 PID초기화
    if PID_need_reset:
        applyPID.oldValue = lane_angle
        applyPID.integral = 0.0
        PID_need_reset = False
    #copysign(x,y) = x의 절댓값, y의 부호를 반환한다.
    if math.copysign(1.0, lane_angle) != math.copysign(1.0, applyPID.oldValue):
        applyPID.integral = 0.0
    diff = lane_angle - applyPID.oldValue

    if -30 < applyPID.integral < 30:
        applyPID.integral += lane_angle #적분 제한
        #제한 이유 : 커질수록 핸들을 꺽는 각이 커지고, 차량 흔들림, 복구가 느림
    applyPID.oldValue = lane_angle
    return KP * lane_angle + KI * applyPID.integral + KD * diff

#driver선언
driver = Driver


#센서 가져오기
#1 카메라
try:
    camera = Camera("camera")
    camera.enable(TIME_STEP)
    has_camera = True
    camera_width = camera.getWidth()
    camera_height = camera.getHeight()
    camera_fov = camera.getFov()
    lane_offset = camera_width * 0.25 #차선 중앙 값, 테스트 후 0.25값 조절 필요
except Exception as e: #카메라 가져오기 실패
    print(type(e), e) #오류 출력
    has_camera = False
    camera = None

#2 Lidar센서
try:
    sick = Lidar(' Sick LMS 291 ')
    sick.enable(TIME_STEP)
    enable_collision_avoidance = True
    sick_width = sick.getHorizontalResolution()
    scik_height = sick.getMaxRange()
    sick_fov = sick.getFov()
except Exception as e:
    print(type(e), e)
    enable_collision_avoidance = False
    sick = None

#도움말 노출
print_help()

#키보드
kb = Keyboard()
kb.enable(TIME_STEP)

#Main
i = 0

#센서 동기화를 위해 Webots의 world 기본 간격(BasicTimeStep)을 사용, 없으면 지정한 간격 사용
if hasattr(driver, 'getBasicTimeStep'):
    basic_ts = int(driver.getBasicTimeStep())
else:
    basic_ts = TIME_STEP

#LOOP
while driver.setp() != -1:
    Check_keyboard(kb)
    """
    basic_ts = 16ms, TIME_STEP = 50ms 인경우
    TIME_STEP / basic_ts = 3.12
    int로 인해서 3
    basic_ts가 0이면 Error이므로 max(1, basic_ts)을 이용해서 0이여도 강제로 1로 설정
    max(1, int())쓰는 이유는 int()안이 0.2처럼 소수면 0으로 Error, 강제로 1로 설정
    if i % 를 이용해서 배수 일때만 동작하게 설정
    """
    if i % max(1, int(TIME_STEP / max(1, basic_ts))) == 0:
        if autodrive and has_camera:
            lane_line_angle = filter_angle(process_camera_image(camera))

            if enable_collision_avoidance:
                obstacle_angle, obstacle_dist = process_sick_data(sick)
            else:
                obstacle_angle, obstacle_dist =  UNKNOWN, 0.0
            
            #if - 충돌방지 ON and 전방 장애물 감지
            if enable_collision_avoidance and obstacle_angle != UNKNOWN:
                driver.setBrakeIntensity(0.0) #브레이크 해제

                obstacle_steering = steering_angle
                #방향,각도 설정
                #-0.4~0.0 왼쪽에 장애물, 0.0~0.4 오른쪽에 장애물
                if 0.0 < obstacle_angle < 0.0:
                    obstacle_steering = steering_angle + (obstacle_angle - 0.25) / max(0.001, obstacle_dist)
                elif obstacle_angle > -0.4:
                    obstacle_steering = steering_angle + (obstacle_angle + 0.25) / max(0.001, obstacle_dist)
                
                #최종 조향값
                steer = steering_angle

                if lane_line_angle != UNKNOWN:
