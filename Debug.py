from vehicle import Driver
from controller import Camera, Lidar, GPS, Keyboard, DistanceSensor
import math
import numpy as np
import cv2

#1. 중앙선(노란석)인식 가능함 -> 있으면 노란색으로 주행, 흰색점선은 인식 못함 -> 대체재로 2번 방법 생각중...
#2. 장애물 회피해서 2차선 도로일 경우 끝경계를 도로의 색(67,64,64)이 아닌 경계를 찾아 중앙을 주행하게
#   but 2차선 이상의 거리면?
#3. 가드레일이 있으면 DistanceSensor을 이용해서 거리 이격하여 주행
# but 주행 중, 램프(Ramp)처럼 본선에 합류하는 거나 빠지는 길이 있다면?
#4. setCruisingSpeed로 속도를 조절하는데 setBrakeIntensity가 1.0이여도 왜 차량이 멈추지 않는지?

"""
1. 중앙선(노란선) 인식해서 중앙선 침범 못하게 저지
2. openCV로 
 속도 제어는 브레이크 밟는 알고리즘을 setCruisingSpeed에 대입해서 조절 통일 
"""

#── GPS 좌표계 단순화 & STEP 지정 & UNKNOWN ──────────────
X,Y,Z = 0,1,2
TIME_STEP = 50
UNKNOWN = 9999.99

#── PID게인 ──────────────
KP = 0.25
KI = 0.006
KD = 2.0
PID_need_reset = False

#── 필터크기 ──────────────
FILTER_SIZE = 3

#── 플래그 ──────────────
has_camera = False
collision_avoidance = False #충돌 방지 기능
enable_display = False

#── 센서 객체 생성 ──────────────
#Camera변수
left_camera = None #왼쪽 카메라
right_camera = None #오른쪽 카메라
camera_width = -1
camera_height = -1

#Lidar
sick = None
sick_width = -1
sick_fov = -1.0

#── 주행 상태 ──────────────
speed = 0.0
steering_angle = 0.0

#── 상태(장애물 회피) ──────────────
STATE_NORMAL = "NORMAL" #기본 상태
STATE_AVOID  = "AVOID"  #장애물회피 상태
STATE_RETURN = "RETURN" #차선복귀 상태
drive_state = STATE_NORMAL
avoid_direction = 1 # -1 : 왼쪽 회피, 1 : 오른쪽 회피
state_timer = 0
AVOID_STEPS = 35  #회피 최소 유지 스텝
RETURN_STEPS = 70 #복귀 지속 스텝

#── HSV색상 범위 ──────────────
LOWER_WHITE = np.array([0, 0, 180])
UPPER_WHITE = np.array([180, 60, 255])

LOWER_YELLOW = np.array([20,100, 100])
UPPER_YELLOW = np.array([35, 255, 255])

#── 도움말 ──────────────
def print_help():
    print("You can drive this car!")
    print("[UP]/[DOWN] - accelerate/slow down")

#── 속도 조절 ──────────────
def set_speed(kmh : float):
    global speed
    kmh = max(0.0, min(kmh, 150.0))
    speed = kmh
    driver.setCruisingSpeed(speed)

#── Keyboard 조작 ──────────────
def check_keyboard(kb: Keyboard):
    key = kb.getKey()
    if key == Keyboard.UP:
        set_speed(speed+1.0)
    elif key == Keyboard.DOWN:
        set_speed(speed-1.0)

#── PID ──────────────
def applyPID(error):
    if not hasattr(applyPID, "prev"):
        applyPID.prev = 0
        applyPID.integral = 0
    
    diff = error - applyPID.prev
    applyPID.integral += error

    applyPID.prev = error

    return KP*error + KI*applyPID.integral + KD*diff

#── 조향 안정화 ──────────────
steering_angle = 0
def set_steering(angle):
    global steering_angle
    #변화제한
    if angle - steering_angle > 0.1:
        angle = steering_angle + 0.1
    if angle - steering_angle < -0.1:
        angle = steering_angle - 0.1
    #최대조향각
    angle = max(-0.5, min(angle,0.5))
    steering_angle = angle
    driver.setSteeringAngle(angle)

#── smoothing ──────────────
smooth_angle = 0
def filter_angle(new_angle):
    global smooth_angle
    smooth_angle = 0.7*smooth_angle + 0.3*new_angle
    return smooth_angle

# ── 차선 인식 (HSV 방식 적용) ────────────────────────
def detect_lane(img):
    roi = img[int(camera_height*0.65):camera_height-1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)
    yellow_mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)

    white_pixels = np.where(white_mask>0)
    yellow_pixels = np.where(yellow_mask>0)

    white_x = UNKNOWN
    yellow_x = UNKNOWN

    if len(white_pixels[1]) > 40:
        white_x = int(np.mean(white_pixels[1]))
    
    if len(yellow_pixels[1]) > 40:
        yellow_x = int(np.mean(yellow_pixels[1]))
    
    return white_x,yellow_x

#── Lidar 처리 ──────────────
def process_lidar():
    ranges = np.array(sick.getRangeImage())
    ranges = np.nan_to_num(ranges,nan=50,posinf=50)

    n = len(ranges)

    front = np.min(ranges[int(n*0.45):int(n*0.55)])
    left = np.min(ranges[int(n*0.6):int(n*0.8)])
    right = np.min(ranges[int(n*0.2):int(n*0.4)])

    return front,left,right

#── 속도 자동 제어 ──────────────
def auto_speed(front):
    if drive_state == STATE_AVOID:
        return 10
    if front < 10:
        return 15
    
    return 30

#── 센서 초기화 ──────────────
driver = Driver()
if hasattr(driver, 'getBasicTimeStep'):
    basic_ts = int(driver.getBasicTimeStep())
else:
    basic_ts = TIME_STEP

#left_Camera
try:
    left_camera = Camera('left_camera')
    left_camera.enable(TIME_STEP)
    camera_width = left_camera.getWidth()
    camera_height = left_camera.getHeight()
    has_camera = True
    print("Left Camera initialized")
except Exception:
    left_camera = None
    print("Warning: Left_Camera not found")
#right_Camera
try:
    right_camera = Camera('right_camera')
    right_camera.enable(TIME_STEP)
    if camera_width == -1:
        camera_width = right_camera.getWidth()
        camera_height = right_camera.getHeight()
    has_camera = True
    print("Right Camera initialized")
except Exception:
    right_camera = None
    print("Warning: Right_Camera not found")

#Lidar
try:
    sick = Lidar('Sick LMS 291')
    sick.enable(TIME_STEP)
    collision_avoidance = True
    sick_width = sick.getHorizontalResolution()
    sick_fov = sick.getFov()
    print("Lidar initialized")
except Exception:
    collision_avoidance = False
    sick = None

print_help()

kb = Keyboard()
kb.enable(TIME_STEP)

#── Main Loop ──────────────
i = 0

while driver.step() != -1:
    check_keyboard(kb)

    if i % max(1, int (TIME_STEP / max(1, basic_ts))) == 0:
        front, left_dist, right_dist = process_lidar()

        white_list = []
        yellow_list = []

        for cam in [left_camera, right_camera]:
            if cam is None:
                continue
            raw = cam.getImage()
            img = np.frombuffer(raw, np.uint8).reshape((camera_height, camera_width, 4))[:,:,3]
            white, yellow = detect_lane(img)
            if white != UNKNOWN:
                white_list.append(white)
            if yellow != UNKNOWN:
                yellow_list.append(yellow)
            
        lane_error = UNKNOWN
        if len(white_list) > 0:
            lane_x = int(np.mean(white_list))
            lane_error = lane_x - camera_width // 2
        
        yellow_correction = 0
        if len(yellow_list) > 0:
            yellow_x = int(np.mean(yellow_list))
            if yellow_x > camera_width * 0.45:
                yellow_correction = 0.15

        if drive_state == STATE_NORMAL:
            if front < 6:
                avoid_direction = -1 if left_dist > right_dist else 1
                drive_state = STATE_AVOID
                state_timer = 0
            else:
                if lane_error != UNKNOWN:
                    driver.setCrusingSpeed(0.0)
                    pid = applyPID(lane_error)
                    steer = filter_angle(pid + yellow_correction)
                    set_steering(steer)
                else:
                    #차선 인식 실패
                    driver.setCrusingSpeed(0.0)
                    PID_need_reset = True
        
        elif drive_state == STATE_AVOID:
            state_timer += 1
            if front > 10 and state_timer > AVOID_STEPS:
                drive_state = STATE_RETURN
                state_timer = 0
            else:
                set_steering(filter_angle(0.3 * avoid_direction))
        
        elif drive_state == STATE_RETURN:
            state_timer += 1
            if state_timer > RETURN_STEPS:
                drive_state = STATE_NORMAL
                PID_need_reset = True
            else:
                set_steering(filter_angle(-0.25 * avoid_direction))
        target = auto_speed(front)
        driver.setCruisingSpeed(min(speed, target))
    
    i += 1
