from vehicle import Driver
from controller import Camera, Lidar, Keyboard
import numpy as np
import cv2
import math

# -----------------------------------
# 기본 설정 O
# -----------------------------------

TIME_STEP = 50
UNKNOWN = 999999

KP = 0.25
KI = 0.006
KD = 2.0

LOWER_WHITE = np.array([0,0,200])
UPPER_WHITE = np.array([180,50,255])

LOWER_YELLOW = np.array([20,100,100])
UPPER_YELLOW = np.array([35,255,255])

# -----------------------------------
# 상태 O
# -----------------------------------

STATE_NORMAL = 0
STATE_AVOID = 1
STATE_RETURN = 2

drive_state = STATE_NORMAL
avoid_direction = 1
state_timer = 0

AVOID_TIME = 30
RETURN_TIME = 60

# -----------------------------------
# PID O
# -----------------------------------

def applyPID(error):

    if not hasattr(applyPID,"prev"):
        applyPID.prev = 0
        applyPID.integral = 0

    diff = error - applyPID.prev
    applyPID.integral += error

    applyPID.prev = error

    return KP*error + KI*applyPID.integral + KD*diff


# -----------------------------------
# 조향 안정화 O
# -----------------------------------

steering_angle = 0

def set_steering(angle):

    global steering_angle

    # 변화 제한
    if angle - steering_angle > 0.1:
        angle = steering_angle + 0.1

    if angle - steering_angle < -0.1:
        angle = steering_angle - 0.1

    # 최대 조향각 제한
    angle = max(-0.5,min(angle,0.5))

    steering_angle = angle

    driver.setSteeringAngle(angle)


# -----------------------------------
# 조향 smoothing O
# -----------------------------------

smooth_angle = 0

def filter_angle(new_angle):

    global smooth_angle

    smooth_angle = 0.7*smooth_angle + 0.3*new_angle

    return smooth_angle


# -----------------------------------
# 차선 탐지 O
# -----------------------------------

def detect_lane(img):

    roi = img[int(cam_height*0.65):cam_height-5,:]

    hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)

    white_mask = cv2.inRange(hsv,LOWER_WHITE,UPPER_WHITE)
    yellow_mask = cv2.inRange(hsv,LOWER_YELLOW,UPPER_YELLOW)

    white_pixels = np.where(white_mask>0)
    yellow_pixels = np.where(yellow_mask>0)

    white_x = UNKNOWN
    yellow_x = UNKNOWN

    if len(white_pixels[1]) > 40:
        white_x = int(np.mean(white_pixels[1]))

    if len(yellow_pixels[1]) > 40:
        yellow_x = int(np.mean(yellow_pixels[1]))

    return white_x,yellow_x


# -----------------------------------
# LiDAR 처리 O 
# -----------------------------------

def process_lidar():

    ranges = np.array(lidar.getRangeImage())
    ranges = np.nan_to_num(ranges,nan=50,posinf=50)

    n = len(ranges)

    front = np.min(ranges[int(n*0.45):int(n*0.55)])
    left  = np.min(ranges[int(n*0.6):int(n*0.8)])
    right = np.min(ranges[int(n*0.2):int(n*0.4)])

    return front,left,right


# -----------------------------------
# 속도 자동 제어 O 
# -----------------------------------

def auto_speed(front):

    if drive_state == STATE_AVOID:
        return 10

    if front < 10:
        return 15

    return 30


# -----------------------------------
# 센서 초기화
# -----------------------------------

driver = Driver()

camera_left = driver.getDevice("camera_left")
camera_right = driver.getDevice("camera_right")

camera_left.enable(TIME_STEP)
camera_right.enable(TIME_STEP)

cam_width = camera_left.getWidth()
cam_height = camera_left.getHeight()

lidar = driver.getDevice("Sick LMS 291")
lidar.enable(TIME_STEP)

keyboard = Keyboard()
keyboard.enable(TIME_STEP)

speed = 30
driver.setCruisingSpeed(speed)

# -----------------------------------
# 메인 루프
# -----------------------------------

while driver.step() != -1:

    # 키보드 속도 조절
    key = keyboard.getKey()

    if key == Keyboard.UP:
        speed += 5

    if key == Keyboard.DOWN:
        speed -= 5

    speed = max(0,min(speed,60))

    # LiDAR 데이터
    front,left_dist,right_dist = process_lidar()

    # 차선 후보 리스트
    white_list = []
    yellow_list = []

    for cam in [camera_left,camera_right]:

        raw = cam.getImage()

        img = np.frombuffer(raw,np.uint8).reshape((cam_height,cam_width,4))[:,:,:3]

        white,yellow = detect_lane(img)

        if white != UNKNOWN:
            white_list.append(white)

        if yellow != UNKNOWN:
            yellow_list.append(yellow)

    # 흰선 기반 차선 유지
    lane_error = UNKNOWN

    if len(white_list) > 0:

        lane_x = int(np.mean(white_list))

        center = cam_width // 2

        lane_error = lane_x - center


    # 노란 중앙선 침범 방지
    yellow_correction = 0

    if len(yellow_list) > 0:

        yellow_x = int(np.mean(yellow_list))

        if yellow_x > cam_width*0.45:
            yellow_correction = 0.15


    # 상태 머신
    if drive_state == STATE_NORMAL:

        if front < 6:

            avoid_direction = -1 if left_dist > right_dist else 1
            drive_state = STATE_AVOID
            state_timer = 0

        else:

            if lane_error != UNKNOWN:

                pid = applyPID(lane_error)

                steer = pid + yellow_correction

                steer = filter_angle(steer)

                set_steering(steer)


    elif drive_state == STATE_AVOID:

        state_timer += 1

        if front > 10 and state_timer > AVOID_TIME:

            drive_state = STATE_RETURN
            state_timer = 0

        else:

            steer = 0.3 * avoid_direction
            set_steering(filter_angle(steer))


    elif drive_state == STATE_RETURN:

        state_timer += 1

        if state_timer > RETURN_TIME:

            drive_state = STATE_NORMAL

        else:

            steer = -0.25 * avoid_direction
            set_steering(filter_angle(steer))


    # 속도 적용
    target = auto_speed(front)

    driver.setCruisingSpeed(min(speed,target))