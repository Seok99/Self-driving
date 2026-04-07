from vehicle import Driver
from controller import Camera, Lidar, GPS, Keyboard
import numpy as np
import cv2

# ───────── 기본 설정 ─────────
TIME_STEP = 50
UNKNOWN = 9999.99

KP = 0.25
KI = 0.006
KD = 2.0

# ───────── 상태 ─────────
STATE_NORMAL = "NORMAL"
STATE_AVOID  = "AVOID"
STATE_RETURN = "RETURN"

drive_state = STATE_NORMAL
avoid_direction = 1
state_timer = 0

AVOID_STEPS = 35
RETURN_STEPS = 70

# ───────── HSV ─────────
LOWER_WHITE = np.array([0, 0, 180])
UPPER_WHITE = np.array([180, 60, 255])

LOWER_YELLOW = np.array([20,100,100])
UPPER_YELLOW = np.array([35,255,255])

# ───────── PID ─────────
def applyPID(error):
    if not hasattr(applyPID, "prev"):
        applyPID.prev = 0
        applyPID.integral = 0

    diff = error - applyPID.prev
    applyPID.integral += error
    applyPID.prev = error

    return KP*error + KI*applyPID.integral + KD*diff

# ───────── 조향 ─────────
steering_angle = 0

def set_steering(angle):
    global steering_angle

    if angle - steering_angle > 0.1:
        angle = steering_angle + 0.1
    if angle - steering_angle < -0.1:
        angle = steering_angle - 0.1

    angle = max(-0.5, min(angle, 0.5))
    steering_angle = angle
    driver.setSteeringAngle(angle)

# ───────── smoothing ─────────
smooth_angle = 0

def filter_angle(new_angle):
    global smooth_angle
    smooth_angle = 0.7*smooth_angle + 0.3*new_angle
    return smooth_angle

# ───────── 차선 인식 ─────────
def detect_lane(img):
    roi = img[int(camera_height*0.5):camera_height-1, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    white_mask = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)
    yellow_mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)

    white_pixels = np.where(white_mask > 0)
    yellow_pixels = np.where(yellow_mask > 0)

    white_x = UNKNOWN
    yellow_x = UNKNOWN

    if len(white_pixels[1]) > 40:
        white_x = int(np.mean(white_pixels[1]))

    if len(yellow_pixels[1]) > 40:
        yellow_x = int(np.mean(yellow_pixels[1]))

    return white_x, yellow_x

# ───────── 라이다 ─────────
def process_lidar():
    ranges = np.array(sick.getRangeImage())
    ranges = np.nan_to_num(ranges, nan=50, posinf=50)

    n = len(ranges)

    front = np.min(ranges[int(n*0.45):int(n*0.55)])
    left  = np.min(ranges[int(n*0.6):int(n*0.8)])
    right = np.min(ranges[int(n*0.2):int(n*0.4)])

    return front, left, right

# ───────── 속도 ─────────
def auto_speed(front):
    if drive_state == STATE_AVOID:
        return 10
    if front < 10:
        return 15
    return 30

# ───────── 초기화 ─────────
driver = Driver()

left_camera = Camera('left_camera')
left_camera.enable(TIME_STEP)

right_camera = Camera('right_camera')
right_camera.enable(TIME_STEP)

camera_width = left_camera.getWidth()
camera_height = left_camera.getHeight()

sick = Lidar('Sick LMS 291')
sick.enable(TIME_STEP)

kb = Keyboard()
kb.enable(TIME_STEP)

speed = 30

# ───────── 메인 루프 ─────────
while driver.step() != -1:

    front, left_dist, right_dist = process_lidar()

    white_list = []
    yellow_list = []

    for cam in [left_camera, right_camera]:
        raw = cam.getImage()

        img = np.frombuffer(raw, np.uint8).reshape((camera_height, camera_width, 4))
        img = img[:, :, :3]  # RGB 유지

        white, yellow = detect_lane(img)

        if white != UNKNOWN:
            white_list.append(white)

        if yellow != UNKNOWN:
            yellow_list.append(yellow)

    lane_error = UNKNOWN

    # 🔥 핵심: 중앙 계산
    if len(white_list) > 0 and len(yellow_list) > 0:
        white_x = int(np.mean(white_list))
        yellow_x = int(np.mean(yellow_list))

        lane_center = (white_x + yellow_x) // 2
        lane_error = lane_center - camera_width // 2

    elif len(white_list) > 0:
        white_x = int(np.mean(white_list))
        lane_error = white_x - camera_width * 0.75  # fallback

    # 🔥 중앙선 침범 방지
    if len(yellow_list) > 0:
        yellow_x = int(np.mean(yellow_list))

        if yellow_x > camera_width * 0.48:
            lane_error += 50  # 강제 오른쪽 이동

    # ───────── 상태 머신 ─────────
    if drive_state == STATE_NORMAL:
        if front < 6:
            avoid_direction = -1 if left_dist > right_dist else 1
            drive_state = STATE_AVOID
            state_timer = 0
        else:
            if lane_error != UNKNOWN:
                pid = applyPID(lane_error)
                steer = filter_angle(pid)
                set_steering(steer)

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
        else:
            set_steering(filter_angle(-0.25 * avoid_direction))

    # ───────── 속도 ─────────
    target = auto_speed(front)
    driver.setCruisingSpeed(min(speed, target))