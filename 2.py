from vehicle import Driver
from controller import Camera, Lidar, GPS, Display, Keyboard
import numpy as np
import cv2
import math

# ── 기본 설정 ────────────────────────────────────────────────
X, Y, Z = 0, 1, 2
TIME_STEP = 50
UNKNOWN = 999999.99

# ── PID 설정 ─────────────────────────────────────────────────
KP = 0.25
KI = 0.006
KD = 2.0
PID_need_reset = False

# ── 플래그 ───────────────────────────────────────────────────
enable_collision_avoidance = False
enable_display = False
has_gps = False
has_camera = False

# ── 센서 객체 ────────────────────────────────────────────────
camera_left = None   # 왼쪽 카메라 : 노란 중앙선 담당
camera_right = None  # 오른쪽 카메라 : 흰색 점선 담당
cam_width = -1
cam_height = -1

sick = None
sick_width = -1
sick_fov = -1.0

gps = None
gps_coords = [0.0, 0.0, 0.0]
gps_speed = 0.0

display = None
speedometer_image = None

# ── 주행 상태 ────────────────────────────────────────────────
speed = 0.0
steering_angle = 0.0
manual_steering = 0
autodrive = True

# ── 상태 머신 (장애물 회피) ───────────────────────────────────
# ChatGPT 코드의 상태 머신 구조 유지
# 단, 타이머 기반 복귀 대신 기존 코드의 Lidar 거리 기반 복귀로 교체
STATE_NORMAL = "NORMAL"
STATE_AVOID  = "AVOID"
STATE_RETURN = "RETURN"
drive_state = STATE_NORMAL
avoid_direction = 1   # -1: 왼쪽 회피, 1: 오른쪽 회피
state_timer = 0
AVOID_STEPS = 35      # 회피 최소 유지 스텝
RETURN_STEPS = 70     # 복귀 지속 스텝

# ── HSV 색상 범위 (ChatGPT 코드에서 가져옴) ───────────────────
# HSV는 BGR보다 조명 변화에 강해서 인식률이 높음
LOWER_WHITE  = np.array([0,   0,   180])
UPPER_WHITE  = np.array([180, 60,  255])
LOWER_YELLOW = np.array([20,  100, 100])
UPPER_YELLOW = np.array([35,  255, 255])

WIPER_OFF, WIPER_SLOW, WIPER_FAST = 0, 1, 2


# ── 헬퍼 함수 ────────────────────────────────────────────────

def print_help():
    print("You can drive this car!")
    print("[LEFT]/[RIGHT] - steer")
    print("[UP]/[DOWN] - accelerate/slow down")
    print("[A] - auto-drive")
    print("[B] - stop")

"""수동 불필요
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
"""

def set_speed(kmh: float):
    global speed
    kmh = max(0.0, min(kmh, 250.0))
    speed = kmh
    print(f"setting speed to {kmh:g} km/h")
    driver.setCruisingSpeed(kmh)


def set_steering_angle(wheel_angle: float):
    global steering_angle
    # 급격한 조향 변화 억제
    if wheel_angle - steering_angle > 0.1:
        wheel_angle = steering_angle + 0.1
    if wheel_angle - steering_angle < -0.1:
        wheel_angle = steering_angle - 0.1
    steering_angle = wheel_angle
    # 최대 조향각 제한 (0.5 rad)
    wheel_angle = max(-0.5, min(wheel_angle, 0.5))
    driver.setSteeringAngle(wheel_angle)

"""수동 불필요
def change_manual_steer_angle(inc: int):
    global manual_steering
    set_autodrive(False)
    new_manual_steering = manual_steering + inc
    if -25.0 <= new_manual_steering <= 25.0:
        manual_steering = new_manual_steering
        set_steering_angle(manual_steering * 0.02)
    if manual_steering == 0:
        print("going straight")
    else:
        print(f"turning {steering_angle:.2f} rad ({'left' if steering_angle < 0 else 'right'})")
"""

def check_keyboard(kb: Keyboard):
    key = kb.getKey()
    if key == Keyboard.UP:
        set_speed(speed + 5.0)
    elif key == Keyboard.DOWN:
        set_speed(speed - 5.0)
    elif key == Keyboard.LEFT:
        change_manual_steer_angle(-1)
    elif key == Keyboard.RIGHT:
        change_manual_steer_angle(+1)
    elif key in (ord('A'), ord('a')):
        set_autodrive(True)
    elif key == ord('B'):
        set_speed(0.0)


# ── 차선 인식 (ChatGPT HSV 방식 적용) ────────────────────────

def detect_white_lane(img) -> float:
    """
    오른쪽 카메라 이미지에서 흰색 차선 오차 반환.
    HSV 마스크 → ROI 픽셀 평균 x → 중심 오차
    반환값: 픽셀 오차 (중앙 기준, 오른쪽이면 양수)
    """
    roi = img[int(cam_height * 0.6):cam_height - 5, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_WHITE, UPPER_WHITE)
    pixels = np.where(mask > 0)
    if len(pixels[1]) < 40:
        return UNKNOWN
    lane_x = int(np.mean(pixels[1]))
    center = roi.shape[1] // 2
    return float(lane_x - center)


def detect_yellow_lane(img) -> float:
    """
    왼쪽 카메라 이미지에서 노란 중앙선 오차 반환.
    반환값: 픽셀 오차 (중앙 기준)
    """
    roi = img[int(cam_height * 0.6):cam_height - 5, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    pixels = np.where(mask > 0)
    if len(pixels[1]) < 40:
        return UNKNOWN
    yellow_x = int(np.mean(pixels[1]))
    center = roi.shape[1] // 2
    return float(yellow_x - center)


def compute_lane_error(white_err: float, yellow_err: float) -> float:
    """
    두 카메라 오차를 합산해 최종 차선 오차 반환.
    - 둘 다 인식 → 평균
    - 한쪽만 인식 → 해당 값 사용 (끝 차선 흰선 없는 경우 포함)
    - 둘 다 실패 → UNKNOWN
    """
    left_valid  = (white_err  != UNKNOWN)
    right_valid = (yellow_err != UNKNOWN)

    if left_valid and right_valid:
        return (white_err + yellow_err) / 2.0
    elif left_valid:
        return white_err
    elif right_valid:
        return yellow_err
    else:
        return UNKNOWN


# ── 조향 스무딩 (ChatGPT 방식) ───────────────────────────────
_smooth_angle = 0.0

def filter_angle(new_angle: float) -> float:
    global _smooth_angle
    if new_angle == UNKNOWN:
        return UNKNOWN
    _smooth_angle = 0.75 * _smooth_angle + 0.25 * new_angle
    return _smooth_angle


# ── PID 제어 (기존 코드 유지) ────────────────────────────────

def applyPID(error: float) -> float:
    global PID_need_reset
    if not hasattr(applyPID, "oldValue"):
        applyPID.oldValue = 0.0
    if not hasattr(applyPID, "integral"):
        applyPID.integral = 0.0

    if PID_need_reset:
        applyPID.oldValue = error
        applyPID.integral = 0.0
        PID_need_reset = False

    if math.copysign(1.0, error) != math.copysign(1.0, applyPID.oldValue):
        applyPID.integral = 0.0

    diff = error - applyPID.oldValue
    if -30 < applyPID.integral < 30:
        applyPID.integral += error
    applyPID.oldValue = error

    return KP * error + KI * applyPID.integral + KD * diff


# ── Lidar 처리 (기존 코드 유지 + numpy 개선) ─────────────────

def process_sick_data() -> tuple:
    """
    장애물 각도, 전방/좌/우 거리 반환.
    기존 코드의 Lidar 거리 기반 로직 유지.
    ChatGPT 코드처럼 좌/우 거리도 추가로 반환해 회피 방향 결정에 활용.
    """
    if sick is None or sick_width <= 0:
        return UNKNOWN, 50.0, 50.0, 50.0

    ranges = np.array(sick.getRangeImage())
    ranges = np.nan_to_num(ranges, nan=50.0, posinf=50.0)
    n = len(ranges)

    front_min = float(np.min(ranges[int(n*0.45):int(n*0.55)]))
    left_min  = float(np.min(ranges[int(n*0.6) :int(n*0.8)]))
    right_min = float(np.min(ranges[int(n*0.2) :int(n*0.4)]))

    # 전방 중앙 영역에서 장애물 각도 계산 (기존 방식 유지)
    HALF_AREA = 20
    start = max(0, int(n / 2 - HALF_AREA))
    end   = min(n, int(n / 2 + HALF_AREA))
    close_mask = ranges[start:end] < 20.0
    indices = np.where(close_mask)[0]

    if len(indices) == 0:
        return UNKNOWN, front_min, left_min, right_min

    avg_idx = float(np.mean(indices + start))
    angle = (avg_idx / n - 0.5) * sick_fov
    return angle, front_min, left_min, right_min


# ── 자동 속도 제어 (ChatGPT 방식) ────────────────────────────

def auto_speed(front_dist: float) -> float:
    if drive_state == STATE_AVOID:
        return 10.0
    if front_dist < 10.0:
        return 15.0
    return 25.0


# ── GPS / Display ─────────────────────────────────────────────

def compute_gps_speed():
    global gps_speed, gps_coords
    coords = gps.getValues()
    gps_speed = gps.getSpeed() * 3.6
    gps_coords = list(coords)


def update_display():
    NEEDLE_LENGTH = 50.0
    display.imagePaste(speedometer_image, 0, 0, False)
    current_speed = driver.getCurrentSpeed()
    if math.isnan(current_speed):
        current_speed = 0.0
    alpha = current_speed / 260.0 * 3.72 - 0.27
    x = int(-NEEDLE_LENGTH * math.cos(alpha))
    y = int(-NEEDLE_LENGTH * math.sin(alpha))
    display.drawLine(100, 95, 100 + x, 95 + y)
    display.drawText(f"GPS coords: {gps_coords[X]:.1f} {gps_coords[Z]:.1f}", 10, 130)
    display.drawText(f"GPS speed:  {gps_speed:.1f}", 10, 140)
    display.drawText(f"State: {drive_state}", 10, 150)


# ── 센서 초기화 ──────────────────────────────────────────────

driver = Driver()
basic_ts = int(driver.getBasicTimeStep()) if hasattr(driver, 'getBasicTimeStep') else TIME_STEP

# 왼쪽 카메라 (노란 중앙선)
try:
    camera_left = Camera('camera_left')
    camera_left.enable(TIME_STEP)
    cam_width  = camera_left.getWidth()
    cam_height = camera_left.getHeight()
    has_camera = True
    print("Left camera initialized.")
except Exception:
    camera_left = None
    print("Warning: camera_left not found.")

# 오른쪽 카메라 (흰색 점선)
try:
    camera_right = Camera('camera_right')
    camera_right.enable(TIME_STEP)
    if cam_width == -1:
        cam_width  = camera_right.getWidth()
        cam_height = camera_right.getHeight()
    has_camera = True
    print("Right camera initialized.")
except Exception:
    camera_right = None
    print("Warning: camera_right not found.")

# Lidar
try:
    sick = Lidar('Sick LMS 291')
    sick.enable(TIME_STEP)
    enable_collision_avoidance = True
    sick_width = sick.getHorizontalResolution()
    sick_fov   = sick.getFov()
    print("Lidar initialized.")
except Exception:
    enable_collision_avoidance = False
    sick = None

# GPS
try:
    gps = GPS('gps')
    gps.enable(TIME_STEP)
    has_gps = True
except Exception:
    has_gps = False
    gps = None

# Display
try:
    display = Display('display')
    enable_display = True
    speedometer_image = display.imageLoad('speedometer.png')
except Exception:
    enable_display = False
    display = None

if has_camera:
    set_speed(50.0)

print_help()
kb = Keyboard()
kb.enable(TIME_STEP)

# ── 메인 루프 ────────────────────────────────────────────────

i = 0

while driver.step() != -1:

    check_keyboard(kb)

    if i % max(1, int(TIME_STEP / max(1, basic_ts))) == 0:

        # ── 센서 데이터 수집 ──────────────────────────────────
        obstacle_angle, front_dist, left_dist, right_dist = process_sick_data()

        # ── 카메라 이미지 처리 ────────────────────────────────
        white_err  = UNKNOWN
        yellow_err = UNKNOWN

        if camera_right is not None:
            raw = camera_right.getImage()
            img_right = np.frombuffer(raw, np.uint8).reshape((cam_height, cam_width, 4))[:, :, :3]
            white_err = detect_white_lane(img_right)

        if camera_left is not None:
            raw = camera_left.getImage()
            img_left = np.frombuffer(raw, np.uint8).reshape((cam_height, cam_width, 4))[:, :, :3]
            yellow_err = detect_yellow_lane(img_left)

        lane_error = compute_lane_error(white_err, yellow_err)

        # ── 상태 머신 ─────────────────────────────────────────
        if drive_state == STATE_NORMAL:

            if enable_collision_avoidance and front_dist < 6.0:
                # 장애물 감지 → 더 넓은 쪽으로 회피
                avoid_direction = -1 if left_dist > right_dist else 1
                drive_state = STATE_AVOID
                state_timer = 0
                print(f"Obstacle! Avoiding to {'left' if avoid_direction == -1 else 'right'}")

            else:
                # 정상 차선 추종
                if lane_error != UNKNOWN:
                    driver.setBrakeIntensity(0.0)
                    pid_out = applyPID(lane_error)
                    steer = filter_angle(pid_out)
                    set_steering_angle(steer)
                else:
                    # 차선 인식 실패 → 감속
                    driver.setBrakeIntensity(0.4)
                    PID_need_reset = True

        elif drive_state == STATE_AVOID:
            state_timer += 1
            # Lidar로 장애물 사라졌는지 확인 후 복귀 (타이머는 최소 유지 시간)
            if front_dist > 10.0 and state_timer > AVOID_STEPS:
                drive_state = STATE_RETURN
                state_timer = 0
                print("Obstacle cleared, returning to lane...")
            else:
                steer = filter_angle(0.4 * avoid_direction)
                set_steering_angle(steer)

        elif drive_state == STATE_RETURN:
            state_timer += 1
            if state_timer > RETURN_STEPS:
                drive_state = STATE_NORMAL
                state_timer = 0
                PID_need_reset = True
                print("Returned to normal driving.")
            else:
                steer = filter_angle(-0.3 * avoid_direction)
                set_steering_angle(steer)

        # ── 속도 적용 ─────────────────────────────────────────
        target_speed = auto_speed(front_dist)
        driver.setCruisingSpeed(min(speed, target_speed))

        # ── GPS / Display 업데이트 ────────────────────────────
        if has_gps:
            compute_gps_speed()
        if enable_display:
            update_display()

    i += 1