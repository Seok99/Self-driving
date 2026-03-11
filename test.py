from vehicle import Driver
from controller import Camera, Lidar, GPS, Keyboard
import math

# ─────────────────────────────────────────────
#  기본 설정
# ─────────────────────────────────────────────
X, Y, Z = 0, 1, 2
TIME_STEP = 50
UNKNOWN = 9999.99

# ── PID 게인 (시뮬레이션 후 조정 권장) ──
KP = 0.25
KI = 0.006
KD = 2.0
PID_need_reset = False

# ── 이동 평균 필터 크기 ──
FILTER_SIZE = 3

# ─────────────────────────────────────────────
#  플래그
# ─────────────────────────────────────────────
has_gps               = False
has_camera            = False
enable_collision_avoidance = False

# ─────────────────────────────────────────────
#  센서 객체 / 변수
# ─────────────────────────────────────────────
camera        = None
camera_width  = -1
camera_height = -1
camera_fov    = -1.0
lane_offset   = 100   # 카메라 초기화 전 기본값 (오류 방지)

sick          = None
sick_width    = -1
sick_fov      = -1.0

gps           = None
gps_coords    = [0.0, 0.0, 0.0]
gps_speed     = 0.0

# ─────────────────────────────────────────────
#  주행 상태
# ─────────────────────────────────────────────
speed          = 0.0
steering_angle = 0.0
autodrive      = True

# ── 차선 변경 상태 머신 ──
# 'follow' : 차선 유지(기본)
# 'changing': 차선 변경 중 (카운터 사용)
LANE_STATE_FOLLOW    = 'follow'
LANE_STATE_TURN_OUT  = 'turn_out'   # 1단계 : 차선 방향으로 꺾기
LANE_STATE_TURN_IN   = 'turn_in'    # 2단계 : 반대 방향으로 복귀
lane_state           = LANE_STATE_FOLLOW
lane_change_counter  = 0
LANE_CHANGE_STEPS    = 15   # 1단계 / 2단계 각각 30스텝 (총 약 3초)
lane_change_steer    = 0.0  # 현재 단계의 조향각
lane_change_return   = 0.0  # 2단계(복귀) 조향각

# ─────────────────────────────────────────────
#  색상 기준값  (BGR 순서 — Webots Camera API)
# ─────────────────────────────────────────────
YELLOW_REF = (95, 187, 203)   # 노란 중앙선
WHITE_REF  = (235, 232, 232)  # 흰 차선

# 흰선은 조명 변화가 크므로 허용 오차를 넉넉하게 설정
YELLOW_THRESHOLD = 30
WHITE_THRESHOLD  = 50   # ← 기존 30에서 50으로 상향 (흰선 인식 핵심 수정)

# ─────────────────────────────────────────────
#  유틸
# ─────────────────────────────────────────────
def print_help():
    print("=== 자율주행 컨트롤러 ===")
    print("[UP]/[DOWN]  : 속도 +5 / -5 km/h")
    print("[A]          : 자동 주행 복귀")


def color_diff(a, b):
    """두 BGR 색상 간의 L1 거리 계산."""
    return sum(abs(a[i] - b[i]) for i in range(3))


# ─────────────────────────────────────────────
#  카메라 처리
# ─────────────────────────────────────────────
def process_camera_image(cam: Camera):
    """
    노란선(중앙선)과 흰선(차선 경계)을 인식해
    (차선 중심 각도, 현재_1차선_여부) 를 반환한다.

    반환값:
        lane_angle (float) : 양수=오른쪽 이탈, 음수=왼쪽 이탈
        is_center_line (bool): 노란 중앙선이 왼쪽에 있으면 True (= 현재 1차선)
    """
    if camera_width <= 0 or camera_height <= 0:
        return UNKNOWN, False

    image = cam.getImage()
    if image is None:
        return UNKNOWN, False

    center = camera_width / 2

    # 화면 하단 40%만 검사 (노면 인식 정확도 향상)
    y_start = int(camera_height * 0.6)

    yellow_left_sum  = 0; yellow_left_count  = 0
    yellow_right_sum = 0; yellow_right_count = 0
    white_left_sum   = 0; white_left_count   = 0
    white_right_sum  = 0; white_right_count  = 0

    for y in range(y_start, camera_height):
        for x in range(camera_width):
            b = Camera.imageGetBlue (image, camera_width, x, y)
            g = Camera.imageGetGreen(image, camera_width, x, y)
            r = Camera.imageGetRed  (image, camera_width, x, y)

            if color_diff((b, g, r), YELLOW_REF) < YELLOW_THRESHOLD:
                if x < center:
                    yellow_left_sum  += x; yellow_left_count  += 1
                else:
                    yellow_right_sum += x; yellow_right_count += 1

            elif color_diff((b, g, r), WHITE_REF) < WHITE_THRESHOLD:
                if x < center:
                    white_left_sum  += x; white_left_count  += 1
                else:
                    white_right_sum += x; white_right_count += 1

    # 노란 중앙선이 왼쪽에 있으면 → 1차선 주행 중
    is_center_line = (yellow_left_count > 0)

    # ── 좌/우 차선 평균 x 좌표 결정 ──
    # 왼쪽 : 노란선 우선, 없으면 흰선
    if yellow_left_count > 0:
        left_avg = yellow_left_sum / yellow_left_count
    elif white_left_count > 0:
        left_avg = white_left_sum / white_left_count
    else:
        left_avg = None

    # 오른쪽 : 흰선 우선, 없으면 노란선
    if white_right_count > 0:
        right_avg = white_right_sum / white_right_count
    elif yellow_right_count > 0:
        right_avg = yellow_right_sum / yellow_right_count
    else:
        right_avg = None

    # ── 차선 중심 계산 ──
    if left_avg is not None and right_avg is not None:
        lane_center = (left_avg + right_avg) / 2
    elif left_avg is not None:
        lane_center = left_avg + lane_offset   # 오른쪽 선이 안 보이면 왼쪽 선 기준으로 추정
    elif right_avg is not None:
        lane_center = right_avg - lane_offset  # 왼쪽 선이 안 보이면 오른쪽 선 기준으로 추정
    else:
        return UNKNOWN, False                  # 양쪽 모두 안 보이면 UNKNOWN

    lane_angle = ((lane_center / camera_width) - 0.5) * camera_fov
    return lane_angle, is_center_line


# ─────────────────────────────────────────────
#  이동 평균 필터
# ─────────────────────────────────────────────
_filter_buffer = [0.0] * FILTER_SIZE

def filter_angle(new_value: float):
    global _filter_buffer
    if new_value == UNKNOWN:
        # UNKNOWN이 들어오면 버퍼 초기화 후 UNKNOWN 반환
        _filter_buffer = [0.0] * FILTER_SIZE
        return UNKNOWN
    # 슬라이딩 윈도우
    _filter_buffer = _filter_buffer[1:] + [new_value]
    return sum(_filter_buffer) / FILTER_SIZE


# ─────────────────────────────────────────────
#  Lidar 처리  (3-Zone : 전방 / 왼쪽 차선 / 오른쪽 차선)
# ─────────────────────────────────────────────
def process_sick_data(sick_dev: Lidar):
    """
    반환값:
        front_dist  (float) : 전방 최근접 장애물 거리 (없으면 UNKNOWN)
        left_clear  (bool)  : 왼쪽 차선 안전 여부
        right_clear (bool)  : 오른쪽 차선 안전 여부
    """
    image = sick_dev.getRangeImage()
    if not image or sick_width <= 0:
        return UNKNOWN, True, True

    mid         = sick_width // 2
    CENTER_HALF = 20   # 전방 탐지 반폭 (인덱스)
    LANE_WIDTH  = 35   # 옆 차선 탐지 폭 (인덱스)
    SAFE_DIST   = 20.0 # 전방 위험 거리 (m)
    SIDE_DIST   = 5.0  # 측면 위험 거리 (m)

    c_start = max(0, mid - CENTER_HALF)
    c_end   = min(sick_width, mid + CENTER_HALF)
    l_start = max(0, c_start - LANE_WIDTH)
    l_end   = c_start
    r_start = c_end
    r_end   = min(sick_width, c_end + LANE_WIDTH)

    # 전방 최근접 거리
    front_dist = UNKNOWN
    for x in range(c_start, c_end):
        r = image[x]
        if r < SAFE_DIST:
            if front_dist == UNKNOWN or r < front_dist:
                front_dist = r

    # 왼쪽 차선 안전 여부
    left_clear = all(image[x] >= SIDE_DIST for x in range(l_start, l_end))

    # 오른쪽 차선 안전 여부
    right_clear = all(image[x] >= SIDE_DIST for x in range(r_start, r_end))

    return front_dist, left_clear, right_clear


# ─────────────────────────────────────────────
#  조향 / 속도 제어
# ─────────────────────────────────────────────
def set_speed(kmh: float):
    global speed
    kmh   = max(0.0, min(kmh, 200.0))
    speed = kmh
    print(f"속도 설정: {kmh:g} km/h")
    driver.setCruisingSpeed(kmh)


def set_steering_angle(wheel_angle: float):
    """변화율 제한 + 최대 조향각 제한 적용."""
    global steering_angle
    # 변화율 제한 : 한 스텝에 최대 ±0.1 rad
    delta = wheel_angle - steering_angle
    if delta > 0.1:
        wheel_angle = steering_angle + 0.1
    elif delta < -0.1:
        wheel_angle = steering_angle - 0.1
    # 절대 한계 ±0.5 rad
    wheel_angle   = max(-0.5, min(0.5, wheel_angle))
    steering_angle = wheel_angle
    driver.setSteeringAngle(wheel_angle)


# ─────────────────────────────────────────────
#  PID 제어기
# ─────────────────────────────────────────────
def applyPID(lane_angle: float):
    global PID_need_reset
    if not hasattr(applyPID, "oldValue"):  applyPID.oldValue  = 0.0
    if not hasattr(applyPID, "integral"):  applyPID.integral  = 0.0

    if PID_need_reset:
        applyPID.oldValue = lane_angle   # Derivative kick 방지
        applyPID.integral = 0.0
        PID_need_reset    = False

    # 부호가 바뀌면 적분 초기화 (오버슈팅 방지)
    if math.copysign(1.0, lane_angle) != math.copysign(1.0, applyPID.oldValue):
        applyPID.integral = 0.0

    diff = lane_angle - applyPID.oldValue

    if -30 < applyPID.integral < 30:
        applyPID.integral += lane_angle

    applyPID.oldValue = lane_angle
    return KP * lane_angle + KI * applyPID.integral + KD * diff


# ─────────────────────────────────────────────
#  키보드
# ─────────────────────────────────────────────
def check_keyboard(kb: Keyboard):
    key = kb.getKey()
    if key == Keyboard.UP:
        set_speed(speed + 5.0)
    elif key == Keyboard.DOWN:
        set_speed(speed - 5.0)
    elif key in (ord('A'), ord('a')):
        global autodrive
        autodrive = True
        print("자동 주행 복귀")


# ─────────────────────────────────────────────
#  GPS
# ─────────────────────────────────────────────
def compute_gps_speed():
    global gps_speed, gps_coords
    gps_coords = list(gps.getValues())
    gps_speed  = gps.getSpeed() * 3.6   # m/s → km/h


# ═════════════════════════════════════════════
#  초기화
# ═════════════════════════════════════════════
driver = Driver()

# 카메라
try:
    camera        = Camera("camera")
    camera.enable(TIME_STEP)
    camera_width  = camera.getWidth()
    camera_height = camera.getHeight()
    camera_fov    = camera.getFov()
    lane_offset   = camera_width * 0.25  # 한쪽 선만 보일 때 차선 폭 추정값
    has_camera    = True
    print("카메라 초기화 완료")
except Exception as e:
    print(f"카메라 초기화 실패: {e}")
    has_camera = False

# Lidar
try:
    sick       = Lidar("Sick LMS 291")
    sick.enable(TIME_STEP)
    sick_width = sick.getHorizontalResolution()
    sick_fov   = sick.getFov()
    enable_collision_avoidance = True
    print("Lidar 초기화 완료")
except Exception as e:
    print(f"Lidar 초기화 실패: {e}")
    enable_collision_avoidance = False

# GPS (선택)
try:
    gps = GPS("gps")
    gps.enable(TIME_STEP)
    has_gps = True
    print("GPS 초기화 완료")
except Exception as e:
    print(f"GPS 초기화 실패 (없어도 동작): {e}")
    has_gps = False

print_help()

if has_camera:
    set_speed(50.0)

kb = Keyboard()
kb.enable(TIME_STEP)

basic_ts = int(driver.getBasicTimeStep()) if hasattr(driver, 'getBasicTimeStep') else TIME_STEP

# ═════════════════════════════════════════════
#  메인 루프
# ═════════════════════════════════════════════
i = 0

while driver.step() != -1:
    check_keyboard(kb)

    if i % max(1, int(TIME_STEP / max(1, basic_ts))) == 0:

        if autodrive and has_camera:

            # ── 1. 센서 데이터 수집 ──
            raw_angle, is_center_line = process_camera_image(camera)
            lane_line_angle           = filter_angle(raw_angle)

            if enable_collision_avoidance:
                front_dist, left_clear, right_clear = process_sick_data(sick)
            else:
                front_dist, left_clear, right_clear = UNKNOWN, True, True

            # ── 2. 제어값 초기화 ──
            brake_intensity = 0.0
            steer           = steering_angle  # 기본: 현재 각도 유지

            # ══════════════════════════════════════
            #  상태 머신 : 1단계 - 차선 방향으로 꺾기
            # ══════════════════════════════════════
            if lane_state == LANE_STATE_TURN_OUT:
                lane_change_counter -= 1
                steer = lane_change_steer   # 꺾는 방향 고정

                # 1단계 완료 → 2단계(반대 방향 복귀)로 전환
                if lane_change_counter <= 0:
                    lane_state          = LANE_STATE_TURN_IN
                    lane_change_counter = LANE_CHANGE_STEPS
                    steer               = lane_change_return
                    print("차선 변경 2단계: 핸들 복귀 중...")

                # ※ 차선 변경 중 Lidar 전방 판단 비활성화
                #   꺾인 방향으로 가드레일을 장애물로 오인하기 때문
                # (brake_intensity 는 0.0 유지)

            # ══════════════════════════════════════
            #  상태 머신 : 2단계 - 반대 방향으로 핸들 복귀
            # ══════════════════════════════════════
            elif lane_state == LANE_STATE_TURN_IN:
                lane_change_counter -= 1
                steer = lane_change_return  # 복귀 방향 고정

                # 2단계 완료 → 차선 유지(follow)로 복귀
                if lane_change_counter <= 0:
                    lane_state     = LANE_STATE_FOLLOW
                    PID_need_reset = True
                    print("차선 변경 완료, 차선 유지 모드로 복귀")

                # (brake_intensity 는 0.0 유지)

            # ══════════════════════════════════════
            #  상태 머신 : 차선 유지 (기본 주행)
            # ══════════════════════════════════════
            elif lane_state == LANE_STATE_FOLLOW:

                if lane_line_angle != UNKNOWN:
                    # 기본 PID 조향
                    line_steer = applyPID(lane_line_angle)

                    # ── 전방 장애물 감지 ──
                    if enable_collision_avoidance and front_dist != UNKNOWN:

                        # 5m 이내 : 무조건 긴급 제동
                        if front_dist < 5.0:
                            brake_intensity = 1.0
                            steer           = line_steer
                            print("충돌 임박! 긴급 제동!")

                        # 15m 이내 : 차선 변경 시도
                        elif front_dist < 15.0:
                            if is_center_line:
                                # 현재 1차선 → 오른쪽으로만 변경 가능
                                if right_clear:
                                    lane_state          = LANE_STATE_TURN_OUT
                                    lane_change_counter = LANE_CHANGE_STEPS
                                    lane_change_steer   = min(steering_angle + 0.15, 0.5)   # 1단계: 오른쪽
                                    lane_change_return  = max(steering_angle - 0.15, -0.5)  # 2단계: 왼쪽 복귀
                                    PID_need_reset      = True
                                    print("전방 장애물! 오른쪽 차선으로 변경")
                                else:
                                    # 오른쪽도 막혀있으면 비례 제동
                                    brake_intensity = min(max((15.0 - front_dist) / 10.0, 0.2), 1.0)
                                    steer           = line_steer
                                    print("회피 불가 - 제동 (1차선)")
                            else:
                                # 현재 2차선 이상 → 왼쪽 우선
                                if left_clear:
                                    lane_state          = LANE_STATE_TURN_OUT
                                    lane_change_counter = LANE_CHANGE_STEPS
                                    lane_change_steer   = max(steering_angle - 0.15, -0.5)  # 1단계: 왼쪽
                                    lane_change_return  = min(steering_angle + 0.15, 0.5)   # 2단계: 오른쪽 복귀
                                    PID_need_reset      = True
                                    print("전방 장애물! 왼쪽 차선으로 변경")
                                elif right_clear:
                                    lane_state          = LANE_STATE_TURN_OUT
                                    lane_change_counter = LANE_CHANGE_STEPS
                                    lane_change_steer   = min(steering_angle + 0.15, 0.5)   # 1단계: 오른쪽
                                    lane_change_return  = max(steering_angle - 0.15, -0.5)  # 2단계: 왼쪽 복귀
                                    PID_need_reset      = True
                                    print("전방 장애물! 오른쪽 차선으로 변경")
                                else:
                                    brake_intensity = min(max((15.0 - front_dist) / 10.0, 0.2), 1.0)
                                    steer           = line_steer
                                    print("회피 불가 - 제동 (2차선)")
                        else:
                            # 장애물 있어도 15m 이상이면 차선 유지
                            steer = line_steer

                    else:
                        # 장애물 없음 → 순수 차선 추종
                        steer = line_steer

                else:
                    # ── 차선 상실 ──
                    brake_intensity = 0.4
                    PID_need_reset  = True
                    print("차선 상실 - 감속")

            # ── 3. 차량에 명령 전달 ──
            driver.setBrakeIntensity(brake_intensity)
            set_steering_angle(steer)

        # ── GPS 업데이트 ──
        if has_gps:
            compute_gps_speed()

    i += 1