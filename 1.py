# Autonomous vehicle controller example (Python port)
# Original: Cyberbotics C controller
# Ported to Python for Webots vehicle Driver API

from vehicle import Driver #차량 전용 컨트롤러
from controller import Camera, Lidar, Display, GPS, Keyboard
import math

# --- Constants & Globals ---
X, Y, Z = 0, 1, 2
TIME_STEP = 50
UNKNOWN = 99999.99

# PID gains (line following)
KP = 0.25
KI = 0.006
KD = 2.0
PID_need_reset = False

# Size of the yellow line angle filter (moving average)
FILTER_SIZE = 3

# Feature flags (auto-detected)
"""런타임에 센서 존재 여부를 감지하여 설정
센서가 없으면 해당 기능을 비활성화하여 오류 방지"""
enable_collision_avoidance = False
enable_display = False
has_gps = False
has_camera = False

# Devices & parameters
"""camera, sick(lidar), display의 변수를 생성하고 객체를 지정하지 않았기때문에 없을때를 대비하여 fov(시야각)을 -1로 함 """
camera = None
camera_width = -1
camera_height = -1
camera_fov = -1.0

sick = None
sick_width = -1
sick_range = -1.0
sick_fov = -1.0

display = None
display_width = 0
display_height = 0
speedometer_image = None

#gps
gps = None
gps_coords = [0.0, 0.0, 0.0]
gps_speed = 0.0 #현재 차량 속도

# State
speed = 0.0
steering_angle = 0.0
manual_steering = 0
autodrive = True

# Wiper modes (Python Driver uses ints: 0=OFF, 1=SLOW, 2=FAST)
WIPER_OFF, WIPER_SLOW, WIPER_FAST = 0, 1, 2


def print_help():
    print("You can drive this car!")
    print("Select the 3D window and then use the cursor keys to:")
    print("[LEFT]/[RIGHT] - steer")
    print("[UP]/[DOWN] - accelerate/slow down")


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


# set target speed

def set_speed(kmh: float):
    global speed
    if kmh > 250.0:
        kmh = 250.0
    speed = kmh
    print(f"setting speed to {kmh:g} km/h")
    driver.setCruisingSpeed(kmh)


# positive: turn right, negative: turn left

def set_steering_angle(wheel_angle: float):
    global steering_angle
    # limit the difference with previous steering_angle (rate limit)
    if wheel_angle - steering_angle > 0.1:
        wheel_angle = steering_angle + 0.1
    if wheel_angle - steering_angle < -0.1:
        wheel_angle = steering_angle - 0.1
    steering_angle = wheel_angle
    # limit absolute range
    if wheel_angle > 0.5:
        wheel_angle = 0.5
    elif wheel_angle < -0.5:
        wheel_angle = -0.5
    driver.setSteeringAngle(wheel_angle)


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


def check_keyboard(kb: Keyboard):
    key = kb.getKey()
    if key == Keyboard.UP:
        set_speed(speed + 5.0)
    elif key == Keyboard.DOWN:
        set_speed(speed - 5.0)
    elif key == Keyboard.RIGHT:
        change_manual_steer_angle(+1)
    elif key == Keyboard.LEFT:
        change_manual_steer_angle(-1)
    elif key in (ord('A'), ord('a')):
        set_autodrive(True)


# compute rgb difference (|a-b| summed across channels)

def color_diff(a, b):
    diff = 0
    for i in range(3):
        d = a[i] - b[i]
        diff += d if d > 0 else -d
    return diff


# returns approximate angle of yellow road line or UNKNOWN

def process_camera_image(cam: Camera):
    # reference color in BGR
    REF = (95, 187, 203)
    num_pixels = camera_height * camera_width
    sumx = 0
    pixel_count = 0

    image = cam.getImage()
    # Iterate linearly over all pixels (BGRA order, 4 bytes per pixel)
    # We'll scan by (x, y) using accessors to avoid manual index math
    for y in range(camera_height):
        for x in range(camera_width):
            b = Camera.imageGetBlue(image, camera_width, x, y)
            g = Camera.imageGetGreen(image, camera_width, x, y)
            r = Camera.imageGetRed(image, camera_width, x, y)
            if color_diff((b, g, r), REF) < 30:
                sumx += x
                pixel_count += 1

    if pixel_count == 0:
        return UNKNOWN

    avg_x = sumx / pixel_count
    # map normalized x in [0,1] then shift center and scale by FOV
    return ((avg_x / camera_width) - 0.5) * camera_fov


# simple moving average filter for angle
_filter_initialized = False
_filter_buffer = [0.0] * FILTER_SIZE

def filter_angle(new_value: float):
    global _filter_initialized, _filter_buffer
    if (not _filter_initialized) or new_value == UNKNOWN:
        _filter_initialized = True
        _filter_buffer = [0.0] * FILTER_SIZE
    else:
        # shift left
        for i in range(FILTER_SIZE - 1):
            _filter_buffer[i] = _filter_buffer[i + 1]

    if new_value == UNKNOWN:
        return UNKNOWN
    _filter_buffer[FILTER_SIZE - 1] = new_value
    return sum(_filter_buffer) / FILTER_SIZE


# returns approximate angle of obstacle or UNKNOWN; also returns obstacle distance via out param

def process_sick_data(sick_dev: Lidar):
    global sick_width, sick_fov
    HALF_AREA = 20  # ~20 degrees wide middle area
    image = sick_dev.getRangeImage()
    if not image or sick_width <= 0:
        return UNKNOWN, 0.0

    sumx = 0
    collision_count = 0
    obstacle_dist = 0.0

    start = int(sick_width / 2 - HALF_AREA)
    end = int(sick_width / 2 + HALF_AREA)
    if start < 0:
        start = 0
    if end > len(image):
        end = len(image)

    for x in range(start, end):
        r = image[x]
        if r < 20.0:
            sumx += x
            collision_count += 1
            obstacle_dist += r

    if collision_count == 0:
        return UNKNOWN, 0.0

    obstacle_dist /= collision_count
    angle = ((sumx / collision_count) / sick_width - 0.5) * sick_fov
    return angle, obstacle_dist


def update_display():
    global speedometer_image
    NEEDLE_LENGTH = 50.0
    # background
    display.imagePaste(speedometer_image, 0, 0, False)

    # needle
    current_speed = driver.getCurrentSpeed()
    if math.isnan(current_speed):
        current_speed = 0.0
    alpha = current_speed / 260.0 * 3.72 - 0.27
    x = int(-NEEDLE_LENGTH * math.cos(alpha))
    y = int(-NEEDLE_LENGTH * math.sin(alpha))
    display.drawLine(100, 95, 100 + x, 95 + y)

    # text
    display.drawText(f"GPS coords: {gps_coords[X]:.1f} {gps_coords[Z]:.1f}", 10, 130)
    display.drawText(f"GPS speed:  {gps_speed:.1f}", 10, 140)


def compute_gps_speed():
    global gps_speed, gps_coords
    coords = gps.getValues()
    speed_ms = gps.getSpeed()
    gps_speed = speed_ms * 3.6
    gps_coords = list(coords)


def applyPID(yellow_line_angle: float):
    global PID_need_reset
    # static-like variables
    if not hasattr(applyPID, "oldValue"):
        applyPID.oldValue = 0.0
    if not hasattr(applyPID, "integral"):
        applyPID.integral = 0.0

    if PID_need_reset:
        applyPID.oldValue = yellow_line_angle
        applyPID.integral = 0.0
        PID_need_reset = False

    # anti-windup: reset integral when sign flips
    if math.copysign(1.0, yellow_line_angle) != math.copysign(1.0, applyPID.oldValue):
        applyPID.integral = 0.0

    diff = yellow_line_angle - applyPID.oldValue

    # limit integral
    if -30 < applyPID.integral < 30:
        applyPID.integral += yellow_line_angle

    applyPID.oldValue = yellow_line_angle
    return KP * yellow_line_angle + KI * applyPID.integral + KD * diff


# --- Initialization ---
driver = Driver()

# Try to attach optional devices safely
try:
    camera = Camera('camera')
    camera.enable(TIME_STEP)
    has_camera = True
    camera_width = camera.getWidth()
    camera_height = camera.getHeight()
    camera_fov = camera.getFov()
except Exception:
    has_camera = False
    camera = None

try:
    sick = Lidar('Sick LMS 291')
    sick.enable(TIME_STEP)
    enable_collision_avoidance = True
    sick_width = sick.getHorizontalResolution()
    sick_range = sick.getMaxRange()
    sick_fov = sick.getFov()
except Exception:
    enable_collision_avoidance = False
    sick = None

try:
    gps = GPS('gps')
    gps.enable(TIME_STEP)
    has_gps = True
except Exception:
    has_gps = False
    gps = None

try:
    display = Display('display')
    enable_display = True
    speedometer_image = display.imageLoad('speedometer.png')
except Exception:
    enable_display = False
    display = None

# Lights & startup
if has_camera:
    set_speed(50.0)

driver.setHazardFlashers(True)
driver.setDippedBeams(True)
driver.setAntifogLights(True)
driver.setWiperMode(WIPER_SLOW)

print_help()

# Keyboard
kb = Keyboard()
kb.enable(TIME_STEP)

# Main loop
i = 0
basic_ts = int(driver.getBasicTimeStep()) if hasattr(driver, 'getBasicTimeStep') else TIME_STEP

while driver.step() != -1:
    # user input
    check_keyboard(kb)

    # update sensors every TIME_STEP
    if i % max(1, int(TIME_STEP / max(1, basic_ts))) == 0:
        if autodrive and has_camera:
            yellow_line_angle = filter_angle(process_camera_image(camera))

            if enable_collision_avoidance:
                obstacle_angle, obstacle_dist = process_sick_data(sick)
            else:
                obstacle_angle, obstacle_dist = UNKNOWN, 0.0

            if enable_collision_avoidance and obstacle_angle != UNKNOWN:
                # obstacle detected
                driver.setBrakeIntensity(0.0)
                obstacle_steering = steering_angle
                if 0.0 < obstacle_angle < 0.4:
                    obstacle_steering = steering_angle + (obstacle_angle - 0.25) / max(0.001, obstacle_dist)
                elif obstacle_angle > -0.4:
                    obstacle_steering = steering_angle + (obstacle_angle + 0.25) / max(0.001, obstacle_dist)

                steer = steering_angle
                if yellow_line_angle != UNKNOWN:
                    line_following_steering = applyPID(yellow_line_angle)
                    if obstacle_steering > 0 and line_following_steering > 0:
                        steer = max(obstacle_steering, line_following_steering)
                    elif obstacle_steering < 0 and line_following_steering < 0:
                        steer = min(obstacle_steering, line_following_steering)
                else:
                    PID_need_reset = True

                set_steering_angle(steer)

            elif yellow_line_angle != UNKNOWN:
                # follow line
                driver.setBrakeIntensity(0.0)
                set_steering_angle(applyPID(yellow_line_angle))
            else:
                # lost line, slow down
                driver.setBrakeIntensity(0.4)
                PID_need_reset = True

        # update auxiliary UI/sensors
        if has_gps:
            compute_gps_speed()
        if enable_display:
            update_display()

    i += 1

# (No explicit cleanup needed in Python)