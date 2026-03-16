from vehicle import Driver
from controller import Camer, Lidar, GPS, Keyboard, DistanceSensor
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

#──GPS 좌표계 단순화 & STEP 지정 & UNKNOWN──────────────
X,Y,Z = 0,1,2
TIME_STEP = 50
UNKNOWN = 9999.99

#──PID게인──────────────
KP = 0.25
KI = 0.006
KD = 2.0
PID_need_reset = False

#──필터크기──────────────
FILTER_SIZE = 3

#──플래그──────────────
has_camera = False
collision_avoidance = False #충돌 방지 기능
enable_display = False

#──센서 객체 생성──────────────
#Camera변수
left_camera = None #왼쪽 카메라
right_camera = None #오른쪽 카메라
camera_width = -1
camera_height = -1

#Lidar
sick = None
sick_width = -1
sick_fov = -1.0

#──주행 상태──────────────
speed = 0.0
steering_angle = 0.0

#──상태(장애물 회피)──────────────
STATE_NORMAL = "NORMAL" #기본 상태
STATE_AVOID  = "AVOID"  #장애물회피 상태
STATE_RETURN = "RETURN" #차선복귀 상태
drive_state = STATE_NORMAL
avoid_direction = 1 # -1 : 왼쪽 회피, 1 : 오른쪽 회피
state_timer = 0
AVOID_STEPS = 35  #회피 최소 유지 스텝
RETURN_STEPS = 70 #복귀 지속 스텝

#──HSV색상 범위──────────────
LOWER_WHITE = np.array([0, 0, 180])
UPPER_WHITE = np.array([180, 60, 255])
LOWER_YELLOW = np.array([20,100, 100])
UPPER_YELLOW = np.array([35, 255, 255])

#──도움말──────────────
def print_help():
    print("You can drive this car!")
    print("[UP]/[DOWN] - accelerate/slow down")

#──속도 조절──────────────
def set_speed(kmh : float):
    global speed
    kmh = max(0.0, min(kmh, 150.0))
    speed = kmh
    driver.setCruisingSpeed(speed)

#──바퀴angle조절──────────────
def set_steering_angle(wheel_angle: float):
    global steering_angle
    #급격한 조향 변화 억제
    if wheel_angle - steering_angle > 0.1:
        wheel_angle = steering_angle + 0.1
    if wheel_angle - steering_angle < -0.1:
        wheel_angle = steering_angle - 0.1
    steering_angle = wheel_angle
    #최대 조향각 설정 -> 0.5rad
    wheel_angle = max(-0.5, min(wheel_angle, 0.5))
    driver.setSteeringAngle(wheel_angle)

#──Keyboard 조작──────────────
def check_keyboard(kb: Keyboard):
    key = kb.getKey()
    if key == Keyboard.UP:
        set_speed(speed+1.0)
    elif key == Keyboard.DOWN:
        set_speed(speed-1.0)
    
# ── 차선 인식 (HSV 방식 적용) ────────────────────────
def detect_white