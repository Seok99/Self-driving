from vehicle import Driver
from controller import Camera, Lidar, Display, GPS
import math
X,Y,Z = 0,1,2
TIME_STEP = 50
UNKNOWN = 9999.99

#PID제어 알고리즘
"""KP = 
KI = 
KD = """ #나중에 설정
PID_need_reset = False

FILTER_SIZE = 3

"""필요한거
GPS
Lidar
camera
"""
#flags
has_gps = False
has_camera = False
enable_collision_avoidance = False

#Lidar
sick = None
sick_width = -1
sick_hight = -1
sick_fov = -1.0

#Camera
camera = None
camera_width = -1
camera_hight = -1
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