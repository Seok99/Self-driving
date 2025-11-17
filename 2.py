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