from vehicle import Driver
from controller import Camer, Lidar, GPS, Keyboard, DistanceSensor
import math

X,Y,Z = 0,1,2
TIME_STEP = 50
UNKNOWN = 9999.99

#1. 중앙선(노란석)인식 가능함 -> 있으면 노란색으로 주행, 흰색점선은 인식 못함 -> 대체재로 2번 방법 생각중...
#2. 장애물 회피해서 2차선 도로일 경우 끝경계를 도로의 색(67,64,64)이 아닌 경계를 찾아 중앙을 주행하게
#   but 2차선 이상의 거리면?
#3. 가드레일이 있으면 DistanceSensor을 이용해서 거리 이격하여 주행
# but 주행 중, 램프(Ramp)처럼 본선에 합류하는 거나 빠지는 길이 있다면?
#4. setCruisingSpeed로 속도를 조절하는데 setBrakeIntensity가 1.0이여도 왜 차량이 멈추지 않는지?
