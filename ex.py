from vehicle import Driver
import numpy as np

driver = Driver()
timestep = int(driver.getBasicTimeStep())

camera = driver.getDevice("camera")
camera.enable(timestep)

width = camera.getWidth()
height = camera.getHeight()

while driver.step() != -1:

    image = camera.getImage()

    img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    img = img[:, :, :3]   # RGBA → RGB

    # 화면 중앙 픽셀
    r, g, b = img[height//2][width//2]

    if r > 200 and g > 200 and b > 200:
        print("True - White line detected")

        # 차량 출발
        driver.setCruisingSpeed(20)

    else:
        print("False")

        # 차량 정지
        driver.setCruisingSpeed(0)