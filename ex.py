from vehicle import Driver
import numpy as np
import cv2

driver = Driver()

camera = driver.getDevice("camera_left")
camera.enable(32)

width = camera.getWidth()
height = camera.getHeight()

speed = 0

while driver.step() != -1:

    image = camera.getImage()

    img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    img = img[:, :, :3]
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    roi = img[int(height*0.3):height, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 흰색 검출
    lower_white = np.array([0,0,180])
    upper_white = np.array([180,60,255])

    mask = cv2.inRange(hsv, lower_white, upper_white)

    white_pixels = cv2.countNonZero(mask)
    

    if white_pixels > 26:
        speed = 20
    else:
        speed = 0

    driver.setCruisingSpeed(speed)

    cv2.imshow("roi", roi)
    cv2.imshow("white mask", mask)

    cv2.waitKey(1)
    print("white_pixels:", white_pixels)