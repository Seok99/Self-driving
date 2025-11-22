from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera")
camera.enable(timestep)

# Step을 반드시 진행
for _ in range(10):
    robot.step(timestep)

img = camera.getImage()
width = camera.getWidth()
height = camera.getHeight()

for y in range(height):
    for x in range(width):
        r = camera.imageGetRed(img, width, x, y)
        g = camera.imageGetGreen(img, width, x, y)
        b = camera.imageGetBlue(img, width, x, y)

print("R:", r, "G:", g, "B:", b)
