from controller import Robot, Lidar

robot = Robot()
timestep = int(robot.getBasicTimeStep())

sick = robot.getDevice('Sick LMS 291')
sick.enable(timestep)

while robot.step(timestep) != -1:
    image = sick.getRangeImage()
    if image:
        mid = len(image) // 2
        print(f"전체 포인트 수: {len(image)}")
        print(f"중앙(정면): {image[mid]:.2f}m")
        print(f"오른쪽 끝: {image[-1]:.2f}m")
        print(f"왼쪽 끝: {image[0]:.2f}m")
        # 오른쪽 절반 전체 출력
        print("오른쪽 구간별 거리:")
        for x in range(mid, len(image), 5):  # 5칸 간격으로 출력
            print(f"  index {x}: {image[x]:.2f}m")

        if image:
            mid = len(image) // 2
            print(f"전체 포인트 수: {len(image)}")
            print(f"중앙(정면) index {mid}: {image[mid]:.2f}m")
            
            # 오른쪽 구간 상세 출력
            print("\n오른쪽 구간 (중앙~끝):")
            for x in range(mid, len(image), 3):  # 3칸 간격
                val = image[x]
                dist = f"{val:.2f}m" if val != float('inf') else "inf"
                print(f"  index {x}: {dist}")
            
            print("\n왼쪽 구간 (끝~중앙):")
            for x in range(0, mid, 3):  # 3칸 간격
                val = image[x]
                dist = f"{val:.2f}m" if val != float('inf') else "inf"
                print(f"  index {x}: {dist}")