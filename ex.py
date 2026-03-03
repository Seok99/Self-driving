def process_camera_image(cam: Camera):
    REF = (67, 64, 64) # Reference(목표) 아주 어두운 회색 (현재 안 쓰임)
    yellow_RGB = (95, 187, 203)
    white_RGB = (255, 255, 255) # [수정] 임시 흰색 RGB 값 추가
    image = cam.getImage()
    center = camera_width / 2
    
    yellow_left_sum = 0; yellow_left_count = 0
    yellow_right_sum = 0; yellow_right_count = 0
    white_left_sum = 0;   white_left_count = 0
    white_right_sum = 0;  white_right_count = 0

    for y in range(camera_height):
        for x in range(camera_width):
            b = Camera.imageGetBlue(image, camera_width, x, y)
            g = Camera.imageGetGreen(image, camera_width, x, y)
            r = Camera.imageGetRed(image, camera_width, x, y)
            
            if color_diff((b,g,r), yellow_RGB) < 30:
                if x < center:
                    yellow_left_sum += x;  yellow_left_count += 1
                else:
                    yellow_right_sum += x; yellow_right_count += 1
                    
            # [수정] 흰색 차선에도 오차 범위(< 30) 추가
            elif color_diff((b,g,r), white_RGB) < 30: 
                if x < center:
                    white_left_sum += x; white_left_count += 1
                else:
                    white_right_sum += x; white_right_count += 1

    # [핵심 추가] 왼쪽에 노란선(중앙선)이 충분히(예: 30픽셀 이상) 보이는가?
    center_yellow_line = (yellow_left_count > 30) 

    # 왼쪽/오른쪽 선의 평균 계산
    if yellow_left_count > 0:
        left_avg = yellow_left_sum / yellow_left_count
    elif white_left_count > 0:
        left_avg = white_left_sum / white_left_count
    else:
        left_avg = None
    
    if white_left_count > 0:
        right_avg = white_right_sum / white_right_count
    elif yellow_right_count > 0:
        # [수정] 분모 오타 수정 (yellow_right_sum -> yellow_right_count)
        right_avg = yellow_right_sum / yellow_right_count
    else:
        right_avg = None
    
    # 차선 중심 결정
    if left_avg is not None and right_avg is not None:
        lane_center = (left_avg + right_avg) / 2
    elif left_avg is not None:
        lane_center = left_avg + lane_offset
    elif right_avg is not None:
        # [수정] 오른쪽 선만 보이면 가상의 중앙은 왼쪽으로 빼야 함 (-)
        lane_center = right_avg - lane_offset 
    else:
        # [수정] 차선을 못 찾았을 때도 반환값 개수를 맞춰줌 (UNKNOWN, False)
        return UNKNOWN, False 
        
    lane_angle = ((lane_center / camera_width) - 0.5) * camera_fov
    
    # [핵심 추가] 계산된 차선 각도와 중앙선 여부를 함께 반환
    return lane_angle, center_yellow_line