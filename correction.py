i = 0
while driver.step() != -1:
    check_keyboard(kb)

    if i % max(1, int(TIME_STEP / max(1, basic_ts))) == 0:

        front, left_dist, right_dist = process_lidar()

        # ChatGPT 방식 - 두 카메라 동일 처리
        white_list = []
        yellow_list = []

        for cam in [left_camera, right_camera]:
            if cam is None:
                continue
            raw = cam.getImage()
            img = np.frombuffer(raw, np.uint8).reshape((camera_height, camera_width, 4))[:, :, :3]
            white, yellow = detect_lane(img)
            if white != UNKNOWN:
                white_list.append(white)
            if yellow != UNKNOWN:
                yellow_list.append(yellow)

        lane_error = UNKNOWN
        if len(white_list) > 0:
            lane_x = int(np.mean(white_list))
            lane_error = lane_x - camera_width // 2

        yellow_correction = 0
        if len(yellow_list) > 0:
            yellow_x = int(np.mean(yellow_list))
            if yellow_x > camera_width * 0.45:
                yellow_correction = 0.15

        # 상태 머신
        if drive_state == STATE_NORMAL:
            if front < 6:
                avoid_direction = -1 if left_dist > right_dist else 1
                drive_state = STATE_AVOID
                state_timer = 0
            else:
                if lane_error != UNKNOWN:
                    driver.setCruisingSpeed(0.0)  # 브레이크 해제
                    pid = applyPID(lane_error)
                    steer = filter_angle(pid + yellow_correction)
                    set_steering(steer)
                else:
                    # 차선 인식 실패
                    driver.setCruisingSpeed(0.0)
                    PID_need_reset = True

        elif drive_state == STATE_AVOID:
            state_timer += 1
            if front > 10 and state_timer > AVOID_STEPS:
                drive_state = STATE_RETURN
                state_timer = 0
            else:
                set_steering(filter_angle(0.3 * avoid_direction))

        elif drive_state == STATE_RETURN:
            state_timer += 1
            if state_timer > RETURN_STEPS:
                drive_state = STATE_NORMAL
                PID_need_reset = True
            else:
                set_steering(filter_angle(-0.25 * avoid_direction))

        target = auto_speed(front)
        driver.setCruisingSpeed(min(speed, target))

    i += 1