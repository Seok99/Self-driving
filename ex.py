# ... (1. 카메라 데이터 받아오기 완료 후) ...

            # 2. 라이다 데이터 받아오기 (복구: 원래 반환값인 각도와 거리로 받기)
            if enable_collision_avoidance:
                obstacle_angle, obstacle_dist = process_sick_data(sick)
            else:
                obstacle_angle, obstacle_dist = UNKNOWN, 0.0
            
            # 초기 제어값 설정
            brake_intensity = 0.0
            steer = steering_angle

            # 3. 상황별 판단 로직
            if lane_line_angle != UNKNOWN:
                # 기본 주행 - 차선 유지 PID 조향각 계산
                line_following_steering = applyPID(lane_line_angle)
                
                # 충돌 방지 기능 ON and 앞에 장애물 발견된 경우
                if enable_collision_avoidance and obstacle_angle != UNKNOWN:
                    
                    # [복구] 원래 짜셨던 똑똑한 비례 회피 조향 로직
                    obstacle_steering = steering_angle
                    if 0.0 < obstacle_angle < 0.4:
                        obstacle_steering = steering_angle + (obstacle_angle - 0.25) / max(0.001, obstacle_dist)
                    elif -0.4 < obstacle_angle <= 0.0: 
                        obstacle_steering = steering_angle + (obstacle_angle + 0.25) / max(0.001, obstacle_dist)

                    # 전방 평균 거리가 15m 이내일 때 회피 판단 시작
                    if obstacle_dist < 15.0:
                        
                        # [핵심] 1차선(노란선)인데 장애물이 있으면 -> 역주행 방지를 위해 무조건 오른쪽 강제 회피
                        if is_center_line:
                            steer = steering_angle - 0.3 
                            PID_need_reset = True
                            
                        # 2차선이면 -> 원래 짜셨던 obstacle_steering을 활용하여 빈 곳으로 부드럽게 회피!
                        else:
                            steer = obstacle_steering
                            PID_need_reset = True
                            
                        # 브레이크 비례 제어 (거리가 가까워질수록 브레이크 강하게)
                        if obstacle_dist < 5.0:
                            brake_intensity = 1.0
                            print("충돌 임박! 긴급 제동!")
                        else:
                            calculated_brake = (15.0 - obstacle_dist) / 10.0
                            brake_intensity = min(max(calculated_brake, 0.0), 0.5)
                            
                    # 거리가 15m 이상이면 평소처럼 차선 유지
                    else:
                        steer = line_following_steering
                
                # 전방에 장애물이 아예 없으면 평소처럼 차선 유지
                else:
                    steer = line_following_steering
            
            # [유지] 차선을 잃은 경우 감속 방어 코드
            else:
                brake_intensity = 0.4
                PID_need_reset = True
                print("차선을 잃었습니다. 감속합니다.")

            # 4. 행동을 차량에 전달
            driver.setBrakeIntensity(brake_intensity)
            set_steering_angle(steer)
            
    i += 1