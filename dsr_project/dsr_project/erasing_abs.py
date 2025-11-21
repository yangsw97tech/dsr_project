import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# 이동 속도 및 가속도
VELOCITY = 100
ACC = 100

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, move_periodic, posx  # 필요한 기능만 임포트

    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)

    # Tool과 TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)


def perform_task():
    """로봇이 수행할 작업 (절대 좌표 기준 + 원본 흐름 적용)"""
    print("Performing task...")
    from DSR_ROBOT2 import (
        release_compliance_ctrl,release_force,
        check_force_condition,
        task_compliance_ctrl,
        set_desired_force,
        set_ref_coord,
        movej,
        movel,wait,movej,get_current_posx,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
        DR_BASE,DR_TOOL,
        DR_MV_MOD_ABS,DR_MV_MOD_REL,
        posx,
        set_user_cart_coord,
        get_tool_force,
    )
    import math # 올림 계산을 위한 math 모듈 임포트

    # =============================================================
    # 1. 절대 좌표 4개 지점 정의 (6-DOF)
    # =============================================================
    p1 = [331.79, 67.310, 330.540, 0, -180, 0]   # 상우 (Top-Right)
    p2 = [615.190, 53.910, 330.540, 0, -180, 0]     # 하우 (Bottom-Right)
    p3 = [605.120, -127.91, 330.540, 0, -180, 0]    # 하좌 (Bottom-Left)
    p4 = [326.180, -117.220, 330.540, 0, -180, 0]       # 상좌 (Top-Left)
    
    # 방향(Orientation)은 p4의 값을 기준으로 고정합니다.
    orientation = [p4[3], p4[4], p4[5]]

    # =============================================================
    # 2. 붓질 횟수 및 간격 계산 (원본 코드 로직 사용)
    # =============================================================
    board_h = 200    # 도마의 세로 길이 (Y방향 총 이동 거리)
    board_w = 300    # 도마의 가로 길이 (X방향 붓질 거리) - *참고용*

    sponge_h = 23    # 지우개(도구)의 폭 (mm)
    
    # Z축 상승 높이
    lift_z_height = 10.0 # 10mm

    # 1. 붓질 횟수 자동 계산 (올림)
    if board_h <= 0 or sponge_h <= 0:
        num_strokes = 1
    else:
        num_strokes = math.ceil(board_h / sponge_h)

    # 2. 미적 정렬을 위한 최적 Y축 이동 간격 계산
    total_overlap = (sponge_h * num_strokes) - board_h
    overlap_count = num_strokes - 1
    
    if overlap_count <= 0:
        y_step_size = 0 # 1회 붓질
    else:
        # y_step_size = 붓 너비 - (총 겹침 / 겹침 횟수)
        y_step_size = sponge_h - (total_overlap / overlap_count)
        
    print(f"계산된 붓질 횟수: {num_strokes}회")
    print(f"Y축 이동 간격: {y_step_size:.1f}mm (총 겹침 {total_overlap}mm을 {overlap_count}회 분배)")
    # =============================================================

    #스펀지의 x,y,z 방향 이동 속도& 가속도
    vel_x = 50;acc_x = 50
    vel_y = 50;acc_y = 50
    vel_z = 30;acc_z = 30


    # 초기 홈 정렬 상태
    home_pose_j = [0, 0, 90, 0, 90, 0]
    movej(home_pose_j, vel = 30, acc = 30)

    # 초기 위치(p4) 위로 이동 (Z축 + 20mm)
    approach_pos = p4[:2] + [p4[2] + lift_z_height + 10] + orientation
    movel(approach_pos, vel=30, acc=30, ref = DR_BASE, mod = DR_MV_MOD_ABS)
    print('작업 시작 위치(p4) 상공으로 이동')


    # --- 힘제어 함수 ---
    def force_control(switch, press_force = 10):
        '''
        force_control(1,10) 와 같이 호출하여 10N 힘 제어 시작
        force_control(0)과 같이 호출하여 힘 제어 종료
        '''
        if switch == 1:
            print(f"  ... 힘 제어 시작 ({press_force}N)")
            set_ref_coord(DR_TOOL) # Tool 좌표계 설정
            task_compliance_ctrl(stx=[3000, 3000, 200, 200, 200, 200])
            wait(0.5)
            # Z축으로만 press_force 만큼 힘 적용
            set_desired_force(fd=[0, 0, press_force, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            
            # 힘이 감지될 때까지 대기 (간략화된 버전)
            # 실제로는 check_force_condition을 루프와 함께 사용해야 할 수 있음
            print("  ... 표면 접촉 대기")
            wait(1.0) # 힘 제어가 표면을 누를 시간 (필요시 'check_force_condition' 루프로 대체)

        elif switch == 0:
            print("  ... 힘 제어 중지")
            release_force()
            release_compliance_ctrl()

    # --- Oiling 함수 (절대좌표 기준 + 원본흐름) ---
    def oiling_move():
        
        # 작업 시작점(p4)으로 이동
        movel(p4, vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        # 작업 시작 (첫 번째 힘 제어)
        force_control(1, 11)
        
        # num_strokes 만큼 붓질 수행
        for i in range(num_strokes):
            
            # 0.0 ~ 1.0 사이의 보간 계수(t) 계산
            # Y축 이동은 p4->p3 (좌측) / p1->p2 (우측) 방향으로 y_step_size 만큼 진행
            # t는 총 Y이동(board_h) 대비 현재 Y위치를 나타냄
            current_y_offset = i * y_step_size
            t = current_y_offset / board_h if board_h > 0 else 0.0
            if t > 1.0: t = 1.0

            # 1. 현재 Stroke의 Start, Mid, End 절대 좌표 계산
            
            # Start (Left)
            start_x = p4[0] + t * (p3[0] - p4[0])
            start_y = p4[1] + t * (p3[1] - p4[1])
            start_pose = [start_x, start_y, p4[2]] + orientation

            # End (Right)
            end_x = p1[0] + t * (p2[0] - p1[0])
            end_y = p1[1] + t * (p2[1] - p1[1])
            end_pose = [end_x, end_y, p1[2]] + orientation

            # Mid (중간)
            mid_x = (start_x + end_x) / 2
            mid_y = (start_y + end_y) / 2
            mid_pose = [mid_x, mid_y, p4[2]] + orientation


            # 2. X축 방향 이동 (왕복 칠하기 + 중간 리프트)
            if i % 2 == 0:
                # 짝수번째: Start -> Mid -> End (L->R)
                print(f"--- {i + 1}/{num_strokes}번째 붓질 (L->R) ---")
                force_control(0) # 힘 제어 중지
                movel(mid_pose, vel = vel_x, acc = acc_x, ref = DR_BASE, mod = DR_MV_MOD_ABS)
                
                movel([0, 0, +lift_z_height,0,0,0], vel = vel_y, acc = acc_y, ref = DR_BASE, mod = DR_MV_MOD_REL) # Z축 상승
                force_control(1,11) # 힘 제어 시작 (하강)
                
                force_control(0) # 힘 제어 중지
                movel(end_pose, vel = vel_x, acc = acc_x, ref = DR_BASE, mod = DR_MV_MOD_ABS)
            
            else:
                # 홀수번째: End -> Mid -> Start (R->L)
                print(f"--- {i + 1}/{num_strokes}번째 붓질 (R->L) ---")
                force_control(0) # 힘 제어 중지
                movel(mid_pose, vel = vel_x, acc = acc_x, ref = DR_BASE, mod = DR_MV_MOD_ABS)
                
                movel([0, 0, +lift_z_height,0,0,0], vel = vel_y, acc = acc_y, ref = DR_BASE, mod = DR_MV_MOD_REL) # Z축 상승
                force_control(1,11) # 힘 제어 시작 (하강)
                
                force_control(0) # 힘 제어 중지
                movel(start_pose, vel = vel_x, acc = acc_x, ref = DR_BASE, mod = DR_MV_MOD_ABS)

            print(f'{i + 1}번째 붓질 완료')

            # 3. Y축 방향 이동 (라인 변경) - 마지막 붓질 후에는 불필요
            if i < num_strokes - 1:
                print("  ... 다음 라인으로 이동 ...")
                force_control(0) # 힘제어 종료
                
                # 3a. Z축 상승
                print("    (1) Z축 상승")
                movel([0, 0, +lift_z_height,0,0,0], vel = vel_y, acc = acc_y, ref = DR_BASE, mod = DR_MV_MOD_REL)
                
                # 3b. 다음 라인 시작점 계산
                next_y_offset = (i + 1) * y_step_size
                t_next = next_y_offset / board_h if board_h > 0 else 0.0
                if t_next > 1.0: t_next = 1.0

                if (i+1) % 2 == 0: # 다음이 짝수 (L->R) 이면
                    # 다음 시작점은 'Start' (Left)
                    next_x = p4[0] + t_next * (p3[0] - p4[0])
                    next_y = p4[1] + t_next * (p3[1] - p4[1])
                    next_lane_start_pose = [next_x, next_y, p4[2]] + orientation
                else: # 다음이 홀수 (R->L) 이면
                    # 다음 시작점은 'End' (Right)
                    next_x = p1[0] + t_next * (p2[0] - p1[0])
                    next_y = p1[1] + t_next * (p2[1] - p1[1])
                    next_lane_start_pose = [next_x, next_y, p1[2]] + orientation

                # 3c. 다음 라인 시작점 상공으로 Y축 이동
                print("    (2) 다음 라인 상공으로 이동")
                next_lane_approach_pose = next_lane_start_pose[:2] + [next_lane_start_pose[2] + lift_z_height] + orientation
                movel(next_lane_approach_pose, vel = vel_y, acc = acc_y, ref = DR_BASE, mod = DR_MV_MOD_ABS)
                
                # 3d. Z축 하강 (힘 제어)
                print("    (3) Z축 하강 (힘 제어)")
                force_control(1,11) #힘제어 시작
                
                print(f'{y_step_size:.1f}mm 다음 라인으로 이동 완료')
            
            else:
                print('--- 모든 붓질 완료 ---')


        print('oiling 작업 완료')   
                
        force_control(0) #힘제어 종료
                
        # z축 상향 이동
        movel([0,0,100,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)
        print('상승 완료')
        
        # 홈으로 복귀
        movej(home_pose_j, vel = 30, acc = 30)
        print('홈 복귀 완료')

    oiling_move()



def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_periodic", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        # 작업 수행 (한 번만 호출)
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()