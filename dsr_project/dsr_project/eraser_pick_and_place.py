
import rclpy
import DR_init

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight" #본인 TCP 이름 설정
ROBOT_TOOL = "GripperDA_v1"  #본인 그리퍼 이름 설정

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 60
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp  # 필요한 기능만 임포트

    # Tool과 TCP 설정
    set_tool(ROBOT_TCP)
    set_tcp(ROBOT_TOOL)

    # 설정된 설정값 출력
    # print("#"*50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#"*50)


def perform_task():
    """로봇이 수행할 작업"""
    print("Performing task...")
    from DSR_ROBOT2 import (
        release_compliance_ctrl, release_force,
        check_force_condition,
        task_compliance_ctrl,
        set_desired_force,
        set_ref_coord,
        get_digital_input, get_digital_output, set_digital_output,
        movej, movel,
        get_current_posx, wait,
        set_user_cart_coord,
        get_tool_force,

        DR_MV_MOD_ABS, DR_MV_MOD_REL,
        DR_MV_RA_DUPLICATE,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
        DR_BASE, DR_TOOL,
        OFF, ON,
        posx, posj,
    )
    import math

    # gripper 조절(추가로 그립할 도구에 따라 추가 구현 가능 -> 집는 두께)
    def gripper(switch):
        # gripper 닫기
        if switch == 0:
            set_digital_output(1,ON)
            set_digital_output(2,OFF)
        # gripper 열기
        elif switch == 1:
            set_digital_output(1,OFF)
            set_digital_output(2,ON)


    # pick_e -----------------------------------------------------------------
    # 지우개 위로 이동
    movel(posx(325.82, 267.29, 424.88, 82.80, 175.80, 80.38), vel = VELOCITY, t=2, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    gripper(1)  # 그리퍼 열기
    wait(1.00)
    # 아래로 내려가서 지우개 집기
    movel(posx(325.82, 267.29, 310.78, 82.80, 175.80, 80.38), vel = VELOCITY, t=2, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    gripper(0)  # 그리퍼 닫기
    wait(1.00)
    # 다시 위로 올리기
    movel(posx(325.82, 267.29, 424.88, 82.80, 175.80, 80.38), vel = VELOCITY, t=2, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    # 지우개 놓을 위치로 이동
    # movel(posx(321.79, 67.31, 400.54, 13.71, 180.00, 12.62), vel = VELOCITY, t=2, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)  
    # --------------------------------------------------------------------------------------
		
    # --- Yaw (Rz) 2.3도 회전 적용 ---
    p1 = [324.380, 67.386, 330.540]
    p2 = [608.089, 65.371, 330.540]
    p4 = [326.180, -117.220, 330.540]
    # p_origin (C 값 165.5 + 2.3 = 167.8)
    p_origin = [326.180, -117.220, 330.540, 157.52, 180, 167.8]

    list_x = []
    for i in range(3):
        list_x.append(p2[i]-p1[i])
    A = 0
    for i in list_x:
        A += i**2
    A = math.sqrt(A)
    list_X = [] # x축 단위벡터
    for i in list_x:
        list_X.append(i/A)
    # print(list_X)

    list_y = []
    for i in range(3):
        list_y.append(p1[i]-p4[i])
    B = 0
    for i in list_y:
        B += i**2
    B = math.sqrt(B)
    list_Y = []  # y축 단위벡터
    for i in list_y:
        list_Y.append(i/B)
    # print(list_Y)

    pos = p_origin
    DR_USER1 = set_user_cart_coord(list_X, list_Y, pos, ref=DR_BASE)

    board_h = 214    #도마의 세로 길이 기입
    board_w = 300   #도마의 가로 길이 기입

    sponge_h = 90    #스펀지의 세로 길이 기입
    sponge_w = 25    #스펀지의 가로 길이 기입

    sponge_h = 23    #지우개의 세로 길이 기입
    sponge_w = 23    #지우개의 가로 길이 기입

    # =============================================================
    # 1. 붓질 횟수 자동 계산 (올림)
    # 200 / 95 = 2.105... -> 올림 (ceil)하여 3회
    num_strokes = math.ceil(board_h / sponge_h)

    # 2. 미적 정렬을 위한 최적 이동 간격 계산
    # (마지막 붓 끝이 도마 끝에 오도록 겹침을 균등하게 분배)
    total_overlap = (sponge_h * num_strokes) - board_h
    overlap_count = num_strokes - 1
    
    if overlap_count <= 0:
        # 붓질 1회로 충분하거나(board_h <= sponge_h), 겹침이 필요없는 경우  
        y_step_size = board_h

    else:
        # y_step_size = 붓 너비 - (총 겹침 / 겹침 횟수)
        y_step_size = sponge_h - (total_overlap / overlap_count)
        
    print(f"계산된 붓질 횟수: {num_strokes}회")
    print(f"Y축 이동 간격: {y_step_size:.1f}mm (총 겹침 {total_overlap}mm을 {overlap_count}회 분배)")
    # =============================================================

    move_x = board_w - 8 # x방향 이동할 거리(이 값을 더해주거나 빼주며 이동)
    # move_x = 350 # x방향 이동할 거리(이 값을 더해주거나 빼주며 이동)
    
    #스펀지의 x,y,z 방향 이동 속도& 가속도
    vel_x = 50;acc_x = 50
    vel_y = 50;acc_y = 50
    vel_z = 30;acc_z = 30


    # 초기 홈 정렬 상태
    # home_pose_j = [0, 0, 90, 0, 90, 0]
    # movej(home_pose_j, vel = 30, acc = 30)

    # 초기 위치로 이동
    init_pos = posx([331.79, 65.310, 350.540, 13.71, 180.0, 12.62]) # 픽앤 플레이스로 도달한 위치의 테스크 공간 좌표 값을 넣어야함 !@!
    movel(init_pos, vel=30, t=2, acc=30, ref = DR_BASE, mod = DR_MV_MOD_ABS)
    print('초기 위치로 이동됨')

    # 지우개 다시 잡기 
    movel([0,0,-20,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)
    gripper(1)  # 그리퍼 열기
    wait(1.00)
    movel([0,0,-10,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)
    gripper(0)  # 그리퍼 닫기
    wait(1.00)
    movel([0,0,+10,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)

    print('지우개 재정렬 및 힘제어 준비 완료')

    set_ref_coord(DR_USER1)

    # 힘제어 함수
    def force_control(switch,press_force = 10):
        #switch on(1) 이면 힘제어를 시작합니다.
        if switch == 1:
            print("힘 제어 시작")
            set_ref_coord(1) # Tool 좌표계 설정
            task_compliance_ctrl(stx=[3000, 3000, 200, 200, 200, 200])  #강성 조절(헐겁게)
            wait(0.5) # 안정화 대기(필수)
            set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

            #힘 감지 체크
            while True:
                
                ret = check_force_condition(DR_AXIS_Z, max=press_force)

                if ret == -1:
                    print("Z축 힘이 press_force N 이상 감지됨.",ret)
                    break
            wait(0.5)

        #switch off(0) 이면 힘제어를 중지합니다.
        elif switch == 0:
            print("힘 제어 중지")
            release_force()
            release_compliance_ctrl()

    def oiling_move():
            # 오일링을 하기 위한 도마 위(좌상단)부터 시작
            force_control(1,14) #힘제어 시작

            for y_move_cnt in range(int(num_strokes)):
                
                # x축 방향 이동 (왕복 칠하기)
                if y_move_cnt %2 == 0:
                    force_control(0) #힘제어 종료
                    movel([move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL) # >>> 방향 이동
                    movel([-move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL) # <<< 방향 이동
                    movel([move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL) # >>> 방향 이동
                else:
                    force_control(0) #힘제어 종료
                    movel([-move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL) # <<< 방향 이동
                    movel([move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL) # >>> 방향 이동
                    movel([-move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL) # <<< 방향 이동

                
                # 이제 y_move_cnt가 0~8까지 증가하므로 붓질 횟수가 올바르게 출력됩니다.
                print(f'{y_move_cnt + 1}번째 붓질 완료')

                # 마지막 붓질 후에는 y 이동 불필요
                if y_move_cnt < num_strokes - 1:
                    movel([0, 0, +10,0,0,0], vel = vel_y,t=1, acc = acc_y, ref = DR_USER1, mod = DR_MV_MOD_REL)
                    # y_step_size 만큼 이동
                    movel([0,-y_step_size, 0,0,0,0], vel = vel_y, t=1, acc = acc_y, ref = DR_USER1, mod = DR_MV_MOD_REL) # vvv 방향 이동
                    force_control(1,14) #힘제어 시작
                    print(f'{y_step_size:.1f}mm 다음 라인으로 이동')
                else:
                    # [성공]
                    print('최종 Y 위치에 도달 (도마 끝에 미적으로 정렬됨)')

            # 루프가 하나로 합쳐졌으므로 중간 movel은 필요 없습니다.
            print('oiling 작업 완료')   
                    
            force_control(0) # 힘제어 종료
                    
            # z축 상향 이동
            movel([0,0,100,0,0,0], vel = vel_z, acc = acc_z, ref = DR_USER1, mod = DR_MV_MOD_REL)
            print('상승 완료')

    oiling_move()
    
    # place_e-----------------------------------------------------------------

    # 지우개 위치로 이동
    movel(posx(330.44, 289.61, 400.54, 64.35, -180.00, 63.26), vel = VELOCITY, t=2, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    # 지우개 내리기
    movel(posx(0.00, 0.00, -85.26, 0.00, 0.00, 0.00), vel = VELOCITY, t=2, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    gripper(1)  # 그리퍼 열기

    #---------------------------------------------------------------------------

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        # 작업 수행
        perform_task()
            
    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
