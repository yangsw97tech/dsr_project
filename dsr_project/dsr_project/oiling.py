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
        release_compliance_ctrl,release_force,
        check_force_condition,
        task_compliance_ctrl,
        set_desired_force,
        set_ref_coord,
        get_digital_input, get_digital_output, set_digital_output,
        movej, movel,
        get_current_posx, wait,
        get_tool_force,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
        DR_BASE, DR_TOOL,
        DR_MV_MOD_ABS,DR_MV_MOD_REL,
        DR_MV_RA_DUPLICATE,
        OFF,ON,
        posx,posj,
    )
    import math

    # 초기 홈 정렬 상태
    home_pose_j = [0, 0, 90, 0, 90, 0]
    movej(home_pose_j, vel = 30, acc = 30)

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

    # pick
    movel(posx(352.89, 241.49, 333.53, 42.12, 179.97, 42.13), vel = VELOCITY, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(0.00, 0.00, 97.75, 0.00, 0.00, 0.00), vel = VELOCITY, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    movel(posx(140.00, -7.98, 0.00, 0.00, 0.00, 0.00), vel = VELOCITY, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    movel(posx(0.00, 0.00, -85.78, 0.00, 0.00, 0.00), vel = VELOCITY, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    
    gripper(0)  # 그리퍼 닫기
    wait(1.00)

    movel(posx(0.00, 0.00, 119.42, 0.00, 0.00, 0.00), vel = VELOCITY, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
    movel(posx(-150.00, -199.10, 0.00, 0.00, 0.00, 0.00), vel = VELOCITY, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)

    board_h = 200   # 도마의 세로 길이 기입
    board_w = 300   # 도마의 가로 길이 기입

    sponge_h = 95    # 스폰지의 세로 길이 기입
    sponge_w = 25    # 스폰지의 가로 길이 기입

     # =============================================================
    # 1. 붓질 횟수 자동 계산 (올림)
    num_strokes = math.ceil(board_h / sponge_h)    # 3
    
    # 2. 미적 정렬을 위한 최적 이동 간격 계산
    total_overlap = (sponge_h * num_strokes) - board_h    # 겹치는 범위: 285-200 = 85
    overlap_count = num_strokes - 1    # 겹치는 개수: 2
    
    if overlap_count <= 0:
        y_step_size = board_h
    else:
        y_step_size = sponge_h - (total_overlap / overlap_count)    # 95-85/2 = 52.5
        
    print(f"계산된 붓질 횟수: {num_strokes}회")     # 3회
    print(f"Y축 이동 간격: {y_step_size:.1f}mm (총 겹침 {total_overlap}mm을 {overlap_count}회 분배)")
    # =============================================================

    # move_x = board_w - sponge_w # x방향 이동할 거리(이 값을 더해주거나 빼주며 이동)
    move_x = board_w - sponge_w     # 275

    #스펀지의 x,y,z 방향 이동 속도& 가속도
    vel_x = 50;acc_x = 50
    vel_y = 50;acc_y = 50
    vel_z = 30;acc_z = 30

    def oiling_move():
        movel([0,0,-65,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)
        movel([move_x+10,0,0,0,0,0], vel = vel_x, acc = acc_x, ref = DR_BASE, mod = DR_MV_MOD_REL)
        movel([0,0,20,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)
        movel([0,-65,0,0,0,0], vel = vel_y, acc = acc_y, ref = DR_BASE, mod = DR_MV_MOD_REL)
        movel([0,0,-20,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)
        movel([-move_x-25,0,0,0,0,0], vel = vel_x, acc = acc_x, ref = DR_BASE, mod = DR_MV_MOD_REL)
        movel([0,0,20,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)
        movel([0,-65,0,0,0,0], vel = vel_y, acc = acc_y, ref = DR_BASE, mod = DR_MV_MOD_REL)
        movel([0,0,-20,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)
        movel([move_x+20,0,0,0,0,0], vel = vel_x, acc = acc_x, ref = DR_BASE, mod = DR_MV_MOD_REL)
    
    oiling_move()

    # place
    movel(posx(598.34, -93.08, 463.79, 42.80, 179.97, 42.81), vel = VELOCITY, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(505.55, 233.19, 463.79, 43.27, 179.97, 43.28), vel = VELOCITY, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
    movel(posx(0.00, 0.00, -116.69, 0.00, 0.00, 0.00), vel = VELOCITY, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)

    gripper(1)  # 그리퍼 열기
    wait(1.00)

    ## 홈위치로 정렬 
    home_pose_j = [0, 0, 90, 0, 90, 0]
    movej(home_pose_j, vel = 30, acc = 30)

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