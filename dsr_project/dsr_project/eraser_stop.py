import rclpy
import DR_init
from std_msgs.msg import String

STATE = "RUNNING"   # RUNNING / PAUSED

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"  # 본인 TCP 이름 설정
ROBOT_TOOL = "GripperDA_v1"  # 본인 그리퍼 이름 설정

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 60
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp  # 필요한 기능만 임포트

    set_tool(ROBOT_TCP)
    set_tcp(ROBOT_TOOL)

    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)


def control_callback(msg: String):
    """STOP / RESUME 명령을 받는 콜백"""
    global STATE

    cmd = msg.data.upper().strip()
    print(f"[ARM CONTROL] 수신: {cmd}")

    if cmd == "STOP":
        if STATE != "PAUSED":
            print(">>> STOP 수신 → STATE=PAUSED (다음 동작부터 대기)")
        STATE = "PAUSED"

    elif cmd == "RESUME":
        if STATE != "RUNNING":
            print(">>> RESUME 수신 → STATE=RUNNING (동작 재개)")
        STATE = "RUNNING"

    else:
        print(f">>> [ARM CONTROL] 알 수 없는 명령: {cmd}")


def wait_until_running(node):
    """STATE가 RUNNING 될 때까지 토픽 콜백 처리하며 대기"""
    global STATE
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        if STATE == "RUNNING":
            break


def perform_task(node):
    """로봇이 수행할 작업 (STOP/RESUME 대응)"""
    print("Performing task with STOP/RESUME support...")
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
        if switch == 0:   # 닫기
            set_digital_output(1, ON)
            set_digital_output(2, OFF)
        elif switch == 1: # 열기
            set_digital_output(1, OFF)
            set_digital_output(2, ON)

    # -------------------- pick_e --------------------
    # 지우개 위로 이동
    wait_until_running(node)
    movel(posx(325.82, 267.29, 424.88, 82.80, 175.80, 80.38),
          vel=VELOCITY, t=2, acc=ACC,
          radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    wait_until_running(node)
    gripper(1)  # 그리퍼 열기
    wait(1.00)

    # 아래로 내려가서 지우개 집기
    wait_until_running(node)
    movel(posx(325.82, 267.29, 310.78, 82.80, 175.80, 80.38),
          vel=VELOCITY, t=2, acc=ACC,
          radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    wait_until_running(node)
    gripper(0)  # 그리퍼 닫기
    wait(1.00)

    # 다시 위로 올리기
    wait_until_running(node)
    movel(posx(325.82, 267.29, 424.88, 82.80, 175.80, 80.38),
          vel=VELOCITY, t=2, acc=ACC,
          radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    # --- Yaw (Rz) 2.3도 회전 적용 ---
    p1 = [324.380, 67.386, 330.540]
    p2 = [608.089, 65.371, 330.540]
    p4 = [326.180, -117.220, 330.540]
    p_origin = [326.180, -117.220, 330.540, 157.52, 180, 167.8]

    list_x = [p2[i]-p1[i] for i in range(3)]
    A = math.sqrt(sum(i**2 for i in list_x))
    list_X = [i/A for i in list_x]

    list_y = [p1[i]-p4[i] for i in range(3)]
    B = math.sqrt(sum(i**2 for i in list_y))
    list_Y = [i/B for i in list_y]

    pos = p_origin
    DR_USER1 = set_user_cart_coord(list_X, list_Y, pos, ref=DR_BASE)

    board_h = 214
    board_w = 300

    # 지우개 크기
    sponge_h = 23
    sponge_w = 23

    # 붓질 파라미터 계산
    num_strokes = math.ceil(board_h / sponge_h)
    total_overlap = (sponge_h * num_strokes) - board_h
    overlap_count = num_strokes - 1

    if overlap_count <= 0:
        y_step_size = board_h
    else:
        y_step_size = sponge_h - (total_overlap / overlap_count)

    print(f"계산된 붓질 횟수: {num_strokes}회")
    print(f"Y축 이동 간격: {y_step_size:.1f}mm (총 겹침 {total_overlap}mm을 {overlap_count}회 분배)")

    move_x = board_w - 8

    vel_x = 50; acc_x = 50
    vel_y = 50; acc_y = 50
    vel_z = 30; acc_z = 30

    # 초기 위치로 이동
    init_pos = posx([331.79, 65.310, 350.540, 13.71, 180.0, 12.62])
    wait_until_running(node)
    movel(init_pos, vel=30, t=2, acc=30, ref=DR_BASE, mod=DR_MV_MOD_ABS)
    print('초기 위치로 이동됨')

    # 지우개 다시 잡기 
    wait_until_running(node)
    movel([0, 0, -20, 0, 0, 0],
          vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
    gripper(1)
    wait(1.00)

    wait_until_running(node)
    movel([0, 0, -10, 0, 0, 0],
          vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
    gripper(0)
    wait(1.00)

    wait_until_running(node)
    movel([0, 0, +10, 0, 0, 0],
          vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)

    print('지우개 재정렬 및 힘제어 준비 완료')

    set_ref_coord(DR_USER1)

    def force_control(switch, press_force=10):
        if switch == 1:
            print("힘 제어 시작")
            set_ref_coord(1)  # Tool 좌표계
            task_compliance_ctrl(stx=[3000, 3000, 200, 200, 200, 200])
            wait(0.5)
            set_desired_force(fd=[0, 0, 15, 0, 0, 0],
                              dir=[0, 0, 1, 0, 0, 0],
                              mod=DR_FC_MOD_REL)

            while True:
                ret = check_force_condition(DR_AXIS_Z, max=press_force)
                if ret == -1:
                    print("Z축 힘이 press_force N 이상 감지됨.", ret)
                    break
            wait(0.5)

        elif switch == 0:
            print("힘 제어 중지")
            release_force()
            release_compliance_ctrl()

    def oiling_move():
        # force_control(1, 14)  # 힘제어 시작

        for y_move_cnt in range(int(num_strokes)):

            if y_move_cnt % 2 == 0:
                # force_control(0)
                wait_until_running(node)
                movel([move_x, 0, 0, 0, 0, 0],
                      vel=vel_x, t=2, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                wait_until_running(node)
                movel([-move_x, 0, 0, 0, 0, 0],
                      vel=vel_x, t=2, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                wait_until_running(node)
                movel([move_x, 0, 0, 0, 0, 0],
                      vel=vel_x, t=2, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
            else:
                # force_control(0)
                wait_until_running(node)
                movel([-move_x, 0, 0, 0, 0, 0],
                      vel=vel_x, t=2, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                wait_until_running(node)
                movel([move_x, 0, 0, 0, 0, 0],
                      vel=vel_x, t=2, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                wait_until_running(node)
                movel([-move_x, 0, 0, 0, 0, 0],
                      vel=vel_x, t=2, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)

            print(f'{y_move_cnt + 1}번째 붓질 완료')

            if y_move_cnt < num_strokes - 1:
                wait_until_running(node)
                movel([0, 0, +10, 0, 0, 0],
                      vel=vel_y, t=1, acc=acc_y, ref=DR_USER1, mod=DR_MV_MOD_REL)
                wait_until_running(node)
                movel([0, -y_step_size, 0, 0, 0, 0],
                      vel=vel_y, t=1, acc=acc_y, ref=DR_USER1, mod=DR_MV_MOD_REL)
                # force_control(1, 14)
                print(f'{y_step_size:.1f}mm 다음 라인으로 이동')
            else:
                print('최종 Y 위치에 도달 (도마 끝에 미적으로 정렬됨)')

        print('oiling 작업 완료')
        # force_control(0)

        wait_until_running(node)
        movel([0, 0, 100, 0, 0, 0],
              vel=vel_z, acc=acc_z, ref=DR_USER1, mod=DR_MV_MOD_REL)
        print('상승 완료')

    # 붓질 동작
    oiling_move()

    # -------------------- place_e --------------------
    wait_until_running(node)
    movel(posx(330.44, 289.61, 400.54, 64.35, -180.00, 63.26),
          vel=VELOCITY, t=2, acc=ACC,
          radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

    wait_until_running(node)
    movel(posx(0.00, 0.00, -85.26, 0.00, 0.00, 0.00),
          vel=VELOCITY, t=2, acc=ACC,
          radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)

    gripper(1)  # 그리퍼 열기
    print("=== 작업 완료 ===")


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    # STOP/RESUME 토픽 구독
    node.create_subscription(
        String,
        "/arm_control",
        control_callback,
        10
    )

    try:
        initialize_robot()
        perform_task(node)

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()