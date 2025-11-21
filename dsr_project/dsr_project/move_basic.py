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
    """로봇이 수행할 작업"""
    print("Performing task...")

    from DSR_ROBOT2 import (
    release_compliance_ctrl,release_force,
    check_force_condition,
    task_compliance_ctrl,
    set_desired_force,
    set_ref_coord,
    movej,
    movel,wait, get_current_posx,
    DR_FC_MOD_REL,
    DR_AXIS_Z,
    DR_BASE,DR_TOOL,
    DR_MV_MOD_ABS,DR_MV_MOD_REL,
    posx,
    set_user_cart_coord
    )

    import math

    #스펀지의 x,y,z 방향 이동 속도& 가속도
    vel_x = 50;acc_x = 50
    vel_y = 50;acc_y = 50
    vel_z = 30;acc_z = 30

    p1 = [321.79, 77.160, 310.540]   # 내자리 기준 상우
    p2 = [618.860, 65.930, 309.570]     # 하우
    p4 = [317.6, -120.8, 307.8]       # 상좌

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

        # 힘제어 함수
    def force_control(switch,press_force = 10):
        '''
        force_control(1,15) 와 같이 호출하여 15N 힘 제어 시작(기본 힘 10N)
        force_control(0)과 같이 호출하여 힘 제어 종료
        '''
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
                print("Z축 힘이 press_force N 이상이 될 때까지 대기...", ret)
                if ret == -1:
                    print("Z축 힘이 press_force N 이상 감지됨.",ret)
                    break
            wait(0.5)

        #switch off(0) 이면 힘제어를 중지합니다.
        elif switch == 0:
            print("힘 제어 중지")
            release_force()
            release_compliance_ctrl()


    pos = [317.6, -120.8, 307.8, 157.52, 180, 165.5]
    DR_USER1 = set_user_cart_coord(list_X, list_Y, pos, ref=DR_BASE)

    # 초기 홈 정렬 상태
    home_pose_j = [0, 0, 90, 0, 90, 0]
    movej(home_pose_j, vel = 30, acc = 30)
    # 초기 위치로 이동
    init_pos = posx([346.97, 48.270, 450.540, 13.71, 180.0, 12.62]) # 픽앤 플레이스로 도달한 위치의 테스크 공간 좌표 값을 넣어야함 !@!
    wait(1.0)
    movel(init_pos, vel=30, acc=30, ref = DR_BASE, mod = DR_MV_MOD_ABS)
    print('초기 위치로 이동됨')

    force_control(1,15) #힘제어 시작
    move_x = 300

    movel([300,0,0,0,0,0], vel = vel_x, acc = acc_x, mod = DR_MV_MOD_REL) # >>> 방향 이동
    # movel([300,0,0,0,0,0], vel = vel_x, acc = acc_x, ref=DR_BASE mod = DR_MV_MOD_REL) # >>> 방향 이동

    wait(2.0)
    force_control(0) #힘제어 종료
    return

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