import rclpy
import DR_init
from std_msgs.msg import String

STATE = "RUNNING"   # 기본 상태

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

def control_callback(msg: String):
    """STOP / RESUME 명령을 받는 콜백"""
    global STATE
    from DSR_ROBOT2 import motion_pause, motion_resume
    cmd = msg.data.upper()
    print(f"[ARM CONTROL] 수신: {cmd}")

    if cmd == "STOP":
        # 이미 PAUSED면 중복으로 호출 안 해도 되지만, 안전하게 한 번만
        if STATE != "PAUSED":
            print(">>> STOP 수신 → motion_pause() + STATE=PAUSED")
            motion_pause()   # 현재 수행 중인 모션 일시정지 :contentReference[oaicite:5]{index=5}
        STATE = "PAUSED"

    elif cmd == "RESUME":
        if STATE != "RUNNING":
            print(">>> RESUME 수신 → motion_resume() + STATE=RUNNING")
            motion_resume()  # 일시정지된 모션 재개 
        STATE = "RUNNING"

    else:
        print(f">>> [ARM CONTROL] 알 수 없는 명령: {cmd}")

def perform_task(node):
    """로봇이 수행할 작업 (STOP/RESUME 대응)"""
    from DSR_ROBOT2 import posx, amovej, amovel, check_motion, MoveResume, MoveStop

    print("Performing task with STOP/RESUME support...")

    # 초기 위치 및 목표 위치 설정
    JReady = [0, 0, 90, 0, 90, 0]
    pos1 = posx([500, 0, 200, 90, 180, 90])

    # 어느 방향으로 갈 차례인지 (0: JReady, 1: pos1)
    phase = 0

    # 현재 모션 진행 중인지 플래그
    motion_in_progress = False

    global STATE

    while rclpy.ok():
        # 1) ROS2 콜백 처리 (STOP/RESUME 수신)
        rclpy.spin_once(node, timeout_sec=0.01)

        # 2) 현재 모션 상태 확인
        if motion_in_progress:
            st = check_motion()   # 0: IDLE, 1: INIT, 2: BUSY :contentReference[oaicite:7]{index=7}
            # print(f"check_motion = {st}")
            if st == 0:           # DR_STATE_IDLE -> 모션 종료
                motion_in_progress = False

        # 3) RUNNING 상태이고, 현재 모션이 없을 때만 다음 명령 내보내기
        if (STATE == "RUNNING") and (not motion_in_progress):
            if phase == 0:
                print(">>> amovej -> JReady")
                amovej(JReady, vel=VELOCITY, acc=ACC)   # 비동기 조인트 모션 :contentReference[oaicite:8]{index=8}
                phase = 1
            else:
                print(">>> amovel -> pos1")
                amovel(pos1, vel=VELOCITY, acc=ACC)     # 비동기 직선 모션
                phase = 0

            motion_in_progress = True
            
        # 4) PAUSED 상태일 때는:
        # - motion_pause()가 콜백에서 이미 호출되어 있어서
        # - 여기서는 그냥 다음 모션을 안 쏘고 대기만 하면 됨

        # 너무 빡세게 돌지 않게 살짝 쉬어줌
        # (실제론 spin_once timeout으로도 어느 정도 쉬지만, 여유로 한 번 더)
        # rclpy.sleep은 없으니까, 필요하면 time.sleep(0.001) 정도 써도 됨.

    # # 반복 동작 수행
    # while True:
    #     # 이동 명령 실행
    #     print("movej")
    #     movej(JReady, vel=VELOCITY, acc=ACC)
    #     print("movel")
    #     movel(pos1, vel=VELOCITY, acc=ACC)
    

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    # STOP/RESUME 명령 구독자 생성
    node.create_subscription(
        String,
        "/arm_control",   # 토픽 이름
        control_callback,
        10
    )

    try:
        # 초기화는 한 번만 수행
        initialize_robot()
        # 작업 수행
        perform_task(node)

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()