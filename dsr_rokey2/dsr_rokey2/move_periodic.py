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

# 로봇 초기화 과정으로 로봇 tool과 tcp를 설정, 이를 위해 필요한 기능만 먼저 import(set_tool이나 set_tcp)
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

# 실질적으로 로봇 동작을 수행할 함수로 필요한 제어함수,변수,상수 등을 import한뒤 동작 로직 코드를 작성
def perform_task():
    from DSR_ROBOT2 import move_periodic, DR_TOOL

    move_periodic(amp =[10,0,0,0,30,0], period=1.0, atime=0.2, repeat=5, ref=DR_TOOL)
	# 코드를 작성하시오.
    return



def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)   #ros2 통신 컨텍스트 생성
    node = rclpy.create_node("move_periodic", namespace=ROBOT_ID)   #노드 생성

    DR_init.__dsr__node = node  #생성된 노드 객체를 DR_init에 할당

    try:
        initialize_robot()  # 초기화는 한 번만 수행
        perform_task()  # 작업 수행 (한 번만 호출)

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()