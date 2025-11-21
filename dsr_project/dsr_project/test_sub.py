# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# import DR_init
# import threading
# import time
# from std_msgs.msg import Bool
# from rclpy.executors import MultiThreadedExecutor

# # --- 로봇 설정 ---
# ROBOT_ID = "dsr01"
# ROBOT_MODEL = "m0609"
# ROBOT_TOOL = "Tool Weight"
# ROBOT_TCP = "GripperDA"

# DR_init.__dsr__id = ROBOT_ID
# DR_init.__dsr__model = ROBOT_MODEL

# # --- 전역 변수 ---
# node_ = None
# trigger_event = threading.Event()
# task_running = False

# def initialize_robot():
#     """초기화: 구조 유지"""
#     global node_
#     logger = node_.get_logger()
#     try:
#         from DSR_ROBOT2 import set_tool, set_tcp
#         set_tool(ROBOT_TOOL)
#         set_tcp(ROBOT_TCP)
#         logger.info("로봇 초기화(Tool/TCP) 완료.")
#         return True
#     except Exception as e:
#         logger.error(f"초기화 실패: {e}")
#         return False

# def perform_task_loop():
#     """
#     [수정됨] 가상 로봇 초기 상태 고려
#     1. movej로 관절 홈 정렬 (0, 0, 90, 0, 90, 0)
#     2. movel로 X축 왕복 테스트
#     """
#     global task_running, trigger_event, node_
#     logger = node_.get_logger()
#     print("[Task Thread] 트리거 대기 중...")
    
#     while rclpy.ok():
#         triggered = trigger_event.wait(timeout=1.0)
#         if not rclpy.ok(): break
        
#         if triggered:
#             if not task_running:
#                 task_running = True
#                 logger.info("[Task Thread] 동작 시퀀스 시작!")
                
#                 try:
#                     # 안전한 내부 임포트
#                     from DSR_ROBOT2 import movej, movel, DR_BASE, DR_MV_MOD_REL
                    
#                     # 1. [안전 확보] 홈 포즈로 관절 정렬 (Blocking)
#                     logger.info(">>> 홈 포즈 이동 중... (0, 0, 90, 0, 90, 0)")
#                     movej([0, 0, 90, 0, 90, 0], vel=30, acc=30, time=3)
                    
#                     logger.info("[Task Thread] 홈 정렬 완료. 1초 대기.")
#                     time.sleep(1.0)
                    
#                     # 2. [동작 테스트] X축 +100mm 이동 (Blocking)
#                     logger.info(">>> X축 +100mm 이동")
#                     movel([100, 0, 0, 0, 0, 0], vel=50, acc=50, time=2, ref=DR_BASE, mod=DR_MV_MOD_REL)
                    
#                     time.sleep(0.5)
                    
#                     # 3. [복귀] X축 -100mm 이동 (Blocking)
#                     logger.info(">>> X축 원위치 복귀")
#                     movel([-100, 0, 0, 0, 0, 0], vel=50, acc=50, time=2, ref=DR_BASE, mod=DR_MV_MOD_REL)
                    
#                     logger.info("[Task Thread] 전체 동작 완료.")

#                 except Exception as e:
#                     logger.error(f"[Task Thread] 동작 중 에러: {e}")
#                 finally:
#                     task_running = False
#                     trigger_event.clear()
#             else:
#                 trigger_event.clear()

# def listener_callback(msg):
#     global trigger_event, node_
#     if msg.data is True:
#         node_.get_logger().info("토픽 수신 -> 트리거 ON")
#         trigger_event.set()

# def ros_spin_thread():
#     """ROS 스레드: MultiThreadedExecutor 필수"""
#     global node_
#     try:
#         executor = MultiThreadedExecutor(num_threads=4)
#         executor.add_node(node_)
#         executor.spin()
#     except Exception as e:
#         if rclpy.ok():
#             node_.get_logger().error(f"Executor 에러: {e}")

# def main(args=None):
#     global node_
#     rclpy.init(args=args)
#     node_ = rclpy.create_node("home_and_x_test", namespace=ROBOT_ID)
#     DR_init.__dsr__node = node_
    
#     spin_thread = None; robot_thread = None

#     try:
#         if not initialize_robot(): raise Exception("Init Fail")

#         node_.create_subscription(Bool, '/trigger_move', listener_callback, 10)

#         robot_thread = threading.Thread(target=perform_task_loop)
#         spin_thread = threading.Thread(target=ros_spin_thread)

#         robot_thread.start()
#         spin_thread.start()
        
#         robot_thread.join()
#         spin_thread.join()

#     except KeyboardInterrupt:
#         print("종료.")
#     finally:
#         if rclpy.ok(): rclpy.shutdown()
#         if spin_thread: spin_thread.join(timeout=1)
#         if robot_thread: robot_thread.join(timeout=1)

# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import DR_init
import threading
import time
from std_msgs.msg import Bool

# [변경 포인트 1] Multi -> Single로 변경
from rclpy.executors import SingleThreadedExecutor

# --- 로봇 설정 (변경 없음) ---
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# --- 전역 변수 (변경 없음) ---
node_ = None
trigger_event = threading.Event()
task_running = False

def initialize_robot():
    """초기화 (변경 없음)"""
    global node_
    logger = node_.get_logger()
    try:
        from DSR_ROBOT2 import set_tool, set_tcp
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        logger.info("로봇 초기화(Tool/TCP) 완료.")
        return True
    except Exception as e:
        logger.error(f"초기화 실패: {e}")
        return False

def perform_task_loop():
    """
    [로직 동일] 성공했던 '홈 정렬 + X축 이동' 코드 그대로 유지
    """
    global task_running, trigger_event, node_
    logger = node_.get_logger()
    print("[Task Thread] 트리거 대기 중...")
    
    while rclpy.ok():
        triggered = trigger_event.wait(timeout=1.0)
        if not rclpy.ok(): break
        
        if triggered:
            if not task_running:
                task_running = True
                logger.info("[Task Thread] SingleThread 테스트 동작 시작!")
                
                try:
                    from DSR_ROBOT2 import movej, movel, DR_BASE, DR_MV_MOD_REL
                    
                    # 1. 홈 포즈 이동
                    logger.info(">>> 홈 포즈 이동 요청 (Blocking...)")
                    # [예상 지점] SingleExecutor라면 여기서 응답을 못 받아 멈출 수 있음
                    movej([0, 0, 90, 0, 90, 0], vel=30, acc=30, time=3)
                    
                    logger.info("[Task Thread] 홈 정렬 완료. 1초 대기.")
                    time.sleep(1.0)
                    
                    # 2. X축 +100mm 이동
                    logger.info(">>> X축 +100mm 이동 요청")
                    movel([100, 0, 0, 0, 0, 0], vel=50, acc=50, time=2, ref=DR_BASE, mod=DR_MV_MOD_REL)
                    
                    time.sleep(0.5)
                    
                    # 3. X축 원위치
                    logger.info(">>> X축 원위치 복귀 요청")
                    movel([-100, 0, 0, 0, 0, 0], vel=50, acc=50, time=2, ref=DR_BASE, mod=DR_MV_MOD_REL)
                    
                    logger.info("[Task Thread] 전체 동작 완료.")

                except Exception as e:
                    logger.error(f"[Task Thread] 동작 중 에러 (Deadlock 의심): {e}")
                finally:
                    task_running = False
                    trigger_event.clear()
            else:
                trigger_event.clear()

def listener_callback(msg):
    """(변경 없음)"""
    global trigger_event, node_
    if msg.data is True:
        node_.get_logger().info("토픽 수신 -> 트리거 ON")
        trigger_event.set()

def ros_spin_thread():
    """
    [변경 포인트 2] SingleThreadedExecutor 사용
    - 스레드는 별도로 돌지만, ROS 메시지 처리 파이프라인이 1개입니다.
    - Blocking 함수(movej)가 응답을 기다릴 때, 이 Executor가 응답을 처리해줄 수 있는지 확인합니다.
    """
    global node_
    print("[ROS Thread] SingleThreadedExecutor 가동 시작 (Deadlock 테스트)...")
    try:
        # 여기가 변경되었습니다.
        executor = SingleThreadedExecutor() 
        executor.add_node(node_)
        executor.spin()
    except Exception as e:
        if rclpy.ok():
            node_.get_logger().error(f"Executor 에러: {e}")

def main(args=None):
    """(변경 없음)"""
    global node_
    rclpy.init(args=args)
    node_ = rclpy.create_node("single_exec_deadlock_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = node_
    
    spin_thread = None; robot_thread = None

    try:
        if not initialize_robot(): raise Exception("Init Fail")

        node_.create_subscription(Bool, '/trigger_move', listener_callback, 10)

        robot_thread = threading.Thread(target=perform_task_loop)
        spin_thread = threading.Thread(target=ros_spin_thread)

        robot_thread.start()
        spin_thread.start()
        
        robot_thread.join()
        spin_thread.join()

    except KeyboardInterrupt:
        print("종료.")
    finally:
        if rclpy.ok(): rclpy.shutdown()
        if spin_thread: spin_thread.join(timeout=1)
        if robot_thread: robot_thread.join(timeout=1)

if __name__ == "__main__":
    main()