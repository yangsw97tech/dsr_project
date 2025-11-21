#!/usr/bin/env python3
# doma_pick_a_server.py  –  BrushingAction 기반 도마 픽킹 액션 서버

import rclpy
from rclpy.node import Node
import DR_init
import threading
import time
import math
from rclpy.executors import MultiThreadedExecutor

# Action 관련
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from my_robot_interfaces.action import BrushingAction


# --- 로봇 설정 상수 ---
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"    # TCP 이름
ROBOT_TOOL = "GripperDA_v1"  # Tool 이름
VELOCITY = 30
ACC = 30
VELOCITY_fast = 60
ACC_fast = 60

# DR_init 기본 설정 (노드 생성 전)
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# --- 전역 변수 (스레드 공유) ---
node_ = None
trigger_event = threading.Event()
task_running = False

set_tool_func = None
set_tcp_func = None

# 액션용 전역
g_current_goal_handle = None
g_final_result = None


def initialize_robot():
    """
    spin 시작 전에, 메인 스레드에서 딱 한 번만 호출되는 초기화 함수.
    여기서 DSR_ROBOT2를 처음 import 하고 Tool/TCP 설정을 한다.
    """
    global set_tool_func, set_tcp_func, node_
    logger = node_.get_logger()

    try:
        from DSR_ROBOT2 import set_tool, set_tcp
        set_tool_func = set_tool
        set_tcp_func = set_tcp
    except Exception as e:
        logger.error(f"[Init] DSR_ROBOT2 import 실패: {e}")
        return False

    logger.info("[Init] Tool/TCP 설정 시작...")
    try:
        # 이름에 맞춰 Tool, TCP 설정
        set_tool_func(ROBOT_TOOL)
        set_tcp_func(ROBOT_TCP)
        logger.info("[Init] 로봇 초기화 완료 (Tool/TCP 설정).")
        return True
    except Exception as e:
        logger.error(f"[Init] set_tool/set_tcp 실행 실패: {e}")
        return False


def perform_task_loop():
    """
    액션 Goal 이 들어왔을 때(trigget_event set),
    도마 픽킹(doma pick) 시퀀스를 1회 수행하는 작업 스레드 루프.
    """
    global task_running, trigger_event, node_, g_current_goal_handle, g_final_result
    logger = node_.get_logger()

    print("[Task Thread] 작업 스레드 시작됨. 트리거 대기 중...")

    while rclpy.ok():
        # Goal 이 들어와서 trigger_event가 set 될 때까지 대기
        triggered = trigger_event.wait(timeout=1.0)

        if not rclpy.ok():
            break

        if not triggered:
            continue  # 다시 대기

        # 이미 작업 중이면 새 Goal 무시
        if task_running:
            logger.warn("[Task Thread] 이미 작업 실행 중. 새 트리거 무시.")
            trigger_event.clear()
            continue

        # 여기부터 실제 doma_pick 1회 동작
        task_running = True
        start_time = time.time()
        logger.info("[Task Thread] 트리거 수신 → doma_picking 동작 1회 실행 시작.")

        try:
            from DSR_ROBOT2 import (
                set_digital_output,
                movej, movel,
                posx, posj,
                DR_BASE,
                DR_MV_MOD_ABS, DR_MV_MOD_REL,
                DR_MV_RA_DUPLICATE,
                OFF, ON,
                get_current_posx,
            )

            # --- gripper 함수 정의 ---
            def gripper(switch: int):
                # gripper 닫기
                if switch == 0:
                    set_digital_output(1, ON)
                    set_digital_output(2, OFF)
                # gripper 열기
                elif switch == 1:
                    set_digital_output(1, OFF)
                    set_digital_output(2, ON)

            # 피드백 전송 함수 (동작 사이사이에 호출)
            feedback_msg = BrushingAction.Feedback()
            def pub_feedback(msg, pct):
                feedback_msg.feedback_string = msg
                feedback_msg.progress_percentage = float(pct)
                if g_current_goal_handle:
                    g_current_goal_handle.publish_feedback(feedback_msg)
                # logger.info(f"[{int(pct)}%] {msg}") # 필요시 주석 해제

            # 홈 자세 이동
            logger.info("[Task Thread] 홈 자세로 이동.")
            home_pose_j = [0, 0, 90, 0, 90, 0]
            movej(home_pose_j, vel=30, acc=30)
            gripper(1)  # gripper 열기
            time.sleep(1.0)

            # -----------------------------
            # doma Pick 시퀀스 (네 코드 그대로)
            # -----------------------------
            logger.info("[Task Thread] doma Pick 시퀀스 시작.")
            # feedback_msg.feedback_string = "도마 픽킹 동작 시작"
            feedback_msg.current_stroke = 0
            pub_feedback("도마 픽킹 위치 접근 중", 10.0)
            if g_current_goal_handle:
                g_current_goal_handle.publish_feedback(feedback_msg)

            movej(posj(-5.50, 5.92, 76.95, -1.54, 66.92, 0.00),
                  vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            
            pub_feedback("접근 경로 이동 (1/2)", 15.0)
            # 도마잡기 전(1)
            movej(posj(-5.50, 5.92, 76.95, -1.54, 66.92, 0.00),
                  vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            
            pub_feedback("접근 경로 이동 (2/2)", 20.0)
            # 도마잡기 전(2)
            movej(posj(-5.31, 26.53, 60.17, 1.18, 64.83, 0.12),
                  vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            
            pub_feedback("그리퍼 정렬 완료", 25.0)
            # 도마 잡기
            movej(posj(-2.21, 31.13, 61.13, 1.06, 59.34, 0.71),
                  vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)

            pub_feedback("도마 파지 중...", 30.0)

            gripper(0)  # gripper 닫기
            time.sleep(1.0)
            pub_feedback("도마 파지 성공", 35.0)

            ## 도마 작업대 위로 이동
            logger.info("[Task Thread] 도마 작업대 위로 이동.")
            pub_feedback("작업대로 이동 중...", 40.0)
            
            movel(posx(636.87, -13.61, 525.82, 0.45, 151.57, 3.42),
                  t=2, vel=VELOCITY_fast, acc=ACC_fast, radius=30.00,
                  ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)

            ## 자세조정 및 내리기
            pub_feedback("작업대 위치 정렬", 50.0)
            movel(posx(334.04, -13.61, 525.83, 0.45, 151.57, 3.42),
                  vel=VELOCITY_fast, acc=ACC_fast, t=2, radius=30.00,
                  ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            
            pub_feedback("안착 위치 하강 (1/6)", 55.0)
            # 도마 내리기 전 (1)
            movej(posj(-24.57, -15.92, 104.11, 34.45, 52.34, -45.38),
                  vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            
            pub_feedback("안착 위치 하강 (2/6)", 60.0)
            # 도마 내리기 전 (2)
            movej(posj(-50.42, 0.29, 108.69, 64.81, 80.28, -97.76),
                  vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            
            pub_feedback("안착 위치 하강 (3/6)", 65.0)
            # 도마 내리기 전 (3)
            movej(posj(-50.13, 27.81, 92.24, 64.66, 104.44, -124.11),
                  vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            
            pub_feedback("안착 위치 하강 (4/6)", 70.0)
            # 도마 내리기 전 (4)
            movej(posj(-45.01, 50.34, 67.31, 63.27, 124.55, -128.64),
                  vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            
            pub_feedback("안착 위치 하강 (5/6)", 75.0)
            # 도마 내리기 전 (5)
            movej(posj(-45.39, 57.76, 68.16, 63.27, 123.92, -135.78),
                  vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            
            pub_feedback("안착 위치 하강 (6/6)", 80.0)
            # 도마 내리기 전 (6)
            movej(posj(-45.31, 61.34, 71.30, 59.33, 121.63, -141.12),
                  vel=VELOCITY, acc=ACC, radius=0.00, ra=DR_MV_RA_DUPLICATE)
            
            pub_feedback("작업대 안착, 이동", 85.0)
            # 도착 (수평 정렬하며 이동)
            movel(posx(465.54, -315.23, 71.47, 90.00, 90.00, 0.00),
                  vel=VELOCITY, acc=ACC, radius=0.00,
                  ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            
            gripper(1)  # gripper 열기
            time.sleep(1.0)
            pub_feedback("도마 놓기 완료", 90.0)

            # 뒤로 빼기 및 이동을 위한 정렬
            movel(posx(0.00, -100.00, 0.00, 0.00, 0.00, 0.00),
                  vel=VELOCITY_fast, acc=ACC_fast, radius=30.00,
                  ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
            
            pub_feedback("안전 거리 확보 중", 95.0)
            movej(posj(-49.51, 54.09, 28.09, 44.63, 130.44, -131.66),
                  vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)

            logger.info("[Task Thread] 홈 복귀.")
            movej(home_pose_j, vel=30, acc=30)

            logger.info("[Task Thread] ✅ doma_picking 시퀀스 1회 완료.")
            pub_feedback("Pick 공정 완료", 100.0)

            # --- Result 채우기 ---
            end_time = time.time()
            result = BrushingAction.Result()
            result.complete_task = True
            result.total_duration = end_time - start_time
            result.final_pose, _ = get_current_posx()

            g_final_result = result
            if g_current_goal_handle and g_current_goal_handle.is_active:
                g_current_goal_handle.succeed()

        except Exception as e:
            if rclpy.ok():
                logger.error(f"[Task Thread] doma_picking 수행 중 예외 발생: {e}")
            # 실패 Result
            result = BrushingAction.Result()
            result.complete_task = False
            result.total_duration = 0.0
            g_final_result = result
            if g_current_goal_handle and g_current_goal_handle.is_active:
                g_current_goal_handle.abort()
        finally:
            task_running = False
            trigger_event.clear()
            g_current_goal_handle = None
            logger.info("[Task Thread] 작업 1회 종료, 다음 트리거 대기.")

    print("[Task Thread] 작업 스레드 루프 종료됨.")


def goal_callback(goal_request):
    """
    새 Goal 수락 여부 결정.
    start_task 가 True이고, 현재 작업 중이 아니면 ACCEPT.
    """
    global task_running, node_
    logger = node_.get_logger()

    if task_running:
        logger.warn("[ROS Thread] 작업이 이미 실행 중이므로 새 Goal을 거부합니다.")
        return GoalResponse.REJECT

    if not goal_request.start_task:
        logger.warn("[ROS Thread] 'start_task'가 False이므로 Goal을 거부합니다.")
        return GoalResponse.REJECT

    logger.info("[ROS Thread] 새 Goal을 수락합니다.")
    return GoalResponse.ACCEPT


def execute_callback(goal_handle):
    """
    Goal 수락 후 실제 작업 실행.
    작업 스레드에 트리거를 던지고, 끝날 때까지 대기한 뒤 Result 반환.
    """
    global task_running, trigger_event, g_current_goal_handle, node_, g_final_result
    logger = node_.get_logger()
    logger.info("[ROS Thread] Goal 실행 시작! 작업 스레드 트리거.")

    g_current_goal_handle = goal_handle
    trigger_event.set()

    # 작업 스레드가 g_current_goal_handle을 None으로 돌려줄 때까지 대기
    while g_current_goal_handle is not None and rclpy.ok():
        time.sleep(0.1)

    logger.info("[ROS Thread] Goal 실행 완료. Result 반환.")
    return g_final_result


def ros_spin_thread():
    """
    ROS 콜백/서비스/액션 처리를 담당하는 스레드.
    MultiThreadedExecutor를 사용해서 ActionServer와 DSR 서비스가 같이 돌도록 함.
    """
    global node_
    print("[ROS Thread] ROS Spin 스레드 시작...")
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node_)

        action_server = ActionServer(
            node_,
            BrushingAction,
            'do_doma_pick_action',          # 액션 이름 (네임스페이스 없이)
            execute_callback=execute_callback,
            goal_callback=goal_callback,
            cancel_callback=None
        )
        print("[ROS Thread] 'do_doma_pick_action' Action 서버 시작됨.")

        executor.spin()
    except Exception as e:
        if rclpy.ok():
            node_.get_logger().error(f"[ROS Thread] Spin 중 예외 발생: {e}")
    finally:
        print("[ROS Thread] ROS Spin 스레드 종료됨.")


def main(args=None):
    global node_

    rclpy.init(args=args)

    # 노드 생성 및 DR_init에 등록
    node_ = rclpy.create_node("doma_pick_action_server", namespace=ROBOT_ID)
    DR_init.__dsr__node = node_

    print(f"노드 '{ROBOT_ID}/doma_pick_action_server' 생성 완료.")

    spin_thread = None
    robot_thread = None

    try:
        # spin 시작 전에 로봇 초기화
        if not initialize_robot():
            raise Exception("로봇 초기화 실패. 프로그램 종료.")

        # 스레드 두 개 생성 (ROS 스핀 / 작업)
        robot_thread = threading.Thread(target=perform_task_loop)
        spin_thread = threading.Thread(target=ros_spin_thread)

        robot_thread.start()
        spin_thread.start()

        robot_thread.join()
        spin_thread.join()

    except KeyboardInterrupt:
        print("\nCtrl+C 감지. 노드를 종료합니다.")
    except Exception as e:
        print(f"예기치 않은 오류 발생: {e}")
        if node_:
            node_.get_logger().fatal(f"메인 스레드 오류: {e}")
    finally:
        print("모든 스레드 종료 및 ROS2 종료 시작...")
        if rclpy.ok():
            rclpy.shutdown()

        if spin_thread and spin_thread.is_alive():
            spin_thread.join(timeout=1.0)
        if robot_thread and robot_thread.is_alive():
            robot_thread.join(timeout=1.0)

        print("ROS2가 종료되었습니다.")


if __name__ == "__main__":
    main()
