#!/usr/bin/env python3
# doma_pick_stop_server.py

import rclpy
from rclpy.node import Node
import DR_init
import threading
import time
import math
import random
from rclpy.executors import MultiThreadedExecutor

# Action 관련
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from my_robot_interfaces.action import BrushingAction

# Pause/Resume 서비스
from std_srvs.srv import SetBool
from dsr_msgs2.srv import MovePause, MoveResume

# --- 로봇 설정 ---
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"
ROBOT_TOOL = "GripperDA_v1"
VELOCITY = 30
ACC = 30
VELOCITY_fast = 60
ACC_fast = 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# --- 전역 변수 ---
node_ = None
trigger_event = threading.Event()
task_running = False
set_tool_func = None
set_tcp_func = None

g_current_goal_handle = None
g_final_result = None

# 서비스 클라이언트
cli_move_pause = None
cli_move_resume = None


def initialize_robot():
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
        set_tool_func(ROBOT_TOOL)
        set_tcp_func(ROBOT_TCP)
        logger.info("[Init] 로봇 초기화 완료.")
        return True
    except Exception as e:
        logger.error(f"[Init] set_tool/set_tcp 실행 실패: {e}")
        return False

# Pause/Resume 콜백
def callback_pause(request, response):
    global node_, cli_move_pause, cli_move_resume
    
    if cli_move_pause is None or cli_move_resume is None:
        response.success = False
        response.message = "Robot Service Clients not initialized"
        return response

    if not cli_move_pause.service_is_ready():
        response.success = False
        response.message = "Robot Services not ready"
        return response

    if request.data: # PAUSE
        node_.get_logger().warn("⛔ Command: PAUSE -> Calling MovePause...")
        req = MovePause.Request()
        cli_move_pause.call_async(req)
        response.message = "Sent MovePause Signal"
        response.success = True
    else:            # RESUME
        node_.get_logger().info("✅ Command: RESUME -> Calling MoveResume...")
        req = MoveResume.Request()
        cli_move_resume.call_async(req)
        response.message = "Sent MoveResume Signal"
        response.success = True
    return response

def perform_task_loop():
    global task_running, trigger_event, node_, g_current_goal_handle, g_final_result
    logger = node_.get_logger()
    print("[Task Thread] 작업 스레드 시작됨. 트리거 대기 중...")

    while rclpy.ok():
        triggered = trigger_event.wait(timeout=1.0)
        if not rclpy.ok(): break
        if not triggered: continue

        if task_running:
            logger.warn("[Task Thread] 이미 작업 실행 중. 새 트리거 무시.")
            trigger_event.clear()
            continue

        task_running = True
        start_time = time.time()
        logger.info("[Task Thread] 트리거 수신 → doma_picking 동작 시작.")

        try:
            # [핵심] amovesj 임포트 확인
            from DSR_ROBOT2 import (
                amovel, amovej, amovesj, check_motion, 
                set_digital_output,
                posx, posj,
                DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL,
                DR_MV_RA_DUPLICATE, OFF, ON,
                get_current_posx, mwait
            )

            def wait_for_motion():
                time.sleep(0.2)
                while check_motion() != 0: 
                    time.sleep(0.1)
                    if not rclpy.ok(): return False
                return True

            def gripper(switch: int):
                if switch == 0: set_digital_output(1, ON); set_digital_output(2, OFF)
                elif switch == 1: set_digital_output(1, OFF); set_digital_output(2, ON)

            feedback_msg = BrushingAction.Feedback()

            # 1. 홈 자세 이동
            logger.info("1. 홈 자세로 이동.")
            home_pose_j = [0, 0, 90, 0, 90, 0]
            amovej(home_pose_j, vel=30, acc=30)
            if not wait_for_motion(): raise Exception("작업 중단")
            
            gripper(1)
            time.sleep(1.0)

            # 2. 접근 및 그립 (amovesj 적용)
            logger.info("2. Pick 접근 (Spline)")
            feedback_msg.feedback_string = "도마 픽킹 접근"
            if g_current_goal_handle: g_current_goal_handle.publish_feedback(feedback_msg)

            pick_path = [
                posj(-5.50, 5.92, 76.95, -1.54, 66.92, 0.00),
                posj(-5.31, 26.53, 60.17, 1.18, 64.83, 0.12),
                posj(-2.21, 31.13, 61.13, 1.06, 59.34, 0.71)
            ]
            amovesj(pick_path, vel=VELOCITY, acc=ACC)
            if not wait_for_motion(): raise Exception("작업 중단")

            gripper(0)
            time.sleep(1.0)

            # 3. 작업대 이동 (amovel)
            logger.info("3. 도마 작업대 위로 이동")
            feedback_msg.feedback_string = "작업대 이동"
            if g_current_goal_handle: g_current_goal_handle.publish_feedback(feedback_msg)

            amovel(posx(636.87, -13.61, 525.82, 0.45, 151.57, 3.42), vel=VELOCITY_fast, acc=ACC_fast, radius=30.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion(): raise Exception("작업 중단")

            # 4. 자세조정 및 내리기 (amovel + amovesj 적용)
            logger.info("4. 자세 조정 및 안착 (Spline)")
            feedback_msg.feedback_string = "안착 자세 조정"
            if g_current_goal_handle: g_current_goal_handle.publish_feedback(feedback_msg)

            # (1) 직선 이동
            amovel(posx(334.04, -13.61, 525.83, 0.45, 151.57, 3.42), vel=VELOCITY_fast, acc=ACC_fast, radius=30.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion(): raise Exception("작업 중단")

            # (2) 관절 경로 묶음 (amovesj 적용) [수정됨]
            place_adjust_path = [
                posj(-24.57, -15.92, 104.11, 34.45, 52.34, -45.38),
                posj(-50.42, 0.29, 108.69, 64.81, 80.28, -97.76),
                posj(-50.13, 27.81, 92.24, 64.66, 104.44, -124.11),
                posj(-45.01, 50.34, 67.31, 63.27, 124.55, -128.64),
                posj(-45.39, 57.76, 68.16, 63.27, 123.92, -135.78),
                posj(-45.31, 61.34, 71.30, 59.33, 121.63, -141.12)
            ]
            amovesj(place_adjust_path, vel=VELOCITY, acc=ACC)
            if not wait_for_motion(): raise Exception("작업 중단")

            # (3) 최종 밀어넣기
            amovel(posx(465.54, -315.23, 71.47, 90.00, 90.00, 0.00), vel=VELOCITY, acc=ACC, radius=0.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion(): raise Exception("작업 중단")

            gripper(1)
            time.sleep(1.0)

            # 5. 뒤로 빼기 및 이동을 위한 정렬 (amovel + amovesj 적용)
            logger.info("5. 복귀")
            feedback_msg.feedback_string = "복귀"
            if g_current_goal_handle: g_current_goal_handle.publish_feedback(feedback_msg)

            # (1) 뒤로 빼기 (직선)
            amovel(posx(0.00, -100.00, 0.00, 0.00, 0.00, 0.00), vel=VELOCITY_fast, acc=ACC_fast, radius=30.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion(): raise Exception("작업 중단")

            # (2) 정렬 및 홈 복귀 (관절 묶음) [수정됨]
            return_path = [
                posj(-49.51, 54.09, 28.09, 44.63, 130.44, -131.66),
                # 홈 포즈 (list 형태이므로 바로 추가 가능)
                [0, 0, 90, 0, 90, 0] 
            ]
            amovesj(return_path, vel=VELOCITY, acc=ACC)
            if not wait_for_motion(): raise Exception("작업 중단")

            logger.info("[Task Thread] ✅ doma_picking 시퀀스 완료.")

            end_time = time.time()
            result = BrushingAction.Result()
            result.complete_task = True
            result.total_duration = end_time - start_time
            result.final_pose, _ = get_current_posx()

            g_final_result = result
            g_current_goal_handle.succeed()

        except Exception as e:
            if rclpy.ok():
                logger.error(f"[Task Thread] 예외 발생: {e}")
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
            logger.info("[Task Thread] 대기 상태 전환.")

    print("[Task Thread] 루프 종료.")

def goal_callback(goal_request):
    global task_running, node_
    logger = node_.get_logger()
    if task_running: return GoalResponse.REJECT
    if not goal_request.start_task: return GoalResponse.REJECT
    logger.info("[ROS Thread] 새 Goal 수락.")
    return GoalResponse.ACCEPT

def execute_callback(goal_handle):
    global task_running, trigger_event, g_current_goal_handle, node_, g_final_result
    logger = node_.get_logger()
    logger.info("[ROS Thread] Goal 실행 시작.")
    g_current_goal_handle = goal_handle
    trigger_event.set()
    while g_current_goal_handle is not None and rclpy.ok():
        time.sleep(0.1)
    logger.info("[ROS Thread] Goal 실행 완료.")
    return g_final_result

def ros_spin_thread():
    global node_
    print("[ROS Thread] ROS Spin 시작...")
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node_)
        action_server = ActionServer(
            node_, BrushingAction, 'do_doma_pick_action',
            execute_callback=execute_callback, goal_callback=goal_callback, cancel_callback=None
        )
        print("[ROS Thread] Action 서버 시작됨.")
        executor.spin()
    except Exception as e:
        if rclpy.ok(): node_.get_logger().error(f"[ROS Thread] Spin 예외: {e}")
    finally:
        print("[ROS Thread] ROS Spin 종료.")

def main(args=None):
    global node_, cli_move_pause, cli_move_resume
    rclpy.init(args=args)
    
    import random
    time.sleep(random.uniform(0.1, 1.0))

    node_ = rclpy.create_node("doma_pick_action_server", namespace=ROBOT_ID)
    DR_init.__dsr__node = node_

    # 서비스 클라이언트
    cli_move_pause = node_.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
    cli_move_resume = node_.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')
    node_.create_service(SetBool, 'custom_pause', callback_pause)

    print(f"노드 '{ROBOT_ID}/doma_pick_action_server' 생성 완료.")

    spin_thread = None
    robot_thread = None

    try:
        if not initialize_robot(): raise Exception("로봇 초기화 실패")
        robot_thread = threading.Thread(target=perform_task_loop)
        spin_thread = threading.Thread(target=ros_spin_thread)
        robot_thread.start()
        spin_thread.start()
        robot_thread.join()
        spin_thread.join()
    except KeyboardInterrupt:
        print("\nCtrl+C 감지. 노드를 종료합니다.")
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()