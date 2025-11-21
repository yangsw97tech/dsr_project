#!/usr/bin/env python3
# doma_place_stop_server.py

import rclpy
from rclpy.node import Node
import DR_init
import threading
import time
import math
import random
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

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
task_done_event = threading.Event()
task_running = False
set_tool_func = None
set_tcp_func = None

g_current_goal_handle = None
g_final_result = None

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

def callback_pause(request, response):
    global node_, cli_move_pause, cli_move_resume
    if cli_move_pause is None or cli_move_resume is None:
        response.success = False; response.message = "Services not init"; return response
    if not cli_move_pause.service_is_ready():
        response.success = False; response.message = "Services not ready"; return response

    if request.data: # PAUSE
        node_.get_logger().warn("⛔ Command: PAUSE -> Calling MovePause...")
        cli_move_pause.call_async(MovePause.Request())
        response.message = "Sent MovePause Signal"
    else:            # RESUME
        node_.get_logger().info("✅ Command: RESUME -> Calling MoveResume...")
        cli_move_resume.call_async(MoveResume.Request())
        response.message = "Sent MoveResume Signal"
    response.success = True
    return response

def perform_task_loop():
    global task_running, trigger_event, task_done_event, node_, g_current_goal_handle, g_final_result
    logger = node_.get_logger()
    print("[Task Thread] 작업 스레드 대기 중...")

    while rclpy.ok():
        triggered = trigger_event.wait(timeout=1.0)
        if not rclpy.ok(): break
        if not triggered: continue

        if task_running:
            logger.warn("[Task Thread] 이미 작업 실행 중.")
            trigger_event.clear()
            continue

        task_running = True
        start_time = time.time()
        logger.info("[Task Thread] 트리거 수신 → doma_place 동작 시작.")

        try:
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
            logger.info("[Task Thread] Place 시퀀스 시작.")
            feedback_msg.feedback_string = "도마 내려놓기 동작"
            feedback_msg.current_stroke = 0
            if g_current_goal_handle: g_current_goal_handle.publish_feedback(feedback_msg)

            # 1. Place 위치 접근 (amovel)
            amovel(posx(490.55, 233.19, 463.79, 43.27, 179.97, 43.28), vel=VELOCITY, acc=ACC, radius=30.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion(): raise Exception("작업 중단")

            # 2. 자세 잡기 (amovesj)
            logger.info("2. 자세 잡기 (Spline)")
            place_ready_path = [
                posj(-25.44, 25.82, 64.53, 14.31, 106.11, -39.22),
                posj(-50.09, 40.70, 63.67, 35.43, 97.23, -91.51),
                posj(-49.66, 72.86, 47.25, 59.27, 131.00, -131.66)
            ]
            amovesj(place_ready_path, vel=VELOCITY, acc=ACC)
            if not wait_for_motion(): raise Exception("작업 중단")

            # 정렬 이동 (amovel)
            amovel(posx(0.00, 100.00, 0.00, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion(): raise Exception("작업 중단")
            
            gripper(0) # 닫기
            time.sleep(1.0)

            # 3. 건조대 진입 및 배치 (amovesj)
            logger.info("3. 건조대 진입 (Spline)")
            insert_path = [
                posj(-45.31, 61.34, 71.30, 59.33, 121.63, -141.12),
                posj(-45.39, 57.76, 68.16, 63.27, 123.92, -135.78),
                posj(-45.01, 50.34, 67.31, 63.27, 124.55, -128.64),
                posj(-50.13, 27.81, 92.24, 64.66, 104.44, -124.11),
                posj(-50.42, 0.29, 108.69, 64.81, 80.28, -97.76),
                posj(-24.57, -15.92, 104.11, 34.45, 52.34, -45.38)
            ]
            amovesj(insert_path, vel=VELOCITY, acc=ACC)
            if not wait_for_motion(): raise Exception("작업 중단")

            # 4. 안착 이동 (amovel)
            logger.info("4. 안착 이동")
            amovel(posx(334.04, -13.61, 525.83, 0.45, 151.57, 3.42), vel=VELOCITY_fast, acc=ACC_fast, radius=30.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion(): raise Exception("작업 중단")

            amovel(posx(636.87, -13.61, 525.82, 0.45, 151.57, 3.42), vel=VELOCITY_fast, acc=ACC_fast, radius=30.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion(): raise Exception("작업 중단")

            # 5. 내려놓기 전 최종 정렬
            logger.info("5. 내려놓기")
            amovej(posj(-2.21, 31.13, 61.13, 1.06, 59.34, 0.71), vel=VELOCITY-10, acc=ACC-10, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion(): raise Exception("작업 중단")

            gripper(1) # 열기
            time.sleep(1.0)

            # 6. 복귀 (amovesj)
            logger.info("6. 복귀 (Spline)")
            return_path = [
                posj(-0.8, 16.18, 69.13, 1.06, 61.25, 0.71),
                [0, 0, 90, 0, 90, 0] # Home
            ]
            amovesj(return_path, vel=VELOCITY, acc=ACC)
            if not wait_for_motion(): raise Exception("작업 중단")

            logger.info("[Task Thread] ✅ doma_place 시퀀스 1회 완료.")

            end_time = time.time()
            result = BrushingAction.Result()
            result.complete_task = True
            result.total_duration = end_time - start_time
            result.final_pose, _ = get_current_posx()
            
            g_final_result = result
            task_done_event.set()

        except Exception as e:
            logger.error(f"[Task Thread] 수행 중 예외: {e}")
            result = BrushingAction.Result()
            result.complete_task = False
            g_final_result = result
            task_done_event.set()
        finally:
            task_running = False
            trigger_event.clear()
            logger.info("[Task Thread] 작업 종료.")

    print("[Task Thread] 종료.")

def goal_callback(goal_request):
    global task_running, node_
    logger = node_.get_logger()
    if task_running: return GoalResponse.REJECT
    if not goal_request.start_task: return GoalResponse.REJECT
    logger.info("[ROS Thread] Goal 수락.")
    return GoalResponse.ACCEPT

def execute_callback(goal_handle):
    global task_running, trigger_event, task_done_event, g_current_goal_handle, node_, g_final_result
    logger = node_.get_logger()
    logger.info("[ROS Thread] Goal 실행 시작.")
    
    g_current_goal_handle = goal_handle
    task_done_event.clear()
    trigger_event.set()
    
    while rclpy.ok():
        if task_done_event.wait(timeout=0.1):
            break
            
    logger.info("[ROS Thread] 작업 완료. Result 전송 준비.")
    
    if g_final_result and g_final_result.complete_task:
        goal_handle.succeed()
    else:
        goal_handle.abort()
        
    # [핵심 수정] Result 전송 전에 1.0초 대기하여 DDS 상태 동기화 시간 확보
    # 이 sleep이 'Client gone away' 에러를 방지합니다.
    time.sleep(1.0)
    
    g_current_goal_handle = None
    return g_final_result

def ros_spin_thread():
    global node_
    print("[ROS Thread] ROS Spin 시작...")
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node_)
        action_server = ActionServer(
            node_, BrushingAction, 'do_doma_place_action',
            execute_callback=execute_callback, 
            goal_callback=goal_callback, 
            cancel_callback=None,
            callback_group=ReentrantCallbackGroup()
        )
        print("[ROS Thread] Action 서버 시작됨.")
        executor.spin()
    except Exception as e:
        if rclpy.ok(): node_.get_logger().error(f"Spin 예외: {e}")
    finally:
        print("ROS Spin 종료.")

def main(args=None):
    global node_, cli_move_pause, cli_move_resume
    rclpy.init(args=args)
    import random; time.sleep(random.uniform(0.1, 1.0))

    node_ = rclpy.create_node("doma_place_action_server", namespace=ROBOT_ID)
    DR_init.__dsr__node = node_

    cli_move_pause = node_.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
    cli_move_resume = node_.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')
    node_.create_service(SetBool, 'custom_pause', callback_pause)

    print(f"노드 '{ROBOT_ID}/doma_place_action_server' 생성 완료.")
    
    try:
        spin_thread = threading.Thread(target=ros_spin_thread)
        spin_thread.start()
        time.sleep(1.0)

        if not initialize_robot(): raise Exception("로봇 초기화 실패")
        
        robot_thread = threading.Thread(target=perform_task_loop)
        robot_thread.start()
        
        robot_thread.join()
        spin_thread.join()
    except KeyboardInterrupt:
        print("\nCtrl+C 감지.")
    finally:
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()