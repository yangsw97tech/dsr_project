#!/usr/bin/env python3
# doma_place_final_server.py

import rclpy
from rclpy.node import Node
import DR_init
import threading
import time
import math
import random
from rclpy.executors import MultiThreadedExecutor

# Action
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from my_robot_interfaces.action import BrushingAction

# Service (Pause/Resume)
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

# [안전 설정] 충돌 감지 임계값 (N)
SAFETY_FORCE_LIMIT = 20.0 

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

node_ = None
trigger_event = threading.Event()
task_running = False
set_tool_func = None
set_tcp_func = None

g_current_goal_handle = None
g_final_result = None
cli_move_pause = None
cli_move_resume = None
g_is_paused = False # Auto Pause 상태 플래그

def initialize_robot():
    global set_tool_func, set_tcp_func, node_
    logger = node_.get_logger()
    try:
        from DSR_ROBOT2 import set_tool, set_tcp
        set_tool_func = set_tool
        set_tcp_func = set_tcp
        set_tool_func(ROBOT_TOOL)
        set_tcp_func(ROBOT_TCP)
        return True
    except Exception as e:
        logger.error(f"[Init] 초기화 실패: {e}")
        return False

def callback_pause(request, response):
    global node_, cli_move_pause, cli_move_resume, g_is_paused
    
    if not cli_move_pause.service_is_ready():
        response.success = False
        return response

    if request.data: # PAUSE
        node_.get_logger().warn("⛔ Command: PAUSE -> Calling MovePause...")
        cli_move_pause.call_async(MovePause.Request())
        g_is_paused = True
    else:            # RESUME
        node_.get_logger().info("✅ Command: RESUME -> Calling MoveResume...")
        cli_move_resume.call_async(MoveResume.Request())
        g_is_paused = False
        
    response.success = True
    return response

def perform_task_loop():
    global task_running, trigger_event, node_, g_current_goal_handle, g_final_result, g_is_paused
    logger = node_.get_logger()
    
    while rclpy.ok():
        triggered = trigger_event.wait(timeout=1.0)
        if not rclpy.ok(): break
        if not triggered: continue

        if task_running:
            trigger_event.clear()
            continue

        task_running = True
        start_time = time.time()
        logger.info("Doma Placing (Safe Mode) 시작")

        try:
            from DSR_ROBOT2 import (
                amovel, amovej, check_motion, set_digital_output,
                posx, posj, DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL,
                DR_MV_RA_DUPLICATE, OFF, ON, get_current_posx,
                task_compliance_ctrl, release_compliance_ctrl,
                get_tool_force, DR_TOOL
            )

            # [Helper] 안전 대기 함수 (퍼센트 제거됨)
            def wait_for_motion_safe():
                global g_is_paused
                time.sleep(0.2) # 초기 상태 반영 대기

                while check_motion() != 0:
                    # 1. 외력 감지 (안전 기능)
                    forces = get_tool_force(DR_TOOL)
                    force_mag = math.sqrt(forces[0]**2 + forces[1]**2 + forces[2]**2)
                    
                    if force_mag > SAFETY_FORCE_LIMIT and not g_is_paused:
                        logger.warn(f"🚨 충돌 감지({force_mag:.1f}N)! -> AUTO PAUSE")
                        cli_move_pause.call_async(MovePause.Request())
                        g_is_paused = True
                    
                    # 2. Pause 상태면 대기
                    while g_is_paused and rclpy.ok():
                        time.sleep(0.5) # Resume 기다림
                    
                    time.sleep(0.1)
                    if not rclpy.ok(): return False
                return True

            def gripper(switch):
                if switch == 0: set_digital_output(1, ON); set_digital_output(2, OFF)
                elif switch == 1: set_digital_output(1, OFF); set_digital_output(2, ON)

            feedback_msg = BrushingAction.Feedback()
            def pub_msg(msg):
                feedback_msg.feedback_string = msg
                feedback_msg.current_stroke = 0
                g_current_goal_handle.publish_feedback(feedback_msg)

            # --- [안전 모드] 유연 제어 켜기 ---
            task_compliance_ctrl(stx=[1000, 1000, 1000, 100, 100, 100])
            logger.info("[Safety] Compliance Mode ON")

            # 1. 이동
            pub_msg("Place 위치로 이동 중...")
            amovel(posx(490.55, 233.19, 463.79, 43.27, 179.97, 43.28), vel=VELOCITY, acc=ACC, radius=30.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion_safe(): raise Exception("Stop")
            
            amovej(posj(-25.44, 25.82, 64.53, 14.31, 106.11, -39.22), vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion_safe(): raise Exception("Stop")

            amovej(posj(-50.09, 40.70, 63.67, 35.43, 97.23, -91.51), vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion_safe(): raise Exception("Stop")

            # 2. 진입
            pub_msg("건조대 진입 중...")
            amovej(posj(-49.66, 72.86, 47.25, 59.27, 131.00, -131.66), vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            wait_for_motion_safe()

            amovel(posx(0.00, 100.00, 0.00, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, radius=0.00, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion_safe(): raise Exception("Stop")

            # 3. 내려놓기
            pub_msg("도마 내려놓기")
            gripper(0) # 닫기(놓기)
            time.sleep(1.0)

            # 4. 복귀
            pub_msg("복귀 중...")
            amovej(posj(-45.31, 61.34, 71.30, 59.33, 121.63, -141.12), vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            wait_for_motion_safe()

            amovej(posj(-45.39, 57.76, 68.16, 63.27, 123.92, -135.78), vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            wait_for_motion_safe()

            amovej(posj(-45.01, 50.34, 67.31, 63.27, 124.55, -128.64), vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            wait_for_motion_safe()

            amovej(posj(-50.13, 27.81, 92.24, 64.66, 104.44, -124.11), vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            wait_for_motion_safe()

            amovej(posj(-50.42, 0.29, 108.69, 64.81, 80.28, -97.76), vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            wait_for_motion_safe()

            amovej(posj(-24.57, -15.92, 104.11, 34.45, 52.34, -45.38), vel=VELOCITY, acc=ACC, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            wait_for_motion_safe()

            amovel(posx(334.04, -13.61, 525.83, 0.45, 151.57, 3.42), vel=VELOCITY_fast, acc=ACC_fast, radius=30.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion_safe(): raise Exception("Stop")

            amovel(posx(636.87, -13.61, 525.82, 0.45, 151.57, 3.42), vel=VELOCITY_fast, acc=ACC_fast, radius=30.00, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion_safe(): raise Exception("Stop")

            amovej(posj(-2.21, 31.13, 61.13, 1.06, 59.34, 0.71), vel=VELOCITY-10, acc=ACC-10, radius=30.00, ra=DR_MV_RA_DUPLICATE)
            if not wait_for_motion_safe(): raise Exception("Stop")

            # Gripper 열기
            gripper(1)
            time.sleep(1.0)

            # 마무리
            amovej(posj(-0.8, 16.18, 69.13, 1.06, 61.25, 0.71), vel=VELOCITY, acc=ACC, radius=0.00, ra=DR_MV_RA_DUPLICATE)
            wait_for_motion_safe()

            # 홈 복귀
            pub_msg("홈 복귀")
            home_pose_j = [0, 0, 90, 0, 90, 0]
            amovej(home_pose_j, vel=30, acc=30)
            if not wait_for_motion_safe(): raise Exception("Stop")

            # --- [안전 모드 종료] ---
            release_compliance_ctrl()
            logger.info("[Safety] Compliance Mode OFF")

            logger.info("완료")
            result = BrushingAction.Result()
            result.complete_task = True
            result.total_duration = time.time() - start_time
            result.final_pose, _ = get_current_posx()
            g_final_result = result
            g_current_goal_handle.succeed()

        except Exception as e:
            logger.error(f"Err: {e}")
            try: release_compliance_ctrl()
            except: pass
            g_final_result = BrushingAction.Result()
            g_final_result.complete_task = False
            if g_current_goal_handle: g_current_goal_handle.abort()
        finally:
            task_running = False
            trigger_event.clear()

def goal_callback(req): return GoalResponse.ACCEPT if not task_running else GoalResponse.REJECT
def execute_callback(gh):
    global g_current_goal_handle, trigger_event
    g_current_goal_handle = gh; trigger_event.set()
    while g_current_goal_handle and rclpy.ok(): time.sleep(0.1)
    return g_final_result
def ros_spin():
    exec = MultiThreadedExecutor(); exec.add_node(node_)
    ActionServer(node_, BrushingAction, 'do_doma_place_action', execute_callback, goal_callback)
    exec.spin()

def main(args=None):
    global node_, cli_move_pause, cli_move_resume
    rclpy.init(args=args)
    import random; time.sleep(random.uniform(0.1, 1.0))
    node_ = rclpy.create_node("doma_place_action_server", namespace=ROBOT_ID)
    DR_init.__dsr__node = node_
    cli_move_pause = node_.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
    cli_move_resume = node_.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')
    node_.create_service(SetBool, 'custom_pause', callback_pause)
    
    if initialize_robot():
        t1 = threading.Thread(target=perform_task_loop)
        t2 = threading.Thread(target=ros_spin)
        t1.start(); t2.start(); t1.join(); t2.join()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__": main()