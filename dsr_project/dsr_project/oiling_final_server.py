#!/usr/bin/env python3
# oiling_final_server.py

import rclpy
from rclpy.node import Node
import DR_init
import threading
import time
import math
import random
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from my_robot_interfaces.action import BrushingAction
from std_srvs.srv import SetBool
from dsr_msgs2.srv import MovePause, MoveResume

# --- 로봇 설정 ---
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight" 
ROBOT_TOOL = "GripperDA_v1"
VELOCITY = 60 
ACC = 60 
Time = 2.0

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
        logger.error(f"초기화 실패: {e}")
        return False

def callback_pause(request, response):
    global node_, cli_move_pause, cli_move_resume
    if not cli_move_pause.service_is_ready():
        response.success = False
        return response

    if request.data:
        node_.get_logger().warn("⛔ PAUSE")
        cli_move_pause.call_async(MovePause.Request())
    else:
        node_.get_logger().info("✅ RESUME")
        cli_move_resume.call_async(MoveResume.Request())
    response.success = True
    return response

def perform_task_loop():
    global task_running, trigger_event, node_, g_current_goal_handle, g_final_result
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
        logger.info("Oiling 작업 시작")

        try:
            from DSR_ROBOT2 import (
                amovel, amovej, check_motion, set_digital_output,
                posx, posj, DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL,
                DR_MV_RA_DUPLICATE, OFF, ON, get_current_posx, wait,
                set_ref_coord, task_compliance_ctrl, set_desired_force,
                release_force, release_compliance_ctrl, check_force_condition,
                DR_FC_MOD_REL, DR_AXIS_Z
            )

            feedback_msg = BrushingAction.Feedback()

            # [추가] 피드백 발행 헬퍼 함수
            def publish_feedback(msg, stroke, percent):
                feedback_msg.feedback_string = msg
                feedback_msg.current_stroke = stroke
                feedback_msg.task_percentage = float(percent)
                if g_current_goal_handle:
                    g_current_goal_handle.publish_feedback(feedback_msg)

            # [Helper] 부드러운 진행률 업데이트
            def wait_and_publish(target_time, start_pct, end_pct, msg, stroke=0):
                duration = target_time if target_time > 0 else 0.5
                t_start = time.time()
                time.sleep(0.2) 

                while check_motion() != 0:
                    elapsed = time.time() - t_start
                    ratio = min(elapsed / duration, 1.0)
                    current_pct = start_pct + (end_pct - start_pct) * ratio
                    
                    publish_feedback(msg, stroke, current_pct)
                    
                    time.sleep(0.1)
                    if not rclpy.ok(): return False
                
                publish_feedback(msg, stroke, end_pct)
                return True

            def gripper(switch):
                if switch == 0: set_digital_output(1, ON); set_digital_output(2, OFF)
                elif switch == 1: set_digital_output(1, OFF); set_digital_output(2, ON)
                wait(1.0)

            # --- 동작 시퀀스 ---
            
            def oiling_pick():
                logger.info("1. 오일 집기 (0% ~ 10%)")
                
                amovel(posx(352.89, 241.49, 431.28, 42.12, 179.97, 42.13), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                if not wait_and_publish(2.0, 0.0, 2.0, "접근 중"): return False

                amovel(posx(140.00, -7.98, 0.00, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                wait_and_publish(1.0, 2.0, 4.0, "위치 조정")
                
                amovel(posx(0.00, 0.00, -85.78, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                wait_and_publish(1.0, 4.0, 6.0, "내려가기")
                
                gripper(0) # Close

                amovel(posx(0.00, 0.00, 119.42, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                wait_and_publish(1.0, 6.0, 8.0, "들어올리기")
                
                amovel(posx(-150.00, -199.10, 0.00, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                wait_and_publish(1.0, 8.0, 10.0, "이동 중")
                return True

            def oiling_move(num_strokes, y_step_size, move_x, vel_x, acc_x, vel_y, acc_y, vel_z, acc_z):
                logger.info("2. 오일 칠하기 (10% ~ 90%)")
                
                start_pct = 10.0; end_pct = 90.0
                pct_per_stroke = (end_pct - start_pct) / num_strokes

                for y_move_cnt in range(int(num_strokes)):
                    s_p = start_pct + (y_move_cnt * pct_per_stroke)
                    e_p = s_p + pct_per_stroke
                    step_p = pct_per_stroke / 3.0 # 한 줄 내 동작 3분할
                    msg = f"오일링: {y_move_cnt+1}/{num_strokes} 줄"

                    # 1. 시작점 이동 (하강)
                    if y_move_cnt == 0:
                        amovel([0,0,-65,0,0,0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        if not wait_and_publish(1.0, s_p, s_p + (step_p*0.2), "작업 위치 진입"): return False

                    # 왕복 로직
                    if y_move_cnt % 2 == 0: # 짝수 줄
                        amovel([move_x+10,0,0,0,0,0], vel=vel_x, acc=acc_x, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        wait_and_publish(2.0, s_p, s_p + step_p, msg, y_move_cnt+1)
                        
                        amovel([0,0,20,0,0,0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        wait_and_publish(0.5, s_p + step_p, s_p + step_p*1.5, msg, y_move_cnt+1)
                        
                    else: # 홀수 줄
                        amovel([-move_x-25,0,0,0,0,0], vel=vel_x, acc=acc_x, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        wait_and_publish(2.0, s_p, s_p + step_p, msg, y_move_cnt+1)
                        
                        amovel([0,0,20,0,0,0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        wait_and_publish(0.5, s_p + step_p, s_p + step_p*1.5, msg, y_move_cnt+1)

                    # 다음 줄 이동
                    if y_move_cnt < num_strokes - 1:
                        amovel([0,-65,0,0,0,0], vel=vel_y, acc=acc_y, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        amovel([0,0,-20,0,0,0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        wait_and_publish(0.5, s_p + step_p*2, e_p, "다음 줄 준비", y_move_cnt+1)
                    
                return True

            def oiling_place():
                logger.info("3. 오일 내려놓기 (90% ~ 100%)")
                
                amovel(posx(598.34, -93.08, 463.79, 42.80, 179.97, 42.81), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                if not wait_and_publish(2.0, 90.0, 93.0, "제자리 이동"): return False

                amovel(posx(505.55, 233.19, 463.79, 43.27, 179.97, 43.28), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                wait_and_publish(2.0, 93.0, 96.0, "안착 위치 이동")

                amovel(posx(0.00, 0.00, -116.69, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                wait_and_publish(2.0, 96.0, 99.0, "내려놓기")

                gripper(1)

                publish_feedback("작업 완료", 0, 100.0) # [수정] 이제 함수가 존재하므로 에러 없음
                return True

            # --- 메인 실행 ---
            board_h = 200; board_w = 300; sponge_h = 95; sponge_w = 25
            num_strokes = math.ceil(board_h / sponge_h)
            total_overlap = (sponge_h * num_strokes) - board_h
            overlap_count = num_strokes - 1
            y_step_size = sponge_h - (total_overlap / overlap_count) if overlap_count > 0 else board_h
            move_x = board_w - sponge_w
            vel_x = 50; acc_x = 50; vel_y = 50; acc_y = 50; vel_z = 30; acc_z = 30

            if not oiling_pick(): raise Exception("Pick Fail")
            if not oiling_move(num_strokes, y_step_size, move_x, vel_x, acc_x, vel_y, acc_y, vel_z, acc_z): raise Exception("Move Fail")
            if not oiling_place(): raise Exception("Place Fail")

            result = BrushingAction.Result()
            result.complete_task = True
            result.total_duration = time.time() - start_time
            result.final_pose, _ = get_current_posx()
            g_final_result = result
            g_current_goal_handle.succeed()

        except Exception as e:
            logger.error(f"Err: {e}")
            g_final_result = BrushingAction.Result()
            g_final_result.complete_task = False
            if g_current_goal_handle: g_current_goal_handle.abort()
        finally:
            task_running = False; trigger_event.clear()

def goal_callback(req): return GoalResponse.ACCEPT if not task_running else GoalResponse.REJECT
def execute_callback(gh):
    global g_current_goal_handle, trigger_event
    g_current_goal_handle = gh; trigger_event.set()
    while g_current_goal_handle and rclpy.ok(): time.sleep(0.1)
    return g_final_result

def ros_spin():
    exec = MultiThreadedExecutor(); exec.add_node(node_)
    ActionServer(node_, BrushingAction, 'do_oiling_action', execute_callback, goal_callback)
    exec.spin()

def main(args=None):
    global node_, cli_move_pause, cli_move_resume
    rclpy.init(args=args)
    import random; time.sleep(random.uniform(0.1, 1.0))
    node_ = rclpy.create_node("oiling_action_server", namespace=ROBOT_ID)
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