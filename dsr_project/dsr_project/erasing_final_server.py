#!/usr/bin/env python3
# erasing_final_server.py
# 수정: Pause 대신 '속도 1%' 감속 적용 -> 멈춤 현상/씹힘 현상 완벽 해결

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

# Service
from std_srvs.srv import SetBool
from dsr_msgs2.srv import MovePause, MoveResume

# --- 로봇 설정 ---
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight" 
ROBOT_TOOL = "GripperDA_v1"
VELOCITY = 60 
ACC = 60 

# [안전 설정]
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

# [상태 플래그] 현재 감속 모드인지 확인
g_slow_mode = False 

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
    logger.info("Tool/TCP 설정 시작...")
    try:
        set_tool_func(ROBOT_TOOL)
        set_tcp_func(ROBOT_TCP)
        logger.info("로봇 초기화 완료.")
        return True
    except Exception as e:
        logger.error(f"[Init] set_tool/set_tcp 실행 실패: {e}")
        return False

def callback_pause(request, response):
    """
    MovePause 대신 속도 조절 함수를 사용하여 멈춤 효과를 냄
    """
    global node_, g_slow_mode
    
    # DSR 함수 임포트 (안전하게)
    try:
        from DSR_ROBOT2 import change_operation_speed
    except:
        response.success = False; return response

    if request.data: # PAUSE 요청 -> 속도 1%
        node_.get_logger().warn("⛔ [CMD] PAUSE -> 속도 1% (정지 효과)")
        change_operation_speed(1)
        g_slow_mode = True
        response.message = "Speed 1%"
    else:            # RESUME 요청 -> 속도 100%
        node_.get_logger().info("✅ [CMD] RESUME -> 속도 100% (정상화)")
        change_operation_speed(100)
        g_slow_mode = False
        response.message = "Speed 100%"
        
    response.success = True
    return response

def perform_task_loop():
    global task_running, trigger_event, node_, g_current_goal_handle, g_final_result, g_slow_mode
    logger = node_.get_logger()
    print("[Task Thread] 작업 스레드 대기 중...")
    
    while rclpy.ok():
        triggered = trigger_event.wait(timeout=1.0)
        if not rclpy.ok(): break
        if not triggered: continue

        if task_running:
            trigger_event.clear(); continue

        task_running = True
        start_time = time.time()
        logger.info("=== Erasing (Speed Control Safety) 시작 ===")
        
        try:
            from DSR_ROBOT2 import (
                amovel, check_motion, change_operation_speed,
                release_compliance_ctrl, release_force,
                check_force_condition, task_compliance_ctrl, set_desired_force,
                set_ref_coord, get_digital_output, set_digital_output,
                movel, get_current_posx, wait, set_user_cart_coord, get_tool_force,
                DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE,
                DR_FC_MOD_REL, DR_AXIS_Z, DR_BASE, DR_TOOL, OFF, ON,
                posx, posj
            )

            # 시작 시 속도 초기화
            change_operation_speed(100)
            g_slow_mode = False

            # [핵심] 속도 제어 기반 대기 함수
            def wait_for_motion():
                global g_slow_mode
                time.sleep(0.2) # 명령 전달 대기

                # 로봇이 움직이는 동안 루프
                while check_motion() != 0:
                    
                    # 1. 충돌 감지 (정상 속도일 때만 체크)
                    if not g_slow_mode:
                        try:
                            forces = get_tool_force(DR_TOOL)
                            force_mag = math.sqrt(forces[0]**2 + forces[1]**2 + forces[2]**2)
                            
                            if force_mag > SAFETY_FORCE_LIMIT:
                                logger.warn(f"🚨 충돌 감지({force_mag:.1f}N) -> 속도 1% 감속")
                                change_operation_speed(1) # 거의 멈춤
                                g_slow_mode = True
                                # 여기서 continue하면 아래 sleep(0.05)를 건너뛰고 바로 루프 재시작
                                continue
                        except: pass

                    # 2. 감속 모드(Pause)면 여기서 대기 효과
                    if g_slow_mode:
                        # 1% 속도라도 움직임 상태(Busy)는 유지되므로 루프 안 끊김!
                        time.sleep(0.5) 
                        continue

                    time.sleep(0.05)
                    if not rclpy.ok(): return False
                
                return True

            def gripper(switch):
                if switch == 0: set_digital_output(1,ON); set_digital_output(2,OFF)
                elif switch == 1: set_digital_output(1,OFF); set_digital_output(2,ON)
                wait(1.0)

            def force_control(switch, press_force=10):
                global g_slow_mode
                if not rclpy.ok(): return False
                if switch == 1:
                    set_ref_coord(1) 
                    # 안전 모드 유지 (부드럽게)
                    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                    wait(0.5) 
                    set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
                    
                    start_chk = time.time()
                    while rclpy.ok() and (time.time() - start_chk < 5.0):
                        # 힘 제어 중 감속되면 대기
                        if g_slow_mode: 
                            time.sleep(0.5); continue

                        if check_force_condition(DR_AXIS_Z, max=press_force) == -1: break
                        time.sleep(0.1)
                    wait(0.5)
                elif switch == 0:
                    release_force()
                    # 안전 강성 복귀
                    task_compliance_ctrl(stx=[200, 200, 200, 100, 100, 100])
                return True

            def oiling_move(DR_USER1, num_strokes, y_step_size, move_x, feedback_msg):
                vel_x=50; acc_x=50; vel_y=50; acc_y=50; vel_z=30; acc_z=30
                if not force_control(1,14): raise Exception("힘제어 실패")

                for y_move_cnt in range(int(num_strokes)):
                    feedback_msg.feedback_string = f"지우개 닦기 ({y_move_cnt+1}/{num_strokes})"
                    feedback_msg.current_stroke = y_move_cnt + 1
                    if g_current_goal_handle: g_current_goal_handle.publish_feedback(feedback_msg)

                    if y_move_cnt %2 == 0:
                        if not force_control(0): raise Exception("Err")
                        amovel([move_x,0, 0,0,0,0], vel=vel_x, time=2.0, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                        amovel([-move_x,0, 0,0,0,0], vel=vel_x, time=2.0, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                        amovel([move_x,0, 0,0,0,0], vel=vel_x, time=2.0, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                    else:
                        if not force_control(0): raise Exception("Err")
                        amovel([-move_x,0, 0,0,0,0], vel=vel_x, time=2.0, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                        amovel([move_x,0, 0,0,0,0], vel=vel_x, time=2.0, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                        amovel([-move_x,0, 0,0,0,0], vel=vel_x, time=2.0, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False

                    logger.info(f'{y_move_cnt+1}줄 완료')

                    if y_move_cnt < num_strokes - 1:
                        amovel([0, 0, +10,0,0,0], vel=vel_y, time=1.0, acc=acc_y, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                        amovel([0,-y_step_size, 0,0,0,0], vel=vel_y, time=1.0, acc=acc_y, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                        if not force_control(1,14): raise Exception("Err")

                logger.info('oiling 완료')   
                if not force_control(0): raise Exception("Err")
                
                amovel([0,0,100,0,0,0], vel=vel_z, acc=acc_z, ref=DR_USER1, mod=DR_MV_MOD_REL)
                if not wait_for_motion(): return False
                return True

            # --- Main Sequence ---
            feedback_msg = BrushingAction.Feedback()

            # 1. 안전 모드 ON
            task_compliance_ctrl(stx=[200, 200, 200, 100, 100, 100])
            logger.info("[Safety] Compliance Mode ON")
            time.sleep(1.0) # 안정화 대기

            # 2. Pick (좌표 동일)
            logger.info("1. 지우개 집기")
            feedback_msg.feedback_string = "지우개 집기"
            if g_current_goal_handle: g_current_goal_handle.publish_feedback(feedback_msg)
            
            amovel(posx(330.44, 289.61, 424.88, 0, -180, 0), vel=VELOCITY, time=2.0, acc=ACC, ref=0, mod=DR_MV_MOD_ABS)
            if not wait_for_motion(): raise Exception("Stop")
            gripper(1)
            amovel(posx(330.44, 289.61, 310.78, 0, -180, 0), vel=VELOCITY, time=2.0, acc=ACC, ref=0, mod=DR_MV_MOD_ABS)
            if not wait_for_motion(): raise Exception("Stop")
            gripper(0)
            amovel(posx(330.44, 289.61, 424.88, 0, -180, 0), vel=VELOCITY, time=2.0, acc=ACC, ref=0, mod=DR_MV_MOD_ABS)
            if not wait_for_motion(): raise Exception("Stop")

            # 3. Coord Setup
            logger.info("2. 좌표계 설정")
            p1 = [331.013, 67.332, 330.540]; p2 = [614.632, 59.905, 330.540]; p4 = [326.180, -117.220, 330.540]
            p_origin = [326.180, -117.220, 330.540, 157.52, 180, 166.7]
            list_x = [p2[i]-p1[i] for i in range(3)]; A = math.sqrt(sum(i**2 for i in list_x)); list_X = [i/A for i in list_x]
            list_y = [p1[i]-p4[i] for i in range(3)]; B = math.sqrt(sum(i**2 for i in list_y)); list_Y = [i/B for i in list_y]
            DR_USER1 = set_user_cart_coord(list_X, list_Y, p_origin, ref=DR_BASE)

            board_h = 214; sponge_h = 23; num_strokes = math.ceil(board_h / sponge_h)
            total_overlap = (sponge_h * num_strokes) - board_h
            # 누락되었던 overlap_count 복구
            overlap_count = num_strokes - 1
            y_step_size = sponge_h - (total_overlap / overlap_count) if overlap_count > 0 else board_h
            move_x = 300 - 8

            # 4. Erasing Loop
            logger.info("3. 닦기 준비")
            amovel(posx([331.79, 65.310, 350.540, 13.71, 180.0, 12.62]), vel=30, time=2.0, acc=30, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            if not wait_for_motion(): raise Exception("Stop")
            
            amovel([0,0,-7,0,0,0], vel=30, acc=30, ref=DR_BASE, mod=DR_MV_MOD_REL); wait_for_motion()
            gripper(1)
            amovel([0,0,-30,0,0,0], vel=30, acc=30, ref=DR_BASE, mod=DR_MV_MOD_REL); wait_for_motion()
            gripper(0)
            amovel([0,0,+10,0,0,0], vel=30, acc=30, ref=DR_BASE, mod=DR_MV_MOD_REL); wait_for_motion()

            set_ref_coord(DR_USER1)
            if not oiling_move(DR_USER1, num_strokes, y_step_size, move_x, feedback_msg): raise Exception("Loop Fail")

            # 5. Place
            logger.info("4. 내려놓기")
            feedback_msg.feedback_string = "지우개 내려놓기"
            if g_current_goal_handle: g_current_goal_handle.publish_feedback(feedback_msg)

            amovel(posx(330.44, 289.61, 400.54, 64.35, -180.00, 63.26), vel=VELOCITY, time=2.0, acc=ACC, ref=0, mod=DR_MV_MOD_ABS)
            if not wait_for_motion(): raise Exception("Stop")
            amovel(posx(0.00, 0.00, -85.26, 0.00, 0.00, 0.00), vel=VELOCITY, time=2.0, acc=ACC, ref=0, mod=DR_MV_MOD_REL)
            if not wait_for_motion(): raise Exception("Stop")
            gripper(1)

            # --- [Safety OFF] ---
            release_compliance_ctrl()
            logger.info("[Safety] Compliance OFF")
            change_operation_speed(100) # 속도 복구

            logger.info("완료")
            result = BrushingAction.Result()
            result.complete_task = True
            result.total_duration = time.time() - start_time
            result.final_pose , _ = get_current_posx(ref=DR_BASE)
            g_final_result = result
            g_current_goal_handle.succeed()

        except Exception as e:
            logger.error(f"Err: {e}")
            try: 
                release_compliance_ctrl()
                change_operation_speed(100)
            except: pass
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
    ActionServer(node_, BrushingAction, 'do_eraser_action', execute_callback, goal_callback)
    exec.spin()

def main(args=None):
    global node_, cli_move_pause, cli_move_resume
    rclpy.init(args=args)
    import random; time.sleep(random.uniform(0.1, 1.0))
    node_ = rclpy.create_node("eraser_action_server", namespace=ROBOT_ID)
    DR_init.__dsr__node = node_
    cli_move_pause = node_.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
    cli_move_resume = node_.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')
    node_.create_service(SetBool, 'custom_pause', callback_pause)
    
    print(f"노드 '{ROBOT_ID}/eraser_action_server' 생성 완료.")

    if initialize_robot():
        t1 = threading.Thread(target=perform_task_loop)
        t2 = threading.Thread(target=ros_spin)
        t1.start(); t2.start(); t1.join(); t2.join()
    if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()