#!/usr/bin/env python3
# brushing_safety_server_v1.3.py

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

# Service / Msg
from std_srvs.srv import SetBool
from dsr_msgs2.srv import MovePause, MoveResume, ChangeOperationSpeed
from dsr_msgs2.msg import RobotState

# --- 로봇 설정 ---
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"
VELOCITY = 100
ACC = 100
Time = 2.0

# --- 안전 설정 ---
SAFETY_FORCE_TRIGGER = 20.0
SAFETY_FORCE_RESET = 10.0
SAFE_SPEED = 1
NORMAL_SPEED = 100

# --- DR_init ---
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# --- 전역 변수 (재정의) ---
node_ = None
trigger_event = threading.Event()
task_running = False
g_current_external_force = [0.0] * 6 # [Fx, Fy, Fz, Mx, My, Mz]
g_is_slow_mode = False
g_last_topic_time = 0.0
g_current_goal_handle = None
g_final_result = None
cli_move_pause = None
cli_move_resume = None
cli_change_speed = None

# Action 관련
g_current_goal_handle = None
g_final_result = None

# 서비스
cli_move_pause = None
cli_move_resume = None
cli_change_speed = None

# 모니터링 데이터
g_current_external_force = [0.0] * 6
g_is_slow_mode = False
g_last_topic_time = 0.0


def initialize_robot():
    """로봇 초기화 (TCP/Tool 설정)"""
    global set_tool_func, set_tcp_func, node_
    logger = node_.get_logger()
    try:
        from DSR_ROBOT2 import set_tool, set_tcp
        set_tool_func = set_tool
        set_tcp_func = set_tcp
    except Exception as e:
        logger.error(f"DSR_ROBOT2 임포트 실패: {e}")
        return False

    logger.info("로봇 초기 설정(Tool, TCP) 시작...")
    try:
        set_tool_func(ROBOT_TOOL)
        set_tcp_func(ROBOT_TCP)
        logger.info("로봇 초기화 완료.")
        return True
    except Exception as e:
        logger.error(f"설정 실패: {e}")
        return False

def robot_state_callback(msg):
    """
    로봇 상태 수신: 토픽명 확인 결과, /dsr01/state가 더 흔합니다.
    외력 필드: f_world_ext_target_torque (Task Force) 사용.
    """
    global g_current_external_force, g_last_topic_time
    # [수정] 최신 로봇 상태 메시지의 World 기준 외력 필드를 사용
    if hasattr(msg, 'f_world_ext_target_torque'):
        g_current_external_force = msg.f_world_ext_target_torque 
    else:
        # 구버전이거나 필드명 오류 시 fallback (차후 디버깅용)
        g_current_external_force = msg.actual_ett 

    g_last_topic_time = time.time()

def safety_monitor_loop():
    """안전 감시 스레드: 외력 감지 시 속도 조절만 담당."""
    global g_current_external_force, g_is_slow_mode, cli_change_speed, node_, g_last_topic_time
    
    logger = node_.get_logger()
    logger.info("[Safety Thread] 감시 시스템 시작 (Trigger: 20N, Reset: 10N)")
    
    # 5초간 토픽 수신 대기 로직은 제거하고 바로 루프 진입 (main에서 ROS가 켜졌다고 가정)
    
    while rclpy.ok():
        try:
            # 1. 외력 크기 계산 (Fx, Fy, Fz만 사용)
            fx, fy, fz = g_current_external_force[0], g_current_external_force[1], g_current_external_force[2]
            force_magnitude = math.sqrt(fx**2 + fy**2 + fz**2)
            
            # 2. 속도 조절 서비스 준비 확인
            if cli_change_speed is not None and cli_change_speed.service_is_ready():
                
                # 위험 감지 (20N 초과)
                if force_magnitude > SAFETY_FORCE_TRIGGER and not g_is_slow_mode:
                    logger.warn(f"⚠️ 충돌 감지 ({force_magnitude:.1f}N)! 속도 {SAFE_SPEED}%로 제한.")
                    req = ChangeOperationSpeed.Request()
                    req.speed = int(SAFE_SPEED)
                    cli_change_speed.call_async(req)
                    g_is_slow_mode = True
                
                # 안전 복귀 (10N 미만)
                elif force_magnitude < SAFETY_FORCE_RESET and g_is_slow_mode:
                    logger.info(f"✅ 외력 해제 ({force_magnitude:.1f}N). 속도 {NORMAL_SPEED}% 복귀.")
                    req = ChangeOperationSpeed.Request()
                    req.speed = int(NORMAL_SPEED)
                    cli_change_speed.call_async(req)
                    g_is_slow_mode = False
            
            time.sleep(0.1) # 10Hz 감시 주기
            
        except Exception as e:
            logger.error(f"[Safety Thread] 오류: {e}")
            time.sleep(1.0)

def callback_pause(request, response):
    """Pause/Resume 서비스"""
    global node_, cli_move_pause, cli_move_resume
    
    if not cli_move_pause.service_is_ready():
        response.success = False; response.message = "Robot Service not ready"; return response

    if request.data: # PAUSE
        node_.get_logger().warn("⛔ Command: PAUSE Triggered")
        cli_move_pause.call_async(MovePause.Request())
        response.message = "Paused"; response.success = True
    else:            # RESUME
        node_.get_logger().info("✅ Command: RESUME Triggered")
        cli_move_resume.call_async(MoveResume.Request())
        response.message = "Resumed"; response.success = True
    return response

def perform_task_loop():
    """Task 스레드 (로봇 동작 시퀀스)"""
    global task_running, trigger_event, node_, g_current_goal_handle, g_final_result
    logger = node_.get_logger()
    
    while rclpy.ok():
        triggered = trigger_event.wait(timeout=1.0)
        if not rclpy.ok(): break
        
        if triggered:
            if not task_running:
                task_running = True
                start_time = time.time()
                logger.info("[Task Thread] 작업 시퀀스 시작.")
                
                try:
                    from DSR_ROBOT2 import (
                        amovel, amovec, check_motion, posx, set_user_cart_coord,
                        set_digital_output, get_current_posx, amovej,
                        DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, ON, OFF, 
                        DR_MV_RA_DUPLICATE
                    )

                    def pub_feedback(msg, pct):
                        if g_current_goal_handle:
                            fb = BrushingAction.Feedback()
                            fb.feedback_string = msg
                            fb.progress_percentage = float(pct)
                            g_current_goal_handle.publish_feedback(fb)

                    def wait_with_feedback(msg, target_time, start_pct, end_pct):
                        time.sleep(0.1)
                        start_t = time.time()
                        while check_motion() != 0:
                            elapsed = time.time() - start_t
                            ratio = elapsed / target_time if target_time > 0 else 1.0
                            if ratio > 1.0: ratio = 1.0
                            cur = start_pct + (end_pct - start_pct) * ratio
                            pub_feedback(msg, cur)
                            time.sleep(0.1)
                            if not rclpy.ok(): return False
                        pub_feedback(msg, end_pct)
                        return True

                    def gripper(switch):
                        if switch == 0: set_digital_output(1, ON); set_digital_output(2, OFF)
                        elif switch == 1: set_digital_output(1, OFF); set_digital_output(2, ON)

                    # --- 동작 정의 ---
                    def brush_pick():
                        logger.info('[Step 1] Pick 시작')
                        amovel(posx(325.82, 267.29, 424.88, 82.80, 175.80, 80.38), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        if not wait_with_feedback("도구 집기 이동", Time, 0.0, 3.0): return False
                        
                        amovel(posx(365.81, 241.49, 424.88, 42.53, 179.97, 42.54), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        if not wait_with_feedback("접근 중", Time, 3.0, 5.0): return False
                        
                        amovel(posx(0.00, 0.00, -98.31, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        if not wait_with_feedback("그리퍼 하강", Time, 5.0, 8.0): return False

                        gripper(0); time.sleep(1.0) 
                        
                        amovel(posx(348.81, 241.49, 418.61, 42.19, 179.97, 42.20), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        wait_with_feedback("상승 중", Time, 8.0, 9.0)
                        
                        amovel(posx(-35.70, -208.74, 0.00, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        wait_with_feedback("이동 중", Time, 9.0, 10.0)
                        
                        amovel(posx(313.12, 32.75, 388.61, 42.27, 179.97, 42.28), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        wait_with_feedback("Pick 완료 위치", Time, 10.0, 10.0)
                        return True

                    def brushing_move_2(DR_USER1, board_w, num_strokes, y_step_size):
                        logger.info('[Step 2] 원형 브러싱 시작')
                        vel_x = 60; acc_x = 60; beta = 5
                        P_1_init = posx([273.64,20.97,418.61-beta,5.05,-179.58,4.44])
                        P_2_init = posx([323.64,20.97,388.61-beta,5.05,-179.58,4.44])
                        P_3_init = posx([373.64,20.97,418.61-beta,5.05,-179.58,4.44])
                        list_P_init = [P_1_init, P_2_init, P_3_init]

                        start_pct = 10.0; end_pct = 50.0
                        pct_step = (end_pct - start_pct) / num_strokes
                        delta = 25
                        
                        for y_cnt in range(int(num_strokes)):
                            s_pct = start_pct + (y_cnt * pct_step)
                            e_pct = s_pct + pct_step
                            
                            P_1, P_2, P_3 = [p.copy() for p in list_P_init]
                            list_P = [P_1,P_2,P_3]; x_cnt = 0
                            
                            while delta * x_cnt < board_w:
                                msg = f"원형 브러싱 {y_cnt+1}/{num_strokes}"
                                amovel(P_1, time=0.5, vel=vel_x, acc=acc_x, mod=DR_MV_MOD_ABS)
                                if not wait_with_feedback(msg, 0.5, s_pct, s_pct): return False
                                
                                amovec(P_2, P_3, time=0.5, vel=vel_x, acc=acc_x, mod=DR_MV_MOD_ABS)
                                if not wait_with_feedback(msg, 0.5, s_pct, e_pct): return False
                                x_cnt += 1
                                for P in list_P: P[0] += delta
                            
                            for P in list_P_init: P[1] -= abs(y_step_size)
                            if y_cnt < num_strokes - 1:
                                amovel(list_P_init[0], vel=vel_x, acc=acc_x, mod=DR_MV_MOD_ABS)
                                wait_with_feedback("다음 줄 이동", 1.0, e_pct, e_pct)

                        amovel([0,0,30,0,0,0], vel=60, acc=60, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        wait_with_feedback("상승", 1.0, 50.0, 50.0)
                        
                        amovej([0, 0, 90, 0, 90, 0], vel=30, acc=30, time=Time)
                        if not wait_with_feedback("홈 정렬", Time, 50.0, 50.0): return False
                        return True

                    def brushing_move_1(DR_USER1, board_w, num_strokes, y_step_size, move_x):
                        logger.info('[Step 3] 직선 브러싱 시작')
                        vel_x = 60; acc_x = 60; vel_y = 50; acc_y = 50
                        
                        amovel([323.64,20.97,388.61,5.05,-179.58,4.44], vel=50, acc=50, time=Time, mod=DR_MV_MOD_ABS)
                        if not wait_with_feedback("위치 잡기", Time, 50.0, 50.0): return False

                        start_pct = 50.0; end_pct = 90.0
                        pct_step = (end_pct - start_pct) / num_strokes
                        
                        for y_cnt in range(int(num_strokes)):
                            s_pct = start_pct + (y_cnt * pct_step)
                            e_pct = s_pct + pct_step
                            amovel([move_x,0, 0,0,0,0], vel=vel_x, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                            if not wait_with_feedback(f"직선 브러싱 {y_cnt+1}/{num_strokes}", 7.0, s_pct, e_pct): return False
                            
                            amovel([0,0,15,0,0,0], vel=vel_x, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                            wait_with_feedback("들기", 0.5, e_pct, e_pct)
                            
                            if y_cnt < num_strokes - 1:
                                amovel([-move_x,-y_step_size, 0,0,0,0], vel=vel_y, acc=acc_y, ref=DR_USER1, mod=DR_MV_MOD_REL)
                                wait_with_feedback("복귀", 1.5, e_pct, e_pct)
                                
                                amovel([0,0,-15,0,0,0], vel=vel_x, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                                wait_with_feedback("내리기", 0.5, e_pct, e_pct)

                        amovel([0,0,50,0,0,0], vel=60, acc=60, time=Time, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        wait_with_feedback("완료 상승", 1.0, 90.0, 90.0)
                        return True

                    def brush_place():
                        logger.info('[Step 4] Place 시작')
                        amovel(posx(598.34, -93.08, 463.79, 42.80, 179.97, 42.81), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        if not wait_with_feedback("반납 위치 이동", Time, 90.0, 93.0): return False
                        
                        amovel(posx(352.89, 241.49, 463.79, 42.12, 179.97, 42.13), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        wait_with_feedback("접근", Time, 93.0, 95.0)
                        
                        amovel(posx(0.00, 0.00, -130.26, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        wait_with_feedback("하강", Time, 95.0, 99.0)

                        gripper(1); time.sleep(1.0) 
                        pub_feedback("반납 완료", 100.0)
                        return True

                    # --- MAIN EXECUTION ---
                    p1_pos = [321.79, 77.160, 310.540]; p2_pos = [618.860, 65.930, 309.570]; p4_pos = [317.6, -120.8, 307.8]
                    list_x = [p2_pos[i] - p1_pos[i] for i in range(3)]; A = math.sqrt(sum(i**2 for i in list_x)); list_X = [i / A for i in list_x]
                    list_y = [p1_pos[i] - p4_pos[i] for i in range(3)]; B = math.sqrt(sum(i**2 for i in list_y)); list_Y = [i / B for i in list_y]
                    pos = [317.6, -120.8, 307.8, 157.52, 180, 165.5]
                    DR_USER1 = set_user_cart_coord(list_X, list_Y, pos, ref=DR_BASE)
                    logger.info(f"사용자 좌표계 등록 완료 ID: {DR_USER1}")

                    board_h = 200; board_w = 300; brush_h = 110
                    num_strokes = math.ceil(board_h / brush_h); total_overlap = (brush_h * num_strokes) - board_h
                    overlap_count = num_strokes - 1; y_step_size = brush_h - (total_overlap / overlap_count) if overlap_count > 0 else board_h
                    alpha = 10; move_x = board_w + alpha

                    if not rclpy.ok(): raise Exception("Stop")
                    
                    if not brush_pick(): raise Exception("Pick Fail")
                    if not brushing_move_2(DR_USER1, board_w, num_strokes, y_step_size): raise Exception("Brushing 2 Fail")
                    if not brushing_move_1(DR_USER1, board_w, num_strokes, y_step_size, move_x): raise Exception("Brushing 1 Fail")
                    if not brush_place(): raise Exception("Place Fail")

                    result = BrushingAction.Result()
                    result.complete_task = True
                    result.total_duration = time.time() - start_time
                    result.final_pose, _ = get_current_posx()
                    g_final_result = result
                    g_current_goal_handle.succeed()
                    logger.info("모든 작업 완료.")

                except Exception as e:
                    logger.error(f"작업 중 오류: {e}")
                    if g_current_goal_handle and g_current_goal_handle.is_active:
                        g_current_goal_handle.abort()
                finally:
                    task_running = False
                    trigger_event.clear()
                    g_current_goal_handle = None
            else:
                trigger_event.clear()

def goal_callback(goal_request):
    global task_running
    if task_running or not goal_request.start_task: return GoalResponse.REJECT
    return GoalResponse.ACCEPT

def execute_callback(goal_handle):
    global g_current_goal_handle, g_final_result, trigger_event
    g_current_goal_handle = goal_handle
    trigger_event.set()
    while g_current_goal_handle is not None and rclpy.ok(): time.sleep(0.1)
    return g_final_result

def ros_spin_thread():
    global node_
    print("[ROS Thread] ROS Spin 스레드 시작...")
    executor = None # executor를 함수 내에서 정의
    try:
        executor = MultiThreadedExecutor(num_threads=4) 
        executor.add_node(node_)
        
        ActionServer(
            node_,
            BrushingAction,
            'do_brushing_action',
            execute_callback=execute_callback,
            goal_callback=goal_callback,
            cancel_callback=None
        )
        print("[ROS Thread] Action 서버 준비 완료.")
        executor.spin()
    except Exception as e:
        if rclpy.ok(): 
            node_.get_logger().error(f"[ROS Thread] Spin 중 예외 발생: {e}")
    finally:
        if executor:
            executor.shutdown() # Executor 안전 종료
        print("[ROS Thread] ROS Spin 스레드 종료됨.")

def main(args=None):
    global node_, cli_move_pause, cli_move_resume, cli_change_speed
    
    rclpy.init(args=args)
    time.sleep(1.0)
    
    # [수정된 부분] 1. 노드 생성 및 전역 할당
    # 이 단계를 initialize_robot() 호출보다 위로 옮겨야 합니다.
    node_ = rclpy.create_node("brushing_action_server", namespace=ROBOT_ID)
    DR_init.__dsr__node = node_ 
    
    # [2] 초기화 (이제 node_.get_logger()가 작동합니다)
    if not initialize_robot():
        print("로봇 초기화 실패. 종료합니다.")
        return
    
    # [3] 클라이언트 및 구독자 생성
    cli_change_speed = node_.create_client(ChangeOperationSpeed, f'/{ROBOT_ID}/motion/change_operation_speed')
    cli_move_pause = node_.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
    cli_move_resume = node_.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')
    node_.create_service(SetBool, 'custom_pause', callback_pause)
    
    # [4] 토픽 구독 설정 (다른 부분은 동일)
    node_.create_subscription(RobotState, f'/{ROBOT_ID}/state', robot_state_callback, 10)
    
    # 스레드 시작
    robot_thread = threading.Thread(target=perform_task_loop)
    spin_thread = threading.Thread(target=ros_spin_thread)
    safety_thread = threading.Thread(target=safety_monitor_loop) 

    robot_thread.start()
    spin_thread.start()
    safety_thread.start()
    
    try:
        # Main Thread 대기 (ROS2 Shutdown issue fix)
        while rclpy.ok():
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nCtrl+C 감지. 노드를 종료합니다.")
        
    finally:
        # 안전 종료 로직은 그대로 유지
        if rclpy.ok(): rclpy.shutdown()
        # ... (나머지 join 로직은 그대로 유지) ...