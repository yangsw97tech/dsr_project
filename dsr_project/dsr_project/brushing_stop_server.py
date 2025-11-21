# brushing_action_server.py

import rclpy
from rclpy.node import Node
import DR_init
import threading
import time
import math
import random
from rclpy.executors import MultiThreadedExecutor

# Action 관련 모듈
from rclpy.action import ActionServer, GoalResponse, CancelResponse
# 사용자 Action 인터페이스
from my_robot_interfaces.action import BrushingAction

# Pause/Resume 구현을 위한 서비스 모듈
from std_srvs.srv import SetBool
# (주의) 사용자 환경에 맞는 msg 패키지 사용 (dsr_msgs 또는 dsr_msgs2)
from dsr_msgs2.srv import MovePause, MoveResume 

# --- 로봇 설정 상수 ---
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"
VELOCITY = 100
ACC = 100
Time = 2

# --- DR_init 설정 ---
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# --- 전역 변수 ---
node_ = None
trigger_event = threading.Event()
task_running = False
set_tool_func = None
set_tcp_func = None

# Action 관련 전역 변수
g_current_goal_handle = None
g_final_result = None

# 서비스 클라이언트 (로봇 제어용)
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
        logger.error(f"로봇 초기화 중 DSR_ROBOT2 임포트 실패: {e}")
        return False
    logger.info("로봇 초기 설정(Tool, TCP)을 시작합니다...")
    try:
        set_tool_func(ROBOT_TOOL)
        set_tcp_func(ROBOT_TCP)
        logger.info("로봇 초기화 완료.")
        return True
    except Exception as e:
        logger.error(f"set_tool/set_tcp 실행 실패: {e}")
        return False

def callback_pause(request, response):
    """
    GUI/Terminal에서 호출하는 중계 서비스
    실제 로봇의 MovePause / MoveResume 서비스를 호출함
    """
    global node_, cli_move_pause, cli_move_resume
    
    # 클라이언트 연결 확인
    if cli_move_pause is None or cli_move_resume is None:
        response.success = False
        response.message = "Robot Service Clients not initialized"
        return response

    if not cli_move_pause.service_is_ready() or not cli_move_resume.service_is_ready():
        node_.get_logger().error("Robot Services (MovePause/Resume) not ready!")
        response.success = False
        response.message = "Robot Services not connected"
        return response

    if request.data: # True = PAUSE 요청
        node_.get_logger().warn("⛔ Command: PAUSE -> Calling MovePause...")
        
        req = MovePause.Request()
        cli_move_pause.call_async(req) # 비동기 호출
        
        response.message = "Sent MovePause Signal (Robot Pausing...)"
        response.success = True
            
    else:            # False = RESUME 요청
        node_.get_logger().info("✅ Command: RESUME -> Calling MoveResume...")
        
        req = MoveResume.Request()
        cli_move_resume.call_async(req) # 비동기 호출
        
        response.message = "Sent MoveResume Signal (Robot Resuming...)"
        response.success = True
            
    return response

def perform_task_loop():
    """
    '작업 스레드'의 메인 루프.
    amovel(비동기) + check_motion 루프를 사용하여 Pause 신호 처리를 보장함.
    """
    global task_running, trigger_event, node_, g_current_goal_handle, g_final_result
    
    logger = node_.get_logger()
    print("[Task Thread] 작업 스레드 시작됨. 트리거 대기 중...")
    
    while rclpy.ok():
        triggered = trigger_event.wait(timeout=1.0)
        
        if not rclpy.ok():
            break
        
        if triggered:
            if not task_running:
                task_running = True
                start_time = time.time()
                
                logger.info("[Task Thread] 트리거 수신! 작업 로직을 실행합니다.")
                
                try:
                    # [핵심] amovel, amovec, check_motion 임포트
                    from DSR_ROBOT2 import (
                        amovel, amovec, check_motion, # 비동기 명령
                        posx, set_user_cart_coord,
                        set_digital_output, get_current_posx,
                        movej, # movej는 안전상 필요시 사용 (보통 홈 이동 등)
                        DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, ON, OFF, 
                        DR_MV_RA_DUPLICATE
                    )

                    # [Helper] 피드백 함수
                    def pub_feedback(msg, pct):
                        feedback_msg.feedback_string = msg
                        feedback_msg.progress_percentage = float(pct)
                        g_current_goal_handle.publish_feedback(feedback_msg)

                    # --- 메인 시퀀스 실행 ---
                    feedback_msg = BrushingAction.Feedback()

                    # --- [Helper] 비동기 동작 대기 함수 ---
                    def wait_for_motion():
                        """
                        로봇이 동작을 완료할 때까지 대기하는 함수.
                        sleep을 사용하여 ROS 스레드가 Pause 신호를 처리할 틈을 줌.
                        """
                        # 1. 명령 전달 후 로봇 상태가 Busy(2)가 될 때까지 약간 대기
                        time.sleep(0.2) 
                        
                        # 2. 동작 중(check_motion != 0)인 동안 루프
                        # 0: Idle(완료), 1: Init, 2: Busy
                        while check_motion() != 0:
                            # 이 sleep 동안 Pause 서비스 콜백이 실행될 수 있음
                            time.sleep(0.1) 
                            if not rclpy.ok(): return False
                        return True
                    
                    # ★ [신규] 스마트 대기 함수 (점진적 증가) ★
                    # target_time: 이 동작이 대략 몇 초 걸리는지 (amovel의 time=2.0 등과 맞춤)
                    # start_pct -> end_pct: 이 동작 동안 40%에서 60%로 올리겠다
                    def wait_with_feedback(msg, target_time, start_pct, end_pct):
                        time.sleep(0.1) # 동작 시작 대기
                        start_t = time.time()
                        
                        # 로봇이 움직이는 동안(check_motion != 0) 루프
                        while check_motion() != 0:
                            elapsed = time.time() - start_t
                            
                            # 진행률 계산 (시간 비율)
                            ratio = elapsed / target_time
                            if ratio > 1.0: ratio = 1.0 # 100% 초과 방지
                            
                            current_pct = start_pct + (end_pct - start_pct) * ratio
                            
                            # 피드백 전송
                            pub_feedback(msg, current_pct)
                            
                            time.sleep(0.1) # 0.1초 간격 업데이트
                            if not rclpy.ok(): return False
                            
                        # 동작이 예상보다 빨리 끝나도, 마지막엔 목표치(end_pct)를 한번 딱 보내줌
                        pub_feedback(msg, end_pct)
                        return True

                    def gripper(switch):
                        if switch == 0: set_digital_output(1, ON); set_digital_output(2, OFF)
                        elif switch == 1: set_digital_output(1, OFF); set_digital_output(2, ON)

                    # --- [수정] brush_pick (amovel 적용) ---
                    def brush_pick():
                        # pub_feedback("브러싱 도구 집기 시작", 9.0)
                        logger.info(f'[Task Thread] brushing pick 시작')
                        
                        amovel(posx(325.82, 267.29, 424.88, 82.80, 175.80, 80.38), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        # if not wait_for_motion(): return False
                        if not wait_with_feedback("도구 집으러 이동 중...", Time, 0.0, 3.0): return False
                        
                        amovel(posx(365.81, 241.49, 424.88, 42.53, 179.97, 42.54), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        # if not wait_for_motion(): return False
                        if not wait_with_feedback("접근 중...", Time, 3.0, 5.0): return False
                        
                        amovel(posx(0.00, 0.00, -98.31, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        # if not wait_for_motion(): return False
                        if not wait_with_feedback("그리퍼 하강 중...", Time, 5.0, 8.0): return False
                        if not wait_for_motion(): return False

                        gripper(0); time.sleep(1.00) 
                        
                        amovel(posx(348.81, 241.49, 418.61, 42.19, 179.97, 42.20), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        if not wait_for_motion(): return False
                        
                        amovel(posx(-35.70, -208.74, 0.00, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        if not wait_for_motion(): return False
                        
                        amovel(posx(313.12, 32.75, 388.61, 42.27, 179.97, 42.28), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        if not wait_for_motion(): return False
                        
                        logger.info(f'[Task Thread] brushing pick 완료')
                        return True

                    # --- [수정] brush_place (amovel 적용) 90%~100%---
                    def brush_place():
                        logger.info(f'[Task Thread] brushing place 시작')
                        # (1) 이동 (90% -> 95%)
                        amovel(posx(598.34, -93.08, 463.79, 42.80, 179.97, 42.81), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        # if not wait_for_motion(): return False
                        if not wait_with_feedback("도구 내려놓기 위치 이동...", Time, 90.0, 93.0): return False
                        
                        amovel(posx(352.89, 241.49, 463.79, 42.12, 179.97, 42.13), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        # if not wait_for_motion(): return False
                        if not wait_with_feedback("위치 조정 중...", Time, 93.0, 95.0): return False
                        
                        pub_feedback("내려놓기 하강", 95.0)
                        amovel(posx(0.00, 0.00, -130.26, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        # if not wait_for_motion(): return False
                        if not wait_with_feedback("내려놓기 하강 중...", Time, 95.0, 99.0): return False

                        
                        gripper(1); time.sleep(1.00) 
                        pub_feedback("내려놓기 완료", 100.0)
                        logger.info(f'[Task Thread] brushing place 완료')
                        return True

                    # [수정] brushing_move_1 (직선: 50%~90%)
                    def brushing_move_1(DR_USER1, board_w, num_strokes, y_step_size, move_x):
                        logger.info("[Task Thread] brushing_move_1 시작")
                        vel_x = 60; acc_x = 60; vel_y = 50; acc_y = 50
                        
                        amovel([323.64,20.97,388.61,5.05,-179.58,4.44], vel = 50, acc = 50, time = Time, mod = DR_MV_MOD_ABS)
                        if not wait_for_motion(): return False
                        
                        logger.info('[Task Thread] 브러싱 시작 위치 배치 완료')

                        start_pct = 50.0; end_pct = 90.0
                        total_strokes = int(num_strokes)
                        pct_per_stroke = (end_pct - start_pct) / total_strokes # 1회당 증가량

                        for y_move_cnt in range(int(num_strokes)):
                            # 퍼센트 계산 및 피드백
                            # current_pct = start_pct + ((end_pct - start_pct) * (y_move_cnt / total_strokes))
                            feedback_msg.current_stroke = y_move_cnt + 1
                            # pub_feedback(f"직선 붓질 {y_move_cnt + 1}/{total_strokes} 진행 중", current_pct)
                            
                            # 이번 스트로크의 시작%와 끝%
                            s_pct = start_pct + (y_move_cnt * pct_per_stroke)
                            e_pct = s_pct + pct_per_stroke

                            # (1) 메인 붓질 동작 (s_pct -> e_pct의 90%만큼 진행)
                            mid_pct = s_pct + (pct_per_stroke * 0.9)

                            amovel([move_x,0, 0,0,0,0], vel = vel_x, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL)
                            # if not wait_for_motion(): return False
                            # 붓질하는 동안 게이지 상승
                            if not wait_with_feedback(f"직선 붓질 {y_move_cnt+1}/{total_strokes}", 7.0, s_pct, mid_pct): return False
                            
                            amovel([0,0,15,0,0,0], vel = vel_x, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL)
                            if not wait_for_motion(): return False
                            
                            if not y_move_cnt == total_strokes -1:
                                amovel([-move_x,-y_step_size, 0,0,0,0], vel = vel_y, acc = acc_y, ref = DR_USER1, mod = DR_MV_MOD_REL)
                                # if not wait_for_motion(): return False
                                if not wait_with_feedback(f"직선 붓질 {y_move_cnt+1}/{total_strokes} (복귀)", 1.5, mid_pct, e_pct): return False
                                
                                amovel([0,0,-15,0,0,0], vel = vel_x, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL)
                                if not wait_for_motion(): return False
                            else:
                                # 마지막 줄이면 바로 끝 퍼센트로
                                pub_feedback(f"직선 붓질 {y_move_cnt+1}/{total_strokes} 완료", e_pct)

                            logger.info(f'[Task Thread] {y_move_cnt + 1}번째 붓질 완료')
                        
                        logger.info('--[Task Thread] brushing1 작업 완료--')   
                        amovel([0,0,50,0,0,0], vel = 60, acc = 60, time = Time, ref = DR_BASE, mod = DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                        
                        logger.info('[Task Thread] 상승 완료')
                        return True

                    # --- [수정] brushing_move_2 (amovel, amovec 적용) ---
                    # [수정] brushing_move_2 (원형: 10%~50%)
                    def brushing_move_2(DR_USER1, board_w, num_strokes, y_step_size):

                        logger.info("[Task Thread] brushing_move_2 시작")
                        vel_x = 60; acc_x = 60; beta = 5
                        P_1_init = posx([273.64,20.97,418.61-beta,5.05,-179.58,4.44])
                        P_2_init = posx([323.64,20.97,388.61-beta,5.05,-179.58,4.44])
                        P_3_init = posx([373.64,20.97,418.61-beta,5.05,-179.58,4.44])
                        list_P_init = [P_1_init, P_2_init, P_3_init]

                        start_pct = 10.0
                        end_pct = 50.0
                        total_strokes = int(num_strokes)
                        pct_per_stroke = (end_pct - start_pct) / total_strokes

                        # 동그라미 간격 정의 (Inner Loop 계산용)
                        delta = 25
                        circles_per_row = math.ceil(board_w / delta) # 1줄에 그릴 동그라미 개수
                        # 동그라미 1개당 할당량 (예: 10% / 6개 = 1.6%)
                        pct_per_circle = pct_per_stroke / circles_per_row 

                        for y_move_cnt in range(int(num_strokes)):
                            # 퍼센트 계산 및 피드백
                            # current_pct = start_pct + ((end_pct - start_pct) * (y_move_cnt / total_strokes))
                            feedback_msg.current_stroke = y_move_cnt + 1
                            # pub_feedback(f"원형 붓질 {y_move_cnt + 1}/{total_strokes} 진행 중", current_pct)

                            # [계산] 이번 스트로크 구간
                            s_pct = start_pct + (y_move_cnt * pct_per_stroke)
                            e_pct = s_pct + pct_per_stroke

                            P_1 = P_1_init.copy(); P_2 = P_2_init.copy(); P_3 = P_3_init.copy()
                            list_P = [P_1,P_2,P_3]; delta = 25; x_move_cnt = 0

                            # 스트로크 시작 시점 업데이트
                            pub_feedback(f"원형 붓질 {y_move_cnt+1}/{total_strokes} 진행 중", s_pct)

                            while delta * x_move_cnt < board_w:
                                # 이번 동그라미의 시작/중간/끝 퍼센트 계산
                                circle_s_pct = s_pct + (x_move_cnt * pct_per_circle)
                                circle_e_pct = circle_s_pct + pct_per_circle
                                circle_mid_pct = circle_s_pct + (pct_per_circle * 0.5)

                                msg = f"원형 붓질 {y_move_cnt+1}/{total_strokes} (원 {x_move_cnt+1})"

                                amovel(P_1, time = 0.5,vel = vel_x, acc = acc_x, mod = DR_MV_MOD_ABS)
                                # if not wait_for_motion(): return False
                                if not wait_with_feedback(msg, 0.5, circle_s_pct, circle_mid_pct): return False
                                
                                # amovec 사용 (없으면 에러날 수 있으므로 try-except로 처리 가능하지만 보통 있음)
                                amovec(P_2, P_3, time = 0.5,vel = vel_x, acc = acc_x, mod = DR_MV_MOD_ABS)
                                # if not wait_for_motion(): return False
                                if not wait_with_feedback(msg, 0.5, circle_mid_pct, circle_e_pct): return False
                                
                                x_move_cnt += 1
                                for P in list_P: P[0] += delta

                            logger.info(f'[Task Thread] {y_move_cnt + 1}번째 붓질 완료')
                            for P in list_P_init: P[1] -= abs(y_step_size)

                            if y_move_cnt < total_strokes-1:
                                amovel(list_P_init[0], vel = vel_x, acc = acc_x, mod = DR_MV_MOD_ABS)
                                if not wait_for_motion(): return False
                            else:
                                pub_feedback(f"원형 붓질 완료", e_pct)

                        logger.info('--[Task Thread] brushing2 작업 완료--')
                        amovel([0,0,30,0,0,0], vel = 60, acc = 60, ref = DR_BASE, mod = DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                        
                        logger.info('[Task Thread] 상승 완료')
                        
                        # 홈 이동 (movej)도 비동기 + 대기 방식으로 처리 권장
                        # 여기서는 movej(sync)를 유지하려면, Pause시 반응이 늦을 수 있음.
                        # 가능하면 amovej + wait_for_motion으로 바꾸는 것이 좋음.
                        home_pose_j = [0, 0, 90, 0, 90, 0]
                        from DSR_ROBOT2 import amovej
                        amovej(home_pose_j, vel=30, acc=30, time=Time)
                        if not wait_for_motion(): return False
                        
                        return True

                    # --- 메인 로직 실행 ---
                    
                    # 좌표계 설정
                    p1_pos = [321.79, 77.160, 310.540]; p2_pos = [618.860, 65.930, 309.570]; p4_pos = [317.6, -120.8, 307.8]
                    list_x = [p2_pos[i] - p1_pos[i] for i in range(3)]; A = math.sqrt(sum(i**2 for i in list_x)); list_X = [i / A for i in list_x]
                    list_y = [p1_pos[i] - p4_pos[i] for i in range(3)]; B = math.sqrt(sum(i**2 for i in list_y)); list_Y = [i / B for i in list_y]
                    pos = [317.6, -120.8, 307.8, 157.52, 180, 165.5]
                    DR_USER1 = set_user_cart_coord(list_X, list_Y, pos, ref=DR_BASE)
                    logger.info(f"[Task Thread] 사용자 좌표계 'DR_USER1' (ID: {DR_USER1}) 생성 완료.")

                    # 브러싱 계산
                    board_h = 200; board_w = 300; brush_h = 110
                    num_strokes = math.ceil(board_h / brush_h); total_overlap = (brush_h * num_strokes) - board_h
                    overlap_count = num_strokes - 1; y_step_size = brush_h - (total_overlap / overlap_count) if overlap_count > 0 else board_h
                    alpha = 10; move_x = board_w + alpha
                    logger.info(f"[Task Thread] 계산된 붓질 횟수: {num_strokes}회, Y 스텝: {y_step_size:.1f}mm")

                    # --- 실행 시퀀스 ---
                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 1. brush_pick 시작")
                    feedback_msg.feedback_string = "현재 \"붓 집기\" 작업중"
                    feedback_msg.current_stroke = 0
                    g_current_goal_handle.publish_feedback(feedback_msg)
                    
                    if not brush_pick(): raise Exception("brush_pick 실패")

                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 2. brushing_move_2 시작")
                    
                    if not brushing_move_2(DR_USER1, board_w, num_strokes, y_step_size): raise Exception("brushing_move_2 실패")
                    
                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 3. brushing_move_1 시작")
                    
                    if not brushing_move_1(DR_USER1, board_w, num_strokes, y_step_size, move_x): raise Exception("brushing_move_1 실패")
                    
                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 4. brush_place 시작")
                    feedback_msg.feedback_string = "현재 \"붓 내려놓기\" 작업중"
                    feedback_msg.current_stroke = 0
                    g_current_goal_handle.publish_feedback(feedback_msg)
                    
                    if not brush_place(): raise Exception("brush_place 실패")
                    
                    logger.info("[Task Thread] 전체 브러싱 시퀀스 1회 완료.")
                    
                    end_time = time.time()
                    result = BrushingAction.Result()
                    result.complete_task = True
                    result.total_duration = end_time - start_time
                    result.final_pose , _ = get_current_posx() 
                    g_final_result = result
                    g_current_goal_handle.succeed()

                except Exception as e:
                    if rclpy.ok():
                        logger.error(f"[Task Thread] 작업 시퀀스 중 예외 발생: {e}")
                    if g_current_goal_handle and g_current_goal_handle.is_active:
                        result = BrushingAction.Result()
                        result.complete_task = False
                        g_final_result = result
                        g_current_goal_handle.abort()
                finally:
                    task_running = False
                    trigger_event.clear()
                    g_current_goal_handle = None 
                    logger.info("[Task Thread] 작업 스레드 종료됨 (다음 요청 대기).")
            else:
                node_.get_logger().warn("[Task Thread] 작업이 이미 실행 중. 중복 트리거 무시.")
                trigger_event.clear()
    
    print("[Task Thread] 작업 스레드 루프 종료됨.")


def goal_callback(goal_request):
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
    global task_running, trigger_event, g_current_goal_handle, node_, g_final_result
    logger = node_.get_logger()
    logger.info("[ROS Thread] Goal 실행 시작! '작업 스레드'를 트리거합니다.")

    g_current_goal_handle = goal_handle
    trigger_event.set()
    
    # 작업이 끝날 때까지 대기
    while g_current_goal_handle is not None and rclpy.ok():
        time.sleep(0.1)

    logger.info("[ROS Thread] Goal 실행 완료. (작업 스레드 종료 확인)")
    return g_final_result

def ros_spin_thread():
    global node_
    print("[ROS Thread] ROS Spin 스레드 시작...")
    try:
        executor = MultiThreadedExecutor(num_threads=4) 
        executor.add_node(node_)
        
        action_server = ActionServer(
            node_,
            BrushingAction,
            'do_brushing_action', 
            execute_callback=execute_callback, 
            goal_callback=goal_callback, 
            cancel_callback=None
        )
        print("[ROS Thread] 'do_brushing_action' Action 서버 시작됨.")
        
        executor.spin()
    except Exception as e:
        if rclpy.ok():
            node_.get_logger().error(f"[ROS Thread] Spin 중 예외 발생: {e}")
    finally:
        print("[ROS Thread] ROS Spin 스레드 종료됨.")


def main(args=None):
    global node_, cli_move_pause, cli_move_resume
    
    rclpy.init(args=args)
    
    delay = random.uniform(0.1, 1.0) 
    print(f"[Oiling Server] 드라이버 충돌 방지를 위해 {delay:.2f}초 대기...")
    time.sleep(delay)

    node_ = rclpy.create_node("brushing_action_server", namespace=ROBOT_ID)
    
    # [중요] DSR 라이브러리에 노드 연결
    DR_init.__dsr__node = node_
    
    # [중요] 로봇 서비스 클라이언트 생성 (Pause/Resume)
    cli_move_pause = node_.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
    cli_move_resume = node_.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')

    # [중요] 제어용 서비스 서버 생성
    node_.create_service(SetBool, 'custom_pause', callback_pause)
    
    print(f"노드 '{ROBOT_ID}/brushing_action_server' 생성 완료.")
    
    spin_thread = None
    robot_thread = None

    try:
        if not initialize_robot():
            raise Exception("로봇 초기화 실패. 프로그램을 종료합니다.")

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