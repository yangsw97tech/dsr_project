# oiling_action_server.py

import rclpy
from rclpy.node import Node
import DR_init
import threading
import time
import math
from rclpy.executors import MultiThreadedExecutor

# Action 관련 모듈
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from my_robot_interfaces.action import BrushingAction

# [추가] Pause/Resume 서비스
from std_srvs.srv import SetBool
from dsr_msgs2.srv import MovePause, MoveResume

# --- 로봇 설정 상수 ---
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"
ROBOT_TOOL = "GripperDA_v1"
VELOCITY = 60
ACC = 60
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

# [추가] Pause/Resume 콜백
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

    if request.data: # True = PAUSE
        node_.get_logger().warn("⛔ Command: PAUSE -> Calling MovePause...")
        req = MovePause.Request()
        cli_move_pause.call_async(req)
        response.message = "Sent MovePause Signal"
        response.success = True
    else:            # False = RESUME
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
        
        if triggered:
            if not task_running:
                task_running = True
                start_time = time.time()
                
                logger.info("[Task Thread] 트리거 수신! 'oiling' 로직을 1회 실행합니다.")
                
                try:
                    # [핵심] amovel, check_motion 추가 임포트
                    from DSR_ROBOT2 import (
                        amovel, check_motion, # 비동기 명령
                        movel, wait, movej, posx, set_digital_output, get_current_posx,
                        DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, ON, OFF, 
                        DR_MV_RA_DUPLICATE
                    )
                    
                    # --- [Helper] 비동기 대기 함수 ---
                    def wait_for_motion():
                        """로봇 동작 완료 대기 (Pause 틈 확보)"""
                        time.sleep(0.2) # 초기 대기
                        while check_motion() != 0: # 0: Idle, 1: Init, 2: Busy
                            time.sleep(0.1)
                            if not rclpy.ok(): return False
                        return True

                    def gripper(switch):
                        if switch == 0: set_digital_output(1, ON); set_digital_output(2, OFF)
                        elif switch == 1: set_digital_output(1, OFF); set_digital_output(2, ON)
                        wait(1.0) # 그리퍼 동작 시간 대기

                    # --- [수정] 피드백 함수 ---
                    feedback_msg = BrushingAction.Feedback()
                    def pub_feedback(msg, pct):
                        feedback_msg.feedback_string = msg
                        feedback_msg.progress_percentage = float(pct)
                        g_current_goal_handle.publish_feedback(feedback_msg)

                    # ★ [신규] 스마트 대기 함수 (시간 기반 퍼센트 증가) ★
                    def wait_with_feedback(msg, target_time, start_pct, end_pct):
                        time.sleep(0.1) 
                        start_t = time.time()
                        while check_motion() != 0:
                            elapsed = time.time() - start_t
                            ratio = elapsed / target_time
                            if ratio > 1.0: ratio = 1.0
                            
                            # 현재 퍼센트 계산 (선형 보간)
                            current_pct = start_pct + (end_pct - start_pct) * ratio
                            pub_feedback(msg, current_pct)
                            
                            time.sleep(0.1)
                            if not rclpy.ok(): return False
                        
                        # 동작 완료 후 목표값 전송
                        pub_feedback(msg, end_pct)
                        return True

                    # --- Oiling 내부 함수 (amovel 적용) ---
                    
                    def oiling_pick(logger):
                        logger.info(f'[Task Thread] oiling_pick 시작')
                        
                        # (1) 이동 (0% -> 4%)
                        amovel(posx(352.89, 241.49, 431.28, 42.12, 179.97, 42.13), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        if not wait_with_feedback("오일링 도구 집으러 이동...", 2.0, 0.0, 4.0): return False
                        
                        # (2) 접근 (4% -> 6%)
                        amovel(posx(140.00, -7.98, 0.00, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        if not wait_with_feedback("도구 정렬 중...", 2.0, 4.0, 6.0): return False
                        
                        # (3) 하강 (6% -> 9%)
                        amovel(posx(0.00, 0.00, -85.78, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        if not wait_with_feedback("그리퍼 하강 중...", 2.0, 6.0, 9.0): return False
                        
                        print("그리퍼 닫기")
                        gripper(0)
                        pub_feedback("파지 끝, 들어올리는 중", 10.0)
                        
                        # (4) 상승 및 이동 준비
                        amovel(posx(0.00, 0.00, 119.42, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); wait_for_motion()
                        amovel(posx(-150.00, -199.10, 0.00, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); wait_for_motion()
                        
                        logger.info(f'[Task Thread] oiling_pick 완료')
                        return True

                    def oiling_move(logger, num_strokes, y_step_size, move_x, vel_x, acc_x, vel_y, acc_y, vel_z, acc_z):
                        logger.info(f'[Task Thread] oiling_move 시작')
                        
                        feedback_msg = BrushingAction.Feedback()

                        # --- 1번째 줄 (10% -> 36%) ---
                        # feedback_msg.feedback_string = f"현재 \"오일링\" 작업 \"{1}\"번째 줄 진행중"
                        # g_current_goal_handle.publish_feedback(feedback_msg)
                        feedback_msg.current_stroke = 1
                        pub_feedback("오일링 1번째 줄 작업 준비", 10.0)
                        
                        amovel([0,0,-65,0,0,0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                        
                        amovel([move_x+10,0,0,0,0,0], vel=vel_x, acc=acc_x, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        if not wait_with_feedback("오일링 1번째 줄 바르는 중...", 6.0, 10.0, 36.0): return False
                        
                        amovel([0,0,20,0,0,0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                        
                        # --- 2번째 줄 ---
                        # feedback_msg.feedback_string = f"현재 \"오일링\" 작업 \"{2}\"번째 줄 진행중"
                        feedback_msg.current_stroke = 2
                        # g_current_goal_handle.publish_feedback(feedback_msg)

                        amovel([0,-65,0,0,0,0], vel=vel_y, acc=acc_y, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        if not wait_with_feedback("오일링 2번째 줄 이동 중...", 1.5, 36.0, 40.0): return False
                        
                        amovel([0,0,-20,0,0,0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                        
                        amovel([-move_x-25,0,0,0,0,0], vel=vel_x, acc=acc_x, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        if not wait_with_feedback("오일링 2번째 줄 바르는 중...", 6.0, 40.0, 63.0): return False
                        
                        amovel([0,0,20,0,0,0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False

                        # --- 3번째 줄 ---
                        feedback_msg.feedback_string = f"현재 \"오일링\" 작업 \"{3}\"번째 줄 진행중"
                        feedback_msg.current_stroke = 3
                        g_current_goal_handle.publish_feedback(feedback_msg)

                        amovel([0,-65,0,0,0,0], vel=vel_y, acc=acc_y, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        if not wait_with_feedback("오일링 3번째 줄 이동 중...", 1.5, 63.0, 66.0): return False
                        
                        amovel([0,0,-20,0,0,0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                        
                        amovel([move_x+20,0,0,0,0,0], vel=vel_x, acc=acc_x, ref=DR_BASE, mod=DR_MV_MOD_REL)
                        # if not wait_for_motion(): return False
                        if not wait_with_feedback("오일링 3번째 줄 바르는 중...", 6.0, 66.0, 90.0): return False
                        
                        logger.info(f'[Task Thread] oiling_move 완료')
                        return True

                    def oiling_place(logger):
                        logger.info(f'[Task Thread] oiling_place 시작')
                        
                        amovel(posx(598.34, -93.08, 463.79, 42.80, 179.97, 42.81), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        # if not wait_for_motion(): return False
                        if not wait_with_feedback("도구 내려놓기 이동 중...", 2.0, 90.0, 94.0): return False
                        
                        amovel(posx(505.55, 233.19, 463.79, 43.27, 179.97, 43.28), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        # if not wait_for_motion(): return False
                        if not wait_with_feedback("위치 정렬 중...", 1.5, 94.0, 96.0): return False
                        
                        amovel(posx(0.00, 0.00, -116.69, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        # if not wait_for_motion(): return False
                        if not wait_with_feedback("하강 중...", 2.0, 96.0, 99.0): return False
                        
                        gripper(1)
                        if not wait_with_feedback("하강 중...", 2.0, 96.0, 99.0): return False

                        logger.info(f'[Task Thread] oiling_place 완료')
                        return True

                    # --- Oiling 로직의 메인 시퀀스 ---
                    
                    board_h = 200; board_w = 300; sponge_h = 95; sponge_w = 25
                    num_strokes = math.ceil(board_h / sponge_h)
                    total_overlap = (sponge_h * num_strokes) - board_h
                    overlap_count = num_strokes - 1
                    y_step_size = sponge_h - (total_overlap / overlap_count) if overlap_count > 0 else board_h
                    move_x = board_w - sponge_w
                    vel_x = 50; acc_x = 50; vel_y = 50; acc_y = 50; vel_z = 30; acc_z = 30
                    
                    # --- 실행 ---
                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 1. oiling_pick 시작")
                    feedback_msg.feedback_string = "현재 \"오일 집기\" 작업중"
                    feedback_msg.current_stroke = 0
                    g_current_goal_handle.publish_feedback(feedback_msg)
                    
                    if not oiling_pick(logger): raise Exception("oiling_pick 실패")

                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 2. oiling_move 시작")
                    
                    if not oiling_move(logger, num_strokes, y_step_size, move_x, vel_x, acc_x, vel_y, acc_y, vel_z, acc_z): raise Exception("oiling_move 실패")
                    
                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 3. oiling_place 시작")
                    feedback_msg.feedback_string = "현재 \"오일 내려놓기\" 작업중"
                    feedback_msg.current_stroke = 0
                    g_current_goal_handle.publish_feedback(feedback_msg)
                    
                    if not oiling_place(logger): raise Exception("oiling_place 실패")
                    
                    logger.info("[Task Thread] 전체 오일링 시퀀스 1회 완료.")
                    pub_feedback("오일링(Oiling) 공정 완료.", 100.0)
                    
                    end_time = time.time()
                    result = BrushingAction.Result()
                    result.complete_task = True
                    result.total_duration = end_time - start_time
                    result.final_pose, _ = get_current_posx()
                    
                    g_final_result = result 
                    g_current_goal_handle.succeed() 

                except Exception as e:
                    if rclpy.ok():
                        logger.error(f"[Task Thread] 작업 시퀀스 중 예외 발생: {e}")
                    
                    result = BrushingAction.Result()
                    result.complete_task = False
                    g_final_result = result
                    
                    if g_current_goal_handle and g_current_goal_handle.is_active:
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
    while g_current_goal_handle is not None and rclpy.ok():
        time.sleep(0.1)
    logger.info("[ROS Thread] Goal 실행 완료.")
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
            'do_oiling_action', 
            execute_callback=execute_callback,
            goal_callback=goal_callback,
            cancel_callback=None
        )
        print("[ROS Thread] 'do_oiling_action' Action 서버 시작됨.")
        
        executor.spin()
    except Exception as e:
        if rclpy.ok():
            node_.get_logger().error(f"[ROS Thread] Spin 중 예외 발생: {e}")
    finally:
        print("[ROS Thread] ROS Spin 스레드 종료됨.")


def main(args=None):
    global node_, cli_move_pause, cli_move_resume
    
    rclpy.init(args=args)
    
    import random
    delay = random.uniform(0.1, 1.0) 
    print(f"[Oiling Server] 드라이버 충돌 방지를 위해 {delay:.2f}초 대기...")
    time.sleep(delay)

    node_ = rclpy.create_node("oiling_action_server", namespace=ROBOT_ID)
    DR_init.__dsr__node = node_
    
    cli_move_pause = node_.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
    cli_move_resume = node_.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')
    node_.create_service(SetBool, 'custom_pause', callback_pause)
    
    print(f"노드 '{ROBOT_ID}/oiling_action_server' 생성 완료.")
    
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