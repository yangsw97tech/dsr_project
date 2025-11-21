# erasing_action_server.py

import rclpy
from rclpy.node import Node
import DR_init
import threading
import time
import math
from rclpy.executors import MultiThreadedExecutor

# Action 관련 모듈
from rclpy.action import ActionServer, GoalResponse, CancelResponse
# 사용자 Action 인터페이스
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

# --- DR_init 설정 ---
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# --- 전역 변수 ---
node_ = None
trigger_event = threading.Event()
task_running = False
set_tool_func = None
set_tcp_func = None

# Action 관련
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
                logger.info("[Task Thread] 트리거 수신! 작업 시작.")
                
                try:
                    # [핵심] amovel, check_motion 추가
                    from DSR_ROBOT2 import (
                        amovel, check_motion, # 비동기 명령
                        release_compliance_ctrl, release_force,
                        check_force_condition, task_compliance_ctrl, set_desired_force,
                        set_ref_coord, get_digital_input, get_digital_output, set_digital_output,
                        movej, movel, get_current_posx, wait, set_user_cart_coord, get_tool_force,
                        DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE,
                        DR_FC_MOD_REL, DR_AXIS_Z, DR_BASE, DR_TOOL, OFF, ON,
                        posx, posj
                    )

                    # --- [Helper] 비동기 대기 함수 ---
                    def wait_for_motion():
                        time.sleep(0.2) # 상태 반영 대기
                        while check_motion() != 0: # 0: Idle, 1: Init, 2: Busy
                            time.sleep(0.1)
                            if not rclpy.ok(): return False
                        return True

                    # gripper 조절
                    def gripper(switch):
                        if switch == 0:
                            set_digital_output(1,ON); set_digital_output(2,OFF)
                        elif switch == 1:
                            set_digital_output(1,OFF); set_digital_output(2,ON)
                        wait(1.0)

                    # --- 메인 시퀀스 실행 ---
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
                            current_pct = start_pct + (end_pct - start_pct) * ratio
                            pub_feedback(msg, current_pct)
                            time.sleep(0.1)
                            if not rclpy.ok(): return False
                        pub_feedback(msg, end_pct)
                        return True

                    # 힘제어 함수 (동기 방식 유지 - 짧은 시간이라 영향 적음)
                    def force_control(switch, press_force = 10):
                        if not rclpy.ok(): return False
                        if switch == 1:
                            logger.info("힘 제어 시작")
                            set_ref_coord(1) 
                            task_compliance_ctrl(stx=[3000, 3000, 200, 200, 200, 200])
                            wait(0.5) 
                            set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

                            while rclpy.ok():
                                ret = check_force_condition(DR_AXIS_Z, max=press_force)
                                if ret == -1: # 조건 만족 시 리턴값 확인 필요 (매뉴얼상 0 or 1일수도 있음)
                                    logger.info(f"Z축 힘 감지됨.")
                                    break
                                time.sleep(0.1) # 루프 과부하 방지
                            wait(0.5)

                        elif switch == 0:
                            logger.info("힘 제어 중지")
                            release_force()
                            release_compliance_ctrl()
                        return True

                    # oiling_move (amovel 적용)
                    def oiling_move(DR_USER1, num_strokes, y_step_size, move_x, feedback_msg):
                        vel_x = 50; acc_x = 50
                        vel_y = 50; acc_y = 50
                        vel_z = 30; acc_z = 30

                        if not force_control(1,14): raise Exception("힘제어 시작 실패")

                        # 퍼센트 구간 설정
                        start_pct = 10.0
                        end_pct = 90.0
                        total = int(num_strokes)
                        pct_per_stroke = (end_pct - start_pct) / total

                        for y_move_cnt in range(int(num_strokes)):

                            # 진행률 계산
                            # current_pct = start_pct + ((end_pct - start_pct) * (y_move_cnt / total_strokes))
                            feedback_msg.current_stroke = y_move_cnt + 1

                            # 이번 줄의 구간 계산
                            s_pct = start_pct + (y_move_cnt * pct_per_stroke)
                            e_pct = s_pct + pct_per_stroke
                            
                            # 한 줄은 3번의 왕복 운동 (왔다-갔다-왔다) -> 3등분
                            step1 = s_pct + (pct_per_stroke * 0.33)
                            step2 = s_pct + (pct_per_stroke * 0.66)

                            # pub_feedback(f"지우개 작업 {y_move_cnt + 1}/{total_strokes} 줄 진행 중", current_pct)
                            msg = f"지우개 작업 {y_move_cnt + 1}/{total} 줄"

                            # x축 방향 이동 (왕복)
                            # movel -> amovel + wait_for_motion 변환
                            if y_move_cnt %2 == 0:
                                if not force_control(0): raise Exception("힘제어 종료 실패")
                                
                                amovel([move_x,0, 0,0,0,0], vel=vel_x, time=2.0, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                                # if not wait_for_motion(): return False
                                if not wait_with_feedback(msg, 2.0, s_pct, step1): return False
                                
                                amovel([-move_x,0, 0,0,0,0], vel=vel_x, time=2.0, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                                # if not wait_for_motion(): return False
                                if not wait_with_feedback(msg, 2.0, step1, step2): return False
                                
                                amovel([move_x,0, 0,0,0,0], vel=vel_x, time=2.0, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                                if not wait_with_feedback(msg, 2.0, step2, e_pct): return False
                            else:
                                if not force_control(0): raise Exception("힘제어 종료 실패")
                                
                                amovel([-move_x,0, 0,0,0,0], vel=vel_x, time=2.0, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                                if not wait_with_feedback(msg, 2.0, s_pct, step1): return False
                                
                                amovel([move_x,0, 0,0,0,0], vel=vel_x, time=2.0, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                                if not wait_with_feedback(msg, 2.0, step1, step2): return False
                                
                                amovel([-move_x,0, 0,0,0,0], vel=vel_x, time=2.0, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                                if not wait_with_feedback(msg, 2.0, step2, e_pct): return False

                            logger.info(f'[Task Thread] {y_move_cnt + 1}번째 붓질 완료')

                            if y_move_cnt < num_strokes - 1:
                                amovel([0, 0, +10,0,0,0], vel=vel_y, time=1.0, acc=acc_y, ref=DR_USER1, mod=DR_MV_MOD_REL)
                                if not wait_for_motion(): return False
                                
                                amovel([0,-y_step_size, 0,0,0,0], vel=vel_y, time=1.0, acc=acc_y, ref=DR_USER1, mod=DR_MV_MOD_REL)
                                if not wait_for_motion(): return False
                                
                                if not force_control(1,14): raise Exception("힘제어 시작 실패")
                                logger.info(f'[Task Thread] 다음 라인으로 이동')
                            else:
                                logger.info('[Task Thread] 최종 Y 위치에 도달')

                        logger.info('[Task Thread] erasing 작업 완료')   
                        if not force_control(0): raise Exception("힘제어 종료 실패")
                        
                        amovel([0,0,100,0,0,0], vel=vel_z, acc=acc_z, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        if not wait_for_motion(): return False
                        
                        pub_feedback("지우개 작업 완료, 복귀중", 90.0)
                        logger.info('[Task Thread] 상승 완료')
                        return True
                    
                    # --- 메인 시퀀스 실행 ---
                    feedback_msg = BrushingAction.Feedback()
                    feedback_msg.current_stroke = 0

                    # --- 메인 시퀀스 실행 ---
                    # 1. Pick (0% ~ 10%)
                    logger.info("[Task Thread] 1. 지우개 집기 시작")
                    feedback_msg.current_stroke = 0
                    # pub_feedback("지우개 집기 시작", 0.0) # [시작점]
                    
                    amovel(posx(330.44, 289.61, 424.88, 0, -180, 0), vel=VELOCITY, time=2.0, acc=ACC, ref=0, mod=DR_MV_MOD_ABS)
                    # if not wait_for_motion(): raise Exception("작업 중단")
                    if not wait_with_feedback("지우개 집으러 이동...", 2.0, 0.0, 4.0): raise Exception("작업 중단")
                    
                    gripper(1)
                    pub_feedback("지우개 위치로 하강", 5.0) # [중간]
                    
                    amovel(posx(330.44, 289.61, 310.78, 0, -180, 0), vel=VELOCITY, time=2.0, acc=ACC, ref=0, mod=DR_MV_MOD_ABS)
                    # if not wait_for_motion(): raise Exception("작업 중단")
                    if not wait_with_feedback("지우개 위치 하강...", 2.0, 4.0, 7.0): raise Exception("작업 중단")
                    
                    gripper(0)
                    
                    amovel(posx(330.44, 289.61, 424.88, 0, -180, 0), vel=VELOCITY, time=2.0, acc=ACC, ref=0, mod=DR_MV_MOD_ABS)
                    # if not wait_for_motion(): raise Exception("작업 중단")
                    if not wait_with_feedback("지우개 들어올림...", 2.0, 7.0, 10.0): raise Exception("작업 중단")

                    pub_feedback("지우개 파지 완료", 10.0) # [Pick 완료]
                    
                    # 2. 좌표계 설정
                    logger.info("[Task Thread] 2. 좌표계 설정")
                    p1 = [331.013, 67.332, 330.540]
                    p2 = [614.632, 59.905, 330.540]
                    p4 = [326.180, -117.220, 330.540]
                    p_origin = [326.180, -117.220, 330.540, 157.52, 180, 166.7]
                    
                    list_x = [p2[i]-p1[i] for i in range(3)]; A = math.sqrt(sum(i**2 for i in list_x)); list_X = [i/A for i in list_x]
                    list_y = [p1[i]-p4[i] for i in range(3)]; B = math.sqrt(sum(i**2 for i in list_y)); list_Y = [i/B for i in list_y]
                    
                    DR_USER1 = set_user_cart_coord(list_X, list_Y, p_origin, ref=DR_BASE)
                    logger.info(f"[Task Thread] 사용자 좌표계 'DR_USER1' (ID: {DR_USER1}) 생성.")

                    board_h = 214; board_w = 300; sponge_h = 23
                    num_strokes = math.ceil(board_h / sponge_h)
                    total_overlap = (sponge_h * num_strokes) - board_h
                    overlap_count = num_strokes - 1
                    y_step_size = sponge_h - (total_overlap / overlap_count) if overlap_count > 0 else board_h
                    move_x = board_w - 8
                    vel_z = 30; acc_z = 30
                    
                    # 3. 닦기 동작
                    logger.info("[Task Thread] 3. 닦기 동작 준비")
                    init_pos = posx([331.79, 65.310, 350.540, 13.71, 180.0, 12.62])
                    
                    amovel(init_pos, vel=30, time=2.0, acc=30, ref=DR_BASE, mod=DR_MV_MOD_ABS)
                    if not wait_for_motion(): raise Exception("작업 중단")

                    amovel([0,0,-7,0,0,0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
                    if not wait_for_motion(): raise Exception("작업 중단")
                    
                    gripper(1)
                    
                    amovel([0,0,-30,0,0,0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
                    if not wait_for_motion(): raise Exception("작업 중단")
                    
                    gripper(0)
                    
                    amovel([0,0,+10,0,0,0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
                    if not wait_for_motion(): raise Exception("작업 중단")

                    logger.info('[Task Thread] 지우개 재정렬 완료')
                    set_ref_coord(DR_USER1)

                    # 닦기 함수 호출
                    if not oiling_move(DR_USER1, num_strokes, y_step_size, move_x, feedback_msg):
                        raise Exception("oiling_move 함수 실행 실패")
                    
                    # 4. place_e (90% ~ 100%)
                    logger.info("[Task Thread] 4. 지우개 내려놓기")
                    # pub_feedback("지우개 제자리로 이동 중...", 90.0)

                    amovel(posx(330.44, 289.61, 400.54, 64.35, -180.00, 63.26), vel=VELOCITY, time=2.0, acc=ACC, ref=0, mod=DR_MV_MOD_ABS)
                    # if not wait_for_motion(): raise Exception("작업 중단")
                    if not wait_with_feedback("제자리로 이동 중...", 2.0, 90.0, 95.0): raise Exception("작업 중단")
                    
                    amovel(posx(0.00, 0.00, -85.26, 0.00, 0.00, 0.00), vel=VELOCITY, time=2.0, acc=ACC, ref=0, mod=DR_MV_MOD_REL)
                    # if not wait_for_motion(): raise Exception("작업 중단")
                    if not wait_with_feedback("내려놓기 하강 중...", 2.0, 95.0, 99.0): raise Exception("작업 중단")
                    
                    # pub_feedback("내려놓기 위치 도달", 95.0) # [중간]

                    gripper(1)
                    
                    pub_feedback("연마(Erasing) 공정 완료.", 100.0)
                    logger.info("[Task Thread] 전체 지우개 작업 시퀀스 1회 완료.")
                    
                    end_time = time.time()
                    result = BrushingAction.Result()
                    result.complete_task = True
                    result.total_duration = end_time - start_time
                    result.final_pose , _ = get_current_posx(ref=DR_BASE) 
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
    if task_running: return GoalResponse.REJECT
    if not goal_request.start_task: return GoalResponse.REJECT
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
            node_, BrushingAction, 'do_eraser_action', 
            execute_callback=execute_callback, goal_callback=goal_callback, cancel_callback=None
        )
        print("[ROS Thread] 'do_eraser_action' Action 서버 시작됨.")
        executor.spin()
    except Exception as e:
        if rclpy.ok(): node_.get_logger().error(f"[ROS Thread] Spin 중 예외 발생: {e}")
    finally:
        print("[ROS Thread] ROS Spin 스레드 종료됨.")

def main(args=None):
    global node_, cli_move_pause, cli_move_resume
    rclpy.init(args=args)

    import random
    time.sleep(random.uniform(0.1, 1.0))

    node_ = rclpy.create_node("eraser_action_server", namespace=ROBOT_ID)
    DR_init.__dsr__node = node_
    
    cli_move_pause = node_.create_client(MovePause, f'/{ROBOT_ID}/motion/move_pause')
    cli_move_resume = node_.create_client(MoveResume, f'/{ROBOT_ID}/motion/move_resume')
    node_.create_service(SetBool, 'custom_pause', callback_pause)
    
    print(f"노드 '{ROBOT_ID}/eraser_action_server' 생성 완료.")
    
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