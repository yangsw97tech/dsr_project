#Action_server.py (Eraser 동작 + 일시정지 V5 - 5초 후 자동 재개, 초기화 오류 수정)
# V1(global STATE) 기준으로, 'STOP' 수신 시 5초간 일시정지 후
# 'Task Thread'가 스스로 'RUNNING'으로 복귀 (RESUME 토픽 불필요)

import rclpy
from rclpy.node import Node
import DR_init
import threading
import time
import math
from rclpy.executors import MultiThreadedExecutor

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from my_robot_interfaces.action import BrushingAction
from std_msgs.msg import String

# --- 로봇 설정 상수 (실행 확인된 '실제 로봇' 기준) ---
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight" # 'test_action_server.py'의 성공 값
ROBOT_TCP = "GripperDA"  # 'test_action_server.py'의 성공 값
VELOCITY = 60 # 'eraser_stop.py'의 값
ACC = 60 # 'eraser_stop.py'의 값

# --- DR_init 설정 (유지) ---
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# --- 전역 변수 (유지) ---
node_ = None
trigger_event = threading.Event()
task_running = False
set_tool_func = None
set_tcp_func = None
g_current_goal_handle = None
g_final_result = None

# --- [V1 방식] 일시정지를 위한 전역 상태 변수 ---
STATE = "RUNNING"   # RUNNING / PAUSED

def initialize_robot():
    # (유지)
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

# --- ⬇️ [수정] 'ROS 스레드'가 실행할 콜백 (STOP만 처리) ⬇️ ---
def control_callback(msg: String):
    """STOP 명령만 처리하는 콜백 (RESUME은 자동 처리)"""
    global STATE, node_

    cmd = msg.data.upper().strip()
    logger = node_.get_logger()
    
    if cmd == "STOP":
        if STATE != "PAUSED":
            logger.warn(">>> STOP 수신 → STATE=PAUSED (일시정지)")
        STATE = "PAUSED"
    elif cmd == "RESUME":
        logger.info(">>> RESUME 수신 (현재 V5 모드에서는 무시됨. 자동 재개됩니다.)")
    else:
        logger.warn(f">>> [ARM CONTROL] 알 수 없는 명령: {cmd}")
# --- ⬆️ [수정] ⬆️ ---


# --- ⬇️ [수정] '작업 스레드'가 사용할 '5초 자동 재개' 함수 ⬇️ ---
def wait_until_running():
    """
    STATE가 'PAUSED'면 5초간 대기 후 'RUNNING'으로 스스로 복귀
    """
    global STATE, node_
    logger = node_.get_logger()
    
    if STATE == "RUNNING":
        return # 이미 RUNNING 상태면 즉시 리턴

    # --- PAUSED 상태(STATE == "PAUSED")인 경우, 대기 시작 ---
    logger.warn("[Task Thread] PAUSED. 5초 후 자동으로 작업을 재개합니다...")
    if g_current_goal_handle and g_current_goal_handle.is_active:
        feedback_msg = BrushingAction.Feedback()
        feedback_msg.feedback_string = "동작 일시중지됨. 5초 후 자동 재개..."
        feedback_msg.current_stroke = -1 # 일시정지 상태
        g_current_goal_handle.publish_feedback(feedback_msg)

    # 3. [핵심 수정] 5초간 대기 (time.sleep은 GIL을 해제함)
    time.sleep(5.0) 

    # --- 대기 종료 (자동 재개) ---
    logger.info("[Task Thread] 5초 경과. Auto-Resuming...")
    STATE = "RUNNING" # 스스로 RUNNING 상태로 복귀
    
    if g_current_goal_handle and g_current_goal_handle.is_active:
        feedback_msg = BrushingAction.Feedback()
        feedback_msg.feedback_string = "동작을 자동 재개합니다."
        feedback_msg.current_stroke = 0 
        g_current_goal_handle.publish_feedback(feedback_msg)
# --- ⬆️ [수정] ⬆️ ---


def perform_task_loop():
    """
    (유지) '작업 스레드'의 메인 루프.
    'wait_until_running'이 '자동 재개' 방식으로 수정됨.
    """
    global task_running, trigger_event, node_, g_current_goal_handle, g_final_result
    
    logger = node_.get_logger()
    print("[Task Thread] 작업 스레드 시작됨. 트리거 대기 중... (Eraser '일시정지 V5' 버전)")
    
    while rclpy.ok():
        triggered = trigger_event.wait(timeout=1.0)
        
        if not rclpy.ok():
            break
        
        if triggered:
            if not task_running:
                task_running = True
                start_time = time.time()
                
                logger.info("[Task Thread] 트리거 수신! 'eraser_pick_and_place' (일시정지) 로직을 1회 실행합니다.")
                
                try:
                    from DSR_ROBOT2 import (
                        release_compliance_ctrl, release_force,
                        check_force_condition,
                        task_compliance_ctrl,
                        set_desired_force,
                        set_ref_coord,
                        get_digital_input, get_digital_output, set_digital_output,
                        movej, movel,
                        get_current_posx, wait,
                        set_user_cart_coord,
                        get_tool_force,
                        DR_MV_MOD_ABS, DR_MV_MOD_REL,
                        DR_MV_RA_DUPLICATE,
                        DR_FC_MOD_REL,
                        DR_AXIS_Z,
                        DR_BASE, DR_TOOL,
                        OFF, ON,
                        posx, posj,
                    )

                    # gripper 조절
                    def gripper(switch):
                        if switch == 0: set_digital_output(1,ON); set_digital_output(2,OFF)
                        elif switch == 1: set_digital_output(1,OFF); set_digital_output(2,ON)
                    
                    # 힘제어 함수 (일시정지 로직 수정)
                    def force_control(switch, press_force = 10):
                        if not rclpy.ok(): return False 
                        
                        if switch == 1:
                            logger.info("힘 제어 시작")
                            wait_until_running(); set_ref_coord(1) 
                            wait_until_running(); task_compliance_ctrl(stx=[3000, 3000, 200, 200, 200, 200])
                            wait(0.5)
                            wait_until_running(); set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

                            while rclpy.ok():
                                # [유지] 힘 감지 루프가 돌 때마다 PAUSE 상태인지 체크
                                wait_until_running() 
                                
                                ret = check_force_condition(DR_AXIS_Z, max=press_force)
                                if ret == -1:
                                    logger.info(f"Z축 힘이 {press_force}N 이상 감지됨. {ret}")
                                    break
                                time.sleep(0.01) # 루프 지연 (DSR 명령 과부하 방지)
                            wait(0.5)

                        elif switch == 0:
                            logger.info("힘 제어 중지")
                            wait_until_running(); release_force()
                            wait_until_running(); release_compliance_ctrl()
                        
                        return rclpy.ok()

                    # oiling_move 함수 (wait_until_running 추가)
                    def oiling_move(DR_USER1, num_strokes, y_step_size, move_x, feedback_msg):
                        vel_x = 50; acc_x = 50
                        vel_y = 50; acc_y = 50
                        vel_z = 30; acc_z = 30

                        wait_until_running()
                        if not force_control(1,14): raise Exception("힘제어 시작 실패")

                        for y_move_cnt in range(int(num_strokes)):
                            
                            wait_until_running() # 피드백 전에도 체크
                            feedback_msg.feedback_string = f"현재 \"지우개 닦기\" 작업 \"{y_move_cnt + 1}\"번째 줄 진행중"
                            feedback_msg.current_stroke = y_move_cnt + 1
                            g_current_goal_handle.publish_feedback(feedback_msg)

                            if y_move_cnt %2 == 0:
                                wait_until_running(); 
                                if not force_control(0): raise Exception("힘제어 종료 실패")
                                wait_until_running(); movel([move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                                wait_until_running(); movel([-move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                                wait_until_running(); movel([move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                            else:
                                wait_until_running(); 
                                if not force_control(0): raise Exception("힘제어 종료 실패")
                                wait_until_running(); movel([-move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                                wait_until_running(); movel([move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                                wait_until_running(); movel([-move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")

                            logger.info(f'[Task Thread] {y_move_cnt + 1}번째 붓질 완료')

                            if y_move_cnt < num_strokes - 1:
                                wait_until_running(); movel([0, 0, +10,0,0,0], vel = vel_y, t=1, acc = acc_y, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                                wait_until_running(); movel([0,-y_step_size, 0,0,0,0], vel = vel_y, t=1, acc = acc_y, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                                wait_until_running(); 
                                if not force_control(1,14): raise Exception("힘제어 시작 실패")
                                logger.info(f'[Task Thread] {y_step_size:.1f}mm 다음 라인으로 이동')
                            else:
                                logger.info('[Task Thread] 최종 Y 위치에 도달')

                        logger.info('[Task Thread] oiling 작업 완료')   
                        wait_until_running()
                        if not force_control(0): raise Exception("힘제어 종료 실패")
                        
                        wait_until_running()
                        movel([0,0,100,0,0,0], vel = vel_z, acc = acc_z, ref = DR_USER1, mod = DR_MV_MOD_REL);
                        if not rclpy.ok(): raise Exception("작업 중단")
                        logger.info('[Task Thread] 상승 완료')
                        return True
                    
                    # --- [메인 로직] ---

                    feedback_msg = BrushingAction.Feedback()

                    # 1. pick_e ------------------------------------------------
                    logger.info("[Task Thread] 1. 지우개 집기 시작")
                    wait_until_running()
                    feedback_msg.feedback_string = "현재 \"지우개 집기\" 작업중"
                    feedback_msg.current_stroke = 0
                    g_current_goal_handle.publish_feedback(feedback_msg)
                    
                    wait_until_running(); movel(posx(325.82, 267.29, 424.88, 82.80, 175.80, 80.38), vel = VELOCITY, t=2, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_ABS);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    wait_until_running(); gripper(1); wait(1.00)
                    if not rclpy.ok(): raise Exception("작업 중단")
                    wait_until_running(); movel(posx(325.82, 267.29, 310.78, 82.80, 175.80, 80.38), vel = VELOCITY, t=2, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_ABS);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    wait_until_running(); gripper(0); wait(1.00)
                    if not rclpy.ok(): raise Exception("작업 중단")
                    wait_until_running(); movel(posx(325.82, 267.29, 424.88, 82.80, 175.80, 80.38), vel = VELOCITY, t=2, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_ABS);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    
                    # 2. 좌표계 설정 및 계산 ---------------------------------------
                    logger.info("[Task Thread] 2. 사용자 좌표계 설정 및 계산 시작")
                    wait_until_running()
                    feedback_msg.feedback_string = "현재 \"좌표계 설정\" 작업중"
                    g_current_goal_handle.publish_feedback(feedback_msg)

                    p1 = [324.380, 67.386, 330.540]; p2 = [608.089, 65.371, 330.540]; p4 = [326.180, -117.220, 330.540]
                    p_origin = [326.180, -117.220, 330.540, 157.52, 180, 167.8]
                    
                    list_x = [p2[i]-p1[i] for i in range(3)]; A = math.sqrt(sum(i**2 for i in list_x)); list_X = [i/A for i in list_x]
                    list_y = [p1[i]-p4[i] for i in range(3)]; B = math.sqrt(sum(i**2 for i in list_y)); list_Y = [i/B for i in list_y]
                    
                    wait_until_running(); DR_USER1 = set_user_cart_coord(list_X, list_Y, p_origin, ref=DR_BASE)
                    logger.info(f"[Task Thread] 사용자 좌표계 'DR_USER1' (ID: {DR_USER1}) 생성 완료.")

                    board_h = 214; board_w = 300; sponge_h = 23; sponge_w = 23
                    
                    num_strokes = math.ceil(board_h / sponge_h)
                    total_overlap = (sponge_h * num_strokes) - board_h
                    overlap_count = num_strokes - 1
                    y_step_size = sponge_h - (total_overlap / overlap_count) if overlap_count > 0 else board_h
                    
                    logger.info(f"[Task Thread] 계산된 붓질 횟수: {num_strokes}회, Y 스텝: {y_step_size:.1f}mm")
                    move_x = board_w - 8
                    vel_x = 50; acc_x = 50; vel_y = 50; acc_y = 50; vel_z = 30; acc_z = 30
                    
                    # 3. 닦기 동작 시작 ------------------------------------------
                    logger.info("[Task Thread] 3. 닦기 동작 준비")
                    
                    init_pos = posx([331.79, 65.310, 350.540, 13.71, 180.0, 12.62])
                    wait_until_running(); movel(init_pos, vel=30, t=2, acc=30, ref = DR_BASE, mod = DR_MV_MOD_ABS);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info('[Task Thread] 닦기 초기 위치로 이동됨')

                    # 지우개 다시 잡기 
                    wait_until_running(); movel([0,0,-20,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    wait_until_running(); gripper(1); wait(1.00)
                    if not rclpy.ok(): raise Exception("작업 중단")
                    wait_until_running(); movel([0,0,-10,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    wait_until_running(); gripper(0); wait(1.00)
                    if not rclpy.ok(): raise Exception("작업 중단")
                    wait_until_running(); movel([0,0,+10,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL);
                    if not rclpy.ok(): raise Exception("작업 중단")

                    logger.info('[Task Thread] 지우개 재정렬 및 힘제어 준비 완료')
                    wait_until_running(); set_ref_coord(DR_USER1)

                    if not oiling_move(DR_USER1, num_strokes, y_step_size, move_x, feedback_msg):
                        raise Exception("oiling_move 함수 실행 실패")
                    
                    # 4. place_e -----------------------------------------------
                    logger.info("[Task Thread] 4. 지우개 내려놓기 시작")
                    wait_until_running()
                    feedback_msg.feedback_string = "현재 \"지우개 내려놓기\" 작업중"
                    feedback_msg.current_stroke = 0
                    g_current_goal_handle.publish_feedback(feedback_msg)

                    wait_until_running(); movel(posx(330.44, 289.61, 400.54, 64.35, -180.00, 63.26), vel = VELOCITY, t=2, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_ABS);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    wait_until_running(); movel(posx(0.00, 0.00, -85.26, 0.00, 0.00, 0.00), vel = VELOCITY, t=2, acc = ACC, radius=0.00, ref=0, mod=DR_MV_MOD_REL);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    wait_until_running(); gripper(1); wait(1.00)
                    
                    # --- [메인 로직 끝] ---

                    logger.info("[Task Thread] 전체 지우개 작업 시퀀스 1회 완료.")
                    
                    # (유지) '작업 스레드'가 성공 Result를 전송
                    end_time = time.time()
                    result = BrushingAction.Result()
                    result.complete_task = True
                    result.total_duration = end_time - start_time
                    result.final_pose , _ = get_current_posx() # 최종 위치 반환
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
    """(유지)"""
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
    """(유지)"""
    global task_running, trigger_event, g_current_goal_handle, node_, g_final_result
    logger = node_.get_logger()
    logger.info("[ROS Thread] Goal 실행 시작! '작업 스레드'를 트리거합니다.")

    g_current_goal_handle = goal_handle
    trigger_event.set()
    
    while g_current_goal_handle is not None and rclpy.ok():
        time.sleep(0.1)

    logger.info("[ROS Thread] Goal 실행 완료. (작업 스레드 종료 확인)")
    return g_final_result

def ros_spin_thread():
    """(유지)"""
    global node_
    print("[ROS Thread] ROS Spin 스레드 시작...")
    try:
        executor = MultiThreadedExecutor(num_threads=4) 
        executor.add_node(node_)
        
        action_server = ActionServer(
            node_,
            BrushingAction,
            'do_eraser_action',
            execute_callback=execute_callback,
            goal_callback=goal_callback,
            cancel_callback=None
        )
        print("[ROS Thread] 'do_eraser_action' Action 서버 시작됨.")
        
        print("[ROS Thread] '/arm_control' 토픽 구독 시작.")
        node_.create_subscription(
            String,
            "/arm_control",
            control_callback,
            10
        )
        
        executor.spin()
    except Exception as e:
        if rclpy.ok():
            node_.get_logger().error(f"[ROS Thread] Spin 중 예외 발생: {e}")
    finally:
        print("[ROS Thread] ROS Spin 스레드 종료됨.")


def main(args=None):
    """(수정) [!!!] 빠뜨렸던 DR_init.__dsr__node = node_ 라인 추가"""
    global node_
    
    rclpy.init(args=args)
    
    node_ = rclpy.create_node("eraser_action_server", namespace=ROBOT_ID)
    
    # --- ⬇️ [핵심 수정] 오류 수정 ⬇️ ---
    DR_init.__dsr__node = node_
    # --- ⬆️ [핵심 수정] ⬆️ ---
    
    print(f"노드 '{ROBOT_ID}/eraser_action_server' 생성 완료.")
    
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