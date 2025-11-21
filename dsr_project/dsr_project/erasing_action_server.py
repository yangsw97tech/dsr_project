#Action_server.py (Eraser 동작 이식 버전)

import rclpy
from rclpy.node import Node
import DR_init
import threading
import time
import math
from rclpy.executors import MultiThreadedExecutor

# 1. (유지) Action 관련 모듈 임포트
from rclpy.action import ActionServer, GoalResponse, CancelResponse
# 2. (유지) 사용자가 정의한 Action 인터페이스 임포트 (요청대로 BrushingAction 유지)
from my_robot_interfaces.action import BrushingAction

# --- 로봇 설정 상수 (eraser_pick_and_place.py 기준으로 변경) ---
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight" #본인 TCP 이름 설정
ROBOT_TOOL = "GripperDA_v1"  #본인 그리퍼 이름 설정
VELOCITY = 60 # eraser_pick_and_place.py 값
ACC = 60 # eraser_pick_and_place.py 값
# Time = 2 (eraser 스크립트는 이 변수를 사용하지 않음)

# --- DR_init 설정 (유지) ---
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# --- 전역 변수 (유지) ---
node_ = None
trigger_event = threading.Event()
task_running = False
set_tool_func = None
set_tcp_func = None

# 3. (유지) 현재 처리 중인 Goal Handle을 '작업 스레드'와 공유하기 위한 전역 변수
g_current_goal_handle = None
g_final_result = None # (유지) 최종 결과를 execute_callback에 전달하기 위한 전역 변수


def initialize_robot():
    # (유지) 구조 변경 없음. (위에서 변경된 TCP, TOOL 상수를 사용함)
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

def perform_task_loop():
    """
    (수정) '작업 스레드'의 메인 루프.
    trigger_event를 기다리다가, 깨어나면 'eraser_pick_and_place' 로직을 실행합니다.
    """
    global task_running, trigger_event, node_, g_current_goal_handle, g_final_result
    
    logger = node_.get_logger()
    print("[Task Thread] 작업 스레드 시작됨. 트리거 대기 중... (Eraser 버전)")
    
    while rclpy.ok():
        triggered = trigger_event.wait(timeout=1.0)
        
        if not rclpy.ok():
            break
        
        if triggered:
            if not task_running:
                task_running = True
                
                # (유지) 작업 시작 시간 기록
                start_time = time.time()
                
                logger.info("[Task Thread] 트리거 수신! 'eraser_pick_and_place' 로직을 1회 실행합니다.")
                
                try:
                    from DSR_ROBOT2 import (
                        release_compliance_ctrl, release_force,
                        check_force_condition,
                        task_compliance_ctrl,
                        set_desired_force,
                        set_ref_coord,
                        get_digital_input, get_digital_output, set_digital_output,
                        movej, movel,
                        get_current_posx, wait, # get_current_posx는 result를 위해 유지
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
                    # --- ⬆️ [수정] DSR 임포트 ⬆️ ---


                    # --- ⬇️ [수정] eraser_pick_and_place.py의 헬퍼 함수 ⬇️ ---
                    
                    # gripper 조절
                    def gripper(switch):
                        # gripper 닫기
                        if switch == 0:
                            set_digital_output(1,ON)
                            set_digital_output(2,OFF)
                        # gripper 열기
                        elif switch == 1:
                            set_digital_output(1,OFF)
                            set_digital_output(2,ON)
                        # (추가) 안전 대기
                        wait(1.0) # gripper 동작 시간 대기

                    # 힘제어 함수
                    def force_control(switch, press_force = 10):
                        if not rclpy.ok(): return False # (추가) 안전 체크
                        
                        #switch on(1) 이면 힘제어를 시작합니다.
                        if switch == 1:
                            logger.info("힘 제어 시작")
                            set_ref_coord(1) # Tool 좌표계 설정
                            task_compliance_ctrl(stx=[3000, 3000, 200, 200, 200, 200])  #강성 조절(헐겁게)
                            wait(0.5) # 안정화 대기(필수)
                            set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

                            #힘 감지 체크
                            while rclpy.ok():
                                ret = check_force_condition(DR_AXIS_Z, max=press_force)
                                if ret == -1:
                                    logger.info(f"Z축 힘이 {press_force}N 이상 감지됨. {ret}")
                                    break
                            wait(0.5)

                        #switch off(0) 이면 힘제어를 중지합니다.
                        elif switch == 0:
                            logger.info("힘 제어 중지")
                            release_force()
                            release_compliance_ctrl()
                        
                        return rclpy.ok()

                    # oiling_move 함수 (피드백 추가)
                    def oiling_move(DR_USER1, num_strokes, y_step_size, move_x, feedback_msg):
                        vel_x = 50; acc_x = 50
                        vel_y = 50; acc_y = 50
                        vel_z = 30; acc_z = 30

                        # 오일링을 하기 위한 도마 위(좌상단)부터 시작
                        if not force_control(1,14): raise Exception("힘제어 시작 실패") #힘제어 시작

                        for y_move_cnt in range(int(num_strokes)):
                            
                            # --- (추가) 피드백 전송 ---
                            feedback_msg.feedback_string = f"현재 \"지우개 닦기\" 작업 \"{y_move_cnt + 1}\"번째 줄 진행중"
                            feedback_msg.current_stroke = y_move_cnt + 1
                            g_current_goal_handle.publish_feedback(feedback_msg)
                            # ---

                            # x축 방향 이동 (왕복 칠하기)
                            if y_move_cnt %2 == 0:
                                if not force_control(0): raise Exception("힘제어 종료 실패") #힘제어 종료
                                movel([move_x,0, 0,0,0,0], vel = vel_x, time=2.0, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                                movel([-move_x,0, 0,0,0,0], vel = vel_x, time=2.0, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                                movel([move_x,0, 0,0,0,0], vel = vel_x, time=2.0, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                            else:
                                if not force_control(0): raise Exception("힘제어 종료 실패") #힘제어 종료
                                movel([-move_x,0, 0,0,0,0], vel = vel_x, time=2.0, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                                movel([move_x,0, 0,0,0,0], vel = vel_x, time=2.0, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                                movel([-move_x,0, 0,0,0,0], vel = vel_x, time=2.0, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")

                            logger.info(f'[Task Thread] {y_move_cnt + 1}번째 붓질 완료')

                            # 마지막 붓질 후에는 y 이동 불필요
                            if y_move_cnt < num_strokes - 1:
                                movel([0, 0, +10,0,0,0], vel = vel_y, time=1.0, acc = acc_y, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                                # y_step_size 만큼 이동
                                movel([0,-y_step_size, 0,0,0,0], vel = vel_y, time=1.0, acc = acc_y, ref = DR_USER1, mod = DR_MV_MOD_REL);
                                if not rclpy.ok(): raise Exception("작업 중단")
                                if not force_control(1,14): raise Exception("힘제어 시작 실패") #힘제어 시작
                                logger.info(f'[Task Thread] {y_step_size:.1f}mm 다음 라인으로 이동')
                            else:
                                logger.info('[Task Thread] 최종 Y 위치에 도달')

                        logger.info('[Task Thread] oiling 작업 완료')   
                        if not force_control(0): raise Exception("힘제어 종료 실패") # 힘제어 종료
                        
                        # z축 상향 이동
                        movel([0,0,100,0,0,0], vel = vel_z, acc = acc_z, ref = DR_USER1, mod = DR_MV_MOD_REL);
                        if not rclpy.ok(): raise Exception("작업 중단")
                        logger.info('[Task Thread] 상승 완료')
                        return True
                    
                    # --- ⬆️ [수정] eraser_pick_and_place.py의 헬퍼 함수 ⬆️ ---
                    

                    # --- ⬇️ [수정] eraser_pick_and_place.py의 메인 동작 ⬇️ ---

                    # (유지) 피드백 메시지 객체 생성
                    feedback_msg = BrushingAction.Feedback()

                    # 1. pick_e ------------------------------------------------
                    logger.info("[Task Thread] 1. 지우개 집기 시작")
                    feedback_msg.feedback_string = "현재 \"지우개 집기\" 작업중"
                    feedback_msg.current_stroke = 0
                    g_current_goal_handle.publish_feedback(feedback_msg)
                    
                    # 330.44, 289.61, 400.54  -> 원래 place
                    # 325.82, 267.29,  -> 원래 pick
                    movel(posx(330.44, 289.61, 424.88, 0, -180, 0), vel = VELOCITY, time=2.0, acc = ACC, ref=0, mod=DR_MV_MOD_ABS);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    gripper(1)  # 그리퍼 열기
                    if not rclpy.ok(): raise Exception("작업 중단")
                    movel(posx(330.44, 289.61, 310.78, 0, -180, 0), vel = VELOCITY, time=2.0, acc = ACC, ref=0, mod=DR_MV_MOD_ABS);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    gripper(0)  # 그리퍼 닫기
                    if not rclpy.ok(): raise Exception("작업 중단")
                    movel(posx(330.44, 289.61, 424.88, 0, -180, 0), vel = VELOCITY, time=2.0, acc = ACC, ref=0, mod=DR_MV_MOD_ABS);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    
                    # 2. 좌표계 설정 및 계산 ---------------------------------------
                    logger.info("[Task Thread] 2. 사용자 좌표계 설정 및 계산 시작")
                    feedback_msg.feedback_string = "현재 \"좌표계 설정\" 작업중"
                    g_current_goal_handle.publish_feedback(feedback_msg)

                    # --- Yaw (Rz) -1.5도 회전 (직교 보정) ---
                    # X축 Z회전: -1.5000도, X/Y 내각: 90.00도
                    p1 = [331.013, 67.332, 330.540]
                    p2 = [614.632, 59.905, 330.540]
                    p4 = [326.180, -117.220, 330.540]
                    # p_origin (C 값 165.5 + 1.207 = 166.7)
                    p_origin = [326.180, -117.220, 330.540, 157.52, 180, 166.7]
                    
                    list_x = [p2[i]-p1[i] for i in range(3)]; A = math.sqrt(sum(i**2 for i in list_x)); list_X = [i/A for i in list_x]
                    list_y = [p1[i]-p4[i] for i in range(3)]; B = math.sqrt(sum(i**2 for i in list_y)); list_Y = [i/B for i in list_y]
                    
                    DR_USER1 = set_user_cart_coord(list_X, list_Y, p_origin, ref=DR_BASE)
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
                    
                    # 초기 위치로 이동
                    init_pos = posx([331.79, 65.310, 350.540, 13.71, 180.0, 12.62])
                    movel(init_pos, vel=30, time=2.0, acc=30, ref = DR_BASE, mod = DR_MV_MOD_ABS);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info('[Task Thread] 닦기 초기 위치로 이동됨')

                    # 지우개 다시 잡기 
                    movel([0,0,-7,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    gripper(1)  # 그리퍼 열기
                    if not rclpy.ok(): raise Exception("작업 중단")
                    movel([0,0,-30,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    gripper(0)  # 그리퍼 닫기
                    if not rclpy.ok(): raise Exception("작업 중단")
                    movel([0,0,+10,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL);
                    if not rclpy.ok(): raise Exception("작업 중단")

                    logger.info('[Task Thread] 지우개 재정렬 및 힘제어 준비 완료')

                    set_ref_coord(DR_USER1)

                    # (수정) oiling_move 호출
                    if not oiling_move(DR_USER1, num_strokes, y_step_size, move_x, feedback_msg):
                        raise Exception("oiling_move 함수 실행 실패")
                    
                    # 4. place_e -----------------------------------------------
                    logger.info("[Task Thread] 4. 지우개 내려놓기 시작")
                    feedback_msg.feedback_string = "현재 \"지우개 내려놓기\" 작업중"
                    feedback_msg.current_stroke = 0
                    g_current_goal_handle.publish_feedback(feedback_msg)

                    # 지우개 위치로 이동
                    movel(posx(330.44, 289.61, 400.54, 64.35, -180.00, 63.26), vel = VELOCITY, time=2.0, acc = ACC, ref=0, mod=DR_MV_MOD_ABS);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    # 지우개 내리기
                    movel(posx(0.00, 0.00, -85.26, 0.00, 0.00, 0.00), vel = VELOCITY, time=2.0, acc = ACC, ref=0, mod=DR_MV_MOD_REL);
                    if not rclpy.ok(): raise Exception("작업 중단")
                    gripper(1)  # 그리퍼 열기
                    if not rclpy.ok(): raise Exception("작업 중단")
                    
                    # --- ⬆️ [수정] eraser_pick_and_place.py의 메인 동작 ⬆️ ---

                    logger.info("[Task Thread] 전체 지우개 작업 시퀀스 1회 완료.")
                    
                    # (유지) '작업 스레드'가 성공 Result를 전송
                    end_time = time.time()
                    result = BrushingAction.Result()
                    result.complete_task = True
                    result.total_duration = end_time - start_time
                    result.final_pose , _ = get_current_posx(ref=DR_BASE) # 최종 위치 반환
                    g_final_result = result # (유지) 결과를 전역 변수에 저장
                    g_current_goal_handle.succeed() # (유지)

                except Exception as e:
                    if rclpy.ok():
                        logger.error(f"[Task Thread] 작업 시퀀스 중 예외 발생: {e}")
                    # (유지) '작업 스레드'가 실패 Result를 전송
                    if g_current_goal_handle and g_current_goal_handle.is_active:
                        result = BrushingAction.Result()
                        result.complete_task = False
                        g_final_result = result # (유지) 결과를 전역 변수에 저장
                        g_current_goal_handle.abort() # (유지)
                finally:
                    # (유지) 작업 완료 후 상태 리셋
                    task_running = False
                    trigger_event.clear()
                    g_current_goal_handle = None 
                    logger.info("[Task Thread] 작업 스레드 종료됨 (다음 요청 대기).")
            else:
                node_.get_logger().warn("[Task Thread] 작업이 이미 실행 중. 중복 트리거 무시.")
                trigger_event.clear()
    
    print("[Task Thread] 작업 스레드 루프 종료됨.")


def goal_callback(goal_request):
    """
    (유지) 새 Goal 수락 여부만 결정하는 콜백
    """
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
    """
    (유지) Goal을 수락한 후, 실제 작업을 실행하고 완료될 때까지 대기하는 콜백.
    """
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
    """(유지) 'ROS 스레드'의 메인 루프 (Executor 사용)"""
    global node_
    print("[ROS Thread] ROS Spin 스레드 시작...")
    try:
        executor = MultiThreadedExecutor(num_threads=4) 
        executor.add_node(node_)
        
        # 5. (수정) ActionServer 이름 변경
        action_server = ActionServer(
            node_,
            BrushingAction,
            'do_eraser_action', # <-- Action 이름 변경
            execute_callback=execute_callback,
            goal_callback=goal_callback,
            cancel_callback=None
        )
        print("[ROS Thread] 'do_eraser_action' Action 서버 시작됨.")
        
        executor.spin()
    except Exception as e:
        if rclpy.ok():
            node_.get_logger().error(f"[ROS Thread] Spin 중 예외 발생: {e}")
    finally:
        print("[ROS Thread] ROS Spin 스레드 종료됨.")


def main(args=None):
    """(유지) 메인 함수: 노드 생성 -> 초기화 -> 스레드 2개 분리 실행"""
    global node_
    
    rclpy.init(args=args)
    
    # DSR 드라이버 충돌을 피하기 위해 서버마다 랜덤 지연 시간 추가
    # 5개 서버가 동시에 set_tool/set_tcp를 호출하는 것을 방지
    import random
    delay = random.uniform(0.1, 1.0) 
    print(f"[Oiling Server] 드라이버 충돌 방지를 위해 {delay:.2f}초 대기...")
    time.sleep(delay)

    # (수정) 노드 이름 변경
    node_ = rclpy.create_node("eraser_action_server", namespace=ROBOT_ID)
    DR_init.__dsr__node = node_
    
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