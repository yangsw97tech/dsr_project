#Action_server.py

import rclpy
from rclpy.node import Node
import DR_init
import threading
import time
import math
from rclpy.executors import MultiThreadedExecutor

# 1. (수정) Action 관련 모듈 임포트
from rclpy.action import ActionServer, GoalResponse, CancelResponse
# 2. (수정) 사용자가 정의한 Action 인터페이스 임포트 (패키지명 가정)
from my_robot_interfaces.action import BrushingAction

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

# 3. (추가) 현재 처리 중인 Goal Handle을 '작업 스레드'와 공유하기 위한 전역 변수
g_current_goal_handle = None
g_final_result = None # (추가) 최종 결과를 execute_callback에 전달하기 위한 전역 변수


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

def perform_task_loop():
    """
    (유지) '작업 스레드'의 메인 루프.
    trigger_event를 기다리다가, 깨어나면 brushing_final 로직을 실행합니다.
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
                
                # (추가) 작업 시작 시간 기록
                start_time = time.time()
                
                logger.info("[Task Thread] 트리거 수신! 'brushing_final' 로직을 1회 실행합니다.")
                
                try:
                    # DSR 라이브러리 임포트 (get_current_posx 추가)
                    from DSR_ROBOT2 import (
                        movel, wait, movej, movec, posx, set_user_cart_coord,
                        set_digital_output, get_current_posx, # <--- get_current_posx 추가
                        DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, ON, OFF, 
                        DR_MV_RA_DUPLICATE
                    )

                    # --- 5. (내부 함수) gripper 정의 ---
                    def gripper(switch):
                        if switch == 0: set_digital_output(1, ON); set_digital_output(2, OFF)
                        elif switch == 1: set_digital_output(1, OFF); set_digital_output(2, ON)

                    # --- 6. (내부 함수) brush_pick 정의 ---
                    def brush_pick():
                        logger.info(f'[Task Thread] brushing pick 시작')
                        if not rclpy.ok(): return False
                        movel(posx(325.82, 267.29, 424.88, 82.80, 175.80, 80.38), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        if not rclpy.ok(): return False
                        movel(posx(365.81, 241.49, 424.88, 42.53, 179.97, 42.54), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        if not rclpy.ok(): return False
                        movel(posx(0.00, 0.00, -98.31, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        gripper(0); time.sleep(1.00) 
                        if not rclpy.ok(): return False
                        movel(posx(348.81, 241.49, 418.61, 42.19, 179.97, 42.20), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        if not rclpy.ok(): return False
                        movel(posx(-35.70, -208.74, 0.00, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        if not rclpy.ok(): return False
                        movel(posx(313.12, 32.75, 388.61, 42.27, 179.97, 42.28), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        logger.info(f'[Task Thread] brushing pick 완료')
                        return True

                    # --- 7. (내부 함수) brush_place 정의 ---
                    def brush_place():
                        logger.info(f'[Task Thread] brushing place 시작')
                        if not rclpy.ok(): return False
                        movel(posx(598.34, -93.08, 463.79, 42.80, 179.97, 42.81), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        if not rclpy.ok(): return False
                        movel(posx(352.89, 241.49, 463.79, 42.12, 179.97, 42.13), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        if not rclpy.ok(): return False
                        movel(posx(0.00, 0.00, -130.26, 0.00, 0.00, 0.00), vel=VELOCITY, acc=ACC, time=Time, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        gripper(1); time.sleep(1.00) 
                        if not rclpy.ok(): return False
                        logger.info(f'[Task Thread] brushing place 완료')
                        # logger.info(f'[Task Thread] brushing place 이후 홈정렬')
                        # home_pose_j = [0, 0, 90, 0, 90, 0]
                        # movej(home_pose_j, vel = 30, acc = 30, time = Time)
                        return True

                    # --- 8. (내부 함수) brushing_move_1 정의 ---
                    def brushing_move_1(DR_USER1, board_w, num_strokes, y_step_size, move_x):
                        logger.info("[Task Thread] brushing_move_1 시작")
                        vel_x = 60; acc_x = 60; vel_y = 50; acc_y = 50
                        
                        if not rclpy.ok(): return False
                        movel([323.64,20.97,388.61,5.05,-179.58,4.44], vel = 50, acc = 50, time = Time, mod = DR_MV_MOD_ABS)
                        logger.info('[Task Thread] 브러싱 시작 위치 배치 완료')

                        # 9. (수정) 피드백 메시지 객체는 한 번만 생성
                        feedback_msg = BrushingAction.Feedback()

                        for y_move_cnt in range(int(num_strokes)):
                            
                            # --- ⬇️ 여기를 수정했습니다 (포맷 변경) ⬇️ ---
                            feedback_msg.feedback_string = f"현재 \"직선 붓질\" 작업 \"{y_move_cnt + 1}\"번째 줄 진행중"
                            feedback_msg.current_stroke = y_move_cnt + 1
                            # --- ⬆️ 여기까지 수정 ⬆️ ---
                            
                            g_current_goal_handle.publish_feedback(feedback_msg) # 피드백 전송
                            
                            if not rclpy.ok(): return False
                            movel([move_x,0, 0,0,0,0], vel = vel_x, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL)
                            if not rclpy.ok(): return False
                            movel([0,0,15,0,0,0], vel = vel_x, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL)
                            
                            if not y_move_cnt == int(num_strokes)-1:
                                if not rclpy.ok(): return False
                                movel([-move_x,-y_step_size, 0,0,0,0], vel = vel_y, acc = acc_y, ref = DR_USER1, mod = DR_MV_MOD_REL)
                                if not rclpy.ok(): return False
                                movel([0,0,-15,0,0,0], vel = vel_x, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL)

                            logger.info(f'[Task Thread] {y_move_cnt + 1}번째 붓질 완료')
                        
                        logger.info('--[Task Thread] brushing1 작업 완료--')   
                        movel([0,0,50,0,0,0], vel = 60, acc = 60, time = Time, ref = DR_BASE, mod = DR_MV_MOD_REL)
                        logger.info('[Task Thread] 상승 완료')
                        return True

                    # --- 9. (내부 함수) brushing_move_2 정의 ---
                    def brushing_move_2(DR_USER1, board_w, num_strokes, y_step_size):
                        logger.info("[Task Thread] brushing_move_2 시작")
                        vel_x = 60; acc_x = 60; beta = 5
                        P_1_init = posx([273.64,20.97,418.61-beta,5.05,-179.58,4.44])
                        P_2_init = posx([323.64,20.97,388.61-beta,5.05,-179.58,4.44])
                        P_3_init = posx([373.64,20.97,418.61-beta,5.05,-179.58,4.44])
                        list_P_init = [P_1_init, P_2_init, P_3_init]

                        # 9. (수정) 피드백 메시지 객체는 한 번만 생성
                        feedback_msg = BrushingAction.Feedback()

                        for y_move_cnt in range(int(num_strokes)):
                            
                            # --- ⬇️ 여기를 수정했습니다 (포맷 변경) ⬇️ ---
                            feedback_msg.feedback_string = f"현재 \"원형 붓질\" 작업 \"{y_move_cnt + 1}\"번째 줄 진행중"
                            feedback_msg.current_stroke = y_move_cnt + 1
                            # --- ⬆️ 여기까지 수정 ⬆️ ---
                            
                            g_current_goal_handle.publish_feedback(feedback_msg) # 피드백 전송

                            P_1 = P_1_init.copy(); P_2 = P_2_init.copy(); P_3 = P_3_init.copy()
                            list_P = [P_1,P_2,P_3]; delta = 50; x_move_cnt = 0

                            while delta * x_move_cnt < board_w:
                                if not rclpy.ok(): return False
                                movel(P_1, time = 0.5,vel = vel_x, acc = acc_x, mod = DR_MV_MOD_ABS)
                                if not rclpy.ok(): return False
                                movec(P_2, P_3, time = 0.5,vel = vel_x, acc = acc_x, mod = DR_MV_MOD_ABS)
                                x_move_cnt += 1
                                for P in list_P: P[0] += delta

                            logger.info(f'[Task Thread] {y_move_cnt + 1}번째 붓질 완료')
                            for P in list_P_init: P[1] -= abs(y_step_size)

                            if not y_move_cnt == int(num_strokes)-1:
                                if not rclpy.ok(): return False
                                movel(list_P_init[0], vel = vel_x, acc = acc_x, mod = DR_MV_MOD_ABS)
                                
                        logger.info('--[Task Thread] brushing2 작업 완료--')
                        movel([0,0,30,0,0,0], vel = 60, acc = 60, ref = DR_BASE, mod = DR_MV_MOD_REL)
                        logger.info('[Task Thread] 상승 완료')
                        home_pose_j = [0, 0, 90, 0, 90, 0]
                        movej(home_pose_j, vel = 30, acc = 30, time = Time)
                        return True

                    # --- 10. (핵심) brushing_final.py의 메인 로직 실행 ---
                    # home_pose_j = [0, 0, 90, 0, 90, 0]
                    # movej(home_pose_j, vel = 30, acc = 30)

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
                    
                    feedback_msg = BrushingAction.Feedback()

                    # --- 11. (핵심) 전체 시퀀스 호출 및 피드백 전송 ---
                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 1. brush_pick 시작")
                    
                    # --- ⬇️ 여기를 수정했습니다 (픽/플레이스 포맷) ⬇️ ---
                    feedback_msg.feedback_string = "현재 \"붓 집기\" 작업중"
                    feedback_msg.current_stroke = 0
                    g_current_goal_handle.publish_feedback(feedback_msg)
                    if not brush_pick(): raise Exception("brush_pick 실패")

                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 2. brushing_move_2 시작")
                    
                    # --- ⬇️ 여기를 삭제했습니다 (0번째 피드백 제거) ⬇️ ---
                    # (불필요한 피드백 전송 코드 제거)
                    # --- ⬆️ 여기까지 삭제 ⬆️ ---
                    
                    if not brushing_move_2(DR_USER1, board_w, num_strokes, y_step_size): raise Exception("brushing_move_2 실패")
                    
                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 3. brushing_move_1 시작")
                    
                    # --- ⬇️ 여기를 삭제했습니다 (0번째 피드백 제거) ⬇️ ---
                    # (불필요한 피드백 전송 코드 제거)
                    # --- ⬆️ 여기까지 삭제 ⬆️ ---
                    
                    if not brushing_move_1(DR_USER1, board_w, num_strokes, y_step_size, move_x): raise Exception("brushing_move_1 실패")
                    
                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 4. brush_place 시작")
                    
                    # --- ⬇️ 여기를 수정했습니다 (픽/플레이스 포맷) ⬇️ ---
                    feedback_msg.feedback_string = "현재 \"붓 내려놓기\" 작업중"
                    feedback_msg.current_stroke = 0
                    # --- ⬆️ 여기까지 수정 ⬆️ ---
                    
                    g_current_goal_handle.publish_feedback(feedback_msg)
                    if not brush_place(): raise Exception("brush_place 실패")
                    
                    logger.info("[Task Thread] 전체 브러싱 시퀀스 1회 완료.")
                    
                    # 10. (수정) '작업 스레드'가 성공 Result를 전송
                    end_time = time.time()
                    result = BrushingAction.Result()
                    result.complete_task = True
                    result.total_duration = end_time - start_time
                    result.final_pose , _ = get_current_posx() # 최종 위치 반환
                    g_final_result = result # (수정) 결과를 전역 변수에 저장
                    g_current_goal_handle.succeed() # (수정) 인자 없이 호출

                except Exception as e:
                    if rclpy.ok():
                        logger.error(f"[Task Thread] 작업 시퀀스 중 예외 발생: {e}")
                    # 10. (수정) '작업 스레드'가 실패 Result를 전송
                    if g_current_goal_handle and g_current_goal_handle.is_active:
                        result = BrushingAction.Result()
                        result.complete_task = False
                        g_final_result = result # (수정) 결과를 전역 변수에 저장
                        g_current_goal_handle.abort() # (수정) 인자 없이 호출
                finally:
                    # 11. (수정) 작업 완료 후 상태 리셋
                    task_running = False
                    trigger_event.clear()
                    # Goal Handle 해제는 execute_callback이 종료된 후 자동으로 되므로,
                    # 여기서는 None으로 설정하여 while 루프를 종료시키는 역할만 함
                    g_current_goal_handle = None 
                    logger.info("[Task Thread] 작업 스레드 종료됨 (다음 요청 대기).")
            else:
                node_.get_logger().warn("[Task Thread] 작업이 이미 실행 중. 중복 트리거 무시.")
                trigger_event.clear()
    
    print("[Task Thread] 작업 스레드 루프 종료됨.")


def goal_callback(goal_request):
    """
    (수정) 새 Goal 수락 여부만 결정하는 콜백
    """
    global task_running, node_
    logger = node_.get_logger()

    # 4. (수정) Goal 요청을 받았을 때의 처리
    if task_running:
        # 4. (수정) 이미 작업 중이면 새 Goal은 거부(Abort)
        logger.warn("[ROS Thread] 작업이 이미 실행 중이므로 새 Goal을 거부합니다.")
        return GoalResponse.REJECT
    
    if not goal_request.start_task:
        logger.warn("[ROS Thread] 'start_task'가 False이므로 Goal을 거부합니다.")
        return GoalResponse.REJECT

    logger.info("[ROS Thread] 새 Goal을 수락합니다.")
    return GoalResponse.ACCEPT

def execute_callback(goal_handle):
    """
    (신규) Goal을 수락한 후, 실제 작업을 실행하고 완료될 때까지 대기하는 콜백.
    이 함수가 종료되어야 Action 서버가 Goal 처리가 끝났다고 인지합니다.
    """
    global task_running, trigger_event, g_current_goal_handle, node_, g_final_result
    logger = node_.get_logger()
    logger.info("[ROS Thread] Goal 실행 시작! '작업 스레드'를 트리거합니다.")

    # 4. (수정) '작업 스레드'가 사용할 수 있도록 Goal Handle을 전역 변수에 저장
    g_current_goal_handle = goal_handle
    
    # '작업 스레드'에 '시작' 신호 전송
    trigger_event.set()
    
    # [핵심] '작업 스레드'가 g_current_goal_handle을 None으로 만들 때까지(즉, 작업이 끝날 때까지) 대기
    while g_current_goal_handle is not None and rclpy.ok():
        time.sleep(0.1)

    logger.info("[ROS Thread] Goal 실행 완료. (작업 스레드 종료 확인)")
    # 이 함수가 리턴될 때 goal_handle의 최종 상태가 결정되어 있어야 합니다.
    return g_final_result

def ros_spin_thread():
    """(유지) 'ROS 스레드'의 메인 루프 (Executor 사용)"""
    global node_
    print("[ROS Thread] ROS Spin 스레드 시작...")
    try:
        executor = MultiThreadedExecutor(num_threads=4) 
        executor.add_node(node_)
        
        # 5. (수정) 토픽 구독 대신 ActionServer 생성
        action_server = ActionServer(
            node_,
            BrushingAction,
            'do_brushing_action', # Action 이름
            execute_callback=execute_callback, # Goal 실행 콜백
            goal_callback=goal_callback, # Goal 수락/거부 콜백
            cancel_callback=None # 취소 콜백 (필요 시 구현)
        )
        print("[ROS Thread] 'do_brushing_action' Action 서버 시작됨.")
        
        executor.spin()
    except Exception as e:
        if rclpy.ok():
            node_.get_logger().error(f"[ROS Thread] Spin 중 예외 발생: {e}")
    finally:
        print("[ROS Thread] ROS Spin 스레드 종료됨.")


def main(args=None):
    """메인 함수: 노드 생성 -> 초기화 -> 스레드 2개 분리 실행"""
    global node_
    
    rclpy.init(args=args)
    
    # DSR 드라이버 충돌을 피하기 위해 서버마다 랜덤 지연 시간 추가
    # 5개 서버가 동시에 set_tool/set_tcp를 호출하는 것을 방지
    import random
    delay = random.uniform(0.1, 1.0) 
    print(f"[Oiling Server] 드라이버 충돌 방지를 위해 {delay:.2f}초 대기...")
    time.sleep(delay)

    node_ = rclpy.create_node("brushing_action_server", namespace=ROBOT_ID)
    DR_init.__dsr__node = node_
    
    print(f"노드 '{ROBOT_ID}/brushing_action_server' 생성 완료.")
    
    spin_thread = None
    robot_thread = None

    try:
        if not initialize_robot():
            raise Exception("로봇 초기화 실패. 프로그램을 종료합니다.")

        # 6. (수정) 토픽 구독자 생성 코드 제거됨 (ros_spin_thread로 이동)

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