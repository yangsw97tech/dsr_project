#oiling_a_server.py

import rclpy
from rclpy.node import Node
import DR_init
import threading
import time
import math
from rclpy.executors import MultiThreadedExecutor

# 1. Action 관련 모듈 임포트
from rclpy.action import ActionServer, GoalResponse, CancelResponse
# 2. (중요) 사용자가 정의한 'my_robot_interfaces'의 'BrushingAction'을 재사용
from my_robot_interfaces.action import BrushingAction

# --- 로봇 설정 상수 (oiling.py 기준) ---
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
# (핵심 수정) 'test_a_server.py'와 동일하게 최종 결과를 전달하기 위한 전역 변수 추가
g_final_result = None 


def initialize_robot():
    """로봇 초기화 (DSR_ROBOT2 지연 임포트)"""
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
    trigger_event를 기다리다가, 깨어나면 'oiling' 로직을 실행합니다.
    """
    # (핵심 수정) g_final_result 전역 변수 사용
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
                
                logger.info("[Task Thread] 트리거 수신! 'oiling' 로직을 1회 실행합니다.")
                
                try:
                    from DSR_ROBOT2 import (
                        movel, wait, movej, posx, set_digital_output, get_current_posx,
                        DR_BASE, DR_MV_MOD_ABS, DR_MV_MOD_REL, ON, OFF, 
                        DR_MV_RA_DUPLICATE
                    )
                    
                    # --- Oiling 내부 함수 정의 ---
                    
                    def gripper(switch):
                        if switch == 0: set_digital_output(1, ON); set_digital_output(2, OFF)
                        elif switch == 1: set_digital_output(1, OFF); set_digital_output(2, ON)

                    def oiling_pick(logger):
                        logger.info(f'[Task Thread] oiling_pick 시작')
                        if not rclpy.ok(): return False
                        movel(posx(352.89, 241.49, 431.28, 42.12, 179.97, 42.13), vel = VELOCITY, acc = ACC, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        # if not rclpy.ok(): return False
                        # movel(posx(0.00, 0.00, 97.75, 0.00, 0.00, 0.00), vel = VELOCITY, acc = ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        if not rclpy.ok(): return False
                        movel(posx(140.00, -7.98, 0.00, 0.00, 0.00, 0.00), vel = VELOCITY, acc = ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        if not rclpy.ok(): return False
                        movel(posx(0.00, 0.00, -85.78, 0.00, 0.00, 0.00), vel = VELOCITY, acc = ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        
                        gripper(0)
                        time.sleep(1.00) 
                        if not rclpy.ok(): return False

                        movel(posx(0.00, 0.00, 119.42, 0.00, 0.00, 0.00), vel = VELOCITY, acc = ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        if not rclpy.ok(): return False
                        movel(posx(-150.00, -199.10, 0.00, 0.00, 0.00, 0.00), vel = VELOCITY, acc = ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        logger.info(f'[Task Thread] oiling_pick 완료')
                        return True

                    def oiling_move(logger, num_strokes, y_step_size, move_x, vel_x, acc_x, vel_y, acc_y, vel_z, acc_z):
                        logger.info(f'[Task Thread] oiling_move 시작')
                        
                        feedback_msg = BrushingAction.Feedback()

                        # --- 1번째 줄 ---
                        feedback_msg.feedback_string = f"현재 \"오일링\" 작업 \"{1}\"번째 줄 진행중"
                        feedback_msg.current_stroke = 1
                        g_current_goal_handle.publish_feedback(feedback_msg)
                        
                        if not rclpy.ok(): return False
                        movel([0,0,-65,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)
                        if not rclpy.ok(): return False
                        movel([move_x+10,0,0,0,0,0], vel = vel_x, acc = acc_x, ref = DR_BASE, mod = DR_MV_MOD_REL)
                        if not rclpy.ok(): return False
                        movel([0,0,20,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)
                        
                        # --- 2번째 줄 ---
                        feedback_msg.feedback_string = f"현재 \"오일링\" 작업 \"{2}\"번째 줄 진행중"
                        feedback_msg.current_stroke = 2
                        g_current_goal_handle.publish_feedback(feedback_msg)

                        if not rclpy.ok(): return False
                        movel([0,-65,0,0,0,0], vel = vel_y, acc = acc_y, ref = DR_BASE, mod = DR_MV_MOD_REL)
                        if not rclpy.ok(): return False
                        movel([0,0,-20,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)
                        if not rclpy.ok(): return False
                        movel([-move_x-25,0,0,0,0,0], vel = vel_x, acc = acc_x, ref = DR_BASE, mod = DR_MV_MOD_REL)
                        if not rclpy.ok(): return False
                        movel([0,0,20,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)

                        # --- 3번째 줄 ---
                        feedback_msg.feedback_string = f"현재 \"오일링\" 작업 \"{3}\"번째 줄 진행중"
                        feedback_msg.current_stroke = 3
                        g_current_goal_handle.publish_feedback(feedback_msg)

                        if not rclpy.ok(): return False
                        movel([0,-65,0,0,0,0], vel = vel_y, acc = acc_y, ref = DR_BASE, mod = DR_MV_MOD_REL)
                        if not rclpy.ok(): return False
                        movel([0,0,-20,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)
                        if not rclpy.ok(): return False
                        movel([move_x+20,0,0,0,0,0], vel = vel_x, acc = acc_x, ref = DR_BASE, mod = DR_MV_MOD_REL)
                        
                        logger.info(f'[Task Thread] oiling_move 완료')
                        return True

                    def oiling_place(logger):
                        logger.info(f'[Task Thread] oiling_place 시작')
                        if not rclpy.ok(): return False
                        movel(posx(598.34, -93.08, 463.79, 42.80, 179.97, 42.81), vel = VELOCITY, acc = ACC, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        if not rclpy.ok(): return False
                        movel(posx(505.55, 233.19, 463.79, 43.27, 179.97, 43.28), vel = VELOCITY, acc = ACC, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                        if not rclpy.ok(): return False
                        movel(posx(0.00, 0.00, -116.69, 0.00, 0.00, 0.00), vel = VELOCITY, acc = ACC, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
                        
                        gripper(1)
                        time.sleep(1.00) 
                        if not rclpy.ok(): return False

                        logger.info(f'[Task Thread] oiling_place 완료')
                        logger.info(f'[Task Thread] oiling_place 이후 홈정렬')
                        # home_pose_j = [0, 0, 90, 0, 90, 0]
                        # movej(home_pose_j, vel = 30, acc = 30)
                        return True

                    # --- Oiling 로직의 메인 시퀀스 ---
                    # home_pose_j = [0, 0, 90, 0, 90, 0]
                    # movej(home_pose_j, vel = 30, acc = 30)

                    board_h = 200; board_w = 300; sponge_h = 95; sponge_w = 25
                    num_strokes = math.ceil(board_h / sponge_h)
                    total_overlap = (sponge_h * num_strokes) - board_h
                    overlap_count = num_strokes - 1
                    y_step_size = sponge_h - (total_overlap / overlap_count) if overlap_count > 0 else board_h
                    move_x = board_w - sponge_w
                    vel_x = 50; acc_x = 50; vel_y = 50; acc_y = 50; vel_z = 30; acc_z = 30
                    logger.info(f"[Task Thread] 오일링 계산 완료: {num_strokes}회 (계산상), Y간격 {y_step_size:.1f}mm")
                    
                    feedback_msg = BrushingAction.Feedback()

                    # --- Oiling 전체 시퀀스 호출 (피드백 포함) ---
                    
                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 1. oiling_pick 시작")
                    feedback_msg.feedback_string = "현재 \"오일 집기\" 작업중"
                    feedback_msg.current_stroke = 0
                    g_current_goal_handle.publish_feedback(feedback_msg)
                    if not oiling_pick(logger): raise Exception("oiling_pick 실패")

                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 2. oiling_move 시작")
                    # (oiling_move 시작 전 중복 피드백 제거)
                    if not oiling_move(logger, num_strokes, y_step_size, move_x, vel_x, acc_x, vel_y, acc_y, vel_z, acc_z): raise Exception("oiling_move 실패")
                    
                    if not rclpy.ok(): raise Exception("작업 중단")
                    logger.info("[Task Thread] 3. oiling_place 시작")
                    feedback_msg.feedback_string = "현재 \"오일 내려놓기\" 작업중"
                    feedback_msg.current_stroke = 0
                    g_current_goal_handle.publish_feedback(feedback_msg)
                    if not oiling_place(logger): raise Exception("oiling_place 실패")
                    
                    logger.info("[Task Thread] 전체 오일링 시퀀스 1회 완료.")
                    
                    # (핵심 수정) 'test_a_server.py'와 동일하게 Result 객체 생성
                    end_time = time.time()
                    result = BrushingAction.Result()
                    result.complete_task = True
                    result.total_duration = end_time - start_time
                    result.final_pose, _ = get_current_posx()
                    
                    # (핵심 수정) Result를 전역 변수에 저장
                    g_final_result = result 
                    
                    # (핵심 수정) succeed()는 인자 없이 호출
                    g_current_goal_handle.succeed() 

                except Exception as e:
                    if rclpy.ok():
                        logger.error(f"[Task Thread] 작업 시퀀스 중 예외 발생: {e}")
                    
                    # (핵심 수정) 실패 Result 생성 및 전역 변수에 저장
                    result = BrushingAction.Result()
                    result.complete_task = False
                    g_final_result = result
                    
                    # (핵심 수정) abort()는 인자 없이 호출
                    if g_current_goal_handle and g_current_goal_handle.is_active:
                        g_current_goal_handle.abort()
                
                finally:
                    # (핵심 수정) '작업 스레드'가 종료됨을 'ROS 스레드'에 알림
                    task_running = False
                    trigger_event.clear()
                    g_current_goal_handle = None 
                    logger.info("[Task Thread] 작업 스레드 종료됨 (다음 요청 대기).")
            else:
                node_.get_logger().warn("[Task Thread] 작업이 이미 실행 중. 중복 트리거 무시.")
                trigger_event.clear()
    
    print("[Task Thread] 작업 스레드 루프 종료됨.")


# (핵심 수정) 'test_a_server.py'와 동일하게 goal_callback 추가
def goal_callback(goal_request):
    """(신규) 새 Goal 수락 여부만 결정하는 콜백"""
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

# (핵심 수정) 'test_a_server.py'와 동일하게 execute_callback 수정
def execute_callback(goal_handle):
    """(수정) Goal을 수락한 후, 실제 작업을 실행하고 완료될 때까지 대기"""
    global task_running, trigger_event, g_current_goal_handle, node_, g_final_result
    logger = node_.get_logger()
    logger.info("[ROS Thread] Goal 실행 시작! '작업 스레드'를 트리거합니다.")

    g_current_goal_handle = goal_handle
    trigger_event.set()
    
    # (핵심 수정) '작업 스레드'가 끝날 때까지 (g_current_goal_handle이 None이 될 때까지) 대기
    while g_current_goal_handle is not None and rclpy.ok():
        time.sleep(0.1)

    logger.info("[ROS Thread] Goal 실행 완료. (작업 스레드 종료 확인)")
    
    # (핵심 수정) '작업 스레드'가 g_final_result에 저장해둔 최종 Result 객체를 반환
    return g_final_result


def ros_spin_thread():
    """(유지) 'ROS 스레드'의 메인 루프 (Executor 사용)"""
    global node_
    print("[ROS Thread] ROS Spin 스레드 시작...")
    try:
        executor = MultiThreadedExecutor(num_threads=4) 
        executor.add_node(node_)
        
        # (핵심 수정) 'test_a_server.py'와 동일하게 goal_callback 및 execute_callback 등록
        action_server = ActionServer(
            node_,
            BrushingAction,
            'do_oiling_action', # <-- [중요!] 'do_brushing_action'에서 'do_oiling_action'으로 변경!
            execute_callback=execute_callback,
            goal_callback=goal_callback,
            cancel_callback=None
        )
        print("[ROS Thread] 'do_oiling_action' Action 서버 시작됨.") # <-- 로그도 같이 수정
        
        executor.spin()
    except Exception as e:
        if rclpy.ok():
            node_.get_logger().error(f"[ROS Thread] Spin 중 예외 발생: {e}")
    finally:
        print("[ROS Thread] ROS Spin 스레드 종료됨.")


def main(args=None):
    """(유지) 메인 함수: 스레드 2개 분리 실행"""
    global node_
    
    rclpy.init(args=args)
    
    node_ = rclpy.create_node("oiling_action_server", namespace=ROBOT_ID)
    DR_init.__dsr__node = node_
    
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