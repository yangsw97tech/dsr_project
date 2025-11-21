#!/usr/bin/env python3
import rclpy
# import DR_init  <--- [제거] (main에서 임포트)
import time
import math
import threading # <--- 멀티스레드용 잠금 임포트

from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from my_robot_interfaces.action import BrushingStatus # <--- BrushingStatus

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"
ROBOT_TOOL = "GripperDA_v1"
VELOCITY = 60
ACC = 60

# <--- [제거] DR_init 글로벌 설정


class BrushingActionServer(Node):
    """ BrushingStatus 인터페이스, 멀티스레드 Executor 기반 """

    def __init__(self):
        super().__init__("brushing_action_server", namespace=ROBOT_ID)
        
        self.state = "RUNNING"  # RUNNING / PAUSED
        self._state_lock = threading.Lock() # <--- self.state 보호용 잠금
        
        # <--- [제거] DR_init.__dsr__node = self
        
        # <추가> 로봇 초기화 플래그
        self._robot_initialized = False

        # STOP/RESUME 토픽 구독
        self.sub_control = self.create_subscription(
            String,
            "/arm_control",
            self.control_callback,
            10
        )
        
        self._action_server = ActionServer(
            self,
            BrushingStatus,
            'brushing_task',
            self.execute_callback
        )
        
        self.get_logger().info(f"'{ROBOT_ID}/brushing_task' 액션 서버 준비됨 (멀티스레드).")

    def initialize_robot(self):
        """로봇의 Tool과 TCP를 설정 (execute_callback에서 1회 호출됨)"""
        if self._robot_initialized:
            return
            
        self.get_logger().info("로봇 초기화 (set_tool, set_tcp) 시작...")
        # 이 시점에는 main에서 DR_init.node가 설정되었으므로 임포트가 안전함
        from DSR_ROBOT2 import set_tool, set_tcp

        set_tool(ROBOT_TCP)
        set_tcp(ROBOT_TOOL)
        self._robot_initialized = True
        
        self.get_logger().info("Initializing robot with the following settings:")
        self.get_logger().info(f"ROBOT_ID: {ROBOT_ID}")
        self.get_logger().info(f"ROBOT_MODEL: {ROBOT_MODEL}")
        self.get_logger().info(f"ROBOT_TCP: {ROBOT_TCP}")
        self.get_logger().info(f"ROBOT_TOOL: {ROBOT_TOOL}")
        self.get_logger().info("#" * 50)
        self.get_logger().info("로봇 초기화 완료.")

    def control_callback(self, msg: String):
        """STOP / RESUME 명령을 받는 콜백 (Thread-safe)"""
        cmd = msg.data.upper().strip()
        self.get_logger().info(f"[ARM CONTROL] 수신: {cmd}")

        # <중요> 스레드 잠금으로 self.state 보호
        with self._state_lock:
            if cmd == "STOP":
                if self.state != "PAUSED":
                    self.get_logger().warn(">>> STOP 수신 → STATE=PAUSED")
                self.state = "PAUSED"
            elif cmd == "RESUME":
                if self.state != "RUNNING":
                    self.get_logger().info(">>> RESUME 수신 → STATE=RUNNING")
                self.state = "RUNNING"
            else:
                self.get_logger().warn(f">>> [ARM CONTROL] 알 수 없는 명령: {cmd}")

    def wait_until_running(self):
        """STATE가 RUNNING 될 때까지 대기 (Thread-safe, busy-wait 방지)"""
        
        # 현재 상태를 안전하게 확인
        with self._state_lock:
            is_paused = (self.state == "PAUSED")

        if is_paused:
            self.get_logger().info("...동작 일시중지 (PAUSED). RESUME 대기 중...")

        while rclpy.ok():
            with self._state_lock:
                if self.state == "RUNNING":
                    break
            
            # 멀티스레드: spin_once() 대신 time.sleep() 사용
            time.sleep(0.01) 
        
        return True

    def execute_callback(self, goal_handle):
        """액션 Goal을 실행하는 메인 콜백."""
        
        # <수정> 로봇 초기화를 여기서 실행 (데드락 방지)
        if not self._robot_initialized:
            try:
                self.get_logger().info("첫 Goal 수신. 로봇 초기화를 시도합니다...")
                self.initialize_robot() # set_tool, set_tcp 호출
            except Exception as e:
                self.get_logger().error(f"로봇 초기화 실패: {e}")
                goal_handle.abort()
                return BrushingStatus.Result(complete_task=False)

        self.get_logger().info(f"액션 Goal 수신: start_task={goal_handle.request.start_task}")

        if not goal_handle.request.start_task:
            goal_handle.abort()
            return BrushingStatus.Result(complete_task=False)

        start_time = time.time()
        result = BrushingStatus.Result()

        try:
            # --- (DSR_ROBOT2 임포트) ---
            from DSR_ROBOT2 import (
                release_compliance_ctrl, release_force, check_force_condition,
                task_compliance_ctrl, set_desired_force, set_ref_coord,
                get_digital_input, get_digital_output, set_digital_output,
                movej, movel, get_current_posx, wait, set_user_cart_coord,
                DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE,
                DR_FC_MOD_REL, DR_AXIS_Z, DR_BASE, DR_TOOL, OFF, ON, posx,
            )

            # (이하 싱글스레드 버전과 로직 동일)
            # gripper 조절
            def gripper(switch):
                if switch == 0:   # 닫기
                    set_digital_output(1, ON)
                    set_digital_output(2, OFF)
                elif switch == 1: # 열기
                    set_digital_output(1, OFF)
                    set_digital_output(2, ON)

            # -------------------- pick_e --------------------
            self.wait_until_running() 
            movel(posx(325.82, 267.29, 424.88, 82.80, 175.80, 80.38), vel=VELOCITY, t=2, acc=ACC)
            self.wait_until_running() 
            gripper(1); wait(1.00)
            self.wait_until_running() 
            movel(posx(325.82, 267.29, 310.78, 82.80, 175.80, 80.38), vel=VELOCITY, t=2, acc=ACC)
            self.wait_until_running() 
            gripper(0); wait(1.00)
            self.wait_until_running() 
            movel(posx(325.82, 267.29, 424.88, 82.80, 175.80, 80.38), vel=VELOCITY, t=2, acc=ACC)
            
            # --- Yaw (Rz) 2.3도 회전 적용 ---
            p1 = [324.380, 67.386, 330.540]; p2 = [608.089, 65.371, 330.540]
            p4 = [326.180, -117.220, 330.540]; p_origin = [326.180, -117.220, 330.540, 157.52, 180, 167.8]
            list_x = [p2[i]-p1[i] for i in range(3)]; A = math.sqrt(sum(i**2 for i in list_x))
            list_X = [i/A for i in list_x]
            list_y = [p1[i]-p4[i] for i in range(3)]; B = math.sqrt(sum(i**2 for i in list_y))
            list_Y = [i/B for i in list_y]
            pos = p_origin
            DR_USER1 = set_user_cart_coord(list_X, list_Y, pos, ref=DR_BASE)

            board_h = 214; board_w = 300
            sponge_h = 23; sponge_w = 23
            num_strokes = math.ceil(board_h / sponge_h); total_overlap = (sponge_h * num_strokes) - board_h
            overlap_count = num_strokes - 1
            if overlap_count <= 0: y_step_size = board_h
            else: y_step_size = sponge_h - (total_overlap / overlap_count)
            self.get_logger().info(f"계산된 붓질 횟수: {num_strokes}회, Y간격: {y_step_size:.1f}mm")
            move_x = board_w - 8
            vel_x = 50; acc_x = 50; vel_y = 50; acc_y = 50; vel_z = 30; acc_z = 30

            # 초기 위치로 이동
            init_pos = posx([331.79, 65.310, 350.540, 13.71, 180.0, 12.62])
            self.wait_until_running() 
            movel(init_pos, vel=30, t=2, acc=30, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            self.get_logger().info('초기 위치로 이동됨')

            # 지우개 다시 잡기 
            self.wait_until_running(); movel([0, 0, -20, 0, 0, 0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
            gripper(1); wait(1.00)
            self.wait_until_running(); movel([0, 0, -10, 0, 0, 0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
            gripper(0); wait(1.00)
            self.wait_until_running(); movel([0, 0, +10, 0, 0, 0], vel=vel_z, acc=acc_z, ref=DR_BASE, mod=DR_MV_MOD_REL)
            self.get_logger().info('지우개 재정렬 완료')

            set_ref_coord(DR_USER1)

            def oiling_move():
                for y_move_cnt in range(int(num_strokes)):
                    feedback_msg = BrushingStatus.Feedback()
                    feedback_msg.current_stroke = y_move_cnt + 1
                    goal_handle.publish_feedback(feedback_msg)
                    self.get_logger().info(f"피드백 전송: {y_move_cnt + 1}번째 붓질 시작")

                    if y_move_cnt % 2 == 0:
                        self.wait_until_running(); movel([move_x, 0, 0, 0, 0, 0], vel=vel_x, t=2, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        self.wait_until_running(); movel([-move_x, 0, 0, 0, 0, 0], vel=vel_x, t=2, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        self.wait_until_running(); movel([move_x, 0, 0, 0, 0, 0], vel=vel_x, t=2, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                    else:
                        self.wait_until_running(); movel([-move_x, 0, 0, 0, 0, 0], vel=vel_x, t=2, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        self.wait_until_running(); movel([move_x, 0, 0, 0, 0, 0], vel=vel_x, t=2, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        self.wait_until_running(); movel([-move_x, 0, 0, 0, 0, 0], vel=vel_x, t=2, acc=acc_x, ref=DR_USER1, mod=DR_MV_MOD_REL)

                    self.get_logger().info(f'{y_move_cnt + 1}번째 붓질 완료')

                    if y_move_cnt < num_strokes - 1:
                        self.wait_until_running(); movel([0, 0, +10, 0, 0, 0], vel=vel_y, t=1, acc=acc_y, ref=DR_USER1, mod=DR_MV_MOD_REL)
                        self.wait_until_running(); movel([0, -y_step_size, 0, 0, 0, 0], vel=vel_y, t=1, acc=acc_y, ref=DR_USER1, mod=DR_MV_MOD_REL)
                    else:
                        self.get_logger().info('최종 Y 위치 도달')
                
                self.get_logger().info('oiling 작업 완료')
                self.wait_until_running(); movel([0, 0, 100, 0, 0, 0], vel=vel_z, acc=acc_z, ref=DR_USER1, mod=DR_MV_MOD_REL)

            oiling_move()

            # -------------------- place_e --------------------
            self.wait_until_running(); movel(posx(330.44, 289.61, 400.54, 64.35, -180.00, 63.26), vel=VELOCITY, t=2, acc=ACC)
            self.wait_until_running(); movel(posx(0.00, 0.00, -85.26, 0.00, 0.00, 0.00), vel=VELOCITY, t=2, acc=ACC, mod=DR_MV_MOD_REL)
            gripper(1); self.get_logger().info("=== 작업 완료 ===")

            # --- [액션 결과 전송] ---
            end_time = time.time()
            final_pose_tuple = get_current_posx() 
            
            result.complete_task = True
            result.final_pose = list(final_pose_tuple[0]) # <--- 수정: [0] (pos)만 사용
            result.total_duration = end_time - start_time
            
            goal_handle.succeed()
            self.get_logger().info("액션 성공 (Succeeded)")

        except Exception as e:
            self.get_logger().error(f"액션 실행 중 예외 발생: {e}")
            goal_handle.abort()
            result.complete_task = False
            result.total_duration = time.time() - start_time
            result.final_pose = [0.0] * 6 

        return result


def main(args=None):
    """메인 함수: 멀티스레드 스핀 (안전한 초기화)"""
    rclpy.init(args=args)
    
    # 1. 노드 객체 생성
    action_server_node = BrushingActionServer()

    # 2. <--- [핵심 수정] ---
    # 노드 생성 *이후*에 DR_init을 임포트하고 모든 설정을 수행합니다.
    try:
        import DR_init
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = action_server_node # 가장 중요
        
        action_server_node.get_logger().info(f"DR_init 임포트 및 노드 설정 완료.")
        
    except ImportError:
        action_server_node.get_logger().error("DR_init 모듈을 찾을 수 없습니다. 종료합니다.")
        rclpy.shutdown()
        return
    except Exception as e:
        action_server_node.get_logger().error(f"DR_init 설정 중 오류 발생: {e}")
        rclpy.shutdown()
        return

    # 3. Executor 생성 및 노드 추가
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(action_server_node)
    
    action_server_node.get_logger().info("멀티스레드 Executor로 스핀 시작...")

    try:
        # 4. 스핀 시작 (initialize_robot은 Goal 수신 시 호출됨)
        executor.spin()

    except KeyboardInterrupt:
        action_server_node.get_logger().info("사용자에 의해 노드 종료")
    except Exception as e:
        action_server_node.get_logger().error(f"스핀 중 예외 발생: {e}")
    finally:
        action_server_node.get_logger().info("Executor 종료 및 노드 파괴 중...")
        executor.shutdown()
        action_server_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()