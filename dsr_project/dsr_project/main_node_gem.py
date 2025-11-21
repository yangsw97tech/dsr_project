#!/usr/bin/env python3
# board_main_coordinator.py (수정 버전)
#
# 도마 가공 전체 공정(Pick -> Erasing -> Brushing -> Oiling -> Place)을
# 하나의 메인 노드에서 순차 실행하거나,
# 각 공정을 개별 실행할 수 있는 ROS2 rclpy 노드.
#
# (제공받은 코드를 기반으로 5단계 시퀀스로 확장)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String

from my_robot_interfaces.action import BrushingAction   # 모든 서버가 이 액션 타입을 재사용

# --- 실행 명령 상수 정의 ---
CMD_START_ALL = "START_ALL"
# (개별 디버깅용 명령)
CMD_START_DOMA_PICK = "START_DOMA_PICK"    # (신규 추가)
CMD_START_ERASING = "START_ERASING"
CMD_START_BRUSHING = "START_BRUSHING"
CMD_START_OILING = "START_OILING"
CMD_START_DOMA_PLACE = "START_DOMA_PLACE"  # (신규 추가)


class BoardMainCoordinator(Node):
    """
    GUI 모니터링 노드에서 오는 명령을 받아,
    - Pick → Erasing → Brushing → Oiling → Place 전체 공정을 실행하거나
    - 개별 공정만 실행하는 '메인 노드'
    """

    def __init__(self):
        super().__init__('board_main_coordinator')

        # --- 총 5개의 액션 서버에 연결할 클라이언트 ---
        
        # 1. 도마 Pick (신규 추가)
        self._doma_pick_client = ActionClient(
            self,
            BrushingAction,
            '/dsr01/do_doma_pick_action' # Doma_a_pick_server.py 에서 정의된 이름
        )
        
        # 2. 연마 (Erasing)
        self._eraser_client = ActionClient(
            self,
            BrushingAction,
            '/dsr01/do_eraser_action'
        )
        
        # 3. 브러싱 (Brushing)
        self._brushing_client = ActionClient(
            self,
            BrushingAction,
            '/dsr01/do_brushing_action'
        )
        
        # 4. 오일링 (Oiling)
        self._oiling_client = ActionClient(
            self,
            BrushingAction,
            '/dsr01/do_oiling_action'
        )
        
        # 5. 도마 Place (신규 추가)
        self._doma_place_client = ActionClient(
            self,
            BrushingAction,
            '/dsr01/do_doma_place_action' # Doma_a_place_server.py 에서 정의된 이름
        )

        # GUI로부터 메인 명령 받는 토픽 (토픽 이름은 GUI와 일치시켜야 함)
        self._cmd_sub = self.create_subscription(
            String,
            '/main_task_cmd',   # GUI에서 여기에 String으로 명령 publish
            self.cmd_callback,
            10
        )

        # 상태 변수 (제공된 구조)
        self.busy = False           # 현재 작업 중 여부
        self.pending_command = None # 새로 들어온 명령 (대기 중)

        self.get_logger().info("--- BoardMainCoordinator 노드 (v2.0) 시작 ---")
        self.get_logger().info(f"'/main_task_cmd' 토픽으로 명령 대기 중...")
        self.get_logger().info(f"사용 가능 명령: {CMD_START_ALL}, {CMD_START_DOMA_PICK}, {CMD_START_ERASING}, ...")

    # ------------------------------------------------------------------
    # GUI → 메인노드 명령 콜백
    # ------------------------------------------------------------------
    def cmd_callback(self, msg: String):
        cmd = msg.data.strip().upper()

        if self.busy:
            self.get_logger().warn(
                f"이미 작업 실행 중입니다. 새 명령({cmd})은 무시합니다."
            )
            return

        # (수정) 신규 명령 포함
        if cmd not in (CMD_START_ALL, 
                       CMD_START_DOMA_PICK, CMD_START_ERASING,
                       CMD_START_BRUSHING, CMD_START_OILING,
                       CMD_START_DOMA_PLACE):
            self.get_logger().warn(f"알 수 없는 명령 수신: {cmd}")
            return

        self.get_logger().info(f"명령 수신: {cmd}")
        self.pending_command = cmd

    # ------------------------------------------------------------------
    # 액션 호출 공통 함수 (수정 불필요 - 완벽한 함수)
    # ------------------------------------------------------------------
    def _run_action_once(self, client: ActionClient, action_label: str) -> bool:
        """
        특정 액션 서버에 Goal 한 번 보내고, 결과를 기다렸다가
        성공/실패를 bool 로 반환.
        """
        # 서버 대기
        self.get_logger().info(f"[{action_label}] Action 서버를 기다리는 중...")
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"[{action_label}] Action 서버 응답이 없습니다.")
            return False

        # Goal 메시지 생성
        goal_msg = BrushingAction.Goal()
        goal_msg.start_task = True

        self.get_logger().info(f"[{action_label}] 작업 시작을 요청합니다...")

        # 피드백 콜백 (각 작업의 피드백을 동일한 콜백으로 로깅)
        def feedback_cb(feedback_msg):
            feedback = feedback_msg.feedback
            self.get_logger().info(
                f"[{action_label}] 피드백: {feedback.feedback_string}"
            )

        # Goal 전송 (비동기)
        send_goal_future = client.send_goal_async(
            goal_msg,
            feedback_callback=feedback_cb
        )

        # Goal 수락/거부까지 기다림
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"[{action_label}] Goal이 거부되었습니다.")
            return False

        self.get_logger().info(f"[{action_label}] Goal 수락됨. 결과를 기다립니다...")

        # 결과(Result) 기다리기
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result_response = get_result_future.result()
        if not result_response:
             self.get_logger().error(f"[{action_label}] 결과 수신 실패 (null response).")
             return False
             
        result = result_response.result

        if result and result.complete_task:
            self.get_logger().info(
                f"[{action_label}] 작업 완료(성공). "
                f"총 소요 시간: {result.total_duration:.2f} 초, "
                f"최종 포즈: {result.final_pose}"
            )
            return True
        else:
            self.get_logger().error(f"[{action_label}] 작업 실패!")
            return False

    # ------------------------------------------------------------------
    # 각 공정 개별 실행 함수 (디버깅용)
    # ------------------------------------------------------------------
    
    # (신규 추가)
    def run_doma_pick_only(self):
        self.get_logger().info("=== [디버깅] 도마 Pick 작업만 실행 ===")
        ok = self._run_action_once(self._doma_pick_client, "DOMA_PICK")
        if not ok:
            self.get_logger().error("도마 Pick 작업 중단/실패.")
        self.get_logger().info("=== 도마 Pick 작업 종료 ===")

    def run_eraser_only(self):
        self.get_logger().info("=== [디버깅] 연마 작업만 실행 ===")
        ok = self._run_action_once(self._eraser_client, "ERASING")
        if not ok:
            self.get_logger().error("연마 작업 중단/실패.")
        self.get_logger().info("=== 연마 작업 종료 ===")

    def run_brushing_only(self):
        self.get_logger().info("=== [디버깅] 브러싱 작업만 실행 ===")
        ok = self._run_action_once(self._brushing_client, "BRUSHING")
        if not ok:
            self.get_logger().error("브러싱 작업 중단/실패.")
        self.get_logger().info("=== 브러싱 작업 종료 ===")

    def run_oiling_only(self):
        self.get_logger().info("=== [디버깅] 오일링 작업만 실행 ===")
        ok = self._run_action_once(self._oiling_client, "OILING")
        if not ok:
            self.get_logger().error("오일링 작업 중단/실패.")
        self.get_logger().info("=== 오일링 작업 종료 ===")

    # (신규 추가)
    def run_doma_place_only(self):
        self.get_logger().info("=== [디버깅] 도마 Place 작업만 실행 ===")
        ok = self._run_action_once(self._doma_place_client, "DOMA_PLACE")
        if not ok:
            self.get_logger().error("도마 Place 작업 중단/실패.")
        self.get_logger().info("=== 도마 Place 작업 종료 ===")

    # ------------------------------------------------------------------
    # (핵심 수정) 전체 공정 실행 (Pick → Erasing → Brushing → Oiling → Place)
    # ------------------------------------------------------------------
    def run_all_sequence(self):
        self.get_logger().info("=== 전체 공정 시작: PICK → ERASING → BRUSHING → OILING → PLACE ===")

        # 1. 도마 Pick
        if not self._run_action_once(self._doma_pick_client, "DOMA_PICK"):
            self.get_logger().error("DOMA_PICK 실패. 전체 공정을 중단합니다.")
            return

        # 2. 연마 (Erasing)
        if not self._run_action_once(self._eraser_client, "ERASING"):
            self.get_logger().error("ERASING 실패. 전체 공정을 중단합니다.")
            return

        # 3. 브러싱 (Brushing)
        if not self._run_action_once(self._brushing_client, "BRUSHING"):
            self.get_logger().error("BRUSHING 실패. 전체 공정을 중단합니다.")
            return

        # 4. 오일링 (Oiling)
        if not self._run_action_once(self._oiling_client, "OILING"):
            self.get_logger().error("OILING 실패. 전체 공정을 중단합니다.")
            return

        # 5. 도마 Place
        if not self._run_action_once(self._doma_place_client, "DOMA_PLACE"):
            self.get_logger().error("DOMA_PLACE 실패.")
            return

        self.get_logger().info("=== ✅ 전체 공정 (5단계) 모두 성공적으로 완료 ===")

    # ------------------------------------------------------------------
    # 메인 실행 루프 (제공된 구조)
    # ------------------------------------------------------------------
    def run(self):
        """
        rclpy.spin() 대신, 직접 spin_once + 명령 처리 루프를 돌리는 구조.
        """
        while rclpy.ok():
            # GUI에서 오는 토픽 콜백 처리
            rclpy.spin_once(self, timeout_sec=0.1)

            # 처리할 명령이 있고, 아직 작업 중이 아니라면 실행
            if not self.busy and self.pending_command is not None:
                cmd = self.pending_command
                self.pending_command = None
                self.busy = True # (상태 변경) 작업 시작

                try:
                    # (수정) 명령 분기 처리
                    if cmd == CMD_START_ALL:
                        self.run_all_sequence()
                    elif cmd == CMD_START_DOMA_PICK:
                        self.run_doma_pick_only()
                    elif cmd == CMD_START_ERASING:
                        self.run_eraser_only()
                    elif cmd == CMD_START_BRUSHING:
                        self.run_brushing_only()
                    elif cmd == CMD_START_OILING:
                        self.run_oiling_only()
                    elif cmd == CMD_START_DOMA_PLACE:
                        self.run_doma_place_only()
                    else:
                        self.get_logger().warn(f"처리할 수 없는 명령: {cmd}")
                except Exception as e:
                    self.get_logger().error(f"명령 실행 중 예외 발생: {e}")
                finally:
                    self.busy = False # (상태 변경) 작업 완료


def main(args=None):
    rclpy.init(args=args)
    node = BoardMainCoordinator()
    try:
        node.run() # spin() 대신 run() 호출
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt: 메인 노드 종료 요청")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()