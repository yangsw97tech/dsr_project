#!/usr/bin/env python3
# board_main_coordinator.py
#
# 도마 가공 전체 공정:
#   doma_pick -> erasing -> brushing -> oiling -> doma_place
# 를 하나의 메인 노드에서 순차 실행하거나,
# 각 공정을 개별 실행할 수 있는 ROS2 rclpy 노드.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String

# 공정용(연마/브러싱/오일링) 액션 인터페이스
from my_robot_interfaces.action import BrushingAction

CMD_START_ALL      = "START_ALL"
# (개별 디버깅용 명령)
CMD_START_DOMA_PICK = "START_DOMA_PICK"
CMD_START_ERASING  = "START_ERASING"
CMD_START_BRUSHING = "START_BRUSHING"
CMD_START_OILING   = "START_OILING"
CMD_START_DOMA_PLACE = "START_DOMA_PLACE"


class BoardMainCoordinator(Node):
    """
    GUI 모니터링 노드에서 오는 명령을 받아,
    - doma_pick -> erasing -> brushing -> oiling -> doma_place 전체 공정을 실행하거나
    - 개별 공정만 실행하는 '메인 노드'
    """

    def __init__(self):
        super().__init__('board_main_coordinator')

        # --------- 액션 클라이언트들 ---------
        # 1) Doma pick / place (doma_interfaces 사용)
        self._doma_pick_client = ActionClient(
            self,
            BrushingAction,
            '/dsr01/do_doma_pick_action'
        )
        self._doma_place_client = ActionClient(
            self,
            BrushingAction,
            '/dsr01/do_doma_place_action'
        )

        # 2) Erasing / Brushing / Oiling (my_robot_interfaces 사용)
        self._eraser_client = ActionClient(
            self,
            BrushingAction,
            '/dsr01/do_eraser_action'
        )
        self._brushing_client = ActionClient(
            self,
            BrushingAction,
            '/dsr01/do_brushing_action'
        )
        self._oiling_client = ActionClient(
            self,
            BrushingAction,
            '/dsr01/do_oiling_action'
        )

        # GUI로부터 메인 명령 받는 토픽
        self._cmd_sub = self.create_subscription(
            String,
            '/main_task_cmd',   # GUI에서 여기에 String으로 명령 publish
            self.cmd_callback,
            10
        )

        # [신규] GUI 로그창으로 메시지를 보내기 위한 Publisher
        self._gui_log_pub = self.create_publisher(
            String,
            '/gui_log',         # GUI가 이 토픽을 구독함
            10
        )

        # 상태 변수
        self.busy = False           # 현재 작업 중 여부
        self.pending_command = None # 새로 들어온 명령 (대기 중)

        self.get_logger().info("BoardMainCoordinator 노드가 시작되었습니다.")

    # ------------------------------------------------------------------
    # 헬퍼: GUI로 로그 전송
    # ------------------------------------------------------------------
    def publish_gui_log(self, text: str):
        """GUI 화면의 로그창에 텍스트를 띄우기 위해 토픽 발행"""
        msg = String()
        msg.data = text
        self._gui_log_pub.publish(msg)

    # ------------------------------------------------------------------
    # GUI → 메인노드 명령 콜백
    # ------------------------------------------------------------------
    def cmd_callback(self, msg: String):
        cmd = msg.data.strip().upper()

        if self.busy:
            warn_msg = f"이미 작업 실행 중입니다. 새 명령({cmd})은 무시합니다."
            self.get_logger().warn(warn_msg)
            self.publish_gui_log(f"⚠️ {warn_msg}")
            return

        if cmd not in (CMD_START_ALL, CMD_START_DOMA_PICK, CMD_START_ERASING,
                       CMD_START_BRUSHING, CMD_START_OILING, CMD_START_DOMA_PLACE):
            self.get_logger().warn(f"알 수 없는 명령 수신: {cmd}")
            return

        self.get_logger().info(f"명령 수신: {cmd}")
        self.publish_gui_log(f"📥 명령 수신됨: {cmd}")
        self.pending_command = cmd

    # ------------------------------------------------------------------
    # 액션 호출 공통 함수
    # ------------------------------------------------------------------
    def _run_action_once(self,
                         client: ActionClient,
                         action_label: str,
                         goal_type):
        """
        특정 액션 서버에 Goal 한 번 보내고, 결과를 기다렸다가
        성공/실패를 bool 로 반환.

        goal_type: BrushingAction.Goal
        """
        # 서버 대기
        self.get_logger().info(f"[{action_label}] Action 서버를 기다리는 중...")
        self.publish_gui_log(f"[{action_label}] 서버 연결 대기중...")
        
        if not client.wait_for_server(timeout_sec=10.0):
            err_msg = f"[{action_label}] Action 서버 응답이 없습니다."
            self.get_logger().error(err_msg)
            self.publish_gui_log(f"❌ {err_msg}")
            return False

        # Goal 메시지 생성
        goal_msg = goal_type()
        goal_msg.start_task = True

        self.get_logger().info(f"[{action_label}] 작업 시작을 요청합니다...")
        self.publish_gui_log(f"[{action_label}] 작업 요청 🚀")

        # [핵심] 피드백 콜백 (액션 서버 -> 메인노드 -> GUI 로그)
        def feedback_cb(feedback_msg):
            feedback = feedback_msg.feedback
            # feedback_string은 BrushingAction.action에 정의된 필드
            log_str = f"[{action_label}] 진행중: {feedback.feedback_string}"
            
            # 1. 터미널 출력
            self.get_logger().info(log_str)
            # 2. GUI로 전송
            self.publish_gui_log(log_str)

        # Goal 전송 (비동기)
        send_goal_future = client.send_goal_async(
            goal_msg,
            feedback_callback=feedback_cb
        )

        # Goal 수락/거부까지 기다림
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"[{action_label}] Goal이 거부되었습니다.")
            self.publish_gui_log(f"❌ [{action_label}] 서버가 작업을 거부했습니다.")
            return False

        self.get_logger().info(f"[{action_label}] Goal 수락됨. 결과를 기다립니다...")

        # 결과(Result) 기다리기
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result_wrapper = get_result_future.result()
        if result_wrapper is None:
            self.get_logger().error(f"[{action_label}] 결과 수신 실패.")
            self.publish_gui_log(f"❌ [{action_label}] 결과 수신 실패.")
            return False

        result = result_wrapper.result

        if getattr(result, "complete_task", False):
            total_duration = getattr(result, "total_duration", 0.0)
            # final_pose = getattr(result, "final_pose", [])
            msg = f"[{action_label}] 완료✅ (소요시간: {total_duration:.1f}s)"
            self.get_logger().info(msg)
            self.publish_gui_log(msg)
            return True
        else:
            self.get_logger().error(f"[{action_label}] 작업 실패!")
            self.publish_gui_log(f"❌ [{action_label}] 작업 실패 보고됨.")
            return False

    # ------------------------------------------------------------------
    # 개별 공정 실행 함수
    # ------------------------------------------------------------------
    def run_doma_pick_only(self):
        self.publish_gui_log("=== [디버깅] 도마 Pick 실행 ===")
        ok = self._run_action_once(self._doma_pick_client, "DOMA_PICK", BrushingAction.Goal)
        if not ok: self.publish_gui_log("❌ DOMA_PICK 실패.")

    def run_eraser_only(self):
        self.publish_gui_log("=== [디버깅] 연마(Eraser) 실행 ===")
        ok = self._run_action_once(self._eraser_client, "ERASING", BrushingAction.Goal)
        if not ok: self.publish_gui_log("❌ ERASING 실패.")

    def run_brushing_only(self):
        self.publish_gui_log("=== [디버깅] 브러싱 실행 ===")
        ok = self._run_action_once(self._brushing_client, "BRUSHING", BrushingAction.Goal)
        if not ok: self.publish_gui_log("❌ BRUSHING 실패.")

    def run_oiling_only(self):
        self.publish_gui_log("=== [디버깅] 오일링 실행 ===")
        ok = self._run_action_once(self._oiling_client, "OILING", BrushingAction.Goal)
        if not ok: self.publish_gui_log("❌ OILING 실패.")

    def run_doma_place_only(self):
        self.publish_gui_log("=== [디버깅] 도마 Place 실행 ===")
        ok = self._run_action_once(self._doma_place_client, "DOMA_PLACE", BrushingAction.Goal)
        if not ok: self.publish_gui_log("❌ DOMA_PLACE 실패.")

    # ------------------------------------------------------------------
    # 전체 공정 실행
    # ------------------------------------------------------------------
    def run_all_sequence(self):
        self.publish_gui_log("=== 🏁 전체 공정 시작 (5단계) ===")

        # 1) Doma pick
        if not self._run_action_once(self._doma_pick_client, "DOMA_PICK", BrushingAction.Goal):
            return

        # 2) Erasing
        if not self._run_action_once(self._eraser_client, "ERASING", BrushingAction.Goal):
            return

        # 3) Brushing
        if not self._run_action_once(self._brushing_client, "BRUSHING", BrushingAction.Goal):
            return

        # 4) Oiling
        if not self._run_action_once(self._oiling_client, "OILING", BrushingAction.Goal):
            return

        # 5) Doma place (마무리)
        if not self._run_action_once(self._doma_place_client, "DOMA_PLACE", BrushingAction.Goal):
            return

        self.publish_gui_log("=== 🎉 전체 공정 완료! 수고하셨습니다. ===")

    # ------------------------------------------------------------------
    # 메인 실행 루프
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
                self.busy = True

                try:
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
                    self.publish_gui_log(f"❌ 예외 발생: {e}")
                finally:
                    self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = BoardMainCoordinator()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt: 메인 노드 종료 요청")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()