#!/usr/bin/env python3
# doma_pick_a_client.py – doma_pick 액션을 1회 요청하는 클라이언트

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from my_robot_interfaces.action import BrushingAction


class DomaPickActionClient(Node):
    """
    'do_doma_pick_action' 서버에 도마 픽킹 작업을 요청하는 액션 클라이언트
    """
    def __init__(self):
        super().__init__('doma_pick_action_client')

        # 서버 쪽 노드가 namespace="dsr01" 이기 때문에,
        # 실제 액션 이름은 /dsr01/do_doma_pick_action 이 됨.
        self._action_client = ActionClient(
            self,
            BrushingAction,
            '/dsr01/do_doma_pick_action'
        )

    def send_goal(self):
        """서버에 Goal을 전송하고 결과를 기다리는 메인 함수"""

        self.get_logger().info("Action 서버(/dsr01/do_doma_pick_action)를 기다리는 중...")
        self._action_client.wait_for_server()

        goal_msg = BrushingAction.Goal()
        goal_msg.start_task = True

        self.get_logger().info("'doma_pick' 작업 시작을 요청합니다...")

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal이 서버에 의해 거부되었습니다.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal이 수락되었습니다. 작업 완료를 기다립니다...')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result

        if result.complete_task:
            self.get_logger().info("doma_pick 작업 완료! (성공)")
            self.get_logger().info(f"  총 소요 시간: {result.total_duration:.2f} 초")
            self.get_logger().info(f"  최종 로봇 위치: {result.final_pose}")
        else:
            self.get_logger().error("doma_pick 작업 실패!")

        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """서버에서 보내는 피드백 출력"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"피드백 수신: {feedback.feedback_string}")


def main(args=None):
    rclpy.init(args=args)
    try:
        action_client = DomaPickActionClient()
        action_client.send_goal()
    except KeyboardInterrupt:
        pass
    finally:
        print("doma_pick Action 클라이언트 종료.")


if __name__ == '__main__':
    main()
