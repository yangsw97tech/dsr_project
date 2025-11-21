#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

# 서버와 동일한 액션 인터페이스를 임포트
from my_robot_interfaces.action import BrushingStatus


class BrushingActionClient(Node):

    def __init__(self):
        super().__init__('brushing_action_client')
        
        # 액션 클라이언트 생성
        self._action_client = ActionClient(
            self,
            BrushingStatus,
            '/dsr01/brushing_task'  # <--- 서버의 네임스페이스와 액션 이름
        )
        self.get_logger().info("액션 클라이언트 노드 시작됨")

    def send_goal(self):
        """액션 서버에 Goal을 전송합니다."""
        
        self.get_logger().info("액션 서버를 기다리는 중...")
        self._action_client.wait_for_server()

        goal_msg = BrushingStatus.Goal()
        goal_msg.start_task = True  # <--- Goal 내용 설정

        self.get_logger().info(f"Goal 전송: start_task = {goal_msg.start_task}")

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """서버로부터 피드백을 수신할 때마다 호출됩니다."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'>>> 피드백 수신: {feedback.current_stroke} 번째 붓질 진행 중...'
        )

    def goal_response_callback(self, future):
        """Goal 전송에 대한 서버의 응답(수락/거부)을 처리합니다."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('!!! Goal이 서버에 의해 거부되었습니다.')
            rclpy.shutdown() # Goal 거부 시 종료
            return

        self.get_logger().info('>>> Goal이 서버에 의해 수락되었습니다.')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """서버로부터 최종 결과를 수신했을 때 호출됩니다."""
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('=' * 30)
            self.get_logger().info('✅ 액션 성공 (SUCCEEDED)')
            self.get_logger().info(f'  - 작업 완료: {result.complete_task}')
            self.get_logger().info(f'  - 최종 자세: {list(result.final_pose)}')
            self.get_logger().info(f'  - 총 소요 시간: {result.total_duration:.2f} 초')
            self.get_logger().info('=' * 30)
        
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('=' * 30)
            self.get_logger().error('❌ 액션 중단됨 (ABORTED)')
            self.get_logger().error(f'  - 작업 완료: {result.complete_task}')
            self.get_logger().error(f'  - 총 소요 시간: {result.total_duration:.2f} 초')
            self.get_logger().error('=' * 30)
        
        else:
            self.get_logger().warn(f'액션 상태: {status}')

        self.get_logger().info("클라이언트를 종료합니다.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = BrushingActionClient()
    action_client.send_goal()

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info("클라이언트 종료 (KeyboardInterrupt)")
    finally:
        action_client.destroy_node()
        # (get_result_callback에서 shutdown()이 호출되므로 여기서 또 호출할 필요 없음)

if __name__ == '__main__':
    main()