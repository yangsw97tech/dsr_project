#Action_client.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# 1. (수정) Action 인터페이스 임포트 (패키지명 가정)
from my_robot_interfaces.action import BrushingAction

class BrushingActionClient(Node):
    """
    (신규) 'do_brushing_action' Action을 요청하는 클라이언트 노드
    """
    def __init__(self):
        super().__init__('brushing_action_client')
        
        # 3. (수정) Publisher 대신 ActionClient 생성
        self._action_client = ActionClient(
            self,
            BrushingAction,
            '/dsr01/do_brushing_action') # (중요) 서버의 네임스페이스와 Action 이름을 정확히 기입

    def send_goal(self):
        """서버에 Goal을 전송하고 결과를 기다리는 메인 함수"""
        
        self.get_logger().info("Action 서버를 기다리는 중...")
        # 3. (수정) Action 서버가 켜질 때까지 대기
        self._action_client.wait_for_server()

        # 3. (수정) Goal 메시지 생성
        goal_msg = BrushingAction.Goal()
        goal_msg.start_task = True

        self.get_logger().info("'brushing_final' 작업 시작을 요청합니다...")
        
        # 3. (수정) Goal 전송 (비동기)
        # 4. (수정) Feedback 콜백 등록
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # Goal이 수락/거부될 때까지 대기
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal이 서버에 의해 거부되었습니다.')
            return

        self.get_logger().info('Goal이 수락되었습니다. 작업 완료를 기다립니다...')

        # 3. (수정) 최종 결과(Result)를 기다림
        get_result_future = goal_handle.get_result_async()
        
        # 4. (수정) 결과가 올 때까지 spin하며 대기 (피드백도 이 때 수신됨)
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result

        # 4. (수정) 최종 결과 출력
        if result.complete_task:
            self.get_logger().info(f"작업 완료! (성공)")
            self.get_logger().info(f"  총 소요 시간: {result.total_duration:.2f} 초")
            self.get_logger().info(f"  최종 로봇 위치: {result.final_pose}")
        else:
            self.get_logger().error(f"작업 실패!")
            
        rclpy.shutdown() # 작업 완료 후 종료

    def feedback_callback(self, feedback_msg):
        """
    (신규) 서버로부터 Feedback을 받을 때마다 호출되는 콜백
        """
        # 4. (수정) 피드백(중간 보고) 출력
        
        # --- ⬇️ 여기를 수정했습니다 ⬇️ ---
        # 서버가 보낸 문자열을 그대로 출력
        feedback = feedback_msg.feedback
        self.get_logger().info(f"피드백 수신: {feedback.feedback_string}")
        # --- ⬆️ 여기까지 수정 ⬆️ ---


def main(args=None):
    rclpy.init(args=args)
    try:
        action_client = BrushingActionClient()
        
        # 3. (수정) Goal 전송 함수 호출
        action_client.send_goal()
        
    except KeyboardInterrupt:
        pass
    finally:
        print("Action 클라이언트 종료.")

if __name__ == '__main__':
    main()