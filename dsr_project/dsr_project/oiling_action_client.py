#oiling_a_client.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# (유지) BrushingAction 인터페이스를 'my_robot_interfaces'에서 임포트
from my_robot_interfaces.action import BrushingAction

class OilingActionClient(Node):
    """
    (유지) 'do_brushing_action' 서버에 '오일링' 작업을 요청하는 클라이언트
    """
    def __init__(self):
        # (유지) 노드 이름
        super().__init__('oiling_action_client')
        
        # (유지) ActionClient 설정
        self._action_client = ActionClient(
            self,
            BrushingAction,
            '/dsr01/do_oiling_action') # <-- [중요!] 'do_brushing_action'에서 'do_oiling_action'으로 변경!

    def send_goal(self):
        """서버에 Goal을 전송하고 결과를 기다리는 메인 함수"""
        
        self.get_logger().info("Action 서버(/dsr01/do_oiling_action)를 기다리는 중...") # <-- 로그도 같이 수정
        self._action_client.wait_for_server()

        goal_msg = BrushingAction.Goal()
        goal_msg.start_task = True

        # (유지) 로그 메시지
        self.get_logger().info("'Oiling' 작업 시작을 요청합니다...")
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback) # <--- 수정된 feedback_callback을 사용

        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal이 서버에 의해 거부되었습니다.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal이 수락되었습니다. 작업 완료를 기다립니다...')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        # --- ⬇️ 여기를 test_a_client.py와 동일하게 수정 ⬇️ ---
        
        # 1. (수정) status 변수 제거
        result = get_result_future.result().result

        # 2. (수정) result.complete_task를 직접 확인
        if result.complete_task:
            self.get_logger().info(f"작업 완료! (성공)")
            # 3. (수정) "(서버 리포트)" 문구 제거
            self.get_logger().info(f"  총 소요 시간: {result.total_duration:.2f} 초")
            self.get_logger().info(f"  최종 로봇 위치: {result.final_pose}")
        else:
            self.get_logger().error(f"작업 실패!")
            
        # --- ⬆️ 여기까지 수정 ⬆️ ---
            
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """(수정) test_a_client.py와 동일한 피드백 출력 방식"""
        feedback = feedback_msg.feedback
        
        # --- ⬇️ 여기를 test_a_client.py와 동일하게 수정 ⬇️ ---
        # 4. (수정) current_stroke 제거, '작업=' 등 제거
        self.get_logger().info(f"피드백 수신: {feedback.feedback_string}")
        # --- ⬆️ 여기까지 수정 ⬆️ ---


def main(args=None):
    rclpy.init(args=args)
    try:
        action_client = OilingActionClient()
        action_client.send_goal()
    except KeyboardInterrupt:
        pass
    finally:
        print("Action 클라이언트 종료.")

if __name__ == '__main__':
    main()