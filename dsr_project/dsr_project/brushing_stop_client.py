# brushing_action_client.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import BrushingAction

class BrushingActionClient(Node):
    def __init__(self):
        super().__init__('brushing_action_client')
        
        # [핵심] Action Server 이름 확인 (/dsr01/do_brushing_action)
        self._action_client = ActionClient(
            self, 
            BrushingAction, 
            '/dsr01/do_brushing_action'
        )

    def send_goal(self):
        self.get_logger().info("Waiting for Action Server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action Server not available!")
            return

        goal_msg = BrushingAction.Goal()
        goal_msg.start_task = True

        self.get_logger().info("Sending Goal (Start Brushing)...")
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal Rejected :(')
            return

        self.get_logger().info('Goal Accepted! Task Started.')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 진행 상황 출력
        self.get_logger().info(f"[Feedback] Stroke: {feedback.current_stroke} | Msg: {feedback.feedback_string}")

    def get_result_callback(self, future):
        result = future.result().result
        
        if result.complete_task:
            self.get_logger().info(f"=== Task Completed Successfully! ===")
            self.get_logger().info(f"Total Duration: {result.total_duration:.2f} sec")
            self.get_logger().info(f"Final Pose: {result.final_pose}")
        else:
            self.get_logger().error("=== Task Failed or Aborted ===")
            
        # 작업 끝나면 노드 종료
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = BrushingActionClient()
    
    action_client.send_goal()
    
    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass # rclpy.shutdown() 호출 시 정상 종료 처리
    finally:
        print("Client Node Finished.")

if __name__ == '__main__':
    main()