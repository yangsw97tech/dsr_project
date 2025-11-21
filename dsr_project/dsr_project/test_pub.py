#test_pub.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class TriggerPublisherNode(Node):
    def __init__(self):
        super().__init__('trigger_publisher_node')
        self.publisher_ = self.create_publisher(Bool, '/trigger_move', 10)
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        msg = Bool()
        msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().info(f"발행 완료: {msg.data}")
        self.timer.cancel() #한번 발행하고 나면 종료
        time.sleep(0.5) 
        self.get_logger().info("발행 완료. 노드를 종료.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    try:
        trigger_publisher = TriggerPublisherNode()
        rclpy.spin(trigger_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
             trigger_publisher.destroy_node()
        print("Publisher 노드 종료.")

if __name__ == '__main__':
    main()