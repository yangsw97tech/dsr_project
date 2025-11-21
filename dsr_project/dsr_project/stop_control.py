# test_control.py
import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

# 로봇 ID (서버와 일치해야 함)
ROBOT_ID = "dsr01"

class ControlPanel(Node):
    def __init__(self):
        super().__init__('control_panel_node')
        
        # [핵심 수정] 서비스 경로에 네임스페이스(dsr01) 포함
        # Server가 'dsr01' 네임스페이스 안에 있으므로 서비스명은 '/dsr01/custom_pause'가 됨
        service_name = f'/{ROBOT_ID}/custom_pause'
        
        self.cli_pause = self.create_client(SetBool, service_name)
        
        self.get_logger().info(f"Waiting for service: {service_name} ...")
        
        # 서비스 서버가 뜰 때까지 대기 (1초 간격 확인)
        while not self.cli_pause.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Service '{service_name}' not available, waiting...")
            
        self.get_logger().info("\n=========================================")
        self.get_logger().info(f" [Control Panel Connected to {ROBOT_ID}]")
        self.get_logger().info("  p + Enter : PAUSE  (일시 정지)")
        self.get_logger().info("  r + Enter : RESUME (작업 재개)")
        self.get_logger().info("  q + Enter : QUIT   (종료)")
        self.get_logger().info("=========================================\n")

    def send_command(self, is_pause):
        req = SetBool.Request()
        req.data = is_pause  # True=Pause, False=Resume
        
        # 비동기 호출
        future = self.cli_pause.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ Success: {response.message}")
            else:
                self.get_logger().warn(f"❌ Failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main():
    rclpy.init()
    node = ControlPanel()

    try:
        while rclpy.ok():
            # 사용자 입력 대기
            cmd = input("Command (p/r/q) > ").strip().lower()
            
            if cmd == 'p':
                node.send_command(True)  # PAUSE
            elif cmd == 'r':
                node.send_command(False) # RESUME
            elif cmd == 'q':
                print("Exiting Control Panel...")
                break
            else:
                print("⚠️ Unknown command. Please use 'p', 'r', or 'q'.")
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()