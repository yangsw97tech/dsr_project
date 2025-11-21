import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # 이미지 메시지 타입
import cv2  # OpenCV 라이브러리
from cv_bridge import CvBridge  # ROS와 OpenCV 이미지 변환
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # 1. 퍼블리셔 생성 -> 토픽 이름 'image_raw', 메시지 타입 'Image'
        self.publisher_ = self.create_publisher(Image,'/camera/image_raw', 10)
        
        # 2. 타이머 생성 (30fps -> 1/30 = 0.033초)(45fps -> 0.022초)(60fps -> 0.016초)
        timer_period = 0.016
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 3. OpenCV 웹캠 캡처 초기화
        self.cap = cv2.VideoCapture(0)  # 0번 카메라
        # 웹캠 카메라가 안잡혔을때 오류처리
        if not self.cap.isOpened():
            self.get_logger().error('Could not open video stream')
            raise RuntimeError('Could not open video stream')


        # 3-1. 해상도 낮추기(하드웨어에서 안해주면 사용 불가)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


        # 4. CvBridge 초기화[img와 cv2간의 데이터 변환 담당]
        self.bridge = CvBridge()
        self.get_logger().info('Camera publisher node started.')

    def timer_callback(self):
        # 5. 웹캠에서 프레임 읽기
        ret, frame = self.cap.read()
        #프레임이 읽히면 이미지를 메시지로 변환하고 publish
        if ret:
            # 6. OpenCV 이미지를 ROS Image 메시지로 변환
            # "bgr8": OpenCV의 기본 인코딩 (Blue, Green, Red, 8-bit)
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            
            # (선택 사항) 타임스탬프 설정 -> 이미지 찍히는 시간도 같이 발행하고 싶을 시 사용 -> builtin_interfaces/msg/Time 메시지 형태
            #ros_image.header.stamp = self.get_clock().now().to_msg()
            
            # 7. 이미지 메시지 발행
            self.publisher_.publish(ros_image)
        #프레인 안읽혔을 때 오류 문구
        else:
            self.get_logger().warn('Failed to capture frame')

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # 8. 노드 종료 시 자원 해제
        camera_publisher.cap.release()
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()