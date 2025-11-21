'''
==사전 준비==
sudo apt install ros-humble-cv-bridge ros-humble-image-transport python3-opencv #필요 패키지 설치(터미널)
python3 -c "import cv2; print(cv2.__version__)" #확인

아래 코드에 대한 setup.py 등록(entry_point)
빌드 및 실행

ros2 run image_tools cam2image

'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # 이미지 메시지 타입
import cv2  # OpenCV 라이브러리
from cv_bridge import CvBridge  # ROS와 OpenCV 이미지 변환

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # 1. CvBridge 초기화
        self.bridge = CvBridge()

        # 2. 서브스크라이버 생성
        # 토픽 이름: 'image_raw', 메시지 타입: Image, 콜백 함수: self.image_callback, 큐 사이즈: 10
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # 퍼블리셔와 토픽 이름이 같아야 함
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Image subscriber node started.')

    def image_callback(self, msg):
        try:
            # 3. ROS Image 메시지를 OpenCV 이미지로 변환
            # "bgr8": 퍼블리셔와 동일한 인코딩 사용
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # 4. OpenCV 창에 이미지 표시
        cv2.imshow("Camera View (Subscriber)", cv_image)
        cv2.waitKey(1)  # 1ms 대기 (이게 없으면 창이 업데이트되지 않음)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # 5. 노드 종료 시 자원 해제
        image_subscriber.destroy_node()
        cv2.destroyAllWindows()  # 모든 OpenCV 창 닫기
        rclpy.shutdown()

if __name__ == '__main__':
    main()