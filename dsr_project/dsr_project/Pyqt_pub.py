# -*- coding: utf-8 -*-
#
# process_simulator_node.py
#
# '가구 공정 시스템'의 진행 상황을 시뮬레이션하는 ROS2 노드.
# 샌딩, 브러싱, 오일링 공정 상태와 진행률을 토픽으로 발행함.
#

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import time
import random

class ProcessSimulatorNode(Node):
    """
    로봇의 작업 공정을 시뮬레이션하고 상태를 발행하는 노드
    """
    def __init__(self):
        super().__init__('process_simulator_node')
        
        # 3가지 정보를 발행할 퍼블리셔 생성
        # 1. 현재 공정 단계 (예: sanding, brushing)
        self.step_publisher_ = self.create_publisher(String, '/robot/process_step', 10)
        
        # 2. 현재 공정 진행률 (0-100)
        self.progress_publisher_ = self.create_publisher(Int32, '/robot/process_progress', 10)
        
        # 3. 로봇 시스템 상태 (예: Running, Idle, Error)
        self.status_publisher_ = self.create_publisher(String, '/robot/status', 10)
        
        # 1초마다 `publish_state` 콜백 함수를 실행
        self.timer = self.create_timer(1.0, self.publish_state)
        
        # 시뮬레이션 상태 변수
        self.process_steps = ['sanding', 'brushing', 'oiling', 'idle'] # 공정 순서
        self.current_step_index = 0  # 현재 공정 인덱스
        self.current_progress = 0    # 현재 공정 진행률
        self.current_status = "Running" # 현재 시스템 상태

        self.get_logger().info('공정 시뮬레이터 노드 시작. 1초마다 상태 발행 중...')
    
    def publish_state(self):
        """
        타이머에 의해 1초마다 호출되어 현재 공정 상태를 계산하고 발행
        """
        
        # 1. 상태 업데이트
        if self.current_status == "Running":
            self.current_progress += 10  # 1초에 10%씩 진행 (10초간 공정 진행)

            # 가끔 에러 시뮬레이션 (5% 확률)
            if random.random() < 0.05:
                self.current_status = "Error"
                self.get_logger().error('시뮬레이션 에러 발생!')
            
            # 현재 공정이 100% 완료되면 다음 단계로
            if self.current_progress >= 100:
                self.current_progress = 100 # 100으로 고정
                
                # 다음 공정으로 이동
                self.current_step_index = (self.current_step_index + 1) % len(self.process_steps)
                
                # 마지막 'idle' 단계면 진행률 0으로 리셋, 아니면 다음 공정 시작
                if self.process_steps[self.current_step_index] == 'idle':
                    self.current_progress = 0
                    self.current_status = "Idle" # 'idle' 상태에서는 Idle로 변경
                else:
                    self.current_progress = 0 # 새 공정 시작
                    self.current_status = "Running" # 다음 공정 시작

        elif self.current_status == "Idle":
            # Idle 상태면 5초간 대기 후 다시 'sanding' 부터 시작
            self.get_logger().info('현재 "Idle" 상태. 5초 후 공정 재시작...')
            time.sleep(5) 
            self.current_step_index = 0 # 'sanding'으로
            self.current_progress = 0
            self.current_status = "Running"
            
        elif self.current_status == "Error":
            # 에러 상태면 10초간 대기 후 'Idle'로 변경
            self.get_logger().warn('현재 "Error" 상태. 10초 후 "Idle"로 복구 시도...')
            time.sleep(10)
            self.current_step_index = 3 # 'idle'로
            self.current_progress = 0
            self.current_status = "Idle"

        
        # 2. 메시지 생성
        step_msg = String()
        step_msg.data = self.process_steps[self.current_step_index]
        
        progress_msg = Int32()
        progress_msg.data = self.current_progress
        
        status_msg = String()
        status_msg.data = self.current_status

        # 3. 메시지 발행
        self.step_publisher_.publish(step_msg)
        self.progress_publisher_.publish(progress_msg)
        self.status_publisher_.publish(status_msg)
        
        # 로그 출력
        self.get_logger().info(
            f'발행: [상태: {status_msg.data}] [공정: {step_msg.data}] [진행률: {progress_msg.data}%]'
        )


def main(args=None):
    rclpy.init(args=args)
    
    node = ProcessSimulatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드 종료')
    finally:
        # 노드 파괴 및 rclpy 종료
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()