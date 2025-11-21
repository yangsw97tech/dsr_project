#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
#  emergency_stop_client.py (올바른 버전)
#
#  이 노드는 순수 rclpy로만 작성되어야 하며,
#  'DR_init'을 절대 임포트(import)하지 않습니다.
#

import rclpy
import sys
from rclpy.node import Node
from dsr_msgs2.srv import MoveStop  # MoveStop 서비스 메시지만 임포트

# MoveStop.srv에 정의된 정지 모드 상수
DR_QSTOP_STO = 0  # Quick stop (STO)
DR_QSTOP = 1      # Quick stop
DR_SSTO = 2       # Soft Stop
DR_HOLD = 3       # HOLD stop

class EmergencyStopClient(Node):

    def __init__(self):
        # 네임스페이스 'dsr01'로 노드 초기화
        super().__init__('emergency_stop_client', namespace='dsr01')
        
        # 서비스 클라이언트 생성
        self.cli = self.create_client(MoveStop, 'motion/move_stop')
        
        # 서비스가 준비될 때까지 대기
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('긴급 정지 서비스를 기다리는 중... (/dsr01/motion/move_stop)')
        
        self.req = MoveStop.Request()
        self.get_logger().info('긴급 정지 클라이언트가 준비되었습니다.')

    def send_request(self, stop_mode=DR_QSTOP_STO):
        # 요청할 정지 모드 설정
        self.req.stop_mode = stop_mode
        self.future = self.cli.call_async(self.req)
        self.get_logger().info(f"긴급 정지 요청 전송 (모드: {stop_mode})...")

def main(args=None):
    rclpy.init(args=args)
    
    stop_client = EmergencyStopClient()
    stop_client.send_request(stop_mode=DR_QSTOP_STO) # 가장 빠른 정지

    # 서비스 응답 대기
    while rclpy.ok():
        rclpy.spin_once(stop_client)
        if stop_client.future.done():
            try:
                response = stop_client.future.result()
                if response.success:
                    stop_client.get_logger().info('긴급 정지 명령이 성공적으로 실행되었습니다.')
                else:
                    stop_client.get_logger().warn('긴급 정지 명령 실행에 실패했습니다.')
            except Exception as e:
                stop_client.get_logger().error(f'서비스 호출 실패: {e}')
            break  # 응답을 받았으므로 루프 종료

    stop_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()