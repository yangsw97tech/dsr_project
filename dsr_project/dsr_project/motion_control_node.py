#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
#  motion_control_node.py (수정 버전)
#
#  이 노드는 '일시 정지'와 '동작 재시작' 서비스 클라이언트를
#  하나로 통합하여 관리함.
#  oiling.py와의 충돌을 피하기 위해 'DR_init' 관련 코드를 모두 제거하고
#  순수 rclpy로만 작성됨.
#

import rclpy
import sys
from rclpy.node import Node

# 사용할 서비스 임포트
from dsr_msgs2.srv import MovePause, MoveResume
# 토픽을 통해 명령을 받기 위한 메시지 타입 임포트
from std_msgs.msg import String

# --- DR_init 및 DSR_ROBOT2 관련 코드 모두 제거 ---
# import DR_init (제거)
# ROBOT_ID, ROBOT_MODEL (제거. 이 노드와 무관함)

class MotionControlNode(Node):

    def __init__(self):
        # 노드 초기화 (네임스페이스는 반드시 'dsr01'로 일치시켜야 함)
        super().__init__('motion_control_node', namespace='dsr01')
        
        # --- DR_init 설정 코드 모두 제거 ---

        # --- 서비스 클라이언트 2개 생성 ---
        # 1. 일시 정지 클라이언트
        self.pause_cli = self.create_client(MovePause, 'motion/move_pause')
        # 2. 동작 재시작 클라이언트
        self.resume_cli = self.create_client(MoveResume, 'motion/move_resume')

        # 서비스 요청 메시지 객체 미리 생성
        self.pause_req = MovePause.Request()
        self.resume_req = MoveResume.Request()

        # 두 서비스가 모두 준비될 때까지 대기
        while not self.pause_cli.wait_for_service(timeout_sec=1.0) or \
              not self.resume_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('일시정지/재시작 서비스를 기다리는 중... (/dsr01/motion/...)')
        
        # --- 토픽 구독자 생성 ---
        self.command_subscriber = self.create_subscription(
            String,
            'motion_command',  # 전체 토픽 이름: /dsr01/motion_command
            self.command_callback,
            10)
        
        self.get_logger().info('모션 제어 노드 준비 완료.')
        self.get_logger().info('"/dsr01/motion_command" 토픽으로 "pause" 또는 "resume" 명령을 기다립니다.')

    # 토픽 메시지를 받았을 때 실행될 콜백 함수
    def command_callback(self, msg):
        command = msg.data.lower() # 받은 메시지를 소문자로 변경
        
        if command == 'pause':
            self.get_logger().info('일시 정지 명령 수신. 서비스 호출...')
            future = self.pause_cli.call_async(self.pause_req)
            future.add_done_callback(self.service_response_callback)
            
        elif command == 'resume':
            self.get_logger().info('동작 재시작 명령 수신. 서비스 호출...')
            future = self.resume_cli.call_async(self.resume_req)
            future.add_done_callback(self.service_response_callback)
            
        else:
            self.get_logger().warn(f'알 수 없는 명령: "{msg.data}". "pause" 또는 "resume"을 사용하세요.')

    # 서비스 호출 후 응답을 처리하는 공통 콜백
    def service_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('서비스 호출이 성공적으로 완료되었습니다.')
            else:
                self.get_logger().warn('서비스 호출은 되었으나, 로봇이 실패를 반환했습니다.')
        except Exception as e:
            self.get_logger().error(f'서비스 호출 중 예외 발생: {e}')

def main(args=None):
    rclpy.init(args=args)
    motion_control_node = MotionControlNode()
    rclpy.spin(motion_control_node)
    motion_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()