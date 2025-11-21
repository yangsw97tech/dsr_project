#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
#  move_basic_tester.py
#  (move_basic.py의 에러를 수정하고, 무한 반복 기능을 추가한 버전)
#

import rclpy
import DR_init  # 1. DR_init을 최상단에 임포트
import sys
import math

# --- 로봇 설정 상수 ---
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"
ROBOT_TOOL = "GripperDA_v1"
VELOCITY = 60
ACC = 60

# 2. DR_init의 전역 ID, MODEL 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def perform_task(logger):
    """
    로봇이 수행할 실제 작업 (간단한 왕복 동작)
    """
    global STATE
    # (필수) 이 함수에서 필요한 모든 DSR 모듈 임포트
    try:
        from DSR_ROBOT2 import (
            movej, movel, wait,
            DR_BASE, DR_MV_MOD_ABS,
            posx, posj,
            motion_pause, motion_resume,
        )
    except Exception as e:
        logger.error(f"perform_task 내 DSR_ROBOT2 임포트 실패: {e}")
        return

    logger.info("--- 테스트 작업 시작 ---")

    # 8. 반복 동작에 사용할 위치 정의
    pos_a = posx([346.97, 48.270, 450.540, 13.71, 180.0, 12.62])
    pos_b = posx([546.97, 48.270, 450.540, 13.71, 180.0, 12.62])
        
    vel = 60.0 # 속도
    acc = 30.0 # 가속도

    logger.info("테스트 동작 (A <-> B 반복)을 시작합니다. 'pause' 명령을 테스트하세요.")

    # 9. (필수) 무한 반복 동작
    while rclpy.ok():
        logger.info("-> A 위치로 이동")
        movel(pos_a, vel=vel, acc=acc, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        wait(1.0) # 1초 대기
        
        # 루프 중간에 rclpy.ok()를 다시 확인하여 Ctrl+C로 종료 가능하게 함
        if not rclpy.ok():
            break

        motion_pause()
        # STATE = "PAUSED"
        # motion_resume()
        logger.info("-> B 위치로 이동")
        movel(pos_b, vel=vel, acc=acc, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        wait(1.0) # 1초 대기


def main(args=None):
    # 3. rclpy 초기화
    rclpy.init(args=args)
    
    # 4. 노드 생성
    node = rclpy.create_node("move_basic_tester", namespace=ROBOT_ID)
    logger = node.get_logger()
    
    # 5. (가장 중요) 생성된 노드를 DR_init에 할당
    DR_init.__dsr__node = node
    logger.info(f"'{node.get_name()}' 노드 생성 및 DR_init에 등록 완료.")
    
    # --- 이제 DSR_ROBOT2 임포트가 안전함 ---
    
    try:
        # 6. (필수) 로봇 모드 설정
        from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        logger.info("로봇 모드를 AUTONOMOUS로 설정했습니다. (외부 제어 가능 상태)")
        
        # 7. (필수) Tool/TCP 설정
        #    (move_basic.py의 initialize_robot() 함수 내용)
        from DSR_ROBOT2 import set_tool, set_tcp
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        logger.info(f"TCP: {ROBOT_TCP}, Tool: {ROBOT_TOOL} 설정 완료.")
        
        # 8. (필수) 실제 작업 수행 (무한 반복)
        perform_task(logger)
        
    except ImportError as e:
        logger.error(f"DSR_ROBOT2 라이브러리 임포트 실패: {e}")
    except KeyboardInterrupt:
        logger.info("키보드 인터럽트 (Ctrl+C) 수신. 종료합니다.")
    except Exception as e:
        logger.error(f"노드 실행 중 예외 발생: {e}")
    finally:
        # 9. 작업 완료 후 노드 종료
        logger.info("작업 완료. 노드를 종료합니다.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()