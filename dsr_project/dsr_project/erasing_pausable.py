#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
import DR_init
import threading  # [수정] 스레딩 라이브러리 임포트
from std_msgs.msg import String # [수정] 토픽 메시지 타입 임포트
from rclpy.executors import MultiThreadedExecutor  # MultiThreadedExecutor 임포트

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

# 이동 속도 및 가속도
VELOCITY = 100
ACC = 100

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot(logger):
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp, move_periodic, posx  # 필요한 기능만 임포트

    # 설정된 상수 출력
    logger.info("#" * 50)
    logger.info("Initializing robot with the following settings:")
    logger.info(f"ROBOT_ID: {ROBOT_ID}")
    logger.info(f"ROBOT_MODEL: {ROBOT_MODEL}")
    logger.info(f"ROBOT_TCP: {ROBOT_TCP}")
    logger.info(f"ROBOT_TOOL: {ROBOT_TOOL}")
    logger.info(f"VELOCITY: {VELOCITY}")
    logger.info(f"ACC: {ACC}")
    logger.info("#" * 50)

    # Tool과 TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)


def perform_task(logger):
    """로봇이 수행할 작업 (별도 스레드에서 실행됨)"""
    try:
        logger.info("Performing task...")
        from DSR_ROBOT2 import (
            release_compliance_ctrl,release_force,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            set_ref_coord,
            movej,
            movel,wait,get_current_posx,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,DR_TOOL,
            DR_MV_MOD_ABS,DR_MV_MOD_REL,
            posx,
            set_user_cart_coord,
            get_tool_force,
        )
        import math # 올림 계산을 위한 math 모듈 임포트

        # --- 좌표 설정 (기존 코드와 동일) ---
        p1 = [324.380, 67.386, 330.540]
        p2 = [608.089, 65.371, 330.540]
        p4 = [326.180, -117.220, 330.540]
        p_origin = [326.180, -117.220, 330.540, 157.52, 180, 167.8]

        list_x = []
        for i in range(3):
            list_x.append(p2[i]-p1[i])
        A = 0
        for i in list_x:
            A += i**2
        A = math.sqrt(A)
        list_X = [] # x축 단위벡터
        for i in list_x:
            list_X.append(i/A)

        list_y = []
        for i in range(3):
            list_y.append(p1[i]-p4[i])
        B = 0
        for i in list_y:
            B += i**2
        B = math.sqrt(B)
        list_Y = []  # y축 단위벡터
        for i in list_y:
            list_Y.append(i/B)

        pos = p_origin
        DR_USER1 = set_user_cart_coord(list_X, list_Y, pos, ref=DR_BASE)

        board_h = 207    #도마의 세로 길이 기입
        board_w = 300   #도마의 가로 길이 기입

        sponge_h = 23    #지우개의 세로 길이 기입
        sponge_w = 23    #지우개의 가로 길이 기입

        num_strokes = math.ceil(board_h / sponge_h)
        total_overlap = (sponge_h * num_strokes) - board_h
        overlap_count = num_strokes - 1
        
        if overlap_count <= 0:
            y_step_size = board_h
        else:
            y_step_size = sponge_h - (total_overlap / overlap_count)
            
        logger.info(f"계산된 붓질 횟수: {num_strokes}회")
        logger.info(f"Y축 이동 간격: {y_step_size:.1f}mm")
        
        move_x = board_w - 10
        vel_x = 50;acc_x = 50
        vel_y = 50;acc_y = 50
        vel_z = 30;acc_z = 30

        home_pose_j = [0, 0, 90, 0, 90, 0]
        movej(home_pose_j, vel = 30, acc = 30)

        init_pos = posx([331.79, 67.310, 350.540, 13.71, 180.0, 12.62])
        movel(init_pos, vel=30, acc=30, ref = DR_BASE, mod = DR_MV_MOD_ABS)
        logger.info('초기 위치로 이동됨')

        # [주의] 이 부분은 실제 로봇에 맞게 z 하강 값을 설정해야 합니다.
        # movel([0,0,-20,0,0,0], vel = vel_z, acc = acc_z, ref = DR_BASE, mod = DR_MV_MOD_REL)
        logger.info('하강 완료 (주석 처리됨)')

        set_ref_coord(DR_USER1)

        # 힘제어 함수
        def force_control(switch,press_force = 10):
            if switch == 1:
                logger.info("힘 제어 시작")
                set_ref_coord(1)
                task_compliance_ctrl(stx=[3000, 3000, 200, 200, 200, 200])
                wait(0.5)
                set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

                while rclpy.ok(): # [수정] ros가 종료되면 루프 탈출
                    ret = check_force_condition(DR_AXIS_Z, max=press_force)
                    if ret == -1:
                        logger.info("Z축 힘이 press_force N 이상 감지됨.")
                        break
                    wait(0.1) # [수정] 너무 빠른 루프 방지
                wait(0.5)

            elif switch == 0:
                logger.info("힘 제어 중지")
                release_force()
                release_compliance_ctrl()

        def oiling_move():
            force_control(1,13)
            
            for y_move_cnt in range(int(num_strokes)):
                # [수정] rclpy.ok()를 확인하여 노드 종료 시 작업을 중단
                if not rclpy.ok():
                    logger.info("RCLPY가 종료되어 oiling_move 중단")
                    break
                
                if y_move_cnt %2 == 0:
                    force_control(0)
                    movel([move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL)
                else:
                    force_control(0)
                    movel([-move_x,0, 0,0,0,0], vel = vel_x, t=2, acc = acc_x, ref = DR_USER1, mod = DR_MV_MOD_REL)
                
                logger.info(f'{y_move_cnt + 1}번째 붓질 완료')

                if y_move_cnt < num_strokes - 1:
                    movel([0, 0, +10,0,0,0], vel = vel_y, t=1, acc = acc_y, ref = DR_USER1, mod = DR_MV_MOD_REL)
                    movel([0,-y_step_size, 0,0,0,0], vel = vel_y, t=1, acc = acc_y, ref = DR_USER1, mod = DR_MV_MOD_REL)
                    force_control(1,13)
                    logger.info(f'{y_step_size:.1f}mm 다음 라인으로 이동')
                else:
                    logger.info('최종 Y 위치에 도달')

            logger.info('oiling 작업 완료')   
            force_control(0)
            movel([0,0,100,0,0,0], vel = vel_z, acc = acc_z, ref = DR_USER1, mod = DR_MV_MOD_REL)
            logger.info('상승 완료')
            movej(home_pose_j, vel = 30, acc = 30)
            logger.info('홈 복귀 완료')

        # --- 메인 작업 실행 ---
        oiling_move()

    except Exception as e:
        # 스레드 내에서 예외 발생 시 로거에 기록
        logger.error(f"Task thread에서 예외 발생: {e}")
    finally:
        logger.info("Perform_task 스레드 종료.")


# [수정] STOP/RESUME을 처리할 콜백 함수
def control_callback(msg: String, logger):
    """STOP / RESUME 명령을 받는 콜백"""
    try:
        from DSR_ROBOT2 import DrlPause, DrlResume, drl_script_resume, drl_script_pause, drl_script_run
    except ImportError as e:
        logger.error(f"control_callback 내 DSR_ROBOT2 임포트 실패: {e}")
        return

    cmd = msg.data.upper()
    logger.info(f"[ARM CONTROL] 수신: {cmd}")

    if cmd == "STOP":
        logger.warn(">>> STOP 수신 → motion_pause()")
        # motion_pause()   # 현재 수행 중인 모션 일시정지
        drl_script_pause()
        
    elif cmd == "RESUME":
        logger.info(">>> RESUME 수신 → motion_resume()")
        # motion_resume()  # 일시정지된 모션 재개 
        drl_script_resume()
    else:
        logger.info(f">>> [ARM CONTROL] 알 수 없는 명령: {cmd}")


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("erasing_pausable", namespace=ROBOT_ID)
    logger = node.get_logger() # 로거 사용

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    # [수정] MultiThreadedExecutor 생성
    # 이 실행기는 여러 스레드에서 발생하는 콜백(토픽, 서비스, 액션)을
    # 내부 스레드 풀에서 안전하게 처리합니다.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 콜백에 로거를 전달하도록 구독자 생성
    node.create_subscription(
        String,
        "/arm_control",   # 토픽 이름
        lambda msg: control_callback(msg, logger),
        10
    )

    task_thread = None
    try:
        # 초기화는 메인 스레드에서 한 번만 수행
        initialize_robot(logger)

        # 작업 스레드(perform_task)를 별도 스레드에서 실행
        logger.info("작업 스레드(perform_task)를 시작합니다...")
        task_thread = threading.Thread(target=perform_task, args=(logger,))
        task_thread.daemon = True  # 메인 스레드 종료 시 자식 스레드도 종료
        task_thread.start()

        # [수정] 메인 스레드는 rclpy.spin() 대신 MultiThreadedExecutor를 스핀합니다.
        logger.info("메인 스레드가 MultiThreadedExecutor로 /arm_control 토픽을 수신 대기합니다.")
        executor.spin()

    except KeyboardInterrupt:
        logger.info("\nNode interrupted by user (Ctrl+C). Shutting down...")
    except Exception as e:
        logger.error(f"Executor 스핀 중 예기치 않은 오류 발생: {e}")
    finally:
        logger.info("모든 스레드 및 노드 종료 중...")
        
        # [수정] 스핀이 중지되면 Executor를 종료합니다.
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        
        logger.info("정상적으로 종료되었습니다.")


if __name__ == "__main__":
    main()