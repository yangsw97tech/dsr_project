# -*- coding: utf-8 -*-
#
# monitoring_gui.py
#
# ROS2 토픽을 구독하여 실시간으로 공정 상태를 표시하는 PyQt5 GUI
# ROS2 노드는 별도의 QThread에서 실행됨.
#

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QProgressBar, QTextEdit, QFrame, QGridLayout
)
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, Qt
from PyQt5.QtGui import QFont, QColor, QPalette

# -------------------------------------------------------------------
# ROS2 노드를 실행하기 위한 별도의 스레드
# -------------------------------------------------------------------
class RosWorker(QThread):
    """
    rclpy.spin()을 GUI 스레드와 분리하여 실행하기 위한 QThread.
    ROS 토픽을 구독하고, 데이터 수신 시 PyQt Signal을 발생시킴.
    """
    # GUI 스레드로 데이터를 전달하기 위한 시그널 정의
    step_updated = pyqtSignal(str)
    progress_updated = pyqtSignal(int)
    status_updated = pyqtSignal(str)

    def __init__(self, node_name='monitoring_gui_node'):
        super().__init__()
        self.node_name = node_name
        self.node = None
        self.is_running = True

    def run(self):
        """
        QThread가 start()될 때 실행되는 메인 함수
        """
        try:
            # ROS2 초기화 (QThread 내부에서 실행)
            if not rclpy.ok():
                rclpy.init()
            
            self.node = Node(self.node_name)
            
            # 3개의 토픽을 구독
            self.node.create_subscription(
                String,
                '/robot/process_step',
                self.step_callback,
                10
            )
            self.node.create_subscription(
                Int32,
                '/robot/process_progress',
                self.progress_callback,
                10
            )
            self.node.create_subscription(
                String,
                '/robot/status',
                self.status_callback,
                10
            )
            
            self.node.get_logger().info('모니터링 GUI 노드 스레드 시작.')
            
            # spin()을 실행하여 콜백 함수들이 호출될 수 있도록 대기
            while self.is_running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)

        except Exception as e:
            print(f"RosWorker 스레드 오류: {e}")
        finally:
            if self.node:
                self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            print("RosWorker 스레드 종료.")

    # --- ROS2 토픽 콜백 함수들 ---
    # 이 함수들은 ROS 스레드에서 호출됨

    def step_callback(self, msg):
        # 수신한 데이터를 pyqtSignal로 GUI 스레드에 전달
        self.step_updated.emit(msg.data)

    def progress_callback(self, msg):
        self.progress_updated.emit(msg.data)

    def status_callback(self, msg):
        self.status_updated.emit(msg.data)

    def stop(self):
        """
        스레드를 안전하게 종료
        """
        self.is_running = False
        self.quit()
        self.wait(5000) # 스레드가 종료될 때까지 최대 5초 대기

# -------------------------------------------------------------------
# 메인 GUI 애플리케이션
# -------------------------------------------------------------------
class MonitoringApp(QWidget):
    def __init__(self):
        super().__init__()
        
        # ROS 스레드 워커 생성 및 시작
        self.ros_worker = RosWorker()
        self.ros_worker.start()
        
        # GUI 초기화
        self.init_ui()
        
        # ROS 스레드에서 오는 시그널을 GUI 업데이트 슬롯(함수)에 연결
        self.ros_worker.step_updated.connect(self.update_step)
        self.ros_worker.progress_updated.connect(self.update_progress)
        self.ros_worker.status_updated.connect(self.update_status)

    def init_ui(self):
        """
        GUI 레이아웃 및 위젯 생성
        """
        self.setWindowTitle('실시간 로봇 공정 모니터링 (PyQt5 + ROS2)')
        self.setGeometry(100, 100, 700, 500)
        
        # 폰트 설정
        title_font = QFont("Noto Sans KR", 18, QFont.Bold)
        label_font = QFont("Noto Sans KR", 12)
        status_font = QFont("Noto Sans KR", 14, QFont.Bold)
        
        # --- 레이아웃 설정 ---
        main_layout = QVBoxLayout()
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)

        # 1. 제목
        title_label = QLabel('실시간 공정 모니터링')
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)

        # 2. 상태 표시 그리드
        status_grid = QGridLayout()
        status_grid.setSpacing(15)
        
        # 2-1. 시스템 상태
        status_grid.addWidget(self.create_label('시스템 상태:', label_font), 0, 0)
        self.status_label = QLabel('대기 중...')
        self.status_label.setFont(status_font)
        self.set_status_style(self.status_label, "Idle") # 초기 스타일
        status_grid.addWidget(self.status_label, 0, 1)

        # 2-2. 현재 공정
        status_grid.addWidget(self.create_label('현재 공정:', label_font), 1, 0)
        self.step_label = QLabel('N/A')
        self.step_label.setFont(status_font)
        status_grid.addWidget(self.step_label, 1, 1)
        
        # 2-3. 공정 진행률
        status_grid.addWidget(self.create_label('공정 진행률:', label_font), 2, 0)
        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setFont(label_font)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid grey;
                border-radius: 5px;
                background-color: #EEEEEE;
                height: 25px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #05B8CC;
                width: 20px;
            }
        """)
        status_grid.addWidget(self.progress_bar, 2, 1)
        
        # 그리드 레이아웃을 메인에 추가
        main_layout.addLayout(status_grid)
        
        # 3. 구분선
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        main_layout.addWidget(line)

        # 4. 실시간 로그
        log_label = QLabel('실시간 로그')
        log_label.setFont(label_font)
        main_layout.addWidget(log_label)
        
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setFont(QFont("Consolas", 10))
        self.log_display.setStyleSheet("background-color: #F8F8F8; border: 1px solid #DDD;")
        main_layout.addWidget(self.log_display)
        
        self.setLayout(main_layout)
        
        # 스타일시트 적용 (전체 창)
        self.setStyleSheet("background-color: #FFFFFF;")

    def create_label(self, text, font):
        """헬퍼 함수: 라벨 생성"""
        label = QLabel(text)
        label.setFont(font)
        return label

    def set_status_style(self, label, status):
        """시스템 상태에 따라 라벨의 색상을 변경"""
        if status == "Running":
            label.setStyleSheet("color: #28A745; padding: 5px;") # 초록색
        elif status == "Idle":
            label.setStyleSheet("color: #6C757D; padding: 5px;") # 회색
        elif status == "Error":
            label.setStyleSheet("color: #DC3545; background-color: #F8D7DA; border-radius: 5px; padding: 5px;") # 빨간색
        else:
            label.setStyleSheet("color: #333; padding: 5px;")


    # --- pyqtSlot 데코레이터: ROS 스레드에서 보낸 시그널을 받는 함수 ---
    # 이 함수들은 메인 GUI 스레드에서 안전하게 호출됨

    @pyqtSlot(str)
    def update_step(self, step):
        """ '/robot/process_step' 토픽을 받아 GUI 업데이트 """
        self.step_label.setText(step.capitalize()) # 예: 'sanding' -> 'Sanding'
        self.add_log(f"[공정 변경] {step} 공정을 시작합니다.")

    @pyqtSlot(int)
    def update_progress(self, progress):
        """ '/robot/process_progress' 토픽을 받아 GUI 업데이트 """
        self.progress_bar.setValue(progress)

    @pyqtSlot(str)
    def update_status(self, status):
        """ '/robot/status' 토픽을 받아 GUI 업데이트 """
        self.status_label.setText(status)
        self.set_status_style(self.status_label, status)
        
        if status == "Error":
            self.add_log("[!! 시스템 오류 !!] 로봇 상태를 확인하세요.", "red")
        elif status == "Idle":
            self.add_log("[상태 변경] 시스템이 유휴(Idle) 상태입니다.", "blue")
        elif status == "Running" and self.progress_bar.value() == 0:
            self.add_log("[상태 변경] 시스템이 'Running' 상태로 변경되었습니다.", "green")

    def add_log(self, message, color="black"):
        """로그창에 메시지 추가 (색상 지정 가능)"""
        color_map = {
            "red": "#DC3545",
            "green": "#28A745",
            "blue": "#007BFF",
            "black": "#333333"
        }
        html_color = color_map.get(color, "#333333")
        
        # 텍스트에 HTML 스타일 적용
        self.log_display.append(f'<span style="color:{html_color};">{message}</span>')
        # 스크롤을 항상 맨 아래로 이동
        self.log_display.verticalScrollBar().setValue(self.log_display.verticalScrollBar().maximum())

    def closeEvent(self, event):
        """
        GUI 창이 닫힐 때 ROS 스레드도 안전하게 종료
        """
        self.add_log("애플리케이션 종료 중... ROS 스레드를 중지합니다.", "blue")
        self.ros_worker.stop()
        event.accept()


def main(args=None):
    # PyQt 애플리케이션 생성
    app = QApplication(sys.argv)
    
    # 메인 윈도우 생성 및 표시
    gui = MonitoringApp()
    gui.show()
    
    # 이벤트 루프 시작
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()