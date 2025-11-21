# ui_v2.1
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from dsr_rokey2.action import test_action_client
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QProgressBar, QStatusBar, QTextEdit, QComboBox, QSizePolicy
)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal, QObject, QDateTime
from PyQt5.QtGui import QImage, QPixmap, QFont, QColor
import cv2
import time
import os

# (메시지 및 Signal Emitter 클래스 정의는 기존과 동일)
# ... [RobotCommand, RobotStatus, RobotSignalEmitter 클래스 정의 생략] ...
try:
    from dsr_msgs2.msg import RobotCommand, RobotStatus
except ImportError:
    class RobotCommand:
        def __init__(self):
            self.command = ""
            self.timestamp = time.time()
    class RobotStatus:
        def __init__(self):
            self.stage = "대기"
            self.progress = 0.0
            self.is_running = False

class RobotSignalEmitter(QObject):
    status_update = pyqtSignal(str, float)
    time_update = pyqtSignal(float)
    log_message = pyqtSignal(str)

# ... [VideoThread 및 RobotMonitorNode 클래스 정의 생략] ...

# =========================================================================
# 2. Video Thread (웹캠 스트리밍) - 장치 ID를 인자로 받도록 수정
# =========================================================================
class VideoThread(QThread):
    """웹캠 영상 스트리밍을 위한 별도 스레드"""
    change_pixmap_signal = pyqtSignal(QImage)
    log_signal = pyqtSignal(str)

    def __init__(self, camera_id=1, parent=None): 
        super().__init__(parent)
        self.is_running = False
        self.cap = None
        self.camera_id = camera_id

    def run(self):
        self.log_signal.emit(f"[Video] 웹캠 초기화 시도 (ID: {self.camera_id})...")
        self.cap = cv2.VideoCapture(self.camera_id)

        if not self.cap.isOpened():
            self.log_signal.emit(f"❌ 웹캠 ID {self.camera_id}를 열 수 없습니다.")
            self.is_running = False 
            return

        self.log_signal.emit(f"✅ 웹캠 ID {self.camera_id} 스트리밍 시작.")
        self.is_running = True
        
        while self.is_running:
            ret, frame = self.cap.read()
            if ret:
                rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
                p = convert_to_Qt_format.scaled(640, 480, Qt.KeepAspectRatio)
                self.change_pixmap_signal.emit(p)
            
            QThread.msleep(30) 

        if self.cap:
             self.cap.release()
             self.cap = None 
             self.log_signal.emit("[Video] 웹캠 스레드 종료 및 장치 해제.")

    def start_thread(self):
        """외부에서 스레드를 시작"""
        if not self.isRunning():
            self.start()

    def stop_thread(self):
        """외부에서 스레드를 안전하게 중지"""
        if self.is_running:
            self.is_running = False 
            self.wait()

    def set_camera_id(self, new_id):
        """웹캠 ID만 업데이트 (재시작 시 적용)"""
        self.camera_id = new_id

# ... [RobotMonitorNode 클래스 정의 생략] ...
class RobotMonitorNode(Node):
    ROBOT_STATUS_TOPIC = '/dsr01/robot_status'
    ROBOT_COMMAND_TOPIC = '/dsr01/robot_command'
    
    def __init__(self, signal_emitter):
        super().__init__('robot_monitor_node')
        self.start_time = time.time()
        self.signal_emitter = signal_emitter
        self.is_running = False 
        self.robot_status_sub = self.create_subscription(RobotStatus, self.ROBOT_STATUS_TOPIC, self.on_robot_status_received, 10)
        self.robot_command_pub = self.create_publisher(RobotCommand, self.ROBOT_COMMAND_TOPIC, 10)
        self.ros_timer = self.create_timer(0.1, self.timer_callback)
        self._log("ROS 2 노드 초기화 완료")

    def on_robot_status_received(self, msg: RobotStatus):
        try:
            self.is_running = msg.is_running
            current_stage = msg.stage
            stage_percent = msg.progress
            self.signal_emitter.status_update.emit(current_stage, float(stage_percent))
            if not self.is_running and current_stage == "완료" and stage_percent >= 100.0:
                 self._log("🎉 로봇 작업 시퀀스 완료!")
        except Exception as e:
            self._log(f"❌ 상태 수신 오류: {type(e).__name__} - {str(e)}")

    def send_robot_command(self, command: str):
        try:
            msg = RobotCommand()
            msg.command = command
            msg.timestamp = self.get_clock().now().to_msg() 
            self.robot_command_pub.publish(msg)
            self._log(f"📤 명령 전송: {command}")
        except Exception as e:
            self._log(f"❌ 명령 전송 실패: {str(e)}")

    def _log(self, message: str):
        timestamp = QDateTime.currentDateTime().toString("hh:mm:ss")
        log_msg = f"[{timestamp}] {message}"
        self.signal_emitter.log_message.emit(log_msg)

    def timer_callback(self):
        if self.is_running:
            elapsed_time = time.time() - self.start_time
            self.signal_emitter.time_update.emit(elapsed_time)
        elif self.start_time != 0 and not self.is_running:
             elapsed_time = time.time() - self.start_time
             self.signal_emitter.time_update.emit(elapsed_time)

# =========================================================================
# 5. PyQt Main UI (GUI)
# =========================================================================
class RobotMonitorUI(QMainWindow):
    """메인 모니터링 UI 윈도우"""
    
    # 두산 로봇 아이콘 (텍스트 기호 사용)
    ROBOT_ICON = "🤖" 
    
    def __init__(self, ros_node, signal_emitter):
        super().__init__()
        self.setWindowTitle("도마 연마 모니터링 시스템")
        self.ros_node = ros_node
        self.signal_emitter = signal_emitter
        self.setGeometry(100, 100, 1400, 700)
        
        self.video_thread = None 
        
        # ===== Action Client 생성 =====
        self.action_client = ActionClient(ros_node, test_action_client, 'robot_control')
        self.send_goal_future = None
        self.get_result_future = None

        self.init_ui()
        
        # ROS Signal 연결
        self.signal_emitter.status_update.connect(self.update_progress)
        self.signal_emitter.time_update.connect(self.update_elapsed_time)
        self.signal_emitter.log_message.connect(self.add_log_message)
        
        # UI 구성 후 VideoThread 생성 및 초기 Signal 연결
        self.init_video() 

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        right_layout = QVBoxLayout()

        # --- 로봇 상태 및 제어 ---
        # 1. 상태 레이블
        self.status_label = QLabel("현재 상태: 대기")
        self.status_label.setFont(QFont('Arial', 18, QFont.Bold))
        
        # 2. 진행바 (내부에 % 포함)
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setFixedHeight(30)
        self.progress_bar.setTextVisible(True) 
        
        # 3. 경과 시간
        self.time_label = QLabel("경과 시간: 00:00:00")
        self.time_label.setFont(QFont('Arial', 14))

        left_layout.addWidget(self.status_label)
        left_layout.addWidget(self.progress_bar) 
        left_layout.addWidget(self.time_label)

        # 4. Start / Stop / Webcam ON/OFF 버튼 (QHBoxLayout)
        control_btn_layout = QHBoxLayout()
        
        start_btn = QPushButton("🚀 START")
        stop_btn = QPushButton("⏹️ STOP")
        self.webcam_toggle_btn = QPushButton("📹 WEBCAM OFF") # ON/OFF 통합 버튼
        
        # 버튼 크기 3등분 설정 (stretch factor 1:1:1)
        control_btn_layout.addWidget(start_btn, 1) 
        control_btn_layout.addWidget(stop_btn, 1) 
        control_btn_layout.addWidget(self.webcam_toggle_btn, 1) 
        
        start_btn.clicked.connect(self.start_robot_sequence)
        stop_btn.clicked.connect(self.stop_robot_sequence)
        self.webcam_toggle_btn.clicked.connect(self.toggle_webcam) 
        
        start_btn.setStyleSheet("background-color: #4CAF50; color: white; font-size: 16pt;")
        stop_btn.setStyleSheet("background-color: #F44336; color: white; font-size: 16pt;")
        self.webcam_toggle_btn.setStyleSheet("background-color: #FF9800; color: white; font-size: 16pt;") # 초기 OFF 상태 스타일

        left_layout.addLayout(control_btn_layout)
        
        # 5. 웹캠 선택 (QComboBox) - 버튼 밑으로 이동 및 우측 정렬
        cam_select_layout = QHBoxLayout()
        
        # 왼쪽에 빈 공간을 넣어 오른쪽으로 정렬 (stretch factor)
        cam_select_layout.addStretch(1) 
        
        cam_select_label = QLabel("웹캠 선택:")
        cam_select_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        cam_select_label.setFont(QFont('Arial', 10))
        cam_select_layout.addWidget(cam_select_label) 
        
        self.camera_selector = QComboBox()
        self.camera_selector.setMinimumWidth(100) # 콤보박스 최소 너비 설정
        available_cams = self.find_available_cameras() 
        self.camera_selector.addItems([str(i) for i in available_cams]) 
        if not available_cams:
            self.camera_selector.addItem("0 (장치 없음)") 
        
        self.camera_selector.currentIndexChanged.connect(self.on_camera_selection_changed)
        
        cam_select_layout.addWidget(self.camera_selector)
        # QHBoxLayout 전체를 버튼 크기와 맞추기 위해 stretch factor를 사용하거나, 레이아웃을 조정해야 함.
        # 여기서는 적절한 stretch와 minimum width로 버튼 크기 영역에 맞게 조정함.

        left_layout.addLayout(cam_select_layout) 
        
        # 6. 로그
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont('Courier', 10))
        self.log_text.setFixedHeight(250)
        left_layout.addWidget(QLabel("--- 시스템 로그 ---"))
        left_layout.addWidget(self.log_text)
        
        # --- 비디오 패널 ---
        # 초기 화면은 로봇 아이콘으로 표시
        self.video_label = QLabel(self.ROBOT_ICON) 
        self.video_label.setFont(QFont('Arial', 150)) # 아이콘 크기 키우기
        self.video_label.setFixedSize(640, 480)
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: #111; color: #555; border: 1px solid #555;")
        right_layout.addWidget(self.video_label)
        
        main_layout.addLayout(left_layout, 1)
        main_layout.addLayout(right_layout, 2)
        central.setLayout(main_layout)

        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("UI 초기화 완료", 3000)
        
    def find_available_cameras(self):
        """연결된 웹캠 장치 ID 목록을 반환"""
        available = []
        # 0번부터 5번까지 ID를 테스트
        for i in range(6): 
            cap = cv2.VideoCapture(i)
            # cap.isOpened()만으로는 부족할 수 있으므로, 프레임을 읽어봅니다.
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    available.append(i)
                cap.release()
        return available

    def init_video(self):
        """VideoThread 객체를 생성하고 Signal을 연결합니다."""
        if self.video_thread is not None:
            if self.video_thread.isRunning(): 
                self.video_thread.stop_thread()
            self.video_thread.wait() 
            # 기존 signal 연결 해제
            try: self.video_thread.change_pixmap_signal.disconnect(self.update_image)
            except TypeError: pass
            try: self.video_thread.log_signal.disconnect(self.add_log_message)
            except TypeError: pass

        if hasattr(self, 'camera_selector') and self.camera_selector.count() > 0:
            cam_id = int(self.camera_selector.currentText().split(' ')[0]) # ID만 추출
        else:
            cam_id = 1 
            
        self.video_thread = VideoThread(camera_id=cam_id)
        
        # Video Thread Signal 연결
        self.video_thread.change_pixmap_signal.connect(self.update_image)
        self.video_thread.log_signal.connect(self.add_log_message)
        
        self._set_video_label_off() # OFF 상태로 초기화

    def _set_video_label_off(self):
        """웹캠 OFF 상태일 때 로봇 아이콘으로 화면을 가리는 함수"""
        self.video_label.setText(self.ROBOT_ICON)
        self.video_label.setStyleSheet("background-color: #111; color: #555; border: 1px solid #555;")
        
    def toggle_webcam(self):
        """웹캠 ON/OFF를 토글하는 통합 함수"""
        if self.video_thread is None:
             self.add_log_message("[Webcam] 스레드 초기화 필요.")
             return

        if self.video_thread.isRunning():
            # OFF 동작
            self.video_thread.stop_thread()
            self._set_video_label_off() # 로봇 아이콘 표시 (요청 반영)
            self.webcam_toggle_btn.setText("📹 WEBCAM OFF")
            self.webcam_toggle_btn.setStyleSheet("background-color: #FF9800; color: white; font-size: 16pt;")
            self.add_log_message("[Webcam] 웹캠 OFF.")
        else:
            # ON 동작
            cam_id_text = self.camera_selector.currentText().split(' ')[0]
            if cam_id_text == "0" and "장치 없음" in self.camera_selector.currentText():
                self.add_log_message("[Webcam] ❌ 연결된 장치가 없습니다.")
                return

            cam_id = int(cam_id_text)
            self.video_thread.set_camera_id(cam_id) # 선택된 ID로 재설정
            self.video_thread.start_thread() # 스레드 시작 (run() 실행)
            self.webcam_toggle_btn.setText("🔴 WEBCAM ON")
            self.webcam_toggle_btn.setStyleSheet("background-color: #4CAF50; color: white; font-size: 16pt;")
            self.add_log_message(f"[Webcam] 웹캠 ON (ID: {cam_id}).")

    def on_camera_selection_changed(self):
        """웹캠 선택 콤보박스 값 변경 시"""
        if not self.camera_selector.currentText(): return

        selected_id = self.camera_selector.currentText().split(' ')[0]
        
        if self.video_thread and self.video_thread.isRunning():
            # 웹캠이 켜져 있으면 끄고 다시 ON 버튼을 누르도록 안내
            self.toggle_webcam() # OFF 상태로 만듦
            self.add_log_message(f"[Webcam] ID를 {selected_id}로 변경했습니다. 다시 ON 버튼을 눌러주세요.")
        elif self.video_thread:
            # OFF 상태일 때는 ID만 업데이트
            self.video_thread.set_camera_id(int(selected_id)) 
            self.add_log_message(f"[Webcam] 웹캠 ID를 {selected_id}로 설정했습니다.")

    # ... [나머지 함수는 기존과 동일] ...
    def start_robot_sequence(self):
        self.statusBar.showMessage("작업 시작 명령 전송...", 3000)
        self.ros_node.start_time = time.time()
        self.ros_node.is_running = True 
        self.ros_node.send_robot_command("start")
        self.ros_node._log("[사용자] 작업 시작 명령 전송 🚀")

        # ===== "START_ALL" 신호를 Action Goal로 전송 =====
        self._send_robot_goal("START_ALL")
        self.add_log_message("[START] 로봇 제어 Goal 전송 🚀 (START_ALL)")

    def stop_robot_sequence(self):
        self.statusBar.showMessage("작업 종료 명령 전송...", 3000)
        self.ros_node.send_robot_command("stop")
        self.progress_bar.setValue(0)
        self.ros_node._log("[사용자] 작업 종료 명령 전송 ⏹️")

        # ===== "STOP_ALL" 신호를 Action Goal로 전송 =====
        self._send_robot_goal("STOP_ALL")
        self.ros_node._log("[사용자] 작업 종료 명령 전송 ⏹️")
        self.add_log_message("[STOP] 로봇 제어 Goal 전송 ⏹️ (STOP_ALL)")

    def update_image(self, qt_image):
        self.video_label.setPixmap(QPixmap.fromImage(qt_image))

    def update_progress(self, stage: str, percent: float):
        """로봇 상태 업데이트 및 로딩바 내부 퍼센트 표시"""
        self.status_label.setText(f"현재 상태: {stage}")
        self.progress_bar.setValue(int(percent))
        self.progress_bar.setFormat(f"진행률: {percent:.1f}%") 

    def update_elapsed_time(self, elapsed_seconds: float):
        h = int(elapsed_seconds // 3600)
        m = int((elapsed_seconds % 3600) // 60)
        s = int(elapsed_seconds % 60)
        self.time_label.setText(f"경과 시간: {h:02d}:{m:02d}:{s:02d}")

    def add_log_message(self, message: str):
        current_text = self.log_text.toPlainText()
        lines = current_text.split('\n')
        if len(lines) > 50:
            current_text = '\n'.join(lines[-50:])
            self.log_text.setPlainText(current_text)
        self.log_text.append(message)
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

# =========================================================================
# 6. Main 함수 (기존과 동일)
# =========================================================================
def main():
    app = QApplication(sys.argv)
    signal_emitter = RobotSignalEmitter()
    
    rclpy.init(args=None)
    ros_node = RobotMonitorNode(signal_emitter)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(ros_node)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    main_window = RobotMonitorUI(ros_node, signal_emitter)
    main_window.show()

    try:
        sys.exit(app.exec_())
    finally:
        if hasattr(main_window, 'video_thread') and main_window.video_thread is not None:
            main_window.video_thread.stop_thread()
            main_window.video_thread.wait()
        ros_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()