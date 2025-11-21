import sys
import os
import threading
import time
import re
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_srvs.srv import SetBool 

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QProgressBar, QTextEdit, 
    QGroupBox, QGridLayout
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject, QDateTime
from PyQt5.QtGui import QImage, QPixmap, QFont

# [중요] Qt 플랫폼 강제 설정
os.environ["QT_QPA_PLATFORM"] = "xcb"

# 액션/서비스 임포트 확인
try:
    from my_robot_interfaces.action import BrushingAction
    HAS_ACTION_INTERFACE = True
except ImportError:
    HAS_ACTION_INTERFACE = False
    print("⚠️ [경고] 'my_robot_interfaces' 패키지 없음.")

# ---------------------------------------------------------
# Signals
# ---------------------------------------------------------
class RobotSignalEmitter(QObject):
    log_message = pyqtSignal(str)
    # [수정] Task Index(1~5)와 Raw Percent(float)를 전달하도록 변경
    status_update = pyqtSignal(str, str, int, float) 
    sys_state_update = pyqtSignal(str)
    time_update = pyqtSignal(str)
    video_frame = pyqtSignal(QImage)

# ---------------------------------------------------------
# 1. ROS Node (통합형)
# ---------------------------------------------------------
class RobotMonitorNode(Node):
    def __init__(self, signal_emitter):
        super().__init__('robot_monitor_node')
        self.emitter = signal_emitter
        self.start_time = None
        self.is_working = False
        
        # 카메라 관련 설정
        self.cv_bridge = CvBridge()
        self.camera_enabled = False 

        # 메인 노드 명령 전송용 Pub
        self.cmd_pub = self.create_publisher(String, '/main_task_cmd', 10)
        
        # 카메라 구독
        self.img_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        )
        
        # [수정] stop_control.py와 동일한 서비스 사용
        self.cli_custom_pause = self.create_client(SetBool, '/dsr01/custom_pause')
        
        # [핵심] 공정 단계 매핑 (Topic -> Step Index)
        self.step_map = {
            'do_doma_pick_action':  (1, "도마 집기 (Pick)"),
            'do_eraser_action':     (2, "연마 (Erasing)"),
            'do_brushing_action':   (3, "브러싱 (Brushing)"),
            'do_oiling_action':     (4, "오일링 (Oiling)"),
            'do_doma_place_action': (5, "도마 놓기 (Place)")
        }

        # [중요] 에러 해결: 중복 방지용 변수 초기화
        self.last_log_msg = "" 

        # [추가] 역행 방지용 변수 (가장 높은 단계 기억)
        self.max_step_seen = 0

        if HAS_ACTION_INTERFACE:
            self._subscribe_actions()
            
        self.create_timer(1.0, self.timer_callback)
        self.pause_in_progress = False
        self.resume_in_progress = False

        self._log("ROS 2 통합 GUI 노드 (Smooth Progress 적용) 준비 완료")

    def image_callback(self, msg):
        if not self.camera_enabled: return
        try:
            import cv2
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = rgb_image.strides[0]
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
            scaled = qt_image.scaled(640, 480, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
            self.emitter.video_frame.emit(scaled)
        except Exception: pass

    def toggle_camera_state(self, is_on):
        self.camera_enabled = is_on
        state_str = "켜짐" if is_on else "꺼짐"
        self._log(f"📷 카메라 {state_str}")

    def _subscribe_actions(self):
        FeedbackType = BrushingAction.Impl.FeedbackMessage
        for action_name in self.step_map.keys():
            topic = f'/dsr01/{action_name}/_action/feedback'
            self.create_subscription(
                FeedbackType, topic, 
                lambda msg, name=action_name: self.on_feedback(msg, name), 
                10
            )

    def on_feedback(self, msg, action_name):
        fb = msg.feedback
        text = fb.feedback_string
        
        # [수정] Action Server에서 보내주는 정확한 퍼센트 사용
        # 만약 구버전이라 필드가 없다면 0.0 처리 (getattr 사용)
        raw_percent = getattr(fb, 'progress_percentage', 0.0)
        
        step_idx, step_name_kr = self.step_map[action_name]
        
        # 늦게 도착한 이전 단계 패킷 무시 (Late Packet Filter)
        if step_idx < self.max_step_seen:
            return  # 이미 3단계인데 2단계 데이터가 오면 무시
        
        # 현재 단계가 더 높으면 갱신
        self.max_step_seen = step_idx

        # 작업 상태 관리
        if not self.is_working and (step_idx > 1 or raw_percent > 0):
            self.is_working = True
            self.start_time = time.time()
            self.emitter.sys_state_update.emit("RUNNING 🟢")
            
        if self.is_working:
             self.emitter.sys_state_update.emit("RUNNING 🟢")

        # 전체 완료 감지
        if step_idx == 5 and raw_percent >= 99.0:
             self.is_working = False
             self.emitter.sys_state_update.emit("IDLE (완료)")
             
        # [수정] 로그 중복 방지 로직 추가
        # 1. 로그 메시지 생성
        log_text = f"[{step_name_kr}] {text}"

        # 2. 이전 메시지와 다를 때만 로그 찍기
        if log_text != self.last_log_msg:
            self.emitter.log_message.emit(log_text)
            self.last_log_msg = log_text # 현재 메시지를 저장

        # self.emitter.log_message.emit(f"[{step_name_kr}] {text}")
        # [핵심] UI 스레드로 Raw Data(float) 전송 -> UI가 부드럽게 처리
        self.emitter.status_update.emit(step_name_kr, text, step_idx, raw_percent)

    def send_command(self, cmd):
        self.cmd_pub.publish(String(data=cmd))
        self.emitter.log_message.emit(f"🚀 명령 전송: {cmd}")

        # [추가] 새 명령을 보낼 때는 단계 필터 초기화 (수동 조작 허용)
        self.max_step_seen = 0

        if cmd == "START_ALL":
            self.start_time = time.time()
            self.is_working = True
            self.emitter.sys_state_update.emit("RUNNING 🟢")
            # 초기화 신호 (Step 0)
            self.emitter.status_update.emit("준비 중...", "시작 대기", 0, 0.0)

    # [수정] Custom Pause 사용
    def req_pause(self):
        if not self.cli_custom_pause.service_is_ready():
            self.emitter.log_message.emit("❌ 서비스(custom_pause) 연결 안 됨")
            return
        if self.pause_in_progress: return
        self.pause_in_progress = True
        self.emitter.log_message.emit("⏸️ 일시 정지 요청...")
        req = SetBool.Request()
        req.data = True
        self.cli_custom_pause.call_async(req).add_done_callback(self._pause_cb)
        self.emitter.sys_state_update.emit("PAUSED ⏸️")

    def _pause_cb(self, future):
        self.pause_in_progress = False
        try:
            if future.result().success: self.emitter.log_message.emit("✅ 일시 정지 성공")
        except Exception as e: self.emitter.log_message.emit(f"❌ 실패: {e}")

    def req_resume(self):
        if not self.cli_custom_pause.service_is_ready(): return
        if self.resume_in_progress: return
        self.resume_in_progress = True
        self.emitter.log_message.emit("▶️ 재개 요청...")
        req = SetBool.Request()
        req.data = False
        self.cli_custom_pause.call_async(req).add_done_callback(self._resume_cb)
        self.emitter.sys_state_update.emit("RUNNING 🟢")

    def _resume_cb(self, future):
        self.resume_in_progress = False
        try:
            if future.result().success: self.emitter.log_message.emit("✅ 재개 성공")
        except Exception as e: self.emitter.log_message.emit(f"❌ 실패: {e}")

    def _log(self, msg):
        self.emitter.log_message.emit(msg)

    def timer_callback(self):
        if self.is_working and self.start_time:
            elapsed = time.time() - self.start_time
            h, m, s = int(elapsed // 3600), int((elapsed % 3600) // 60), int(elapsed % 60)
            self.emitter.time_update.emit(f"{h:02}:{m:02}:{s:02}")

# ---------------------------------------------------------
# 3. Main UI
# ---------------------------------------------------------
class RobotMonitorUI(QMainWindow):
    def __init__(self, ros_node, emitter):
        super().__init__()
        self.node = ros_node
        self.emitter = emitter
        self.is_camera_on = False
        
        self.setWindowTitle("Smart Factory Monitor (Smooth Progress)")
        self.setGeometry(100, 100, 1200, 750)
        
        self.setStyleSheet("""
            QMainWindow { background-color: #2b2b2b; color: #ffffff; }
            QLabel { color: #ffffff; }
            QGroupBox { font-weight: bold; border: 1px solid #555; margin-top: 10px; color: #aaaaaa; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }
            QPushButton { background-color: #444; border: 1px solid #555; color: white; padding: 5px; border-radius: 4px; }
            QPushButton:hover { background-color: #555; }
            QPushButton:pressed { background-color: #333; }
            QProgressBar { border: 1px solid #555; border-radius: 5px; text-align: center; color: white; background-color: #1e1e1e; }
            QProgressBar::chunk { background-color: #007bff; width: 10px; }
            QTextEdit { background-color: #1e1e1e; color: #dddddd; border: 1px solid #555; font-family: Consolas; }
        """)
        
        self.init_ui()
        
        # [신규] 부드러운 진행바를 위한 변수
        self.target_total_prog = 0.0
        self.current_total_prog = 0.0
        self.target_step_prog = 0.0
        self.current_step_prog = 0.0
        
        # [신규] 30ms 간격으로 애니메이션 업데이트 타이머
        self.anim_timer = QTimer()
        self.anim_timer.timeout.connect(self.update_progress_animation)
        self.anim_timer.start(30) # 30ms = 약 33fps

        self.connect_signals()
        self.toggle_camera()

    def connect_signals(self):
        self.emitter.log_message.connect(self.append_log)
        self.emitter.status_update.connect(self.set_progress_target) # [수정] 연결 대상 변경
        self.emitter.sys_state_update.connect(self.update_sys_state)
        self.emitter.time_update.connect(self.lbl_time_val.setText)
        self.emitter.video_frame.connect(self.update_video_frame)

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        left_panel = QVBoxLayout()
        
        info_group = QGroupBox("Status Dashboard")
        grid = QGridLayout()
        font_val = QFont("Arial", 12, QFont.Bold)
        
        grid.addWidget(QLabel("System State:"), 0, 0)
        self.lbl_sys_state = QLabel("IDLE ⚪")
        self.lbl_sys_state.setFont(font_val)
        grid.addWidget(self.lbl_sys_state, 0, 1)
        
        grid.addWidget(QLabel("Elapsed Time:"), 0, 2)
        self.lbl_time_val = QLabel("00:00:00")
        self.lbl_time_val.setFont(font_val)
        self.lbl_time_val.setStyleSheet("color: #00ff00;")
        grid.addWidget(self.lbl_time_val, 0, 3)
        
        grid.addWidget(QLabel("Current Task:"), 1, 0)
        self.lbl_cur_task = QLabel("-")
        self.lbl_cur_task.setFont(font_val)
        self.lbl_cur_task.setStyleSheet("color: #00d4ff;")
        grid.addWidget(self.lbl_cur_task, 1, 1, 1, 3)
        
        grid.addWidget(QLabel("Detail:"), 2, 0)
        self.lbl_detail = QLabel("-")
        self.lbl_detail.setStyleSheet("color: #dddddd;")
        grid.addWidget(self.lbl_detail, 2, 1, 1, 3)
        info_group.setLayout(grid)
        left_panel.addWidget(info_group)
        
        prog_group = QGroupBox("Progress")
        prog_layout = QVBoxLayout()
        prog_layout.addWidget(QLabel("Total Progress (전체 공정)"))
        self.prog_bar_total = QProgressBar()
        self.prog_bar_total.setRange(0, 100)
        self.prog_bar_total.setFormat("%p%")
        prog_layout.addWidget(self.prog_bar_total)
        
        prog_layout.addWidget(QLabel("Step Progress (현재 단계)"))
        self.prog_bar_step = QProgressBar()
        self.prog_bar_step.setRange(0, 100)
        self.prog_bar_step.setStyleSheet("QProgressBar::chunk { background-color: #28a745; }")
        prog_layout.addWidget(self.prog_bar_step)
        prog_group.setLayout(prog_layout)
        left_panel.addWidget(prog_group)

        left_panel.addWidget(QLabel("Log Console"))
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        left_panel.addWidget(self.log_text)

        right_panel = QVBoxLayout()
        self.video_frame = QLabel("NO SIGNAL")
        self.video_frame.setAlignment(Qt.AlignCenter)
        self.video_frame.setFixedSize(640, 480)
        self.video_frame.setStyleSheet("background-color: #000; border: 2px solid #444;")
        right_panel.addWidget(self.video_frame)
        
        ctrl_group = QGroupBox("Control Panel")
        vbox_ctrl = QVBoxLayout()
        
        hbox_main = QHBoxLayout()
        btn_start = QPushButton("🚀 START ALL")
        btn_start.setFixedHeight(50)
        btn_start.setStyleSheet("background-color: #007bff; font-weight: bold;")
        btn_start.clicked.connect(lambda: self.node.send_command("START_ALL"))
        
        btn_pause = QPushButton("⏸️ PAUSE")
        btn_pause.setFixedHeight(50)
        btn_pause.setStyleSheet("background-color: #ffc107; color: black; font-weight: bold;")
        btn_pause.clicked.connect(self.node.req_pause)
        
        btn_resume = QPushButton("▶️ RESUME")
        btn_resume.setFixedHeight(50)
        btn_resume.setStyleSheet("background-color: #28a745; font-weight: bold;")
        btn_resume.clicked.connect(self.node.req_resume)
        
        hbox_main.addWidget(btn_start); hbox_main.addWidget(btn_pause); hbox_main.addWidget(btn_resume)
        vbox_ctrl.addLayout(hbox_main)
        
        grid_debug = QGridLayout()
        tasks = [("Pick", "START_DOMA_PICK"), ("Erase", "START_ERASING"), ("Brush", "START_BRUSHING"), ("Oil", "START_OILING"), ("Place", "START_DOMA_PLACE")]
        for i, (label, cmd) in enumerate(tasks):
            btn = QPushButton(f"{i+1}. {label}")
            btn.clicked.connect(lambda _, c=cmd: self.node.send_command(c))
            grid_debug.addWidget(btn, 0, i)
        vbox_ctrl.addLayout(grid_debug)
        ctrl_group.setLayout(vbox_ctrl)
        right_panel.addWidget(ctrl_group)
        
        self.btn_cam = QPushButton("Camera ON/OFF")
        self.btn_cam.clicked.connect(self.toggle_camera)
        right_panel.addWidget(self.btn_cam)

        main_layout.addLayout(left_panel, 4); main_layout.addLayout(right_panel, 6)

    def append_log(self, text):
        self.log_text.append(text)
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    # [신규] 진행률 목표값 설정 (ROS 스레드에서 호출)
    def set_progress_target(self, task, detail, step_idx, raw_percent):
        self.lbl_cur_task.setText(task)
        self.lbl_detail.setText(detail)
        
        # 1. 리셋 명령 감지
        if step_idx == 0:
            self.target_total_prog = 0; self.current_total_prog = 0
            self.target_step_prog = 0; self.current_step_prog = 0
            return

        # 2. 개별 공정 (0~100)
        self.target_step_prog = float(raw_percent)
        
        # 3. 전체 공정 (0~100) 계산
        # 공식: (현재 단계 인덱스 - 1) * 20% + (현재 단계 진행률 * 0.2)
        base_progress = (step_idx - 1) * 20.0
        added_progress = float(raw_percent) * 0.2
        calc_total = base_progress + added_progress
        
        # 역행 방지 (Total은 계속 올라가야 함, 단 단계 변경 직후 제외)
        if calc_total >= self.target_total_prog:
            self.target_total_prog = calc_total
        elif step_idx > 1 and calc_total > (step_idx-2)*20: # 단계가 바뀌었을 때 안전장치
             self.target_total_prog = calc_total

    # [신규] 타이머에 의한 부드러운 업데이트 (UI 스레드)
    def update_progress_animation(self):
        # Step Progress 보간 (속도 0.2)
        diff_step = self.target_step_prog - self.current_step_prog
        if abs(diff_step) > 0.5:
            self.current_step_prog += diff_step * 0.2
        else:
            self.current_step_prog = self.target_step_prog
        self.prog_bar_step.setValue(int(self.current_step_prog))
        
        # Total Progress 보간 (속도 0.1)
        diff_total = self.target_total_prog - self.current_total_prog
        if abs(diff_total) > 0.1:
            self.current_total_prog += diff_total * 0.1
        else:
            self.current_total_prog = self.target_total_prog
        self.prog_bar_total.setValue(int(self.current_total_prog))

    def update_sys_state(self, state):
        self.lbl_sys_state.setText(state)
        color = "#ffffff"
        if "RUNNING" in state: color = "#00ff00"
        elif "PAUSED" in state: color = "#ffff00"
        self.lbl_sys_state.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 12pt;")

    def update_video_frame(self, img):
        self.video_frame.setPixmap(QPixmap.fromImage(img))

    def toggle_camera(self):
        self.is_camera_on = not self.is_camera_on
        self.node.toggle_camera_state(self.is_camera_on)
        if self.is_camera_on:
            self.btn_cam.setText("🔴 카메라 끄기"); self.btn_cam.setStyleSheet("background-color: #d9534f;")
        else:
            self.video_frame.clear(); self.video_frame.setText("Camera OFF")
            self.btn_cam.setText("📹 카메라 켜기"); self.btn_cam.setStyleSheet("background-color: #444;")

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    emitter = RobotSignalEmitter()
    ros_node = RobotMonitorNode(emitter)
    thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    thread.start()
    gui = RobotMonitorUI(ros_node, emitter)
    gui.show()
    try: sys.exit(app.exec_())
    finally: ros_node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()