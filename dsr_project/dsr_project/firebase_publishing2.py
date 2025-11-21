#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db
import threading
import time
import re

from std_msgs.msg import String
from std_srvs.srv import SetBool

try:
    from my_robot_interfaces.action import BrushingAction
    HAS_ACTION = True
except ImportError:
    HAS_ACTION = False
    print("⚠️ [경고] Action Interface 없음")

KEY_FILE = '/home/jy/cobot1_ws/src/doosan-robot2/dsr_project/dsr_project/myfirstdatabase-ecd05-firebase-adminsdk-fbsvc-5f57bc3eeb.json'
DB_URL = "https://myfirstdatabase-ecd05-default-rtdb.firebaseio.com"

class FirebaseBridgeNode(Node):
    def __init__(self):
        super().__init__('firebase_bridge_node')
        
        if not firebase_admin._apps:
            cred = credentials.Certificate(KEY_FILE)
            firebase_admin.initialize_app(cred, {'databaseURL': DB_URL})
        
        self.cmd_pub = self.create_publisher(String, '/main_task_cmd', 10)
        self.cli_custom_pause = self.create_client(SetBool, '/dsr01/custom_pause')

        self.state_data = {
            "system_state": "IDLE",
            "current_task": "대기 중",
            "detail": "-",
            "total_progress": 0,
            "current_step_progress": 0,
            "logs": [],
            "last_updated": time.time()
        }
        
        # [추가됨] 중복 로그 방지 변수
        self.last_log_text = ""

        # [추가] 역행 방지 변수
        self.max_step_seen = 0

        self.update_firebase()

        # [수정] 단계별 인덱스 명확화
        self.step_map = {
            'do_doma_pick_action':  (1, "Pick"),
            'do_eraser_action':     (2, "Erasing"),
            'do_brushing_action':   (3, "Brushing"),
            'do_oiling_action':     (4, "Oiling"),
            'do_doma_place_action': (5, "Place")
        }
        
        if HAS_ACTION:
            self._subscribe_actions()

        self.listen_thread = threading.Thread(target=self.listen_to_firebase, daemon=True)
        self.listen_thread.start()
        self.get_logger().info("🔥 Firebase Bridge Node (Smooth Progress) Started!")

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
        step_idx, step_name = self.step_map[action_name]
        
        # Action Server에서 온 퍼센트값 사용
        step_progress = getattr(fb, 'progress_percentage', 0.0)

        # Late Packet 필터링 (이전 단계 데이터 무시)
        if step_idx < self.max_step_seen:
            return 
        self.max_step_seen = step_idx

        # [수정] 전체 진행률 계산 공식 적용
        # (현재단계-1)*20 + 현재진행률*0.2
        total_progress = ((step_idx - 1) * 20.0) + (step_progress * 0.2)

        # [추가] Total Progress 역행 방지 (DB값 조회 없이 계산값으로 방어)
        # 단, 새 단계 시작(x.0%)일 때는 허용
        current_db_total = self.state_data.get("total_progress", 0)
        if total_progress < current_db_total and step_idx == self.state_data.get("current_step_idx", 0):
             total_progress = current_db_total
        
        if step_idx == 5 and step_progress >= 99.0: 
             total_progress = 100.0
             self.state_data["system_state"] = "IDLE" 
             self.state_data["current_task"] = "작업 완료"
        else:
            self.state_data["system_state"] = "RUNNING"
            self.state_data["current_task"] = step_name

        self.state_data["detail"] = text
        self.state_data["total_progress"] = int(total_progress) # 정수로 변환
        self.state_data["current_step_progress"] = int(step_progress)
        self.state_data["last_updated"] = time.time()
        
        # log_msg = f"[{time.strftime('%H:%M:%S')}] [{step_name}] {text}"
        # self.add_log(log_msg)

        # [수정] 로그 중복 방지
        if text != self.last_log_text:
            log_msg = f"[{time.strftime('%H:%M:%S')}] [{step_name}] {text}"
            self.add_log(log_msg)
            self.last_log_text = text # 업데이트

        self.update_firebase()

    def add_log(self, msg):
        self.state_data["logs"].append(msg)
        if len(self.state_data["logs"]) > 10:
            self.state_data["logs"].pop(0)

    def update_firebase(self):
        try:
            ref = db.reference('robot_status')
            ref.update(self.state_data)
        except Exception as e:
            self.get_logger().error(f"Firebase Update Error: {e}")

    def listen_to_firebase(self):
        ref = db.reference('robot_command')
        ref.listen(self.on_firebase_command)

    def on_firebase_command(self, event):
        if event.data and isinstance(event.data, dict):
            cmd = event.data.get('cmd')
            if cmd:
                self.process_command(cmd)
                db.reference('robot_command').set({}) 

    def process_command(self, cmd):
        self.get_logger().info(f"📥 Web Command: {cmd}")
        self.add_log(f"[{time.strftime('%H:%M:%S')}] 📥 명령 수신: {cmd}")
        
        # [추가] 명령 수신 시 필터 초기화
        self.max_step_seen = 0

        if cmd == "START_ALL":
            self.state_data["total_progress"] = 0
            self.state_data["current_step_progress"] = 0
            self.state_data["system_state"] = "RUNNING"
            self.update_firebase()
        
        if cmd == "PAUSE": self.call_custom_pause(True)
        elif cmd == "RESUME": self.call_custom_pause(False)
        else:
            msg = String()
            msg.data = cmd
            self.cmd_pub.publish(msg)

    def call_custom_pause(self, is_pause):
        if self.cli_custom_pause and self.cli_custom_pause.service_is_ready():
            req = SetBool.Request()
            req.data = is_pause
            self.cli_custom_pause.call_async(req)
            self.state_data["system_state"] = "PAUSED" if is_pause else "RUNNING"
            self.update_firebase()
        else:
            self.get_logger().warn("⚠️ custom_pause 서비스 준비 안 됨")

def main(args=None):
    rclpy.init(args=args)
    node = FirebaseBridgeNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()