#!/usr/bin/env python3
# doma_place_final_client.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import BrushingAction

class DomaPlaceClient(Node):
    def __init__(self):
        super().__init__('doma_place_client')
        self._client = ActionClient(self, BrushingAction, '/dsr01/do_doma_place_action')

    def send_goal(self):
        self.get_logger().info("Waiting for Server...")
        self._client.wait_for_server()
        goal = BrushingAction.Goal()
        goal.start_task = True
        
        self.get_logger().info("Sending Goal (Safety Mode On)...")
        future = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_res_cb)

    def goal_res_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("Goal Rejected")
            return
        self.get_logger().info("Goal Accepted")
        res_future = gh.get_result_async()
        res_future.add_done_callback(self.get_res_cb)

    def get_res_cb(self, future):
        res = future.result().result
        if res.complete_task:
            self.get_logger().info(f"SUCCESS! Time: {res.total_duration:.2f}s")
        else:
            self.get_logger().error("FAILED")
        rclpy.shutdown()

    def feedback_cb(self, msg):
        fb = msg.feedback
        # 퍼센트 제거, 메시지만 출력
        print(f"\r[Safe Place] Status: {fb.feedback_string}   ", end="")

def main(args=None):
    rclpy.init(args=args)
    cli = DomaPlaceClient()
    cli.send_goal()
    try: rclpy.spin(cli)
    except: pass

if __name__ == '__main__': main()