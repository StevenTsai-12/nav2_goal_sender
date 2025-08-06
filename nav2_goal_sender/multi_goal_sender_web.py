# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import threading
import time
from flask import Flask

# ===== 全域控制旗標 =====
stop_flag = False
pause_flag = False

# ===== ROS2 導航節點 =====
class MultiGoalSender(Node):
    def __init__(self):
        super().__init__('multi_goal_sender')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal_handle = None

    def send_goal(self, pose: PoseStamped):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server 不可用')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f'送出目標點: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')
        send_goal_future = self._client.send_goal_async(goal_msg)
        while not send_goal_future.done():
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目標被拒絕')
            return False

        self.get_logger().info('目標已接受，等待導航完成...')
        self.current_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            if stop_flag or pause_flag:
                self.get_logger().warn('中斷中，取消目標...')
                cancel_future = goal_handle.cancel_goal_async()
                while not cancel_future.done():
                    time.sleep(0.1)
                return False
            time.sleep(0.1)

        result = result_future.result().result
        self.get_logger().info(f'導航完成: {result}')
        return True

# ===== 鍵盤控制 =====
def keyboard_listener():
    global stop_flag, pause_flag
    while True:
        cmd = input("輸入指令（Enter=暫停，S=啟動，Q=結束）: ").strip().lower()
        if cmd == "":
            pause_flag = True
            print("已暫停導航（目前目標將取消）")
        elif cmd == "s":
            pause_flag = False
            print("已恢復導航")
        elif cmd == "q":
            stop_flag = True
            print("即將結束程式（目前目標將取消）")
            break

# ===== Flask API 提供給筆電網頁控制使用 =====
app = Flask(__name__)

@app.route('/pause')
def pause():
    global pause_flag
    pause_flag = True
    print("[網頁] 暫停導航")
    return 'Paused'

@app.route('/start')
def start():
    global pause_flag
    pause_flag = False
    print("[網頁] 繼續導航")
    return 'Started'

@app.route('/stop')
def stop():
    global stop_flag
    stop_flag = True
    print("[網頁] 結束導航")
    return 'Stopped'

# ===== 主程式 =====
def main(args=None):
    global stop_flag, pause_flag
    rclpy.init(args=args)
    node = MultiGoalSender()

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=keyboard_listener, daemon=True).start()
    threading.Thread(target=app.run, kwargs={'host': '0.0.0.0', 'port': 5000}, daemon=True).start()

    # ===== 導航點設定（與你原始版本相同） =====
    pose1 = PoseStamped()
    pose1.header.frame_id = 'map'
    pose1.pose.position.x = 0.298
    pose1.pose.position.y = 0.099
    pose1.pose.orientation.z = 0.065
    pose1.pose.orientation.w = 0.997

    pose2 = PoseStamped()
    pose2.header.frame_id = 'map'
    pose2.pose.position.x = 1.580
    pose2.pose.position.y = 0.964
    pose2.pose.orientation.z = 0.598
    pose2.pose.orientation.w = 0.801

    pose2_1 = PoseStamped()
    pose2_1.header.frame_id = 'map'
    pose2_1.pose.position.x = 1.580
    pose2_1.pose.position.y = 0.964
    pose2_1.pose.orientation.z = -0.476
    pose2_1.pose.orientation.w = 0.878

    pose3 = PoseStamped()
    pose3.header.frame_id = 'map'
    pose3.pose.position.x = 4.576
    pose3.pose.position.y = -1.078
    pose3.pose.orientation.z = -0.754
    pose3.pose.orientation.w = 0.656

    pose3_1 = PoseStamped()
    pose3_1.header.frame_id = 'map'
    pose3_1.pose.position.x = 4.576
    pose3_1.pose.position.y = -1.078
    pose3_1.pose.orientation.z = 0.354
    pose3_1.pose.orientation.w = 0.935

    pose4 = PoseStamped()
    pose4.header.frame_id = 'map'
    pose4.pose.position.x = 7.828
    pose4.pose.position.y = 0.878
    pose4.pose.orientation.z = 0.049
    pose4.pose.orientation.w = 0.998

    pose4_1 = PoseStamped()
    pose4_1.header.frame_id = 'map'
    pose4_1.pose.position.x = 7.828
    pose4_1.pose.position.y = 0.678
    pose4_1.pose.orientation.z = 0.952
    pose4_1.pose.orientation.w = -0.221

    poses = [pose1, pose2, pose2_1, pose3, pose3_1, pose4, pose4_1]
    i = 0

    print("多點導航已啟動（可用鍵盤或網頁控制）")

    while not stop_flag:
        if pause_flag:
            time.sleep(0.2)
            continue

        pose = poses[i % len(poses)]
        node.get_logger().info(f'導航至點 {i % len(poses) + 1}')

        success = node.send_goal(pose)
        if not success:
            node.get_logger().warn('導航被取消或失敗')

        i += 1
        time.sleep(1)

    node.destroy_node()
    rclpy.shutdown()
    print("✅ 程式已安全結束")

if __name__ == '__main__':
    main()
