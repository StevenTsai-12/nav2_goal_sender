import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import threading
import time

stop_flag = False
pause_flag = False

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

def main(args=None):
    global stop_flag, pause_flag
    rclpy.init(args=args)
    node = MultiGoalSender()

    # ✅ 背景 spin（關鍵修正）
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # ✅ 鍵盤控制線程
    threading.Thread(target=keyboard_listener, daemon=True).start()

    # 定義導航點 1
    pose1 = PoseStamped()
    pose1.header.frame_id = 'map'
    pose1.pose.position.x = -0.0808
    pose1.pose.position.y = -0.1276
    pose1.pose.orientation.z = 0.0215
    pose1.pose.orientation.w = 0.9998

    # 定義導航點 2
    pose2 = PoseStamped()
    pose2.header.frame_id = 'map'
    pose2.pose.position.x = 1.6056
    pose2.pose.position.y = -0.4224
    pose2.pose.orientation.z = 0.6032
    pose2.pose.orientation.w = 0.7976
    
    # 定義導航點 3
    pose3 = PoseStamped()
    pose3.header.frame_id = 'map'
    pose3.pose.position.x = 4.0037
    pose3.pose.position.y = -1.5735
    pose3.pose.orientation.z = -0.6986
    pose3.pose.orientation.w = 0.7155
    
    # 定義導航點 4
    pose4 = PoseStamped()
    pose4.header.frame_id = 'map'
    pose4.pose.position.x = 8.4905
    pose4.pose.position.y = -1.2175
    pose4.pose.orientation.z =  0.1310
    pose4.pose.orientation.w = 0.9913

    poses = [pose1, pose2, pose3, pose4]
    i = 0

    print("多點導航已啟動。Enter=暫停，S=繼續，Q=結束。")

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
    print("程式已安全結束")

if __name__ == '__main__':
    main()
