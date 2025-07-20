#!/usr/bin/env python3

import rclpy
import math
from rclpy.clock import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# Define your desired goal coordinates and orientation (in degrees)
goal_x = 1.67
goal_y = -0.22
# Format for yaw
# quaternion [x, y, z, w]
# degree [deg]
goal_yaw = [0.0, 0.0, 0.14, 0.99]
quaternion = True

class SimpleGoalNavigator(Node):

    def __init__(self):
        super().__init__('simple_goal_navigator')
        self.navigator = BasicNavigator()
        
        # 如果你的Nav2是從頭啟動的狀態，請使用以下指令，會自動設定原點為Initial Point，並等待Nav2完全啟動！
        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = 0.0
        # initial_pose.pose.position.y = 0.0
        # initial_pose.pose.orientation.z = 0.0
        # initial_pose.pose.orientation.w = 1.0
        # self.navigator.setInitialPose(initial_pose)

        # Wait for Nav2 to become active
        # self.get_logger().info('Waiting for Nav2 to become active...')
        # self.navigator.waitUntilNav2Active()
        # self.get_logger().info('Nav2 is active! Sending goal...')

    def navigate_to_goal(self, x, y, yaw, is_quaternion = False):
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)

        yaw_degree = 0.0;

        if(is_quaternion):
            yaw_degree = math.degrees(euler_from_quaternion([yaw[0], yaw[1], yaw[2], yaw[3]])[2])
            goal_pose.pose.orientation.x = yaw[0]
            goal_pose.pose.orientation.y = yaw[1]
            goal_pose.pose.orientation.z = yaw[2]
            goal_pose.pose.orientation.w = yaw[3]
        else:
            # Convert yaw from degrees to radians and then to quaternion
            yaw_radians = math.radians(yaw[0])
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_radians)
            goal_pose.pose.orientation.x = qx
            goal_pose.pose.orientation.y = qy
            goal_pose.pose.orientation.z = qz
            goal_pose.pose.orientation.w = qw

        self.get_logger().info(f'Attempting to navigate to X:{x}, Y:{y}, Yaw:{yaw_degree} degrees.')
        self.navigator.goToPose(goal_pose)

        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                estimated_time_sec = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                self.get_logger().info(f'Estimated time remaining: {estimated_time_sec:.2f} seconds. Remaining distance: {feedback.distance_remaining:.2f} m')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Goal failed!')
        else:
            self.get_logger().info('Goal has unknown result.')

def main(args=None):
    rclpy.init(args=args)
    navigator_node = SimpleGoalNavigator()

    # Wait user
    input("Press enter to start navigation!")

    navigator_node.navigate_to_goal(goal_x, goal_y, goal_yaw, quaternion)

    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()