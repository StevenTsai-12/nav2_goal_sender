#!/usr/bin/env python3

import rclpy
import math
from rclpy.clock import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# Define a list of poses (x, y, yaw, is_quaternion)
# Format for yaw
# quaternion [x, y, z, w]
# degree [deg]
# The robot will try to navigate through these in order.
# It will not necessarily stop at each intermediate point.
poses_to_visit = [
    (2.0, 1.0, [0.0], False),    # First intermediate point, facing 0 degrees
    (4.0, 3.0, [90.0], False),   # Second intermediate point, facing 90 degrees
    (1.0, 5.0, [180.0], False)   # Final goal point, facing 180 degrees
]

class ThroughPosesNavigator(Node):

    def __init__(self):
        super().__init__('through_poses_navigator')
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

    def navigate_through_poses(self, pose_list):
        # Convert yaw from degrees to radians and then to quaternion for each pose
        goal_poses = []
        for x, y, yaw, is_quaternion in pose_list:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            
            yaw_degree = 0.0;

            if(is_quaternion):
                yaw_degree = math.degrees(euler_from_quaternion([yaw[0], yaw[1], yaw[2], yaw[3]])[2])
                pose.pose.orientation.x = yaw[0]
                pose.pose.orientation.y = yaw[1]
                pose.pose.orientation.z = yaw[2]
                pose.pose.orientation.w = yaw[3]
            else:
                # Convert yaw from degrees to radians and then to quaternion
                yaw_radians = math.radians(yaw[0])
                qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_radians)
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
            self.get_logger().info(f'Pose added X:{x}, Y:{y}, Yaw:{yaw_degree} degrees.')
            goal_poses.append(pose)

        self.get_logger().info(f'Attempting to navigate through {len(goal_poses)} poses.')
        self.navigator.goThroughPoses(goal_poses)

        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                estimated_time_sec = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                self.get_logger().info(f'Estimated time remaining: {estimated_time_sec:.2f} seconds. Remaining distance: {feedback.distance_remaining:.2f} m')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Navigation through poses succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Navigation through poses was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation through poses failed!')
        else:
            self.get_logger().info('Navigation through poses has unknown result.')

def main(args=None):
    rclpy.init(args=args)
    navigator_node = ThroughPosesNavigator()

    # Wait user
    input("Press enter to start navigation!")

    navigator_node.navigate_through_poses(poses_to_visit)

    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()