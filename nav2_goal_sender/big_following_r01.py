#!/usr/bin/env python3

import rclpy
import math
from rclpy.clock import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# Define a list of waypoints (x, y, yaw, is_quaternion)
# Format for yaw
# quaternion [x, y, z, w]
# degree [deg]
# The robot will stop and "achieve" each waypoint before moving to the next.
waypoints_for_patrol = [
    (2.0, 0.0, [0.0], False),    # Waypoint 1
    (2.0, 2.0, [90.0], False),   # Waypoint 2
    (0.0, 2.0, [180.0], False),  # Waypoint 3
    (0.0, 0.0, [-90.0], False)   # Waypoint 4 (back to start, facing south)
]
class WaypointFollowerNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_follower_navigator')
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

    def follow_waypoints(self, waypoint_list):
        # Convert yaw from degrees to radians and then to quaternion for each waypoint
        waypoints = []
        for x, y, yaw, is_quaternion in waypoint_list:
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
            waypoints.append(pose)

        self.get_logger().info(f'Attempting to follow {len(waypoints)} waypoints.')
        self.navigator.followWaypoints(waypoints)

        i = 0
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info(f'Executing current waypoint: {feedback.current_waypoint + 1}/{len(waypoints)}')
                estimated_time_sec = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                self.get_logger().info(f'Estimated time remaining: {estimated_time_sec:.2f} seconds. Remaining distance: {feedback.distance_remaining:.2f} m')
                
                # Example of adding a custom action at a waypoint
                # The Nav2 Waypoint Follower can be configured with plugins
                # to execute specific tasks at each waypoint (e.g., "wait_at_waypoint")
                # This feedback check is just for monitoring progress.
                # if feedback.current_waypoint == 0 and feedback.navigation_duration > Duration(seconds=10).nanoseconds / 1e9:
                #     self.get_logger().info('Robot has been at first waypoint for 10 seconds, cancelling for demo.')
                #     self.navigator.cancelTask()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Waypoint following succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Waypoint following was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Waypoint following failed!')
        else:
            self.get_logger().info('Waypoint following has unknown result.')

def main(args=None):
    rclpy.init(args=args)
    navigator_node = WaypointFollowerNavigator()

    # Wait user
    input("Press enter to start navigation!")

    navigator_node.follow_waypoints(waypoints_for_patrol)

    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()