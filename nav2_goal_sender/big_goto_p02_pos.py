#!/usr/bin/env python3

import threading
import rclpy
import math
from rclpy.clock import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import Buffer, TransformListener
from time import sleep

# Define your desired goal coordinates and orientation (in degrees)
goal_x = 1.65
goal_y = -0.15
# Format for yaw
# quaternion [x, y, z, w]
# degree [deg]
goal_yaw = [0.0, 0.0, 0.13, 0.99]
quaternion = True

class MapToBaseLinkTransform(Node):
    def __init__(self):
        super().__init__('map_to_base_link_transform')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_transform(self):
        # CORRECTED:
        # You want the pose of 'base_footprint' IN the 'map' frame.
        # So, 'target_frame' is 'map'
        # And 'source_frame' is 'base_footprint'
        target_frame = 'map'
        source_frame = 'base_footprint' 

        try:
            # Lookup the transform between the two frames
            transform = self.tf_buffer.lookup_transform(
                target_frame,  # The frame you want the pose *in* (your global reference)
                source_frame,  # The frame whose pose you are looking for (your robot's base)
                rclpy.time.Time(), # Get the latest transform
                Duration(seconds=1.0)
            )

            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # Extract orientation (quaternion)
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            self.get_logger().info(f'Robot Pose in {target_frame} frame:')
            self.get_logger().info(f'  Position: x={x:.2f}, y={y:.2f}, z={z:.2f}')
            self.get_logger().info(f'  Orientation (Quaternion): x={qx:.2f}, y={qy:.2f}, z={qz:.2f}, w={qw:.2f}')

            # If you need Euler angles (roll, pitch, yaw) from the quaternion:
            roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])
            self.get_logger().info(f'  Orientation (Euler): Roll={math.degrees(roll):.2f}, Pitch={math.degrees(pitch):.2f}, Yaw={math.degrees(yaw):.2f} degrees')

            # End this node
            self.get_logger().info(f'Got the position, destroying...')
            return True

        except Exception as ex:
            # The warning message will now accurately reflect the frames it tried to look up
            self.get_logger().warn(f'Could not transform {source_frame} to {target_frame}: {ex}')
            return False

class SimpleGoalNavigator(Node):

    def __init__(self):
        super().__init__('simple_goal_navigator')
        self.navigator = BasicNavigator()
        self.transform_lookup_node = MapToBaseLinkTransform()
        
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
            while not self.transform_lookup_node.get_transform():
                self.get_logger().info('Still getting transform...')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Goal failed!')
        else:
            self.get_logger().info('Goal has unknown result.')

def main(args=None):
    rclpy.init(args=args)

    navigator_node = SimpleGoalNavigator()
    transform_node = navigator_node.transform_lookup_node # Get the reference to the transform node

    # Create a MultiThreadedExecutor to spin both nodes concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(navigator_node)
    executor.add_node(transform_node)

    # Use a separate thread for spinning the executor
    # This allows the main thread to block for user input and then initiate navigation
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Wait for user input to start navigation
    input("Press enter to start navigation!")

    # Now navigate to the goal
    navigator_node.navigate_to_goal(goal_x, goal_y, goal_yaw, quaternion)

    # Clean up
    navigator_node.destroy_node()
    transform_node.destroy_node()
    rclpy.shutdown()
    executor.shutdown() # Ensure the executor is also shut down
    executor_thread.join() # Wait for the executor thread to finish

if __name__ == '__main__':
    main()