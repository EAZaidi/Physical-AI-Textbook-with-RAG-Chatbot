#!/usr/bin/env python3
"""
Send Navigation Goal to Nav2
Module 3, Chapter 3 - Programmatic goal sending
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import sys

class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 action server...')

    def send_goal(self, x, y, yaw):
        """
        Send navigation goal to Nav2.

        Args:
            x: Target X position in map frame (meters)
            y: Target Y position in map frame (meters)
            yaw: Target orientation in map frame (radians)
        """
        # Wait for action server
        self._action_client.wait_for_server()

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set position
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        quat = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]

        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw:.2f} rad')

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance:.2f}m')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return

        self.get_logger().info('Goal accepted by Nav2, navigating...')

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation complete! Result: {result}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Parse command-line arguments
    if len(sys.argv) < 4:
        print('Usage: send_nav_goal.py <x> <y> <yaw>')
        print('Example: send_nav_goal.py 5.0 2.0 1.57')
        print('  x, y: Position in map frame (meters)')
        print('  yaw: Orientation (radians, 0=East, 1.57=North, 3.14=West, -1.57=South)')
        sys.exit(1)

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3])

    node = NavGoalSender()
    node.send_goal(x, y, yaw)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
