#!/usr/bin/env python3
"""
ROS 2 Joint State Publisher Example
Publishes simulated humanoid robot joint states
Prepares students for humanoid robotics capstone (Module 7)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time
import math


class JointStatePublisher(Node):
    """Publishes joint states for a simplified humanoid robot."""

    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        # Timer: publish at 10 Hz
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Joint names for simplified humanoid
        self.joint_names = [
            'neck',
            'left_shoulder', 'left_elbow', 'left_wrist',
            'right_shoulder', 'right_elbow', 'right_wrist',
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle'
        ]

        self.time_step = 0.0

        self.get_logger().info(f'Publishing joint states for {len(self.joint_names)} joints')

    def timer_callback(self):
        """Publish simulated joint states."""
        msg = JointState()

        # Set timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Set joint names
        msg.name = self.joint_names

        # Simulate joint positions (sinusoidal motion)
        msg.position = []
        for i, joint in enumerate(self.joint_names):
            # Simple sinusoidal motion
            angle = 0.5 * math.sin(self.time_step + i * 0.1)
            msg.position.append(angle)

        # Optional: set velocities and efforts
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        # Publish
        self.publisher_.publish(msg)

        self.time_step += 0.1


def main(args=None):
    rclpy.init(args=args)

    joint_state_publisher = JointStatePublisher()

    try:
        rclpy.spin(joint_state_publisher)
    except KeyboardInterrupt:
        pass

    joint_state_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
