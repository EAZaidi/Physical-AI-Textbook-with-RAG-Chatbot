#!/usr/bin/env python3
"""
ROS 2 Joint State Subscriber Example
Subscribes to /joint_states and logs joint information
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSubscriber(Node):
    """Subscribes to joint states and processes them."""

    def __init__(self):
        super().__init__('joint_state_subscriber')

        # Create subscription
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('Listening for joint states...')

    def joint_state_callback(self, msg):
        """Process received joint state message."""
        # Log joint information
        num_joints = len(msg.name)

        self.get_logger().info(f'Received state for {num_joints} joints')

        # Display first 3 joints (to avoid spam)
        for i in range(min(3, num_joints)):
            joint_name = msg.name[i]
            position = msg.position[i] if i < len(msg.position) else 0.0

            self.get_logger().info(
                f'  {joint_name}: {position:.3f} rad'
            )

        if num_joints > 3:
            self.get_logger().info(f'  ... and {num_joints - 3} more joints')


def main(args=None):
    rclpy.init(args=args)

    joint_state_subscriber = JointStateSubscriber()

    try:
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        pass

    joint_state_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
