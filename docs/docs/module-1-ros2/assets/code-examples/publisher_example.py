#!/usr/bin/env python3
"""
ROS 2 Minimal Publisher Example
Publishes string messages to /chatter topic at 1 Hz
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """Simple publisher node that sends string messages."""

    def __init__(self):
        super().__init__('minimal_publisher')

        # Create publisher: (MessageType, topic_name, queue_size)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create timer: callback every 1.0 seconds
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0  # Message counter

        self.get_logger().info('Publisher node started')

    def timer_callback(self):
        """Called periodically to publish messages."""
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.i += 1


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Create node
    minimal_publisher = MinimalPublisher()

    # Spin (keep node running)
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass

    # Cleanup
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
