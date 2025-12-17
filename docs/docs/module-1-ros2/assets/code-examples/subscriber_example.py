#!/usr/bin/env python3
"""
ROS 2 Minimal Subscriber Example
Subscribes to /chatter topic and logs received messages
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """Simple subscriber node that receives string messages."""

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create subscription: (MessageType, topic_name, callback, queue_size)
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Subscriber node started')

    def listener_callback(self, msg):
        """Called when message received on topic."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    # Initialize rclpy
    rclpy.init(args=args)

    # Create node
    minimal_subscriber = MinimalSubscriber()

    # Spin (keep node running)
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass

    # Cleanup
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
