#!/usr/bin/env python3
"""
ROS 2 Service Server Example
Provides /add_two_ints service that adds two integers
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """Service server that adds two integers."""

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service: (ServiceType, service_name, callback)
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Service server ready')

    def add_two_ints_callback(self, request, response):
        """Process service request and return response."""
        response.sum = request.a + request.b

        self.get_logger().info(
            f'Incoming request: a={request.a} b={request.b}'
        )
        self.get_logger().info(f'Returning sum: {response.sum}')

        return response


def main(args=None):
    rclpy.init(args=args)

    service_server = AddTwoIntsServer()

    try:
        rclpy.spin(service_server)
    except KeyboardInterrupt:
        pass

    service_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
