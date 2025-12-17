#!/usr/bin/env python3
"""
ROS 2 Service Client Example
Calls /add_two_ints service with two numbers
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """Service client that calls add_two_ints service."""

    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client: (ServiceType, service_name)
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Client ready')

    def send_request(self, a, b):
        """Send request to service."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        self.get_logger().info(f'Sending request: a={a} b={b}')

        # Call service asynchronously
        future = self.cli.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)

    # Get numbers from command line (or use defaults)
    a = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    b = int(sys.argv[2]) if len(sys.argv) > 2 else 3

    client = AddTwoIntsClient()
    future = client.send_request(a, b)

    # Wait for response
    rclpy.spin_until_future_complete(client, future)

    try:
        response = future.result()
        client.get_logger().info(f'Result: {a} + {b} = {response.sum}')
    except Exception as e:
        client.get_logger().error(f'Service call failed: {e}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
