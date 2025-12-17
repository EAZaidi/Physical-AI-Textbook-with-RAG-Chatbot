#!/usr/bin/env python3
"""
Test script for LLM Planner Node

Tests LLM planning with various commands and validates action sequences.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class LLMPlannerTester(Node):
    def __init__(self):
        super().__init__('llm_planner_tester')

        self.plans = []

        # Subscribe to planned actions
        self.create_subscription(
            String,
            '/planned_actions',
            self.plan_callback,
            10
        )

        # Publisher to send test commands
        self.command_pub = self.create_publisher(
            String,
            '/voice/command',
            10
        )

        self.get_logger().info('LLM Planner Tester initialized')

    def plan_callback(self, msg):
        """Collect generated plans"""
        try:
            actions = json.loads(msg.data)
            self.plans.append(actions)
            self.get_logger().info(f'Received plan with {len(actions)} actions')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON: {e}')

    def test_command(self, command):
        """Send test command and wait for response"""
        self.get_logger().info(f'Testing: "{command}"')

        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

        # Wait for response
        time.sleep(5.0)


def main(args=None):
    rclpy.init(args=args)

    tester = LLMPlannerTester()

    # Test commands
    test_commands = [
        "navigate to the kitchen",
        "pick up the red mug from the table",
        "bring me the book from the bedroom",
        "prepare the room for a meeting",
        "clean the living room"
    ]

    print("\n=== LLM Planner Test ===\n")

    for idx, cmd in enumerate(test_commands, 1):
        print(f"Test {idx}/{len(test_commands)}: {cmd}")
        tester.test_command(cmd)
        rclpy.spin_once(tester, timeout_sec=0.1)

    # Print results
    print(f"\n=== Test Results ===")
    print(f"Total plans generated: {len(tester.plans)}")

    for idx, plan in enumerate(tester.plans, 1):
        print(f"\nPlan {idx}: {test_commands[idx-1]}")
        print(json.dumps(plan, indent=2))

    # Validation
    valid_count = 0
    for plan in tester.plans:
        if isinstance(plan, list) and len(plan) > 0:
            if all('name' in a and 'parameters' in a for a in plan):
                valid_count += 1

    accuracy = (valid_count / len(tester.plans) * 100) if tester.plans else 0
    print(f"\n=== Summary ===")
    print(f"Valid plans: {valid_count}/{len(tester.plans)} ({accuracy:.1f}%)")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
