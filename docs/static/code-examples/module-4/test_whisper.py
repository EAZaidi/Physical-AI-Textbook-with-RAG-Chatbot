#!/usr/bin/env python3
"""
Test script for Whisper Transcription Node

Tests voice transcription accuracy with pre-recorded or live audio.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class WhisperTester(Node):
    def __init__(self):
        super().__init__('whisper_tester')

        self.transcriptions = []

        # Subscribe to transcriptions
        self.create_subscription(
            String,
            '/voice/command',
            self.transcription_callback,
            10
        )

        self.get_logger().info('Whisper Tester initialized. Listening for transcriptions...')

    def transcription_callback(self, msg):
        """Collect transcriptions"""
        transcription = msg.data
        timestamp = time.time()

        self.transcriptions.append({
            'text': transcription,
            'timestamp': timestamp
        })

        self.get_logger().info(f'Received: "{transcription}"')


def main(args=None):
    rclpy.init(args=args)

    tester = WhisperTester()

    # Test commands
    test_commands = [
        "navigate to the kitchen",
        "pick up the red mug",
        "bring me the book from the shelf",
        "open the refrigerator door",
        "place the bottle on the table"
    ]

    print("\n=== Whisper Transcription Test ===")
    print("Please speak the following commands:")
    for idx, cmd in enumerate(test_commands, 1):
        print(f"{idx}. {cmd}")

    print("\nListening... (Press Ctrl+C to stop)\n")

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass

    # Calculate accuracy
    if tester.transcriptions:
        print(f"\n=== Test Results ===")
        print(f"Total transcriptions: {len(tester.transcriptions)}")
        for idx, trans in enumerate(tester.transcriptions, 1):
            print(f"{idx}. {trans['text']}")

        # Compare with expected (manual evaluation)
        print("\nManually evaluate accuracy and latency.")
    else:
        print("No transcriptions received.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
