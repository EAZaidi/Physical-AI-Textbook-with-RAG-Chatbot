#!/usr/bin/env python3
"""
VLA System Capstone Integration

Integrates all components: Whisper, LLM Planner, Visual Grounding, Action Executor.
Provides high-level API and error recovery.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class VLASystem(Node):
    def __init__(self):
        super().__init__('vla_system')

        # Component status tracking
        self.components = {
            'whisper': False,
            'llm_planner': False,
            'visual_grounding': False,
            'action_executor': False
        }

        # Task state
        self.current_task = None
        self.planned_actions = []
        self.current_action_idx = 0
        self.max_retries = 3
        self.retry_count = 0

        # Publishers
        self.command_pub = self.create_publisher(String, '/voice/command', 10)
        self.visual_query_pub = self.create_publisher(String, '/visual_query', 10)

        # Subscribers
        self.create_subscription(String, '/voice/command', self.voice_callback, 10)
        self.create_subscription(String, '/planned_actions', self.actions_callback, 10)
        self.create_subscription(String, '/action_feedback', self.feedback_callback, 10)
        self.create_subscription(String, '/execution_status', self.status_callback, 10)

        # Health monitoring timer
        self.create_timer(5.0, self.check_component_health)

        self.get_logger().info('VLA System initialized')

    def voice_callback(self, msg):
        """Handle incoming voice commands"""
        self.current_task = msg.data
        self.get_logger().info(f'Task received: "{self.current_task}"')
        self.retry_count = 0
        self.current_action_idx = 0

    def actions_callback(self, msg):
        """Handle planned actions from LLM"""
        try:
            self.planned_actions = json.loads(msg.data)
            self.current_action_idx = 0
            self.get_logger().info(f'Received plan with {len(self.planned_actions)} actions')

            # Log action sequence
            for idx, action in enumerate(self.planned_actions):
                self.get_logger().info(f'  {idx+1}. {action["name"]}: {action["parameters"]}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse action sequence: {e}')

    def feedback_callback(self, msg):
        """Handle action execution feedback"""
        feedback = msg.data
        self.get_logger().info(f'Feedback: {feedback}')

        if 'completed' in feedback.lower() or 'success' in feedback.lower():
            self.current_action_idx += 1

            if self.current_action_idx >= len(self.planned_actions):
                self.get_logger().info('✅ Task completed successfully!')
                self.current_task = None
                self.planned_actions = []
            else:
                self.get_logger().info(
                    f'Progress: {self.current_action_idx}/{len(self.planned_actions)} actions'
                )

        elif 'failed' in feedback.lower() or 'error' in feedback.lower():
            self.handle_failure(feedback)

    def status_callback(self, msg):
        """Monitor execution status"""
        status = msg.data
        self.get_logger().info(f'Status: {status}')

        if status == 'sequence_complete':
            self.get_logger().info('✅ Full sequence executed successfully')
            self.current_task = None

    def handle_failure(self, error_msg):
        """Implement error recovery and replanning"""
        self.get_logger().warn(f'Action failed: {error_msg}')

        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().info(f'Retry {self.retry_count}/{self.max_retries}')

            # Strategy 1: Retry same action
            if self.retry_count == 1:
                self.get_logger().info('Retrying same action...')
                # Action executor will retry automatically

            # Strategy 2: Request replanning
            elif self.retry_count == 2:
                self.get_logger().info('Requesting replan from LLM...')
                replan_msg = String()
                replan_msg.data = (
                    f"Previous attempt failed: {error_msg}. "
                    f"Original task: {self.current_task}. "
                    f"Generate alternative plan."
                )
                self.command_pub.publish(replan_msg)

            # Strategy 3: Fallback to safe state
            else:
                self.get_logger().warn('Executing fallback to safe state')
                self.execute_fallback()

        else:
            self.get_logger().error('❌ Max retries exceeded. Task failed.')
            self.current_task = None
            self.retry_count = 0

    def execute_fallback(self):
        """Execute safe fallback behavior"""
        self.get_logger().info('Executing fallback: returning to home position')

        fallback_command = String()
        fallback_command.data = "navigate to home position"
        self.command_pub.publish(fallback_command)

    def check_component_health(self):
        """Periodically check component health"""
        # In real implementation, check if nodes are publishing
        # For now, assume all healthy
        pass

    def execute_voice_command(self, command: str):
        """High-level API: Execute voice command"""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'Executing command: "{command}"')

    def get_task_status(self):
        """Get current task execution status"""
        if self.current_task is None:
            return "No active task"
        else:
            progress = f"{self.current_action_idx}/{len(self.planned_actions)}"
            return f"Task: {self.current_task}, Progress: {progress}"


def main(args=None):
    rclpy.init(args=args)

    try:
        vla_system = VLASystem()

        # Example: Execute demo command after initialization
        time.sleep(2.0)
        vla_system.execute_voice_command("Prepare the room for a meeting")

        rclpy.spin(vla_system)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
