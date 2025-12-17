#!/usr/bin/env python3
"""
LLM Planner Node for ROS 2

Subscribes to /voice/command and uses LLM to generate structured action sequences.
Supports OpenAI, Anthropic, and local models.
Publishes action sequences to /planned_actions.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from pathlib import Path


class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')

        # Declare parameters
        self.declare_parameter('llm_provider', 'openai')  # openai | anthropic | local
        self.declare_parameter('model_name', 'gpt-4o')
        self.declare_parameter('temperature', 0.1)
        self.declare_parameter('max_tokens', 1000)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('prompt_template_path', 'prompts/system_prompt.txt')

        # Get parameters
        self.provider = self.get_parameter('llm_provider').value
        self.model_name = self.get_parameter('model_name').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.timeout = self.get_parameter('timeout').value
        prompt_path = self.get_parameter('prompt_template_path').value

        # Initialize API clients
        self.get_logger().info(f'Initializing LLM provider: {self.provider}')
        self.client = self.initialize_client()

        # Load system prompt
        if os.path.exists(prompt_path):
            with open(prompt_path, 'r') as f:
                self.system_prompt = f.read()
            self.get_logger().info(f'Loaded system prompt from {prompt_path}')
        else:
            self.get_logger().warn(f'Prompt file not found: {prompt_path}, using default')
            self.system_prompt = self.get_default_prompt()

        # ROS 2 setup
        self.subscription = self.create_subscription(
            String,
            '/voice/command',
            self.command_callback,
            10
        )
        self.publisher = self.create_publisher(
            String,
            '/planned_actions',
            10
        )

        # Conversation history (for multi-turn interactions)
        self.conversation_history = []

        self.get_logger().info(f'LLM Planner initialized with {self.provider}/{self.model_name}')

    def initialize_client(self):
        """Initialize LLM API client based on provider"""
        if self.provider == 'openai':
            import openai
            api_key = os.getenv('OPENAI_API_KEY')
            if not api_key:
                raise ValueError('OPENAI_API_KEY environment variable not set')
            return openai.OpenAI(api_key=api_key)

        elif self.provider == 'anthropic':
            import anthropic
            api_key = os.getenv('ANTHROPIC_API_KEY')
            if not api_key:
                raise ValueError('ANTHROPIC_API_KEY environment variable not set')
            return anthropic.Anthropic(api_key=api_key)

        elif self.provider == 'local':
            # For local models using transformers
            from transformers import AutoTokenizer, AutoModelForCausalLM
            import torch

            self.get_logger().info(f'Loading local model: {self.model_name}')
            self.tokenizer = AutoTokenizer.from_pretrained(self.model_name)
            self.local_model = AutoModelForCausalLM.from_pretrained(
                self.model_name,
                torch_dtype=torch.float16,
                device_map='auto'
            )
            return None  # No API client for local models

        else:
            raise ValueError(f'Unsupported LLM provider: {self.provider}')

    def command_callback(self, msg):
        """Handle incoming voice commands"""
        user_command = msg.data
        self.get_logger().info(f'Received command: "{user_command}"')

        try:
            # Generate plan
            action_sequence = self.generate_plan(user_command)

            # Validate
            if self.validate_actions(action_sequence):
                # Publish
                self.publish_actions(action_sequence)
            else:
                self.get_logger().error('Generated invalid action sequence')

        except Exception as e:
            self.get_logger().error(f'Planning failed: {str(e)}')

    def generate_plan(self, user_command):
        """Generate action sequence using LLM"""
        if self.provider == 'openai':
            return self.generate_plan_openai(user_command)
        elif self.provider == 'anthropic':
            return self.generate_plan_anthropic(user_command)
        elif self.provider == 'local':
            return self.generate_plan_local(user_command)

    def generate_plan_openai(self, user_command):
        """Generate plan using OpenAI API"""
        messages = [
            {"role": "system", "content": self.system_prompt},
            {"role": "user", "content": user_command}
        ]

        response = self.client.chat.completions.create(
            model=self.model_name,
            messages=messages,
            temperature=self.temperature,
            max_tokens=self.max_tokens,
            timeout=self.timeout
        )

        action_json = response.choices[0].message.content
        return self.parse_json_response(action_json)

    def generate_plan_anthropic(self, user_command):
        """Generate plan using Anthropic API"""
        message = self.client.messages.create(
            model=self.model_name,
            max_tokens=self.max_tokens,
            messages=[
                {"role": "user", "content": user_command}
            ],
            system=self.system_prompt,
            temperature=self.temperature,
            timeout=self.timeout
        )

        action_json = message.content[0].text
        return self.parse_json_response(action_json)

    def generate_plan_local(self, user_command):
        """Generate plan using local model"""
        import torch

        # Format prompt
        prompt = f"{self.system_prompt}\n\nUser: {user_command}\nAssistant:"

        # Tokenize
        inputs = self.tokenizer(prompt, return_tensors='pt').to(self.local_model.device)

        # Generate
        with torch.no_grad():
            outputs = self.local_model.generate(
                **inputs,
                max_new_tokens=self.max_tokens,
                temperature=self.temperature,
                do_sample=True,
                pad_token_id=self.tokenizer.eos_token_id
            )

        # Decode
        response = self.tokenizer.decode(outputs[0], skip_special_tokens=True)

        # Extract JSON from response
        response = response.split("Assistant:")[-1].strip()
        return self.parse_json_response(response)

    def parse_json_response(self, response_text):
        """Extract and parse JSON from LLM response"""
        # Try to find JSON in response (handle markdown code blocks)
        if '```json' in response_text:
            response_text = response_text.split('```json')[1].split('```')[0].strip()
        elif '```' in response_text:
            response_text = response_text.split('```')[1].split('```')[0].strip()

        # Parse JSON
        try:
            action_sequence = json.loads(response_text)
            return action_sequence
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
            self.get_logger().error(f'Response text: {response_text}')
            raise

    def validate_actions(self, action_sequence):
        """Validate action sequence format"""
        if not isinstance(action_sequence, list):
            self.get_logger().error(f'Action sequence must be list, got {type(action_sequence)}')
            return False

        for idx, action in enumerate(action_sequence):
            if not isinstance(action, dict):
                self.get_logger().error(f'Action {idx} must be dict, got {type(action)}')
                return False

            if 'name' not in action:
                self.get_logger().error(f'Action {idx} missing "name" field')
                return False

            if 'parameters' not in action:
                self.get_logger().error(f'Action {idx} missing "parameters" field')
                return False

        return True

    def publish_actions(self, action_sequence):
        """Publish action sequence to /planned_actions"""
        msg = String()
        msg.data = json.dumps(action_sequence)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published {len(action_sequence)} actions')

    def get_default_prompt(self):
        """Default system prompt if file not found"""
        return """You are a robot task planner. Your job is to decompose natural language commands into executable action sequences.

Available actions:
- navigate_to(x: float, y: float, location: str): Move robot to coordinates
- grasp(object_id: str, position: tuple): Pick up object
- release(position: tuple): Drop object at position
- detect_objects(query: str): Search for objects matching description
- open(object_id: str): Open door or drawer
- close(object_id: str): Close door or drawer

Output format: JSON array of actions
Example:
[
  {"name": "navigate_to", "parameters": {"x": 5.0, "y": 2.0, "location": "kitchen"}},
  {"name": "detect_objects", "parameters": {"query": "red mug"}},
  {"name": "grasp", "parameters": {"object_id": "red_mug_0", "position": [5.2, 2.1, 0.8]}}
]

Generate valid, safe, and feasible action sequences only.
"""


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LLMPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
