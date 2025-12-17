#!/usr/bin/env python3
"""
LangChain Agent for ROS 2 Robot Control

Demonstrates how to use LangChain to create an agent with custom ROS 2 tools.
Implements ReAct pattern with built-in retry logic and conversation memory.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from langchain.agents import Tool, initialize_agent, AgentType
from langchain.chat_models import ChatOpenAI
from langchain.memory import ConversationBufferMemory
import os


class LangChainRobotAgent(Node):
    def __init__(self):
        super().__init__('langchain_robot_agent')

        # Initialize ROS 2 action clients
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2...')
        self.nav2_client.wait_for_server()

        # Known locations (robot's world model)
        self.locations = {
            'kitchen': (5.0, 2.0),
            'living_room': (0.0, 0.0),
            'bedroom': (3.0, 5.0),
            'bathroom': (7.0, 3.0)
        }

        # Detected objects (updated by visual grounding)
        self.detected_objects = {}

        # Subscribe to visual grounding results
        self.create_subscription(
            String,
            '/detected_objects_3d',
            self.detection_callback,
            10
        )

        # Subscribe to voice commands
        self.create_subscription(
            String,
            '/voice/command',
            self.command_callback,
            10
        )

        # Initialize LangChain agent
        self.agent = self.create_agent()

        self.get_logger().info('LangChain Robot Agent initialized')

    def create_agent(self):
        """Create LangChain agent with custom ROS 2 tools"""

        # Define tools
        tools = [
            Tool(
                name="NavigateTo",
                func=self.navigate_to_location,
                description=(
                    "Navigate robot to a location. "
                    "Input should be location name (kitchen, living_room, bedroom, bathroom). "
                    "Returns success/failure message."
                )
            ),
            Tool(
                name="DetectObjects",
                func=self.detect_objects,
                description=(
                    "Search for objects using vision. "
                    "Input should be object description (e.g., 'red mug', 'book'). "
                    "Returns list of detected objects with positions."
                )
            ),
            Tool(
                name="GraspObject",
                func=self.grasp_object,
                description=(
                    "Grasp an object by its ID from DetectObjects. "
                    "Input should be object ID. "
                    "Returns success/failure message."
                )
            ),
            Tool(
                name="ListLocations",
                func=self.list_locations,
                description=(
                    "Get list of known locations the robot can navigate to. "
                    "No input required. "
                    "Returns list of location names."
                )
            )
        ]

        # Initialize LLM
        llm = ChatOpenAI(
            model="gpt-4o",
            temperature=0.1,
            openai_api_key=os.getenv('OPENAI_API_KEY')
        )

        # Initialize memory
        memory = ConversationBufferMemory(
            memory_key="chat_history",
            return_messages=True
        )

        # Create agent
        agent = initialize_agent(
            tools=tools,
            llm=llm,
            agent=AgentType.ZERO_SHOT_REACT_DESCRIPTION,
            verbose=True,
            max_iterations=10,
            memory=memory,
            handle_parsing_errors=True
        )

        return agent

    def command_callback(self, msg):
        """Handle incoming voice commands"""
        command = msg.data
        self.get_logger().info(f'Received command: "{command}"')

        try:
            # Run agent
            result = self.agent.run(command)
            self.get_logger().info(f'Agent result: {result}')
        except Exception as e:
            self.get_logger().error(f'Agent error: {e}')

    def navigate_to_location(self, location: str) -> str:
        """Tool: Navigate to named location"""
        location = location.lower().replace(' ', '_')

        if location not in self.locations:
            return f"Unknown location: {location}. Available: {list(self.locations.keys())}"

        x, y = self.locations[location]
        self.get_logger().info(f'Navigating to {location} at ({x}, {y})')

        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        # Send goal
        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return f"Navigation to {location} rejected by Nav2"

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        return f"Successfully navigated to {location}"

    def detect_objects(self, query: str) -> str:
        """Tool: Detect objects using visual grounding"""
        self.get_logger().info(f'Detecting objects: {query}')

        # Publish query to visual grounding node
        query_msg = String()
        query_msg.data = query

        # In real implementation, publish to /visual_query
        # For now, simulate detection
        if query in self.detected_objects:
            objects = self.detected_objects[query]
            return f"Detected: {objects}"
        else:
            return f"No objects matching '{query}' detected"

    def grasp_object(self, object_id: str) -> str:
        """Tool: Grasp object by ID"""
        self.get_logger().info(f'Grasping object: {object_id}')

        # In real implementation, call MoveIt2
        # For now, simulate
        return f"Successfully grasped {object_id}"

    def list_locations(self, _: str = "") -> str:
        """Tool: List known locations"""
        locations_str = ", ".join(self.locations.keys())
        return f"Known locations: {locations_str}"

    def detection_callback(self, msg):
        """Update detected objects from visual grounding"""
        try:
            import json
            detections = json.loads(msg.data)
            for det in detections:
                self.detected_objects[det['label']] = det
        except Exception as e:
            self.get_logger().error(f'Detection callback error: {e}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LangChainRobotAgent()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
