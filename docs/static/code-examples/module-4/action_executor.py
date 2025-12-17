#!/usr/bin/env python3
"""
Action Executor Node for ROS 2

Subscribes to /planned_actions and executes each action using appropriate ROS 2 action clients.
Supports navigation (Nav2), manipulation (MoveIt2), and perception actions.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from moveit_msgs.msg import MoveGroupGoal
import json
import time


class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')

        # Declare parameters
        self.declare_parameter('navigation_timeout', 30.0)
        self.declare_parameter('manipulation_timeout', 15.0)
        self.declare_parameter('detection_timeout', 5.0)

        # Get parameters
        self.nav_timeout = self.get_parameter('navigation_timeout').value
        self.manip_timeout = self.get_parameter('manipulation_timeout').value
        self.detect_timeout = self.get_parameter('detection_timeout').value

        # Action clients
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav2_client.wait_for_server()
        self.get_logger().info('Nav2 action server connected')

        # MoveIt2 client would be initialized here
        # self.moveit2_client = MoveItClient()

        # Subscriptions
        self.create_subscription(
            String,
            '/planned_actions',
            self.execute_sequence_callback,
            10
        )

        # Publishers
        self.feedback_pub = self.create_publisher(String, '/action_feedback', 10)
        self.status_pub = self.create_publisher(String, '/execution_status', 10)
        self.visual_query_pub = self.create_publisher(String, '/visual_query', 10)

        # State tracking
        self.current_action = None
        self.action_index = 0

        # Subscribe to visual grounding results
        self.detected_objects = {}
        self.create_subscription(
            String,
            '/detected_objects_3d',
            self.detection_callback,
            10
        )

        self.get_logger().info('Action Executor initialized')

    def execute_sequence_callback(self, msg):
        """Execute action sequence from LLM planner"""
        try:
            actions = json.loads(msg.data)
            self.get_logger().info(f'Executing sequence of {len(actions)} actions')

            for idx, action in enumerate(actions):
                self.action_index = idx
                self.current_action = action
                self.get_logger().info(f'Action {idx+1}/{len(actions)}: {action["name"]}')

                success = self.execute_action(action)

                if success:
                    self.publish_feedback(f'Action {idx+1} completed: {action["name"]}')
                else:
                    self.publish_feedback(f'Action {idx+1} failed: {action["name"]}')
                    self.get_logger().error('Sequence execution stopped due to failure')
                    return

            self.get_logger().info('All actions completed successfully')
            self.publish_status('sequence_complete')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse action sequence: {e}')
        except Exception as e:
            self.get_logger().error(f'Execution error: {e}')

    def execute_action(self, action):
        """Execute single action based on action name"""
        action_name = action['name']
        parameters = action['parameters']

        if action_name == 'navigate_to':
            return self.execute_navigation(parameters)
        elif action_name == 'grasp':
            return self.execute_grasp(parameters)
        elif action_name == 'release':
            return self.execute_release(parameters)
        elif action_name == 'detect_objects':
            return self.execute_detection(parameters)
        elif action_name == 'open':
            return self.execute_open(parameters)
        elif action_name == 'close':
            return self.execute_close(parameters)
        else:
            self.get_logger().error(f'Unknown action: {action_name}')
            return False

    def execute_navigation(self, parameters):
        """Execute navigation using Nav2"""
        self.get_logger().info(f'Navigating to {parameters.get("location", "position")}')

        # Validate parameters
        if 'x' not in parameters or 'y' not in parameters:
            self.get_logger().error('Missing x or y coordinates')
            return False

        # Create Nav2 goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(parameters['x'])
        goal_msg.pose.pose.position.y = float(parameters['y'])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0  # Facing forward

        # Send goal
        send_goal_future = self.nav2_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.nav_timeout)

        result = result_future.result()
        if result:
            self.get_logger().info('Navigation successful')
            return True
        else:
            self.get_logger().error('Navigation failed or timed out')
            return False

    def execute_grasp(self, parameters):
        """Execute grasp using MoveIt2"""
        self.get_logger().info(f'Grasping object: {parameters.get("object_id", "unknown")}')

        # Validate parameters
        if 'object_id' not in parameters:
            self.get_logger().error('Missing object_id')
            return False

        # Get object position from visual grounding
        object_id = parameters['object_id']
        if 'position' in parameters:
            position = parameters['position']
        elif object_id in self.detected_objects:
            position = self.detected_objects[object_id]['position']
        else:
            self.get_logger().error(f'Object {object_id} not found in detected objects')
            return False

        # TODO: Call MoveIt2 to plan and execute grasp
        # For now, simulate grasp
        self.get_logger().info(f'Simulating grasp at position {position}')
        time.sleep(2.0)  # Simulate execution time

        return True

    def execute_release(self, parameters):
        """Execute release using MoveIt2"""
        self.get_logger().info(f'Releasing object at {parameters.get("position", "current position")}')

        # Validate parameters
        if 'position' not in parameters:
            self.get_logger().error('Missing release position')
            return False

        position = parameters['position']

        # TODO: Call MoveIt2 to plan and execute release
        # For now, simulate release
        self.get_logger().info(f'Simulating release at position {position}')
        time.sleep(1.5)

        return True

    def execute_detection(self, parameters):
        """Execute object detection using visual grounding"""
        self.get_logger().info(f'Detecting objects: {parameters.get("query", "unknown")}')

        # Validate parameters
        if 'query' not in parameters:
            self.get_logger().error('Missing detection query')
            return False

        query = parameters['query']

        # Publish visual query
        msg = String()
        msg.data = query
        self.visual_query_pub.publish(msg)

        # Wait for detection results
        self.get_logger().info('Waiting for visual grounding results...')
        time.sleep(self.detect_timeout)

        # Check if objects were detected
        if query in self.detected_objects:
            self.get_logger().info(f'Detected: {self.detected_objects[query]}')
            return True
        else:
            self.get_logger().warn(f'No objects detected for query: {query}')
            return False

    def execute_open(self, parameters):
        """Execute open action (door, drawer, etc.)"""
        self.get_logger().info(f'Opening: {parameters.get("object_id", "unknown")}')

        # TODO: Implement opening logic with MoveIt2
        # For now, simulate
        time.sleep(2.0)
        return True

    def execute_close(self, parameters):
        """Execute close action (door, drawer, etc.)"""
        self.get_logger().info(f'Closing: {parameters.get("object_id", "unknown")}')

        # TODO: Implement closing logic with MoveIt2
        # For now, simulate
        time.sleep(2.0)
        return True

    def detection_callback(self, msg):
        """Handle visual grounding detection results"""
        try:
            detections = json.loads(msg.data)
            for detection in detections:
                label = detection['label']
                self.detected_objects[label] = detection
            self.get_logger().info(f'Received {len(detections)} detections')
        except Exception as e:
            self.get_logger().error(f'Failed to parse detections: {e}')

    def publish_feedback(self, message):
        """Publish action feedback"""
        msg = String()
        msg.data = message
        self.feedback_pub.publish(msg)

    def publish_status(self, status):
        """Publish execution status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ActionExecutor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
