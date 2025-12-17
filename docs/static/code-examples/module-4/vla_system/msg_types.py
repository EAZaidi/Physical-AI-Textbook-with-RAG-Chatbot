"""Custom ROS 2 message type definitions and utilities.

This module provides utilities for working with ROS 2 messages
in the VLA system, including message creation and conversion helpers.
"""

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from std_msgs.msg import Header, String
from nav_msgs.msg import Odometry
import rclpy
from rclpy.time import Time


def create_pose(x: float, y: float, z: float = 0.0,
                qx: float = 0.0, qy: float = 0.0,
                qz: float = 0.0, qw: float = 1.0) -> Pose:
    """Create a geometry_msgs/Pose message.

    Args:
        x, y, z: Position coordinates
        qx, qy, qz, qw: Quaternion orientation (default: identity)

    Returns:
        Pose message
    """
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def create_pose_stamped(x: float, y: float, z: float,
                        frame_id: str, node) -> PoseStamped:
    """Create a geometry_msgs/PoseStamped message.

    Args:
        x, y, z: Position coordinates
        frame_id: Reference frame
        node: ROS 2 node (for clock)

    Returns:
        PoseStamped message
    """
    pose_stamped = PoseStamped()
    pose_stamped.header = Header()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.header.stamp = node.get_clock().now().to_msg()
    pose_stamped.pose = create_pose(x, y, z)
    return pose_stamped


def create_detection_3d(object_id: str, class_name: str,
                        pose: Pose, confidence: float) -> Detection3D:
    """Create a vision_msgs/Detection3D message.

    Args:
        object_id: Unique object identifier
        class_name: Object class (e.g., "cup", "book")
        pose: 3D pose of object
        confidence: Detection confidence [0, 1]

    Returns:
        Detection3D message
    """
    detection = Detection3D()

    # Set object hypothesis
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = class_name
    hypothesis.hypothesis.score = confidence
    hypothesis.pose.pose = pose

    detection.results.append(hypothesis)
    detection.id = object_id

    return detection


def create_detection_array(detections: list, frame_id: str,
                          node) -> Detection3DArray:
    """Create a vision_msgs/Detection3DArray message.

    Args:
        detections: List of Detection3D messages
        frame_id: Reference frame
        node: ROS 2 node (for clock)

    Returns:
        Detection3DArray message
    """
    array = Detection3DArray()
    array.header = Header()
    array.header.frame_id = frame_id
    array.header.stamp = node.get_clock().now().to_msg()
    array.detections = detections
    return array


def pose_to_dict(pose: Pose) -> dict:
    """Convert Pose message to dictionary.

    Args:
        pose: Pose message

    Returns:
        Dictionary with position and orientation
    """
    return {
        "position": {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
        },
        "orientation": {
            "x": pose.orientation.x,
            "y": pose.orientation.y,
            "z": pose.orientation.z,
            "w": pose.orientation.w,
        }
    }


def dict_to_pose(d: dict) -> Pose:
    """Convert dictionary to Pose message.

    Args:
        d: Dictionary with position and orientation keys

    Returns:
        Pose message
    """
    pose = Pose()

    pos = d.get("position", {})
    pose.position.x = pos.get("x", 0.0)
    pose.position.y = pos.get("y", 0.0)
    pose.position.z = pos.get("z", 0.0)

    ori = d.get("orientation", {})
    pose.orientation.x = ori.get("x", 0.0)
    pose.orientation.y = ori.get("y", 0.0)
    pose.orientation.z = ori.get("z", 0.0)
    pose.orientation.w = ori.get("w", 1.0)

    return pose
