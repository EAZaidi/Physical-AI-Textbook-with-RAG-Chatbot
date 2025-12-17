#!/usr/bin/env python3
"""
Visual Grounding Node for ROS 2

Uses Grounding DINO for open-vocabulary object detection.
Converts 2D bounding boxes to 3D poses using depth camera.
Publishes detections to /detected_objects_3d.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import json


class VisualGroundingNode(Node):
    def __init__(self):
        super().__init__('visual_grounding_node')

        # Declare parameters
        self.declare_parameter('box_threshold', 0.35)
        self.declare_parameter('text_threshold', 0.25)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('model_config', 'GroundingDINO_SwinT_OGC.py')
        self.declare_parameter('model_checkpoint', 'groundingdino_swint_ogc.pth')

        # Get parameters
        self.box_threshold = self.get_parameter('box_threshold').value
        self.text_threshold = self.get_parameter('text_threshold').value
        self.device = self.get_parameter('device').value
        model_config = self.get_parameter('model_config').value
        model_checkpoint = self.get_parameter('model_checkpoint').value

        # Initialize Grounding DINO model
        self.get_logger().info('Loading Grounding DINO model...')
        try:
            from groundingdino.util.inference import Model
            self.model = Model(
                model_config_path=model_config,
                model_checkpoint_path=model_checkpoint,
                device=self.device
            )
            self.get_logger().info('Grounding DINO loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load Grounding DINO: {e}')
            self.get_logger().warn('Running in simulation mode (no actual detection)')
            self.model = None

        # CV Bridge for ROS image conversion
        self.bridge = CvBridge()

        # Camera intrinsics
        self.camera_fx = None
        self.camera_fy = None
        self.camera_cx = None
        self.camera_cy = None

        # Latest images
        self.latest_rgb = None
        self.latest_depth = None

        # Subscriptions
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        self.query_sub = self.create_subscription(
            String,
            '/visual_query',
            self.query_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            String,
            '/detected_objects_3d',
            10
        )

        self.get_logger().info('Visual Grounding Node initialized')

    def rgb_callback(self, msg):
        """Store latest RGB image"""
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}')

    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')

    def camera_info_callback(self, msg):
        """Store camera intrinsics"""
        if self.camera_fx is None:
            self.camera_fx = msg.k[0]
            self.camera_fy = msg.k[4]
            self.camera_cx = msg.k[2]
            self.camera_cy = msg.k[5]
            self.get_logger().info(f'Camera intrinsics received: fx={self.camera_fx}, fy={self.camera_fy}')

    def query_callback(self, msg):
        """Handle visual grounding queries"""
        query = msg.data
        self.get_logger().info(f'Visual query: "{query}"')

        # Check if we have camera data
        if self.latest_rgb is None:
            self.get_logger().warn('No RGB image available')
            return

        if self.latest_depth is None:
            self.get_logger().warn('No depth image available')
            return

        if self.camera_fx is None:
            self.get_logger().warn('No camera intrinsics available')
            return

        # Run detection
        detections_3d = self.detect_and_localize(query)

        # Publish results
        if detections_3d:
            self.publish_detections(detections_3d)
        else:
            self.get_logger().warn(f'No objects detected for query: "{query}"')

    def detect_and_localize(self, query):
        """Run Grounding DINO and convert to 3D poses"""
        if self.model is None:
            # Simulation mode: return fake detection
            self.get_logger().info('Simulation mode: returning fake detection')
            return [{
                'label': query,
                'confidence': 0.95,
                'position': [1.0, 0.5, 0.8],
                'bbox_2d': [100, 100, 200, 200]
            }]

        try:
            # Run Grounding DINO
            detections = self.model.predict_with_caption(
                image=self.latest_rgb,
                caption=query,
                box_threshold=self.box_threshold,
                text_threshold=self.text_threshold
            )

            # Convert to 3D
            detections_3d = []
            for bbox, score, label in zip(
                detections.xyxy,
                detections.confidence,
                detections.class_names
            ):
                # Get bbox center
                x1, y1, x2, y2 = bbox
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # Get depth at center
                depth = self.latest_depth[cy, cx]

                # Check for valid depth
                if np.isnan(depth) or depth <= 0.0:
                    self.get_logger().warn(f'Invalid depth at ({cx}, {cy}): {depth}')
                    continue

                # Convert to 3D (camera frame)
                x_3d = (cx - self.camera_cx) * depth / self.camera_fx
                y_3d = (cy - self.camera_cy) * depth / self.camera_fy
                z_3d = depth

                detections_3d.append({
                    'label': label,
                    'confidence': float(score),
                    'position': [float(x_3d), float(y_3d), float(z_3d)],
                    'bbox_2d': [float(x1), float(y1), float(x2), float(y2)]
                })

                self.get_logger().info(
                    f'Detected "{label}" at ({x_3d:.2f}, {y_3d:.2f}, {z_3d:.2f}) '
                    f'with confidence {score:.2f}'
                )

            return detections_3d

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
            return []

    def publish_detections(self, detections_3d):
        """Publish 3D detections as JSON"""
        msg = String()
        msg.data = json.dumps(detections_3d)
        self.detection_pub.publish(msg)
        self.get_logger().info(f'Published {len(detections_3d)} detections')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = VisualGroundingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
