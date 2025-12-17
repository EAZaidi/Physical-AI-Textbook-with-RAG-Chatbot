#!/usr/bin/env python3
"""
Isaac ROS Visual SLAM Launch File for Stereo Camera
Feature: 003-isaac-ai-brain
Reference: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch Isaac ROS Visual SLAM with stereo camera configuration.

    Expected input topics (from Isaac Sim):
    - /camera/left/image_raw (sensor_msgs/Image)
    - /camera/right/image_raw (sensor_msgs/Image)
    - /camera/left/camera_info (sensor_msgs/CameraInfo)
    - /camera/right/camera_info (sensor_msgs/CameraInfo)

    Output topics:
    - /visual_slam/tracking/odometry (nav_msgs/Odometry)
    - /visual_slam/tracking/vo_pose_covariance (geometry_msgs/PoseWithCovarianceStamped)
    - /visual_slam/vis/map_points (sensor_msgs/PointCloud2)
    """

    # Launch arguments
    enable_rectified_pose = DeclareLaunchArgument(
        'enable_rectified_pose',
        default_value='True',
        description='Enable pose rectification for improved accuracy'
    )

    enable_imu_fusion = DeclareLaunchArgument(
        'enable_imu_fusion',
        default_value='False',
        description='Enable IMU fusion (requires IMU topic)'
    )

    enable_debug_mode = DeclareLaunchArgument(
        'enable_debug_mode',
        default_value='False',
        description='Enable debug visualization (increases CPU load)'
    )

    # Visual SLAM node
    visual_slam_node = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam',
        namespace='',
        output='screen',
        parameters=[{
            # Camera configuration
            'num_cameras': 2,  # Stereo camera
            'camera_optical_frames': ['camera_left_optical_frame', 'camera_right_optical_frame'],

            # Frame IDs
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_frame': 'odom',

            # SLAM parameters
            'enable_rectified_pose': LaunchConfiguration('enable_rectified_pose'),
            'enable_imu_fusion': LaunchConfiguration('enable_imu_fusion'),
            'enable_debug_mode': LaunchConfiguration('enable_debug_mode'),

            # Tracking parameters
            'enable_observations_view': True,
            'enable_landmarks_view': True,
            'enable_reading_slam_internals': True,
            'enable_slam_visualization': True,

            # Map parameters
            'enable_localization_n_mapping': True,
            'enable_map_preservation': False,  # Start fresh each time (educational)

            # Performance tuning
            'rectified_images': True,
            'enable_ground_constraint_in_odometry': False,  # Humanoid not wheeled
            'enable_ground_constraint_in_slam': False,

            # Feature tracking
            'min_num_images': 2,  # Stereo pair
            'max_num_images': 2,

            # Verbosity
            'verbosity': 3,  # INFO level for students
            'enable_verbosity': True,
        }],
        remappings=[
            # Input image topics (from Isaac Sim)
            ('stereo_camera/left/image', '/camera/left/image_raw'),
            ('stereo_camera/right/image', '/camera/right/image_raw'),
            ('stereo_camera/left/camera_info', '/camera/left/camera_info'),
            ('stereo_camera/right/camera_info', '/camera/right/camera_info'),

            # Output odometry (to Nav2)
            ('visual_slam/tracking/odometry', '/visual_slam/tracking/odometry'),
            ('visual_slam/tracking/vo_pose_covariance', '/visual_slam/tracking/vo_pose_covariance'),

            # Visualization (RViz2)
            ('visual_slam/vis/map_points', '/visual_slam/vis/map_points'),
            ('visual_slam/vis/slam_odometry', '/visual_slam/vis/slam_odometry'),
        ]
    )

    return LaunchDescription([
        enable_rectified_pose,
        enable_imu_fusion,
        enable_debug_mode,
        visual_slam_node
    ])
