#!/usr/bin/env python3
"""
Live Integration: Isaac Sim → Isaac ROS VSLAM
Module 3, Chapter 2 - Real-time VSLAM with Isaac Sim streaming
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    """
    Launch Isaac ROS VSLAM for live Isaac Sim data.

    Prerequisites:
    1. Isaac Sim running with ROS 2 Bridge enabled
    2. Robot with stereo cameras publishing on expected topics
    3. Docker container with Isaac ROS running

    ROS 2 Topics (from Isaac Sim):
    - /camera/left/image_raw
    - /camera/right/image_raw
    - /camera/left/camera_info
    - /camera/right/camera_info
    - /tf
    - /tf_static
    """

    # Get code examples directory
    code_examples_dir = os.path.dirname(os.path.abspath(__file__))

    # Include Isaac ROS VSLAM launch
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(code_examples_dir, 'isaac_ros_vslam_launch.py')
        ),
        launch_arguments={
            'enable_rectified_pose': 'True',
            'enable_imu_fusion': 'False',  # No IMU in this tutorial
            'enable_debug_mode': 'False',  # Set True for debugging
        }.items()
    )

    # Launch RViz2 with pre-configured visualization
    rviz_config_path = os.path.join(code_examples_dir, 'vslam_visualization.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Optional: Static TF publisher (if not provided by Isaac Sim)
    # Publishes map → odom transform
    # Comment out if Isaac Sim publishes this
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_to_odom_publisher',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    # )

    return LaunchDescription([
        vslam_launch,
        rviz_node,
        # static_tf_node,  # Uncomment if needed
    ])


if __name__ == '__main__':
    generate_launch_description()
