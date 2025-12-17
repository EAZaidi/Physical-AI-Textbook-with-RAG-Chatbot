#!/usr/bin/env python3
"""
Master Launch File: Complete Autonomous Navigation Stack
Isaac Sim + Isaac ROS VSLAM + Nav2
Module 3, Chapter 4
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    """
    Launch complete autonomous navigation pipeline.

    Components launched:
    1. Isaac ROS Visual SLAM (perception)
    2. Nav2 Stack (planning + control)
    3. RViz2 (visualization)

    Prerequisites:
    - Isaac Sim running with ROS 2 Bridge enabled
    - Docker container with Isaac ROS running
    - Sensors publishing on expected topics
    """

    code_examples_dir = os.path.dirname(os.path.abspath(__file__))

    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Launch RViz2 for visualization'
    )

    use_rviz = LaunchConfiguration('use_rviz')

    # 1. Isaac ROS Visual SLAM
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(code_examples_dir, 'isaac_ros_vslam_launch.py')
        ),
        launch_arguments={
            'enable_rectified_pose': 'True',
            'enable_debug_mode': 'False',
        }.items()
    )

    # 2. Nav2 Stack (delayed 10 seconds for VSLAM initialization)
    nav2_launch = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(code_examples_dir, 'nav2_bipedal_bringup.launch.py')
            )
        )]
    )

    # 3. RViz2 with full navigation visualization
    rviz_config = os.path.join(code_examples_dir, 'full_navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    return LaunchDescription([
        use_rviz_arg,
        vslam_launch,
        nav2_launch,
        rviz_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
