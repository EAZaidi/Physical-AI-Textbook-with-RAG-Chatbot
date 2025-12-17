#!/usr/bin/env python3
"""
Nav2 Launch File for Bipedal Humanoid Robot
Module 3, Chapter 3
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch Nav2 stack with bipedal-tuned parameters.

    Prerequisites:
    - SLAM map available on /map topic (or use map_server)
    - Localization running (AMCL or Visual SLAM)
    - Sensor data (LiDAR on /scan for obstacle avoidance)
    """

    # Get Nav2 bringup package directory
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Path to custom params file
    params_file = os.path.join(
        os.path.dirname(__file__),
        'nav2_params_bipedal.yaml'
    )

    # Include Nav2 bringup launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'True',  # Use simulation time from rosbag/Isaac Sim
            'autostart': 'True',     # Auto-start all lifecycle nodes
        }.items()
    )

    return LaunchDescription([
        nav2_launch
    ])


if __name__ == '__main__':
    generate_launch_description()
