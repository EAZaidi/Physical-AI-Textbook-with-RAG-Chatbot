# Dockerfile for Gazebo Fortress + ROS 2 Humble
# Based on official OSRF ROS 2 Humble desktop image
# Purpose: Reproducible Gazebo simulation environment for Module 2

FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install Gazebo Fortress and additional tools
RUN apt-get update && apt-get install -y \
    gazebo \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-rosbag2 \
    python3-pip \
    git \
    wget \
    curl \
    nano \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for sensor data processing
RUN pip3 install \
    open3d \
    matplotlib \
    pandas \
    numpy \
    opencv-python

# Create workspace directory
RUN mkdir -p /workspace
WORKDIR /workspace

# Source ROS 2 setup in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

# Default command: launch bash
CMD ["/bin/bash"]
