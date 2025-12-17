# Docker testing environment for Module 1: ROS 2 code examples
# Based on official ROS 2 Humble Desktop image (Ubuntu 22.04)

FROM osrf/ros:humble-desktop

# Set environment variables
ENV LANG=en_US.UTF-8
ENV ROS_DISTRO=humble

# Install additional Python dependencies for code examples
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create workspace for code examples
WORKDIR /workspace

# Source ROS 2 setup in bashrc for interactive use
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Default command: bash (for interactive testing)
CMD ["/bin/bash"]

# Usage:
# Build: docker build -t ros2-module1-testing -f docker/ros2-testing/Dockerfile .
# Run: docker run -it --rm -v $(pwd)/docs/module-1-ros2/assets/code-examples:/workspace ros2-module1-testing
# Test example: python3 /workspace/publisher_example.py
