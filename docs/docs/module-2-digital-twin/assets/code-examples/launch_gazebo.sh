#!/bin/bash

# Launch Gazebo with simple_world.sdf
# Usage: ./launch_gazebo.sh

set -e

echo "Launching Gazebo with simple_world.sdf..."

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# World file path
WORLD_FILE="$SCRIPT_DIR/simple_world.sdf"

if [[ ! -f "$WORLD_FILE" ]]; then
    echo "Error: World file not found: $WORLD_FILE"
    exit 1
fi

# Source ROS 2 if available
if [[ -f "/opt/ros/humble/setup.bash" ]]; then
    source /opt/ros/humble/setup.bash
fi

# Launch Gazebo
gazebo "$WORLD_FILE"
