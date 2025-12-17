#!/bin/bash
# Run Isaac ROS VSLAM on recorded rosbag (offline processing)
# Module 3, Chapter 2

set -e

ROSBAG_DIR="${1:-isaac_sim_demo}"
PLAYBACK_RATE="${2:-0.5}"  # Default: half speed for better processing

if [ ! -d "$ROSBAG_DIR" ]; then
    echo "❌ Rosbag directory not found: $ROSBAG_DIR"
    echo "Usage: $0 <rosbag_directory> [playback_rate]"
    echo "Example: $0 isaac_sim_demo_20231208 0.5"
    exit 1
fi

echo "🚀 Starting Isaac ROS VSLAM in Docker..."
cd "$(dirname "$0")/docker"
docker-compose up -d

echo "⏳ Waiting for container to be ready..."
sleep 3

echo "📡 Launching VSLAM node in container..."
docker exec -d isaac_ros_vslam bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 launch /code-examples/isaac_ros_vslam_launch.py"

echo "⏳ Waiting for VSLAM node to initialize (10 seconds)..."
sleep 10

echo "🔍 Checking VSLAM node status..."
if ! docker exec isaac_ros_vslam bash -c \
  "source /opt/ros/humble/setup.bash && ros2 node list | grep -q visual_slam"; then
    echo "❌ VSLAM node failed to start!"
    echo "Check logs: docker logs isaac_ros_vslam"
    exit 1
fi

echo "✅ VSLAM node running"
echo ""
echo "🎥 Playing rosbag: $ROSBAG_DIR (rate: ${PLAYBACK_RATE}x)"
echo "   Slower playback allows GPU to process every frame"
echo ""

ros2 bag play "$ROSBAG_DIR" --rate "$PLAYBACK_RATE"

echo ""
echo "✅ VSLAM processing complete!"
echo ""
echo "📊 To visualize results:"
echo "   ros2 run rviz2 rviz2 -d /static/code-examples/module-3/vslam_visualization.rviz"
echo ""
echo "📈 To check VSLAM status:"
echo "   ros2 topic echo /visual_slam/status"
echo ""
echo "🛑 To stop Docker container:"
echo "   docker-compose -f $(dirname "$0")/docker/docker-compose.yml down"
