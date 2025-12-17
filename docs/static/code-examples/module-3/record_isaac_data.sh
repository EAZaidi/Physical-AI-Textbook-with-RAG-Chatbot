#!/bin/bash
# Record ROS 2 sensor data from Isaac Sim
# Module 3, Chapter 1

set -e  # Exit on error

# Configuration
DURATION=60  # seconds
OUTPUT_DIR="isaac_sim_demo_$(date +%Y%m%d_%H%M%S)"

echo "🎬 Starting rosbag recording..."
echo "📁 Output directory: ${OUTPUT_DIR}"
echo "⏱️  Duration: ${DURATION} seconds"
echo ""

# Check if ROS 2 topics are available
echo "🔍 Checking for Isaac Sim topics..."
if ! ros2 topic list | grep -q "/camera/left/image_raw"; then
    echo "❌ Isaac Sim ROS 2 topics not detected!"
    echo "   Make sure:"
    echo "   1. Isaac Sim is running"
    echo "   2. ROS 2 Bridge extension is enabled"
    echo "   3. Simulation is playing (not paused)"
    echo "   4. ROS_DOMAIN_ID matches (echo \$ROS_DOMAIN_ID)"
    exit 1
fi

echo "✅ Topics detected:"
ros2 topic list | grep -E "(camera|scan|tf)" | head -10

echo ""
echo "📹 Recording for ${DURATION} seconds..."
echo "   Move the robot in Isaac Sim to capture varied poses!"

# Record rosbag
ros2 bag record -o "${OUTPUT_DIR}" \
  /camera/left/image_raw \
  /camera/right/image_raw \
  /camera/left/camera_info \
  /camera/right/camera_info \
  /scan \
  /tf \
  /tf_static \
  --duration ${DURATION}

echo ""
echo "✅ Recording complete!"
echo "📊 Rosbag info:"
ros2 bag info "${OUTPUT_DIR}"

echo ""
echo "🎥 To play back:"
echo "   ros2 bag play ${OUTPUT_DIR}"
echo ""
echo "👁️  To visualize:"
echo "   ros2 run rviz2 rviz2"
echo "   Add: Image (/camera/left/image_raw)"
echo "   Add: PointCloud2 (/scan)"
echo "   Set Fixed Frame: base_link"
