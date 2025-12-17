#!/bin/bash
# Automated Full Navigation Demo
# Module 3, Chapter 4 - Isaac Sim + Isaac ROS + Nav2

set -e

echo "🚀 Starting Full Navigation Demo..."
echo "=================================================="
echo ""

# 1. Verify Isaac Sim is running
echo "🔍 Step 1: Checking Isaac Sim connection..."
if ! ros2 topic list | grep -q "/camera/left/image_raw"; then
    echo "❌ Isaac Sim not detected!"
    echo "   Please:"
    echo "   1. Open Isaac Sim"
    echo "   2. Load humanoid robot with sensors"
    echo "   3. Enable ROS 2 Bridge extension"
    echo "   4. Press Play button"
    exit 1
fi
echo "✅ Isaac Sim detected (topics publishing)"
echo ""

# 2. Launch Docker container
echo "📦 Step 2: Starting Isaac ROS Docker container..."
cd "$(dirname "$0")/docker"
docker-compose up -d
echo "✅ Docker container running"
sleep 3
echo ""

# 3. Launch full navigation stack
echo "🧠 Step 3: Launching perception + navigation stack..."
docker exec -d isaac_ros_vslam bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 launch /code-examples/isaac_nav_full.launch.py"

echo "⏳ Waiting for system initialization (15 seconds)..."
sleep 15
echo ""

# 4. Verify all nodes running
echo "🔍 Step 4: Verifying system status..."
if docker exec isaac_ros_vslam bash -c \
  "source /opt/ros/humble/setup.bash && \
   ros2 node list" | grep -q "visual_slam"; then
    echo "✅ Isaac ROS VSLAM running"
else
    echo "❌ VSLAM failed to start"
    exit 1
fi

if ros2 node list | grep -q "bt_navigator"; then
    echo "✅ Nav2 running"
else
    echo "❌ Nav2 failed to start"
    exit 1
fi
echo ""

# 5. Display system info
echo "📊 System Status:"
echo "---------------------------------------------------"
echo "🎥 Camera rate: $(ros2 topic hz /camera/left/image_raw --window 10 2>/dev/null || echo 'N/A')"
echo "🗺️  VSLAM rate: $(ros2 topic hz /visual_slam/tracking/odometry --window 10 2>/dev/null || echo 'N/A')"
echo "🎮 Nav2 status: $(ros2 topic echo /diagnostics --once 2>/dev/null | grep -o 'OK' || echo 'Initializing...')"
echo "---------------------------------------------------"
echo ""

# 6. Send navigation goal
echo "🎯 Step 5: Sending navigation goal..."
echo "   Goal: x=15.0m, y=5.0m, yaw=0.0 rad"
python3 ../send_nav_goal.py 15.0 5.0 0.0 &

echo ""
echo "✅ Full Navigation Demo Launched!"
echo "=================================================="
echo ""
echo "📺 Monitor progress:"
echo "   RViz2 should be open with full visualization"
echo "   Watch robot navigate autonomously to goal"
echo ""
echo "📊 Check performance:"
echo "   GPU: watch -n 1 nvidia-smi"
echo "   Topics: ros2 topic hz /cmd_vel"
echo ""
echo "🛑 To stop:"
echo "   docker-compose -f $(dirname "$0")/docker/docker-compose.yml down"
