#!/bin/bash

# VLA System Environment Setup Script
# Sets up Python environment and installs all dependencies for Module 4

set -e

echo "=== VLA System Environment Setup ==="

# Check Python version
python_version=$(python3 --version 2>&1 | awk '{print $2}')
echo "Python version: $python_version"

if ! python3 -c 'import sys; exit(0 if sys.version_info >= (3, 10) else 1)' 2>/dev/null; then
    echo "ERROR: Python 3.10+ required"
    exit 1
fi

# Check ROS 2 installation
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS 2 not sourced. Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "ROS 2 distro: $ROS_DISTRO"

# Create virtual environment (optional)
read -p "Create Python virtual environment? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    python3 -m venv vla_env
    source vla_env/bin/activate
    echo "Virtual environment created and activated"
fi

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install dependencies
echo "Installing Python dependencies..."
pip install -r requirements.txt

# Check API keys
echo ""
echo "=== API Key Configuration ==="

if [ -z "$OPENAI_API_KEY" ]; then
    echo "WARNING: OPENAI_API_KEY not set"
    echo "Set with: export OPENAI_API_KEY='your-key-here'"
fi

if [ -z "$ANTHROPIC_API_KEY" ]; then
    echo "WARNING: ANTHROPIC_API_KEY not set"
    echo "Set with: export ANTHROPIC_API_KEY='your-key-here'"
fi

# Download Grounding DINO model (optional)
echo ""
read -p "Download Grounding DINO model (~300MB)? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "Downloading Grounding DINO..."
    mkdir -p models
    cd models

    # Download config
    wget -q https://raw.githubusercontent.com/IDEA-Research/GroundingDINO/main/groundingdino/config/GroundingDINO_SwinT_OGC.py

    # Download checkpoint
    wget -q https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth

    cd ..
    echo "Grounding DINO model downloaded"
fi

# Test installations
echo ""
echo "=== Testing Installations ==="

python3 -c "import faster_whisper; print('✓ faster-whisper')" || echo "✗ faster-whisper"
python3 -c "import openai; print('✓ openai')" || echo "✗ openai"
python3 -c "import anthropic; print('✓ anthropic')" || echo "✗ anthropic"
python3 -c "import langchain; print('✓ langchain')" || echo "✗ langchain"
python3 -c "import cv2; print('✓ opencv-python')" || echo "✗ opencv-python"
python3 -c "import rclpy; print('✓ rclpy')" || echo "✗ rclpy"

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Next steps:"
echo "1. Set API keys: export OPENAI_API_KEY='...' or export ANTHROPIC_API_KEY='...'"
echo "2. Build ROS 2 package: colcon build --packages-select vla_system"
echo "3. Source workspace: source install/setup.bash"
echo "4. Launch system: ros2 launch vla_system vla_system_launch.py"
echo ""
