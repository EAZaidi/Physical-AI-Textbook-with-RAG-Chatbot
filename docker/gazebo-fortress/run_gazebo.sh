#!/bin/bash

# Gazebo Fortress Docker Launcher
# Supports Linux, WSL2, and macOS with X11 forwarding
# Usage: ./run_gazebo.sh [world_file]

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Gazebo Fortress Docker Launcher ===${NC}"

# Detect platform
PLATFORM="unknown"
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    PLATFORM="linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    PLATFORM="macos"
elif [[ -n "$WSL_DISTRO_NAME" ]]; then
    PLATFORM="wsl2"
else
    echo -e "${RED}Warning: Unknown platform. Attempting Linux-style launch...${NC}"
    PLATFORM="linux"
fi

echo -e "${YELLOW}Detected platform: $PLATFORM${NC}"

# Platform-specific DISPLAY configuration
if [[ "$PLATFORM" == "linux" ]]; then
    DISPLAY_VAR="$DISPLAY"
    # Allow Docker to access X server
    xhost +local:docker > /dev/null 2>&1 || echo -e "${YELLOW}Warning: xhost command not found. GUI may not work.${NC}"
elif [[ "$PLATFORM" == "macos" ]]; then
    DISPLAY_VAR="host.docker.internal:0"
    echo -e "${YELLOW}Note: Ensure XQuartz is running with 'Allow connections from network clients' enabled${NC}"
elif [[ "$PLATFORM" == "wsl2" ]]; then
    # WSL2: Get Windows host IP
    DISPLAY_VAR="$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0"
    echo -e "${YELLOW}Note: Ensure VcXsrv or X410 is running on Windows with 'Disable access control' enabled${NC}"
fi

# Check if DISPLAY is set
if [[ -z "$DISPLAY_VAR" ]]; then
    echo -e "${RED}Error: DISPLAY variable not set. GUI will not work.${NC}"
    echo -e "${YELLOW}For WSL2: export DISPLAY=\$(cat /etc/resolv.conf | grep nameserver | awk '{print \$2}'):0${NC}"
    echo -e "${YELLOW}For macOS: Ensure XQuartz is running${NC}"
    echo -e "${YELLOW}For Linux: DISPLAY should be set automatically${NC}"
    exit 1
fi

echo -e "${GREEN}DISPLAY set to: $DISPLAY_VAR${NC}"

# Get script directory (where docker-compose.yml is located)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Optional: World file argument
WORLD_FILE="${1:-}"
GAZEBO_CMD="gazebo"

if [[ -n "$WORLD_FILE" ]]; then
    if [[ -f "$WORLD_FILE" ]]; then
        echo -e "${GREEN}Loading world file: $WORLD_FILE${NC}"
        GAZEBO_CMD="gazebo $WORLD_FILE"
    else
        echo -e "${RED}Error: World file not found: $WORLD_FILE${NC}"
        exit 1
    fi
fi

# Run Docker container with GUI support
echo -e "${GREEN}Launching Gazebo in Docker...${NC}"

docker run -it --rm \
    --name gazebo_fortress_module2 \
    --env DISPLAY="$DISPLAY_VAR" \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "$SCRIPT_DIR/../..:/workspace:rw" \
    --network host \
    physical-ai-gazebo-fortress:latest \
    bash -c "source /opt/ros/humble/setup.bash && $GAZEBO_CMD"

echo -e "${GREEN}Gazebo closed.${NC}"
