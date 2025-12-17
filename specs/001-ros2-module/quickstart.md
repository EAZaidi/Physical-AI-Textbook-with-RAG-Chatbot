# Quickstart: Module 1 - The Robotic Nervous System (ROS 2)

**Date**: 2025-12-07
**Purpose**: Environment setup and validation guide for Module 1
**Target Audience**: Students preparing to study ROS 2 fundamentals

---

## Prerequisites

Before starting Module 1, ensure you have:

- **Operating System**: Ubuntu 22.04 LTS (native, WSL2, or Docker)
- **Python**: Version 3.10 or later
- **Terminal**: Basic familiarity with command-line (cd, ls, mkdir, source)
- **Text Editor**: VS Code, Sublime, Vim, or any code editor
- **Internet**: For downloading ROS 2 packages and documentation

**Note**: Module 1 focuses on software concepts. No physical robot hardware required.

---

## Setup Option 1: Native Ubuntu 22.04 (Recommended)

### Step 1: Install ROS 2 Humble

Follow official ROS 2 installation instructions:

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop (includes RViz2)
sudo apt update
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep
```

**Source**: [ROS 2 Humble Installation (Ubuntu)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Step 2: Source ROS 2 Workspace

Add to `~/.bashrc` for automatic sourcing:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Verify Installation

```bash
# Check ROS 2 version
ros2 --version
# Expected output: ros2 cli version x.x.x

# Check available commands
ros2 --help

# Test with demo nodes (optional)
ros2 run demo_nodes_py talker
# Open new terminal: ros2 run demo_nodes_py listener
```

---

## Setup Option 2: Docker (Cross-Platform)

### Step 1: Install Docker

- **Ubuntu/Debian**: `sudo apt install docker.io`
- **Windows**: [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop)
- **macOS**: [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop)

### Step 2: Pull ROS 2 Humble Docker Image

```bash
docker pull osrf/ros:humble-desktop
```

### Step 3: Run ROS 2 Container

```bash
# Run interactive container
docker run -it --rm \
  --name ros2-learning \
  osrf/ros:humble-desktop \
  bash

# Inside container, verify installation
ros2 --version
```

### Step 4: Mount Code Examples (for Chapter 3)

```bash
# Clone textbook repository (example)
git clone https://github.com/your-org/physical-ai-humanoid-book.git
cd physical-ai-humanoid-book

# Run container with mounted code examples
docker run -it --rm \
  -v $(pwd)/docs/module-1-ros2/assets/code-examples:/workspace \
  osrf/ros:humble-desktop \
  bash

# Inside container
cd /workspace
python3 publisher_example.py
```

**Source**: [ROS 2 Docker Tutorial](https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)

---

## Setup Option 3: WSL2 (Windows Users)

### Step 1: Enable WSL2

```powershell
# Run in PowerShell as Administrator
wsl --install
# Restart computer
```

### Step 2: Install Ubuntu 22.04 from Microsoft Store

- Open Microsoft Store
- Search "Ubuntu 22.04"
- Install and launch

### Step 3: Follow Native Ubuntu Instructions

Once inside Ubuntu 22.04 WSL2, follow "Setup Option 1" (Native Ubuntu) steps.

**Note**: RViz2 visualization requires X Server on Windows. Install VcXsrv or use Docker instead for visualization.

**Source**: [WSL2 Installation](https://learn.microsoft.com/en-us/windows/wsl/install)

---

## Validation Steps

After setup, verify your environment is ready for Module 1:

### Check 1: ROS 2 Commands Available

```bash
ros2 --version
ros2 topic list
ros2 node list
```

**Expected Output**: Commands execute without errors (node/topic lists may be empty initially).

### Check 2: Python rclpy Available

```bash
python3 -c "import rclpy; print('rclpy version:', rclpy.__version__)"
```

**Expected Output**: `rclpy version: x.x.x` (no import errors)

### Check 3: RViz2 Launches (Optional for Chapter 1-3)

```bash
rviz2
```

**Expected Output**: RViz2 window opens (required for Chapter 4 - URDF visualization)

### Check 4: Run Demo Nodes

Terminal 1:
```bash
ros2 run demo_nodes_py talker
```

Terminal 2:
```bash
ros2 run demo_nodes_py listener
```

**Expected Output**: Talker publishes messages, listener receives and logs them.

---

## Troubleshooting Common Issues

### Issue: `ros2: command not found`

**Solution**: Source ROS 2 workspace:
```bash
source /opt/ros/humble/setup.bash
```
Add to `~/.bashrc` to persist.

### Issue: `ModuleNotFoundError: No module named 'rclpy'`

**Solution**: Ensure ROS 2 is sourced AND Python 3.10+ is used:
```bash
source /opt/ros/humble/setup.bash
python3 --version  # Should be 3.10+
```

### Issue: RViz2 won't launch in WSL2

**Solution**: WSL2 requires X Server (VcXsrv) for GUI apps. Easier to use Docker instead:
```bash
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:humble-desktop \
  rviz2
```

### Issue: Docker container exits immediately

**Solution**: Add `-it` flags for interactive terminal:
```bash
docker run -it osrf/ros:humble-desktop bash
```

---

## Module 1 Learning Path

Once environment is set up, proceed through chapters in order:

1. **Chapter 1: ROS 2 Overview and System Architecture** (conceptual, no coding)
2. **Chapter 2: Nodes, Topics, and Services** (conceptual with code previews)
3. **Chapter 3: Python-to-ROS Control via rclpy** (hands-on coding, requires environment)
4. **Chapter 4: URDF Basics for Humanoid Robots** (URDF editing, RViz2 visualization)

**Estimated Time**: 8-10 hours total (including exercises)

---

## Code Examples Repository Structure

Module 1 code examples will be located at:

```
docs/module-1-ros2/assets/code-examples/
├── publisher_example.py
├── subscriber_example.py
├── service_server_example.py
├── service_client_example.py
├── joint_state_publisher.py
├── joint_state_subscriber.py
├── simple_humanoid.urdf
└── humanoid_with_camera.urdf
```

**Usage**:
```bash
# Navigate to code examples
cd docs/module-1-ros2/assets/code-examples/

# Run publisher example
python3 publisher_example.py

# In new terminal, run subscriber
python3 subscriber_example.py
```

---

## Docker-Based Testing (For Content Developers)

Module 1 code examples are validated in Docker to ensure reproducibility:

### Automated Testing

The project includes an automated test script that validates all code examples in a clean Docker environment:

```bash
# Test all Python examples
./scripts/test-code-examples.sh
```

**How it works**:
1. Builds Docker image from `docker/ros2-testing/Dockerfile` (based on `osrf/ros:humble-desktop`)
2. Mounts code examples directory into container
3. Tests each Python file for syntax validity
4. Reports pass/fail status for each example

**Expected Output**: All examples pass syntax validation

### Manual Testing

Test individual examples:

```bash
# Build Docker image once
docker build -t ros2-module1-testing -f docker/ros2-testing/Dockerfile .

# Test single example
docker run --rm \
  -v $(pwd)/docs/module-1-ros2/assets/code-examples:/workspace \
  ros2-module1-testing \
  bash -c "source /opt/ros/humble/setup.bash && python3 /workspace/publisher_example.py"
```

### CI/CD Integration

GitHub Actions automatically runs code validation on every push:
- **Workflow**: `.github/workflows/test-code-examples.yml`
- **Triggers**: Changes to code examples, Docker config, or test script
- **Process**: Builds Docker image, runs test suite, reports results

**Check CI status**: See Actions tab in GitHub repository

### Adding New Code Examples

When adding new Python code examples:

1. Create file in `docs/module-1-ros2/assets/code-examples/`
2. Ensure it follows ROS 2 conventions (imports rclpy, defines Node class)
3. Run local test: `./scripts/test-code-examples.sh`
4. Verify example appears in test output
5. Push to trigger CI validation

**Quality Gate**: All code examples must pass Docker testing before merge

---

## Next Steps

After completing environment setup:

1. ✅ Verify all validation checks pass
2. ✅ Bookmark official ROS 2 documentation: https://docs.ros.org/en/humble/
3. ✅ Begin Chapter 1: ROS 2 Overview and System Architecture
4. ✅ Join ROS 2 community: https://discourse.ros.org/

---

## Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials (Beginner)](https://docs.ros.org/en/humble/Tutorials/Beginner.html)
- [ROS 2 Docker Images](https://hub.docker.com/r/osrf/ros)
- [ROS 2 Community Forum](https://discourse.ros.org/)
- [Physical AI & Humanoid Robotics Textbook GitHub](https://github.com/your-org/physical-ai-humanoid-book)

---

## Feedback

Encountered setup issues? Please report in GitHub Issues:
https://github.com/your-org/physical-ai-humanoid-book/issues

---

**Last Updated**: 2025-12-07
**ROS 2 Version**: Humble (LTS)
**Target OS**: Ubuntu 22.04 LTS
