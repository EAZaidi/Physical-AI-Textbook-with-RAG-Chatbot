# Quick Start Guide: Module 2 - Digital Twin Environment Setup

**Module**: Module 2 - The Digital Twin (Gazebo & Unity)
**Version**: 1.0
**Last Updated**: 2025-12-07
**Supported Platforms**: Linux (Ubuntu 22.04), Windows 10/11 (WSL2), macOS (Intel/Apple Silicon)

## Overview

This guide provides step-by-step instructions to set up the complete simulation environment for Module 2. By the end, you will have:

✅ **Gazebo Fortress** running in Docker (physics simulation)
✅ **Unity 2022.3 LTS** installed natively (visual environments)
✅ **ROS 2 Humble** with sensor plugins (sensor simulation)
✅ **All dependencies** for code examples (Python packages, visualization tools)

**Estimated Setup Time**: 30-60 minutes (depending on download speeds)

---

## Prerequisites

### Hardware Requirements

**Minimum**:
- CPU: 4 cores (Intel i5 / AMD Ryzen 5 or equivalent)
- RAM: 8 GB
- GPU: Integrated graphics (Intel HD 4000 / AMD Radeon Vega or better)
- Disk Space: 20 GB free (10 GB for Docker images, 8 GB for Unity, 2 GB for dependencies)

**Recommended**:
- CPU: 6+ cores (Intel i7 / AMD Ryzen 7 or equivalent)
- RAM: 16 GB
- GPU: Dedicated graphics (NVIDIA GTX 1060 / AMD RX 580 or better)
- Disk Space: 30 GB free

**Notes**:
- Unity benefits significantly from GPU acceleration (60 FPS target)
- Gazebo in Docker runs well on CPU, but GPU helps for complex scenes
- WSL2 on Windows has limited GPU support for GUI applications (use native Linux if possible)

### Software Prerequisites

- **Docker**: Version 20.10+ (for Gazebo containerization)
- **Git**: Version 2.25+ (for cloning repositories and Unity packages)
- **Python**: 3.8+ (for sensor data processing scripts)

---

## Part 1: Gazebo Fortress Setup (Docker)

### 1.1 Install Docker

#### Linux (Ubuntu 22.04)

```bash
# Update package index
sudo apt update

# Install Docker dependencies
sudo apt install -y ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up Docker repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Verify installation
docker --version  # Should show version 20.10+

# Add user to docker group (avoid using sudo for docker commands)
sudo usermod -aG docker $USER
newgrp docker  # Activate group change immediately
```

#### Windows 10/11 (WSL2)

1. **Install WSL2**:
   ```powershell
   # Run in PowerShell as Administrator
   wsl --install
   wsl --set-default-version 2
   ```

2. **Install Docker Desktop**:
   - Download from: https://www.docker.com/products/docker-desktop
   - Run installer, enable WSL2 backend during setup
   - Restart computer

3. **Verify Installation**:
   ```bash
   # In WSL2 Ubuntu terminal
   docker --version
   docker run hello-world  # Test Docker works
   ```

#### macOS

1. **Install Docker Desktop**:
   - Download from: https://www.docker.com/products/docker-desktop
   - Drag Docker.app to Applications folder
   - Launch Docker Desktop, wait for whale icon to stabilize

2. **Verify Installation**:
   ```bash
   docker --version
   docker run hello-world
   ```

---

### 1.2 Set Up X11 Forwarding (for Gazebo GUI)

Gazebo runs in a Docker container but needs to display GUI on your host machine. This requires X11 server configuration.

#### Linux (Native X11)

```bash
# Allow Docker containers to access X server
xhost +local:docker

# Add to ~/.bashrc to persist across sessions
echo "xhost +local:docker > /dev/null 2>&1" >> ~/.bashrc
```

#### Windows (WSL2) - Option 1: VcXsrv (Free)

1. **Download VcXsrv**:
   - URL: https://sourceforge.net/projects/vcxsrv/
   - Install with default settings

2. **Launch XLaunch**:
   - Display number: `0`
   - Start no client: ✅
   - Disable access control: ✅ (IMPORTANT)
   - Save configuration for future use

3. **Set DISPLAY in WSL2**:
   ```bash
   # Add to ~/.bashrc
   export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
   ```

4. **Restart WSL2 terminal** and verify:
   ```bash
   echo $DISPLAY  # Should show something like 172.x.x.x:0
   ```

#### Windows (WSL2) - Option 2: Windows 11 WSLg (Built-in)

If using Windows 11 with WSLg (GUI support built into WSL2):
```bash
# No additional setup needed, DISPLAY is auto-configured
echo $DISPLAY  # Should show :0 or :1
```

#### macOS - XQuartz

1. **Install XQuartz**:
   ```bash
   brew install --cask xquartz
   ```

2. **Configure XQuartz**:
   - Launch XQuartz (Applications → Utilities → XQuartz)
   - XQuartz → Preferences → Security: ✅ "Allow connections from network clients"
   - Restart XQuartz

3. **Set DISPLAY**:
   ```bash
   # Add to ~/.zshrc or ~/.bash_profile
   export DISPLAY=:0
   xhost +localhost
   ```

---

### 1.3 Pull Gazebo Fortress Docker Image

```bash
# Pull official ROS 2 Humble + Gazebo image (LARGE: ~3-4 GB)
docker pull osrf/ros:humble-desktop-full

# Verify image is available
docker images | grep ros
# Should show: osrf/ros   humble-desktop-full   ...
```

---

### 1.4 Create Gazebo Docker Container Script

Create a reusable script to launch Gazebo with GUI forwarding:

```bash
# Create script file
mkdir -p ~/robotics-workspace/scripts
nano ~/robotics-workspace/scripts/run_gazebo.sh
```

**Paste the following content**:

```bash
#!/bin/bash

# Gazebo Fortress Docker Launcher
# Supports Linux, WSL2, and macOS with X11 forwarding

# Detect platform
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    DISPLAY_VAR="$DISPLAY"
    X11_SOCKET="/tmp/.X11-unix"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    DISPLAY_VAR="host.docker.internal:0"
    X11_SOCKET="/tmp/.X11-unix"
else
    # WSL2 or other
    DISPLAY_VAR="$DISPLAY"
    X11_SOCKET="/tmp/.X11-unix"
fi

# Run Docker container with GUI support
docker run -it --rm \
    --name gazebo_fortress \
    --env DISPLAY="$DISPLAY_VAR" \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume ~/robotics-workspace:/workspace:rw \
    --network host \
    osrf/ros:humble-desktop-full \
    bash -c "source /opt/ros/humble/setup.bash && gazebo"
```

**Make script executable**:
```bash
chmod +x ~/robotics-workspace/scripts/run_gazebo.sh
```

---

### 1.5 Test Gazebo Installation

```bash
# Launch Gazebo (should open GUI window)
~/robotics-workspace/scripts/run_gazebo.sh
```

**Expected Behavior**:
- Gazebo GUI window opens with empty world
- Ground plane visible, camera controls work (click-drag to rotate)
- No error messages in terminal

**Troubleshooting**:

| Issue | Symptoms | Solution |
|-------|----------|----------|
| No GUI window | Gazebo runs but no window appears | Verify X server running (VcXsrv/XQuartz), check `$DISPLAY` variable |
| `cannot connect to X server` | Error in terminal | Run `xhost +local:docker` (Linux) or restart VcXsrv (Windows) |
| Black screen in Gazebo | Window opens but shows black viewport | GPU driver issue; try adding `--gpus all` to docker run (NVIDIA only) |
| Slow performance (< 10 FPS) | Gazebo GUI laggy | Reduce world complexity or run headless mode (no GUI) |

**To exit Gazebo**: Press `Ctrl+C` in terminal or close Gazebo window.

---

## Part 2: Unity 2022.3 LTS Setup (Native)

### 2.1 Install Unity Hub

#### Linux (Ubuntu 22.04)

```bash
# Download Unity Hub AppImage
wget -O ~/Downloads/UnityHub.AppImage https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

# Make executable and move to /opt
chmod +x ~/Downloads/UnityHub.AppImage
sudo mv ~/Downloads/UnityHub.AppImage /opt/UnityHub.AppImage

# Create desktop shortcut (optional)
cat > ~/.local/share/applications/unity-hub.desktop <<EOF
[Desktop Entry]
Name=Unity Hub
Exec=/opt/UnityHub.AppImage
Icon=unity-editor-icon
Type=Application
Categories=Development;
EOF
```

**Launch Unity Hub**:
```bash
/opt/UnityHub.AppImage
```

#### Windows

1. Download Unity Hub installer: https://unity.com/download
2. Run `UnityHubSetup.exe`
3. Install to default location (`C:\Program Files\Unity Hub`)
4. Launch Unity Hub from Start Menu

#### macOS

1. Download Unity Hub: https://unity.com/download
2. Open `.dmg` file, drag Unity Hub to Applications
3. Launch from Applications folder
4. **For Apple Silicon (M1/M2)**: Unity Hub may prompt to install Rosetta 2 (required for some Unity packages)

---

### 2.2 Install Unity 2022.3 LTS

**In Unity Hub**:

1. **Sign in** (create free Unity account if needed)
2. Click **"Installs"** tab → **"Add"** button
3. Select **"Unity 2022.3.XX LTS"** (latest 2022.3 patch version)
4. **Add Modules** (optional but recommended):
   - ✅ **Linux Build Support** (if on Windows/macOS and want cross-platform)
   - ✅ **Documentation** (offline Unity Manual)
   - ❌ Android/iOS/WebGL (not needed for robotics)
5. Click **"Continue"** → Accept license → **"Install"**
6. Wait for installation (10-15 minutes, ~8 GB download)

**Verify Installation**:
- Unity Hub → Installs tab should show **"2022.3.XX LTS"** with green checkmark

---

### 2.3 Install Unity Robotics Hub Packages

Unity Robotics Hub packages are installed per-project (not globally). We'll create a test project and install packages.

**Create New Project**:

1. Unity Hub → **"Projects"** tab → **"New Project"**
2. **Editor Version**: Select "2022.3.XX LTS"
3. **Template**: **"3D (URP)"** (Universal Render Pipeline for better performance)
4. **Project Name**: `RoboticsSimulationTest`
5. **Location**: `~/UnityProjects/` (or `C:\UnityProjects\` on Windows)
6. Click **"Create Project"**

**Install URDF Importer Package**:

1. In Unity Editor, go to **Window → Package Manager**
2. Click **"+"** (top-left) → **"Add package from git URL"**
3. Paste URL: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`
4. Click **"Add"**
5. Wait for installation (1-2 minutes)

**Install ROS TCP Connector Package** (for ROS 2 communication):

1. Package Manager → **"+"** → **"Add package from git URL"**
2. Paste URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
3. Click **"Add"**

**Verify Installation**:
- Package Manager should show:
  - ✅ **URDF Importer** (version 0.7.0+)
  - ✅ **ROS-TCP-Connector** (version 0.7.0+)
- Menu bar should have new option: **"Assets → Import Robot from URDF"**

---

### 2.4 Test Unity Installation

**Create a Simple Scene**:

1. In Unity, create a new GameObject: **GameObject → 3D Object → Cube**
2. In Inspector, add component: **Rigidbody** (enables physics)
3. Press **Play** button (top-center toolbar)
4. **Expected**: Cube falls under gravity

**Check Console for Errors**:
- Window → General → Console
- Should have 0 errors, 0 warnings

**Troubleshooting**:

| Issue | Symptoms | Solution |
|-------|----------|----------|
| Unity crashes on launch | Editor closes immediately after splash screen | Update graphics drivers, try switching to DirectX (Windows) or Vulkan (Linux) in Project Settings |
| Package import fails | Error: "Cannot resolve package..." | Check internet connection, verify Git is installed (`git --version`) |
| Low FPS (< 30) in Play mode | Editor stutters with simple scene | Reduce Quality settings (Edit → Project Settings → Quality → Low), disable shadows |
| URDF import menu missing | No "Import Robot from URDF" option | Verify package installed (Package Manager → Packages: In Project), restart Unity Editor |

---

## Part 3: ROS 2 Humble and Dependencies

### 3.1 Install ROS 2 Humble (Linux Native or WSL2)

**Ubuntu 22.04 (Native or WSL2)**:

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop (includes RViz2, sensor plugins)
sudo apt update
sudo apt install -y ros-humble-desktop-full

# Install additional packages for sensors and Gazebo
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-sensor-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport

# Source ROS 2 setup (add to ~/.bashrc for persistence)
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

**Verify Installation**:
```bash
ros2 --version  # Should show: ros2 cli version 0.25.X
rviz2  # Should launch RViz2 GUI (close after verification)
```

---

### 3.2 Install Python Dependencies

```bash
# Install pip if not already installed
sudo apt install -y python3-pip

# Install Python packages for sensor data processing
pip3 install \
    open3d \
    matplotlib \
    pandas \
    numpy \
    opencv-python

# Verify installations
python3 -c "import open3d; print('Open3D version:', open3d.__version__)"
python3 -c "import matplotlib; print('Matplotlib version:', matplotlib.__version__)"
```

---

### 3.3 Install Visualization Tools (Optional)

**CloudCompare** (for point cloud visualization):
```bash
sudo apt install -y cloudcompare
```

**ROS 2 tools**:
```bash
# rosbag2 (for recording sensor data)
sudo apt install -y ros-humble-rosbag2

# rqt (GUI tools for ROS 2)
sudo apt install -y ros-humble-rqt ros-humble-rqt-common-plugins
```

---

## Part 4: Workspace Setup

### 4.1 Create Robotics Workspace

```bash
# Create workspace directory structure
mkdir -p ~/robotics-workspace/src
cd ~/robotics-workspace

# Clone Module 2 example code (when available)
# git clone <repository-url> src/module-2-examples
```

### 4.2 Build Workspace (if using custom ROS 2 packages)

```bash
cd ~/robotics-workspace
colcon build --symlink-install
source install/setup.bash

# Add to ~/.bashrc for auto-sourcing
echo "source ~/robotics-workspace/install/setup.bash" >> ~/.bashrc
```

---

## Part 5: Validation Checklist

Run through this checklist to verify your setup is complete:

### Gazebo Fortress
- [ ] Docker installed and running (`docker --version`)
- [ ] X server configured (VcXsrv/XQuartz on Windows/macOS, xhost on Linux)
- [ ] Gazebo launches with GUI (`~/robotics-workspace/scripts/run_gazebo.sh`)
- [ ] Empty world visible, camera controls work

### Unity 2022.3 LTS
- [ ] Unity Hub installed and Unity 2022.3.XX LTS shows in Installs tab
- [ ] Test project created and opens without errors
- [ ] URDF Importer package installed (verify in Package Manager)
- [ ] ROS TCP Connector package installed
- [ ] Simple physics test works (cube falls when Play button pressed)

### ROS 2 Humble
- [ ] ROS 2 Humble installed (`ros2 --version`)
- [ ] RViz2 launches without errors (`rviz2`)
- [ ] Gazebo ROS packages installed (`ros2 pkg list | grep gazebo_ros`)
- [ ] Python packages installed (open3d, matplotlib, pandas)

### Environment Variables
- [ ] `$DISPLAY` set correctly (check with `echo $DISPLAY`)
- [ ] ROS 2 sourced in `~/.bashrc` (`source /opt/ros/humble/setup.bash`)
- [ ] Workspace sourced if custom packages built

---

## Troubleshooting Common Issues

### Issue: Gazebo GUI is extremely slow (< 5 FPS)

**Cause**: Limited GPU passthrough in Docker (especially WSL2 on Windows)

**Solution**:
1. Try native Linux installation for best performance
2. On Windows: Use Windows 11 with WSLg for better GPU support
3. Reduce simulation complexity: Fewer objects, simpler meshes
4. Run Gazebo headless (no GUI) and visualize in RViz2 instead

### Issue: Unity crashes when importing URDF

**Cause**: Large mesh files or corrupted URDF

**Solution**:
1. Verify mesh files exist at paths specified in URDF
2. Convert Collada (.dae) meshes to OBJ or FBX format
3. Reduce mesh complexity (simplify in Blender/MeshLab before import)
4. Check Unity Console for specific error messages

### Issue: ROS 2 topics not visible in RViz2

**Cause**: Topic namespace mismatch or message type incompatibility

**Solution**:
```bash
# List all active topics
ros2 topic list

# Echo topic to verify data is publishing
ros2 topic echo /robot/lidar/points

# Check topic type matches RViz2 display
ros2 topic info /robot/lidar/points
```

### Issue: Permission denied when running Docker commands

**Cause**: User not in `docker` group

**Solution**:
```bash
sudo usermod -aG docker $USER
newgrp docker  # Or logout and login again
```

---

## Next Steps

✅ **Environment Setup Complete!**

**Proceed to Chapter 1**: Physics Simulation Fundamentals with Gazebo
- Launch Gazebo and create your first world
- Import humanoid robot from Module 1
- Configure physics parameters

**Reference Documentation**:
- Gazebo Fortress Tutorials: https://gazebosim.org/docs/fortress/tutorials
- Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- ROS 2 Humble Docs: https://docs.ros.org/en/humble/

---

**Quick Start Guide Version**: 1.0
**Last Updated**: 2025-12-07
**Tested On**: Ubuntu 22.04 (native), Windows 11 (WSL2), macOS Monterey (Intel)
**Feedback**: Report issues to course instructors or GitHub repository
