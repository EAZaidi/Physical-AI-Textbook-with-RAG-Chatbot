# Quickstart: Module 3 - The AI-Robot Brain (NVIDIA Isaac)

**Feature**: 003-isaac-ai-brain
**Date**: 2025-12-08
**Prerequisites**: Module 1 (ROS 2 basics) and Module 2 (digital twin simulation) completed

---

## Environment Setup Checklist

### Hardware Requirements
- [ ] NVIDIA GPU: RTX 3060 or higher (8GB+ VRAM)
- [ ] CPU: Intel i7 or AMD Ryzen 7 (8+ cores recommended)
- [ ] RAM: 16GB minimum, 32GB recommended
- [ ] Disk: 50GB free space (Isaac Sim + Docker images)

### Software Requirements
- [ ] **Operating System**: Ubuntu 22.04 LTS (native or WSL2)
- [ ] **NVIDIA Driver**: Version 525 or newer
  ```bash
  nvidia-smi  # Verify GPU is detected
  ```
- [ ] **ROS 2 Humble**: Installed and sourced
  ```bash
  ros2 --version  # Should show: ros2 doctor version 0.10.3
  ```
- [ ] **Docker**: Installed with NVIDIA Container Toolkit
  ```bash
  docker --version  # Should show: Docker version 24.0+
  docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
  ```

---

## Installation Steps

### Step 1: Install Isaac Sim

1. **Download NVIDIA Omniverse Launcher**
   - Visit: https://www.nvidia.com/en-us/omniverse/download/
   - Download launcher for Linux
   - Install: `chmod +x omniverse-launcher-linux.AppImage && ./omniverse-launcher-linux.AppImage`

2. **Install Isaac Sim via Launcher**
   - Open Omniverse Launcher
   - Go to "Exchange" tab → Search "Isaac Sim"
   - Install version **2023.1.1** or newer
   - Wait for installation (~15GB download)

3. **Verify Isaac Sim Installation**
   - Launch Isaac Sim from Launcher
   - Confirm GUI opens (may take 2-3 minutes first launch)
   - Close Isaac Sim

**Estimated Time**: 30-45 minutes (download dependent)

---

### Step 2: Set Up Isaac ROS Docker Environment

1. **Install NVIDIA Container Toolkit**
   ```bash
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
   curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
     sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
     sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
   sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
   sudo systemctl restart docker
   ```

2. **Test GPU Access in Docker**
   ```bash
   docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
   ```
   - Should display GPU information
   - If error: Check NVIDIA driver installation

3. **Pull Isaac ROS Base Image**
   ```bash
   docker pull nvcr.io/nvidia/isaac-ros:humble-ros2_humble_20231122
   ```
   - **Size**: ~10GB (be patient)

4. **Build Custom Isaac ROS Image (Optional)**
   ```bash
   cd specs/003-isaac-ai-brain/contracts
   docker build -t isaac_ros_module3:latest -f Dockerfile.isaac_ros .
   ```

**Estimated Time**: 20-30 minutes (build + download)

---

### Step 3: Install Nav2

1. **Install Nav2 on Host System**
   ```bash
   sudo apt update
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

2. **Verify Nav2 Installation**
   ```bash
   ros2 pkg list | grep nav2
   # Should list ~20 nav2_* packages
   ```

**Estimated Time**: 5 minutes

---

### Step 4: Configure ROS 2 Environment

1. **Set ROS Domain ID (Optional)**
   ```bash
   echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
   source ~/.bashrc
   ```

2. **Enable ROS 2 Simulation Time**
   ```bash
   export ROS_PARAM_USE_SIM_TIME=True
   ```

3. **Test ROS 2 Communication**
   - **Terminal 1**: Start ROS 2 demo
     ```bash
     ros2 run demo_nodes_cpp talker
     ```
   - **Terminal 2**: Listen to topic
     ```bash
     ros2 topic echo /chatter
     ```
   - Should see messages published
   - Press `Ctrl+C` to stop both

**Estimated Time**: 2 minutes

---

## Validation Tests

### Test 1: Isaac Sim ROS 2 Bridge

1. Launch Isaac Sim
2. Window → Extensions → Search "ROS2 Bridge" → Enable
3. Create a simple scene with cube
4. Add OmniGraph:
   - Action Graph → ROS2 Context
   - Add "ROS2 Publish Clock" node
   - Connect and play simulation
5. In terminal:
   ```bash
   ros2 topic list
   # Should see /clock topic
   ```

**Expected Result**: ✅ `/clock` topic visible in `ros2 topic list`

---

### Test 2: Isaac ROS Docker Container

1. Run container with GPU:
   ```bash
   docker run --rm -it --gpus all \
     --network host \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     -e DISPLAY=$DISPLAY \
     nvcr.io/nvidia/isaac-ros:humble-ros2_humble_20231122 \
     bash
   ```

2. Inside container, check GPU:
   ```bash
   nvidia-smi
   # Should show GPU info
   ```

3. Inside container, check ROS 2:
   ```bash
   ros2 topic list
   # Should see topics from host (if simulation running)
   ```

**Expected Result**: ✅ GPU accessible, ✅ ROS 2 topics visible

---

### Test 3: Nav2 Stack

1. Launch Nav2 with default config:
   ```bash
   ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
   ```

2. In RViz2, send "2D Goal Pose"
3. Robot should plan and follow path

**Expected Result**: ✅ Nav2 GUI opens, ✅ Path visualized in RViz2

---

## Troubleshooting

### Issue: Isaac Sim won't launch (black screen)
**Solution**:
- Check GPU driver: `nvidia-smi` should work
- Update driver: https://www.nvidia.com/Download/index.aspx
- Verify Vulkan support: `vulkaninfo | grep "GPU"`

### Issue: Docker container can't access GPU
**Solution**:
- Reinstall NVIDIA Container Toolkit
- Restart Docker daemon: `sudo systemctl restart docker`
- Test with: `docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi`

### Issue: ROS 2 topics not visible between Isaac Sim and host
**Solution**:
- Check `ROS_DOMAIN_ID` matches (both should be 0)
- Disable firewall: `sudo ufw disable` (temporarily for testing)
- Use `cyclonedds` instead of FastDDS:
  ```bash
  sudo apt install ros-humble-rmw-cyclonedds-cpp
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```

### Issue: Nav2 fails to find maps
**Solution**:
- Ensure SLAM or static map is publishing on `/map` topic
- Check with: `ros2 topic echo /map --once`
- Load static map with: `ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=my_map.yaml`

---

## Next Steps

Once all validation tests pass:
1. ✅ Proceed to **Chapter 1**: Isaac Sim — Synthetic Data Generation
2. ✅ Follow step-by-step tutorials in Module 3 chapters
3. ✅ Download code examples from `/static/code-examples/module-3/`

---

## Support Resources

- **Isaac Sim Docs**: https://docs.omniverse.nvidia.com/isaacsim/latest/
- **Isaac ROS Docs**: https://nvidia-isaac-ros.github.io/
- **Nav2 Docs**: https://navigation.ros.org/
- **ROS 2 Humble Docs**: https://docs.ros.org/en/humble/

---

**Quickstart Complete**: Environment ready for Module 3 tutorials!
