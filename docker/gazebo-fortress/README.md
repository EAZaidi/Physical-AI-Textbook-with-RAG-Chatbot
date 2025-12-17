# Gazebo Fortress Docker Environment

**Purpose**: Reproducible Gazebo simulation environment for Module 2: The Digital Twin

## Quick Start

### Build the Docker Image

```bash
cd docker/gazebo-fortress
docker-compose build
```

### Launch Gazebo GUI

```bash
# Method 1: Using run_gazebo.sh (recommended)
./run_gazebo.sh

# Method 2: Using docker-compose
docker-compose run --rm gazebo-fortress

# Method 3: Launch with specific world file
./run_gazebo.sh /workspace/docs/docs/module-2-digital-twin/assets/code-examples/simple_world.sdf
```

## Platform-Specific Setup

### Linux (Ubuntu 22.04)

```bash
# Allow Docker to access X server
xhost +local:docker

# Build and run
docker-compose build
./run_gazebo.sh
```

### Windows (WSL2)

1. **Install X Server** (VcXsrv or X410):
   - Download VcXsrv: https://sourceforge.net/projects/vcxsrv/
   - Launch XLaunch with "Disable access control" enabled

2. **Set DISPLAY in WSL2**:
   ```bash
   export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
   echo 'export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '"'"'{print $2}'"'"'):0' >> ~/.bashrc
   ```

3. **Build and run**:
   ```bash
   docker-compose build
   ./run_gazebo.sh
   ```

**Windows 11 with WSLg**: No X server needed! DISPLAY is auto-configured.

### macOS

1. **Install XQuartz**:
   ```bash
   brew install --cask xquartz
   ```

2. **Configure XQuartz**:
   - Launch XQuartz
   - XQuartz → Preferences → Security: ✅ "Allow connections from network clients"
   - Restart XQuartz

3. **Set DISPLAY and allow connections**:
   ```bash
   export DISPLAY=:0
   xhost +localhost
   ```

4. **Build and run**:
   ```bash
   docker-compose build
   ./run_gazebo.sh
   ```

## What's Included

- **Gazebo Fortress**: Physics simulation engine
- **ROS 2 Humble**: Robotics middleware
- **Sensor plugins**: LiDAR (gpu_ray), depth camera, IMU
- **Python tools**: open3d, matplotlib, pandas, opencv

## Troubleshooting

### Issue: No GUI window appears

**Symptoms**: Gazebo runs but no window visible

**Solutions**:
- **Linux**: Run `xhost +local:docker` before launching
- **Windows WSL2**: Verify VcXsrv is running and DISPLAY is set correctly
- **macOS**: Ensure XQuartz is running and `xhost +localhost` was executed

### Issue: "cannot connect to X server"

**Solutions**:
```bash
# Check DISPLAY variable
echo $DISPLAY

# Linux
xhost +local:docker

# WSL2
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0

# macOS
export DISPLAY=:0
xhost +localhost
```

### Issue: Black screen in Gazebo window

**Cause**: GPU driver issue or missing hardware acceleration

**Solutions**:
- For NVIDIA GPUs: Uncomment GPU section in `docker-compose.yml`
- Reduce scene complexity
- Try software rendering (slower): `LIBGL_ALWAYS_SOFTWARE=1 ./run_gazebo.sh`

### Issue: Slow performance (< 10 FPS)

**Causes**: WSL2 limited GPU support, complex scene

**Solutions**:
- Use native Linux for best performance
- Simplify world files (fewer objects, simpler meshes)
- Close other applications
- Run headless mode (no GUI, visualize in RViz2)

## Testing the Setup

```bash
# Launch Gazebo with empty world (should open GUI)
./run_gazebo.sh

# Launch with custom world
./run_gazebo.sh /workspace/docs/docs/module-2-digital-twin/assets/code-examples/simple_world.sdf

# Run Gazebo inside container interactively
docker-compose run --rm gazebo-fortress
# Inside container:
source /opt/ros/humble/setup.bash
gazebo
```

## File Structure

```
docker/gazebo-fortress/
├── Dockerfile           # Image definition (Gazebo + ROS 2 + tools)
├── docker-compose.yml   # Container configuration
├── run_gazebo.sh        # Launcher script (cross-platform)
└── README.md            # This file
```

## Requirements

- Docker 20.10+ (with BuildKit support)
- 10 GB free disk space (for Docker image)
- X server (VcXsrv/X410 on Windows, XQuartz on macOS, native on Linux)
- 8 GB RAM minimum (16 GB recommended)

## Next Steps

After successful Gazebo launch:
1. Proceed to **Chapter 1: Physics Simulation Fundamentals** in Module 2
2. Create your first Gazebo world
3. Import URDF humanoid models from Module 1

## Support

- Gazebo Fortress Documentation: https://gazebosim.org/docs/fortress/
- ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
- Docker Documentation: https://docs.docker.com/

---

**Version**: 1.0
**Last Updated**: 2025-12-07
**Tested On**: Ubuntu 22.04 (native), Windows 11 (WSL2), macOS Monterey (Intel)
