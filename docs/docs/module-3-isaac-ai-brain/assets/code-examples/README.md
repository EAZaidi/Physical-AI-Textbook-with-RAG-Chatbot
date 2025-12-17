# Module 3: Code Examples

This folder contains all the code examples, scripts, and configuration files referenced in Module 3: The AI-Robot Brain (NVIDIA Isaac).

## 📂 Directory Structure

All downloadable code files for Module 3 are located in:
```
docs/static/code-examples/module-3/
```

This includes:
- Python scripts for Isaac Sim and Isaac ROS
- ROS 2 launch files
- YAML configuration files
- Docker configurations
- Shell scripts for automation

## 🔗 Quick Links

All files are located in `docs/static/code-examples/module-3/`:
- Isaac Sim Scripts - URDF loading, sensor configuration
- Isaac ROS Launch Files - VSLAM and perception pipelines
- Nav2 Configs - Path planning parameters
- Docker Files (in docker/ subdirectory) - Container configurations

## 📋 Available Scripts

### Chapter 1: Isaac Sim
- `isaac_sim_load_robot.py` - Load humanoid URDF programmatically
- `isaac_sim_sensor_config.py` - Configure cameras and LiDAR
- `record_isaac_data.sh` - Record ROS 2 sensor data to rosbag

### Chapter 2: Isaac ROS VSLAM
- `isaac_ros_vslam_launch.py` - Launch Visual SLAM pipeline
- `run_vslam_offline.sh` - Run VSLAM on recorded data
- `vslam_visualization.rviz` - RViz2 configuration for SLAM
- `isaac_sim_vslam_live.launch.py` - Real-time integration

### Chapter 3: Nav2
- `nav2_params_bipedal.yaml` - Navigation parameters tuned for humanoids
- `vslam_to_occupancy.py` - Convert SLAM maps to occupancy grids
- `nav2_bipedal_bringup.launch.py` - Launch Nav2 stack
- `send_nav_goal.py` - Send navigation goals programmatically

### Chapter 4: Integration
- `isaac_nav_full.launch.py` - Master launch file for complete system
- `launch_full_demo.sh` - Automated launch sequence
- `full_navigation.rviz` - Complete RViz2 visualization

## 💾 How to Use

### Download All Files

You can clone the entire repository to get all code examples:

```bash
git clone https://github.com/EAZaidi/Physical-AI-Humanoid-Robotics-Textbook.git
cd Physical-AI-Humanoid-Robotics-Textbook/docs/static/code-examples/module-3/
```

### Download Individual Files

Navigate to the file on GitHub and click "Raw" then save the file, or use `wget`:

```bash
# Example: Download Isaac Sim loading script
wget https://raw.githubusercontent.com/EAZaidi/Physical-AI-Humanoid-Robotics-Textbook/main/docs/static/code-examples/module-3/isaac_sim_load_robot.py
```

### Make Scripts Executable

After downloading Python scripts and shell scripts:

```bash
chmod +x *.py *.sh
```

## 🐳 Docker Configuration

The Docker files in `docker/` directory provide:
- `Dockerfile.isaac_ros` - Isaac ROS container with GPU support
- `docker-compose.yml` - Easy container management

Build and run:
```bash
cd docker/
docker-compose up
```

## 📖 Documentation

Each script includes:
- Inline comments explaining functionality
- Usage instructions in docstrings
- Required dependencies listed at top of file

Refer to the corresponding chapter for detailed explanations and step-by-step tutorials.

## ⚠️ Prerequisites

Before using these scripts, ensure you have:
- Ubuntu 22.04 LTS
- ROS 2 Humble installed
- NVIDIA GPU with drivers (for Isaac ROS)
- Isaac Sim 2023.1.1+ (for Chapter 1)
- Docker + NVIDIA Container Toolkit (for Chapter 2+)

See [Module 3 Introduction](../../) for complete setup instructions.

## 🆘 Troubleshooting

If scripts fail to run:
1. Check Python version: `python3 --version` (should be 3.10)
2. Source ROS 2: `source /opt/ros/humble/setup.bash`
3. Install missing dependencies: `pip install -r requirements.txt` (if provided)
4. Check GPU access: `nvidia-smi`

For module-specific issues, see the Troubleshooting section in each chapter.

## 📝 License

All code examples are provided under the MIT License for educational use.
