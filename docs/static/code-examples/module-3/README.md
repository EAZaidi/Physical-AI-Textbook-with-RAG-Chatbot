# Module 3: Code Examples - The AI-Robot Brain (NVIDIA Isaac)

This directory contains all code examples, configuration files, and Docker setups for Module 3.

## Directory Structure

```
module-3/
├── docker/
│   ├── Dockerfile.isaac_ros       # Isaac ROS Docker image
│   └── docker-compose.yml          # Docker Compose configuration
├── scenes/
│   ├── humanoid_sensor_demo.usd   # Isaac Sim scene for Chapter 1
│   ├── corridor_nav.usd            # Corridor navigation scene
│   └── humanoid_warehouse.usd     # Warehouse exploration scene
├── isaac_sim_load_robot.py         # Load robot URDF into Isaac Sim
├── isaac_sim_sensor_config.py      # Configure sensors programmatically
├── record_isaac_data.sh            # Record rosbag from Isaac Sim
├── isaac_ros_vslam_launch.py       # Isaac ROS Visual SLAM launch file
├── run_vslam_offline.sh            # Run VSLAM on recorded data
├── vslam_visualization.rviz        # RViz2 config for VSLAM
├── isaac_sim_vslam_live.launch.py # Live Isaac Sim → VSLAM integration
├── nav2_params_bipedal.yaml        # Nav2 parameters for humanoid
├── vslam_to_occupancy.py           # Convert VSLAM map to occupancy grid
├── nav2_bipedal_bringup.launch.py # Nav2 launch file
├── send_nav_goal.py                # Send navigation goals programmatically
├── isaac_nav_full.launch.py        # Master launch file (all components)
├── launch_full_demo.sh             # Automated launch script
└── full_navigation.rviz            # RViz2 config for full system
```

## Prerequisites

### Hardware
- NVIDIA GPU (RTX 3060 or better, 8GB+ VRAM)
- Ubuntu 22.04 LTS
- 16GB+ RAM

### Software
- Isaac Sim 2023.1.1+ (via NVIDIA Omniverse)
- Docker with NVIDIA Container Toolkit
- ROS 2 Humble Hawksbill
- Nav2 (installed via apt)

## Quick Start

### 1. Build Docker Image

```bash
cd docker
docker-compose build
```

### 2. Launch Docker Container

```bash
docker-compose up -d
docker exec -it isaac_ros_vslam bash
```

### 3. Run Examples

See individual chapter tutorials in the documentation for detailed instructions.

## Chapter-Specific Files

### Chapter 1: Isaac Sim Synthetic Data
- `isaac_sim_load_robot.py`
- `isaac_sim_sensor_config.py`
- `record_isaac_data.sh`
- `scenes/humanoid_sensor_demo.usd`

### Chapter 2: Isaac ROS VSLAM
- `isaac_ros_vslam_launch.py`
- `run_vslam_offline.sh`
- `vslam_visualization.rviz`
- `isaac_sim_vslam_live.launch.py`

### Chapter 3: Nav2 Path Planning
- `nav2_params_bipedal.yaml`
- `vslam_to_occupancy.py`
- `nav2_bipedal_bringup.launch.py`
- `send_nav_goal.py`

### Chapter 4: Full Integration
- `isaac_nav_full.launch.py`
- `launch_full_demo.sh`
- `full_navigation.rviz`
- `scenes/humanoid_warehouse.usd`

## Troubleshooting

### Docker GPU Access
```bash
# Verify GPU is accessible
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

### ROS 2 Topics Not Visible
```bash
# Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID  # Should be 0

# Try CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### Isaac Sim Connection Issues
- Ensure Isaac Sim ROS 2 Bridge extension is enabled
- Verify `ROS_DOMAIN_ID` matches between Isaac Sim and host
- Check firewall is not blocking UDP ports 7400-7403

## Support

For detailed tutorials and troubleshooting, see the Module 3 chapters in the documentation.

## License

All code examples are provided for educational purposes under the MIT License.
