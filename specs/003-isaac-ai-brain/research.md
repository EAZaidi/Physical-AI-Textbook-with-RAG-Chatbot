# Research: Module 3 - The AI-Robot Brain (NVIDIA Isaac)

**Date**: 2025-12-08
**Feature**: 003-isaac-ai-brain
**Purpose**: Resolve all technical unknowns and establish authoritative sources for Module 3 content

---

## Research Questions

### 1. Isaac Sim Installation & Setup
**Question**: What are the exact installation steps for Isaac Sim 2023.1.1+ on Ubuntu 22.04, and what are the system requirements?

**Decision**: Use NVIDIA Omniverse Launcher method with Isaac Sim 2023.1.1 as minimum version

**Rationale**:
- NVIDIA Omniverse Launcher is the official distribution method
- Isaac Sim 2023.1.1 introduced stable ROS 2 Humble support
- System requirements: RTX 3060 (8GB VRAM minimum), Ubuntu 22.04, NVIDIA Driver 525+

**Alternatives Considered**:
- Docker-only Isaac Sim: Rejected due to complexity of X11 forwarding for GUI tutorials
- Isaac Sim 2022.x: Rejected due to incomplete ROS 2 Humble support

**Authoritative Sources**:
- Installation: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html
- System Requirements: https://docs.omniverse.nvidia.com/isaacsim/latest/installation/requirements.html
- ROS 2 Support: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html

---

### 2. Isaac ROS Docker Setup
**Question**: How should students set up Isaac ROS with GPU acceleration in Docker for ROS 2 Humble?

**Decision**: Use official NVIDIA Isaac ROS Docker images with NVIDIA Container Toolkit

**Rationale**:
- Ensures reproducibility across different hardware
- Handles CUDA/TensorRT dependencies automatically
- Avoids conflicts with host ROS 2 installation

**Alternatives Considered**:
- Native Isaac ROS installation: Rejected due to CUDA dependency conflicts
- Custom Dockerfile from scratch: Rejected in favor of NVIDIA-maintained base images

**Authoritative Sources**:
- Isaac ROS Docker Setup: https://nvidia-isaac-ros.github.io/getting_started/isaac_ros_buildfarm_cdn.html
- Base Images: https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/containers/isaac-ros-dev
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

**Docker Example**:
```dockerfile
FROM nvcr.io/nvidia/isaac-ros:humble-ros2_humble_20231122
# Add project-specific dependencies
```

---

### 3. Isaac ROS Visual SLAM Configuration
**Question**: What are the correct ROS 2 topics, parameters, and launch configurations for Isaac ROS Visual SLAM with stereo cameras?

**Decision**: Use `isaac_ros_visual_slam` package with stereo camera configuration (matching D435i sensor from Module 2)

**Rationale**:
- Stereo VSLAM more robust than monocular for educational demos
- Provides 3D map and 6-DOF pose estimation
- Compatible with RealSense D435i sensor specs used in Module 2

**Key Parameters**:
- Input topics: `/camera/left/image_raw`, `/camera/right/image_raw`, `/camera/left/camera_info`, `/camera/right/camera_info`
- Output topics: `/visual_slam/tracking/odometry` (nav_msgs/Odometry), `/visual_slam/tracking/vo_pose_covariance` (geometry_msgs/PoseWithCovarianceStamped), `/visual_slam/vis/slam_odometry` (nav_msgs/Odometry)
- Map output: `/visual_slam/vis/map_points` (sensor_msgs/PointCloud2)

**Authoritative Sources**:
- Isaac ROS Visual SLAM: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html
- API Reference: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/api/topics.html
- Stereo Camera Setup: https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html

---

### 4. Nav2 Configuration for Bipedal Robots
**Question**: How should Nav2 parameters be tuned for a bipedal humanoid robot's footprint and motion constraints?

**Decision**: Use DWB (Dynamic Window Approach) local planner with circular footprint approximation (0.4m radius)

**Rationale**:
- Circular footprint simplifies collision checking for bipedal stance
- DWB planner allows velocity constraints matching bipedal walking (slower speeds, limited rotation)
- Nav2 behavior trees enable recovery behaviors for humanoid-specific issues (balance recovery)

**Key Nav2 Parameters**:
- Robot footprint: `footprint: [[0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3]]` (or circular radius 0.4m)
- Max velocities: `max_vel_x: 0.5`, `max_vel_theta: 0.5` (slower than wheeled robots)
- Inflation radius: `inflation_radius: 0.5` (conservative for safety)
- Global planner: NavFn or Smac Planner 2D
- Local planner: DWB with TrajectoryGeneratorDifferentialDrive

**Authoritative Sources**:
- Nav2 Configuration Guide: https://navigation.ros.org/configuration/index.html
- DWB Controller: https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html
- Footprint Configuration: https://navigation.ros.org/configuration/packages/costmap-plugins/inflation.html
- Behavior Trees: https://navigation.ros.org/behavior_trees/index.html

---

### 5. Isaac Sim - ROS 2 Bridge Setup
**Question**: How do we configure Isaac Sim to publish sensor data and receive commands via ROS 2 topics?

**Decision**: Use Isaac Sim's built-in ROS 2 Bridge extension with OmniGraph for sensor streaming

**Rationale**:
- Native integration, no external bridge required
- OmniGraph provides visual programming for sensor configurations
- Supports all ROS 2 message types needed (sensor_msgs, geometry_msgs, nav_msgs)

**Setup Steps**:
1. Enable ROS 2 Bridge extension in Isaac Sim
2. Create OmniGraph nodes for camera, LiDAR publishers
3. Configure ROS 2 domain ID (default 0) to match host ROS 2
4. Verify topics with `ros2 topic list`

**Authoritative Sources**:
- ROS 2 Bridge Tutorial: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_navigation.html
- OmniGraph Guide: https://docs.omniverse.nvidia.com/isaacsim/latest/gui_tutorials/tutorial_gui_omnigraph.html
- Sensor Publishing: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_camera.html

---

### 6. Isaac Sim Scene Design Best Practices
**Question**: What are the best practices for creating educational Isaac Sim scenes (warehouse, corridor) for navigation tutorials?

**Decision**: Use Isaac Sim asset library (warehouse props) + custom USD scenes with collision meshes

**Rationale**:
- NVIDIA provides pre-built assets (shelves, boxes, walls) for rapid scene creation
- USD format enables version control and sharing
- Physics-accurate collision meshes ensure realistic navigation simulation

**Scene Requirements**:
- Ground plane with friction material
- Static obstacles with collision shapes
- Lighting (dome light for photorealistic rendering)
- Humanoid robot spawned at known initial pose
- Goal markers (visual aids for students)

**Authoritative Sources**:
- Scene Creation Tutorial: https://docs.omniverse.nvidia.com/isaacsim/latest/gui_tutorials/tutorial_gui_simple_scene.html
- Asset Library: https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/assets/usd_assets_overview.html
- Physics Materials: https://docs.omniverse.nvidia.com/isaacsim/latest/features/physics/physics_materials.html

---

### 7. Integrating Isaac ROS + Nav2 Pipeline
**Question**: What is the correct ROS 2 launch file structure to run Isaac Sim + Isaac ROS VSLAM + Nav2 simultaneously?

**Decision**: Use Python launch files with three nodes: Isaac Sim bridge, Isaac ROS VSLAM, Nav2 stack

**Rationale**:
- Python launch API enables conditional logic (check for GPU, validate topics)
- Separate launch files for each component enables modular testing
- Master launch file composes all three for full demo

**Launch File Structure**:
```python
# launch/isaac_nav_full.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Isaac Sim ROS 2 Bridge (assumed running externally)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('isaac_ros_visual_slam.launch.py')
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource('nav2_bringup.launch.py'),
            launch_arguments={'params_file': 'nav2_params_bipedal.yaml'}.items()
        ),
    ])
```

**Authoritative Sources**:
- ROS 2 Launch System: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html
- Isaac ROS Launch Files: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam/tree/main/launch
- Nav2 Launch Files: https://github.com/ros-planning/navigation2/tree/humble/nav2_bringup/launch

---

### 8. Troubleshooting Common Issues
**Question**: What are the most common pitfalls students encounter, and how do we document solutions?

**Decision**: Create dedicated "Troubleshooting" sections in each chapter addressing known issues

**Common Issues Identified**:
1. **Isaac Sim + Docker networking**: ROS 2 topics not visible between containers
   - Solution: Use `--network host` or configure DDS discovery
2. **GPU memory exhaustion**: Isaac Sim + Isaac ROS exceed 8GB VRAM
   - Solution: Reduce Isaac Sim rendering quality, use smaller scenes
3. **VSLAM initialization failure**: Low-texture environments
   - Solution: Add textured objects, verify camera calibration
4. **Nav2 costmap issues**: Robot footprint incorrect
   - Solution: Visualize footprint in RViz2, adjust inflation radius

**Authoritative Sources**:
- Isaac Sim Troubleshooting: https://docs.omniverse.nvidia.com/isaacsim/latest/known_issues.html
- Isaac ROS Troubleshooting: https://nvidia-isaac-ros.github.io/troubleshooting/index.html
- Nav2 Troubleshooting: https://navigation.ros.org/troubleshooting/index.html

---

## Documentation URLs Summary

| Component | Version | Primary Documentation URL |
|-----------|---------|--------------------------|
| Isaac Sim | 2023.1.1+ | https://docs.omniverse.nvidia.com/isaacsim/latest/ |
| Isaac ROS | 2.0+ | https://nvidia-isaac-ros.github.io/ |
| Isaac ROS Visual SLAM | 2.0+ | https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/ |
| Nav2 | 1.1+ (Humble) | https://navigation.ros.org/ |
| ROS 2 Humble | Humble Hawksbill | https://docs.ros.org/en/humble/ |
| NVIDIA Container Toolkit | Latest | https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/ |

---

## Action Items for Phase 1

Based on research findings:
1. ✅ Create data-model.md defining key entities (Isaac Sim Scene, VSLAM Map, Occupancy Grid, etc.)
2. ✅ Generate contracts/ with example YAML configs (Nav2 params, Isaac ROS launch files)
3. ✅ Write quickstart.md with environment setup checklist
4. ✅ Update agent context with Isaac Sim, Isaac ROS, Nav2 technology references

---

**Research Complete**: All NEEDS CLARIFICATION items resolved. Proceeding to Phase 1 (Design & Contracts).
