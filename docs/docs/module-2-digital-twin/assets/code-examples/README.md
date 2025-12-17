# Code Examples: Module 2 - The Digital Twin

This directory contains all code examples referenced in Module 2 chapters.

## Chapter 1: Gazebo Physics

### World Files (SDF)

**`simple_world.sdf`** - Basic Gazebo world with ground plane and physics
- Usage: `gazebo simple_world.sdf`
- Features: Earth gravity, ground plane with friction, directional lighting

**`physics_tuning.sdf`** - Tuned physics parameters for stable humanoid simulation
- Usage: `gazebo physics_tuning.sdf`
- Features: Optimized ODE solver settings, contact parameters for reduced jitter

### Robot Models (URDF)

**`humanoid_physics.urdf`** - Simple humanoid robot with proper inertial properties
- 7 links: torso, head, 2x upper/lower legs, 2x feet
- 7 joints: neck, 2x hip, 2x knee, 2x ankle
- Properly configured mass and inertia tensors
- Realistic friction on feet

### Launch Scripts

**`launch_gazebo.sh`** - Quick launcher for simple_world.sdf
- Usage: `./launch_gazebo.sh`
- Automatically sources ROS 2 if available

## Chapter 3: Sensor Simulation

### Sensor Configurations (SDF)

**`lidar_config.sdf`** - LiDAR sensor (gpu_ray) configuration
- 360° horizontal scan, 360 samples (1° resolution)
- Range: 0.1-30m, 10 Hz update rate
- Gaussian noise: σ=0.01m (1cm)
- ROS 2 topic: `/robot/lidar/scan`

**`depth_camera_config.sdf`** - Depth camera plugin configuration
- 640×480 resolution, 60° FOV
- Range: 0.5-5.0m, 30 Hz update rate
- Gaussian noise: σ=0.007m (7mm)
- ROS 2 topics: `/robot/camera/rgb/image_raw`, `/robot/camera/depth/image_raw`

**`imu_config.sdf`** - IMU sensor configuration
- 100 Hz update rate
- Accel noise: σ=0.01 m/s², Gyro noise: σ=0.001 rad/s
- ROS 2 topic: `/robot/imu/data`

### Data Collection Scripts (Python)

**`collect_sensor_data.py`** - ROS 2 node for monitoring all sensor streams
- Subscribes to LiDAR, depth camera, and IMU topics
- Displays live statistics (message counts, data ranges)

**`export_pointcloud.py`** - Convert sensor_msgs/PointCloud2 to PCD format
- Collects 100 scans, aggregates to single point cloud
- Uses Open3D for PCD export
- Output: `output.pcd` (viewable in CloudCompare, MeshLab)

**`export_imu_csv.py`** - Export IMU data to CSV
- Records 1000 samples (~10 seconds at 100 Hz)
- Fields: timestamp, ax, ay, az, gx, gy, gz, qx, qy, qz, qw
- Output: `imu_data.csv`

**`plot_imu_data.py`** - Visualize IMU data with matplotlib
- 2-panel plot: acceleration (top) + angular velocity (bottom)
- Includes statistics (mean acceleration for gravity check, noise levels)
- Usage: `python3 plot_imu_data.py [input_file.csv]`
- Output: `03-imu-plot.png`

## Chapter 2: Unity Scenes

### Unity Scripts (C#)

**`CameraController.cs`** - WASD + mouse camera controller
- Controls: WASD (move), Mouse (look), Q/E (up/down), Shift (sprint), Esc (unlock cursor)
- Attach to: Main Camera GameObject
- Parameters: moveSpeed (5.0 m/s), lookSensitivity (2.0)

**`SimpleJointController.cs`** - Basic joint position control using ArticulationBody
- Controls: Arrow keys (hip/knee joints), R (reset), Space (toggle control)
- Attach to: Robot root GameObject
- Parameters: Stiffness (10000), Damping (1000), Max Force (500 N⋅m)
- Demonstrates Unity ArticulationBody PD control

**`PushDetector.cs`** - Detect and log robot-object collisions
- Controls: P (apply push force), C (show collision stats)
- Attach to: Robot or interactive objects
- Displays: Impact force, contact points, relative velocity
- Optional: Visual feedback (color flash), audio feedback

### Unity Setup

**`unity_project_setup.md`** - Complete Unity 2022.3 LTS installation guide
- Unity Hub and Editor installation (Windows, macOS, Linux)
- Unity Robotics Hub packages (ROS TCP Connector, URDF Importer)
- Project configuration (physics, quality settings, folder structure)
- Basic scene setup (camera, ground, lighting)
- Troubleshooting guide for common issues
- Estimated setup time: 30-45 minutes

## Usage Notes

### Running Examples in Docker

```bash
# From repository root
cd docker/gazebo-fortress

# Build Docker image
docker-compose build

# Launch with simple world
./run_gazebo.sh

# Launch with custom world
./run_gazebo.sh /workspace/docs/docs/module-2-digital-twin/assets/code-examples/simple_world.sdf
```

### Running Examples Natively

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Launch world files
gazebo simple_world.sdf
gazebo physics_tuning.sdf

# Spawn humanoid robot (requires ROS 2 + gazebo_ros)
ros2 run gazebo_ros spawn_entity.py -file humanoid_physics.urdf -entity simple_humanoid -x 0 -y 0 -z 1.0
```

## File Organization

```
code-examples/
├── README.md                    # This file
├── simple_world.sdf             # Chapter 1 - Basic Gazebo world
├── humanoid_physics.urdf        # Chapter 1 - 7-link humanoid model
├── launch_gazebo.sh             # Chapter 1 - Launcher script
├── physics_tuning.sdf           # Chapter 1 - Optimized physics params
├── lidar_config.sdf             # Chapter 3 - LiDAR sensor config
├── depth_camera_config.sdf      # Chapter 3 - Depth camera config
├── imu_config.sdf               # Chapter 3 - IMU sensor config
├── collect_sensor_data.py       # Chapter 3 - Sensor monitoring node
├── export_pointcloud.py         # Chapter 3 - PCD export script
├── export_imu_csv.py            # Chapter 3 - IMU CSV export
├── plot_imu_data.py             # Chapter 3 - IMU visualization
├── CameraController.cs          # Chapter 2 - Unity camera script
├── SimpleJointController.cs     # Chapter 2 - Unity joint controller
├── PushDetector.cs              # Chapter 2 - Unity collision detector
└── unity_project_setup.md       # Chapter 2 - Unity setup guide
```

## Testing Examples

All examples should be tested in the documented Docker environment:

```bash
# Verify world files load without errors
gazebo simple_world.sdf --verbose

# Check URDF validity
check_urdf humanoid_physics.urdf

# Validate SDF syntax
gz sdf -k physics_tuning.sdf
```

## Troubleshooting

### Gazebo doesn't launch

- Verify Docker setup: see `docker/gazebo-fortress/README.md`
- Check DISPLAY variable: `echo $DISPLAY`
- Ensure X server running (VcXsrv on Windows, XQuartz on macOS)

### Robot falls through ground

- Check inertial properties in URDF (mass > 0, inertia tensors valid)
- Verify collision geometry exists on all links
- Increase ground plane contact stiffness (kp parameter)

### Simulation is slow/jittery

- Reduce max_step_size in physics configuration
- Increase solver iterations (iters parameter)
- Simplify collision geometry (use primitives instead of meshes)

---

**Module**: Module 2 - The Digital Twin
**Last Updated**: 2025-12-08
**Version**: 1.1 - All code examples complete
**Total Files**: 15 code examples + 1 documentation
