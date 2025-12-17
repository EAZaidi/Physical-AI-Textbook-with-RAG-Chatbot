# Data Model: Module 2 - The Digital Twin (Gazebo & Unity)

**Feature**: Module 2 Digital Twin Simulation
**Created**: 2025-12-07
**Source**: spec.md (Key Entities section)

## Overview

This document defines the key entities and concepts students will learn in Module 2. These entities represent the core components of physics simulation (Gazebo) and high-fidelity visualization (Unity) for humanoid robot digital twins.

## Entity Definitions

### 1. Gazebo World

**Description**: A complete simulation environment containing physics engine configuration, environmental models (ground, walls, obstacles), lighting, and sensor plugins.

**Attributes**:
- `name` (string): Unique identifier for the world (e.g., "simple_humanoid_world")
- `physics_engine` (enum): Physics solver type (ODE, Bullet, DART, Simbody)
- `gravity` (vector3): Gravitational acceleration in m/s² (default: [0, 0, -9.81])
- `step_size` (float): Simulation time step in seconds (default: 0.001s)
- `real_time_factor` (float): Target simulation speed relative to real-time (default: 1.0)
- `models` (list<Model>): Environmental objects (ground plane, walls, obstacles)
- `plugins` (list<Plugin>): Gazebo plugins for sensors, actuators, or custom logic
- `lighting` (list<Light>): Light sources (directional, point, spot)

**Relationships**:
- **Contains** one Physics Engine
- **Contains** zero or more Robot Models
- **Contains** zero or more environmental Models (ground, obstacles)
- **Contains** zero or more Sensor Plugins

**File Format**: SDF (Simulation Description Format) `.world` or `.sdf` files

**Example**:
```xml
<world name="simple_humanoid_world">
  <physics type="ode">
    <gravity>0 0 -9.81</gravity>
    <max_step_size>0.001</max_step_size>
  </physics>
  <model name="ground_plane">
    <static>true</static>
    <!-- geometry and collision definitions -->
  </model>
</world>
```

---

### 2. Unity Scene

**Description**: A 3D environment containing GameObjects (robot, obstacles, cameras), lighting, materials, physics settings, and visual assets optimized for high-fidelity rendering.

**Attributes**:
- `name` (string): Scene identifier (e.g., "HumanoidSimulationScene")
- `rendering_pipeline` (enum): URP (Universal Render Pipeline), HDRP (High Definition), Built-in
- `physics_timestep` (float): Fixed timestep for PhysX physics (default: 0.02s)
- `gravity` (vector3): Gravity in m/s² (default: [0, -9.81, 0])
- `game_objects` (list<GameObject>): All scene entities (robots, environment, cameras)
- `lighting` (Lighting): Global illumination settings (ambient, skybox, directional light)
- `materials` (list<Material>): Surface shaders and textures

**Relationships**:
- **Contains** zero or more Robot Models (as Articulation Bodies)
- **Contains** zero or more interactive GameObjects (obstacles, manipulation targets)
- **Contains** zero or more Camera GameObjects (for visualization or sensor simulation)
- **References** Material Properties for all rendered surfaces

**File Format**: Unity `.unity` scene file (binary or YAML)

**Example Hierarchy**:
```
HumanoidSimulationScene
├── Lighting (Directional Light + Ambient)
├── Environment
│   ├── Floor (GameObject with BoxCollider)
│   ├── Walls (GameObject with MeshRenderer)
│   └── Obstacles (GameObject with Rigidbody)
├── HumanoidRobot (ArticulationBody hierarchy)
└── Cameras
    ├── MainCamera (Free-look)
    └── RobotCamera (First-person on robot head)
```

---

### 3. Physics Engine

**Description**: The computational component responsible for collision detection, constraint solving, and numerical integration of dynamics. Simulates how objects move and interact under forces.

**Attributes**:
- `solver_type` (enum): ODE (Open Dynamics Engine), Bullet, DART, PhysX
- `gravity` (vector3): Global gravitational acceleration
- `max_step_size` (float): Maximum time step for integration (stability vs. performance)
- `solver_iterations` (int): Number of constraint solver iterations (higher = more accurate, slower)
- `contact_max_correcting_velocity` (float): Maximum velocity correction for penetrating contacts
- `contact_surface_layer` (float): Depth of surface layer for contacts (affects stability)

**Relationships**:
- **Belongs to** exactly one Gazebo World or Unity Scene
- **Applies to** all Robot Models and dynamic objects in the environment
- **Uses** Material Properties to determine friction and restitution

**Gazebo Example** (ODE):
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
    </solver>
  </ode>
</physics>
```

**Unity Example** (PhysX via Project Settings):
- Fixed Timestep: 0.02s
- Default Solver Iterations: 6
- Default Solver Velocity Iterations: 1

**Common Issues**:
- **Jitter**: Solver iterations too low, objects vibrate at contacts
- **Explosions**: Time step too large, constraints violated
- **Slow simulation**: Solver iterations too high, excessive computation

---

### 4. Sensor Plugin

**Description**: A software module that simulates sensor behavior by processing simulation data (3D geometry, physics state) and outputting realistic sensor readings (point clouds, images, IMU data).

**Attributes**:
- `sensor_type` (enum): LiDAR (gpu_ray/ray), depth_camera, camera, imu, contact, force_torque
- `update_rate` (float): Sensor publishing frequency in Hz (e.g., 10 Hz for LiDAR, 100 Hz for IMU)
- `topic_name` (string): ROS 2 topic for sensor data publication (e.g., "/lidar/points")
- `frame_id` (string): TF frame for sensor coordinate system (e.g., "lidar_link")
- `noise_model` (NoiseModel): Gaussian noise parameters (mean, std_dev) or more complex models
- `sensor_specific_params` (dict): Range limits (LiDAR), resolution (camera), FOV (depth camera)

**Relationships**:
- **Attached to** a Robot Model link (parent link determines sensor pose)
- **Publishes to** ROS 2 topics (data consumed by RViz2, algorithms, or rosbag)
- **Uses** Physics Engine state (for IMU) or rendered geometry (for LiDAR/cameras)

**Gazebo LiDAR Example**:
```xml
<sensor name="lidar" type="gpu_ray">
  <update_rate>10</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>6.28</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
    </range>
  </ray>
  <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>~/out:=lidar/points</remapping>
    </ros>
  </plugin>
</sensor>
```

**Unity LiDAR Example** (via Unity Robotics Visualizations):
- Component: `LidarSensor.cs` (custom script or package-provided)
- Parameters: Range (30m), Angular Resolution (1°), Update Rate (10 Hz)
- Output: Published to ROS 2 via ROS TCP Connector as `sensor_msgs/PointCloud2`

---

### 5. Robot Model

**Description**: The digital representation of a humanoid robot consisting of links (rigid bodies), joints (kinematic constraints), collision geometry, visual meshes, and sensor mounts.

**Attributes**:
- `name` (string): Robot identifier (e.g., "simple_humanoid")
- `links` (list<Link>): Rigid bodies with mass, inertia, collision, and visual geometry
- `joints` (list<Joint>): Connections between links (revolute, prismatic, fixed) with limits and dynamics
- `sensors` (list<SensorPlugin>): Attached sensor plugins (LiDAR on head, IMU in torso, cameras)
- `base_link` (string): Root link name (e.g., "base_link" or "torso")
- `coordinate_frame` (Transform): Pose of the robot in world coordinates

**Relationships**:
- **Defined by** URDF (Unified Robot Description Format) from Module 1
- **Placed in** Gazebo World or Unity Scene
- **Has many** Material Properties (one per link surface)
- **Has many** Sensor Plugins (attached to specific links)

**Link Attributes**:
- `mass` (float): Total mass in kg
- `inertia` (matrix3x3): Moment of inertia tensor in kg·m²
- `collision_geometry` (Geometry): Simplified shapes for physics (boxes, cylinders, meshes)
- `visual_geometry` (Geometry): High-fidelity meshes for rendering

**Joint Attributes**:
- `type` (enum): revolute (hinge), prismatic (slider), fixed, continuous
- `axis` (vector3): Rotation/translation axis
- `limits` (JointLimits): Position limits, velocity limits, effort limits
- `damping` (float): Joint damping coefficient
- `friction` (float): Joint friction coefficient

**Example URDF Snippet** (torso link with IMU):
```xml
<link name="torso">
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.1" iyy="0.1" izz="0.05" ixy="0" ixz="0" iyz="0"/>
  </inertial>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.4"/>
    </geometry>
  </collision>
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/torso.dae"/>
    </geometry>
  </visual>
</link>

<gazebo reference="torso">
  <sensor name="imu" type="imu">
    <update_rate>100</update_rate>
    <!-- IMU plugin configuration -->
  </sensor>
</gazebo>
```

---

### 6. Material Properties

**Description**: Surface characteristics that define how objects interact physically (friction, restitution) and how they appear visually (color, texture, reflectance).

**Attributes**:

**Physical Properties**:
- `friction_coefficient` (float): Coulomb friction (0 = frictionless, 1+ = high friction)
- `restitution` (float): Bounciness (0 = inelastic, 1 = perfectly elastic)
- `contact_stiffness` (float): Spring constant for contact force calculation (advanced)
- `contact_damping` (float): Damping for contact oscillations (advanced)

**Visual Properties**:
- `albedo_color` (RGB): Base color (e.g., [0.8, 0.8, 0.8] for light gray)
- `texture_map` (string): File path to diffuse texture image (PNG, JPG)
- `metallic` (float): Metallic appearance (0 = dielectric, 1 = metal)
- `roughness` (float): Surface roughness (0 = mirror, 1 = matte)
- `normal_map` (string): Normal map for surface detail (optional)

**Relationships**:
- **Applied to** Robot Model links or environmental objects
- **Used by** Physics Engine for collision response
- **Rendered by** Gazebo/Unity graphics pipeline

**Gazebo Example** (ground plane material):
```xml
<collision name="collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>   <!-- friction coefficient -->
        <mu2>1.0</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

**Unity Example** (PBR material on floor):
- Shader: Standard (Metallic workflow)
- Albedo: Concrete texture (PNG)
- Metallic: 0.0
- Smoothness: 0.3 (roughness = 0.7)
- PhysicMaterial: Friction = 0.6, Bounciness = 0.0

**Educational Importance**: Understanding material properties is critical for realistic humanoid locomotion (foot friction prevents slipping) and manipulation (gripper friction enables grasping).

---

### 7. Point Cloud

**Description**: A 3D data structure representing LiDAR sensor output as a collection of points in Cartesian space, each with (x, y, z) coordinates and optional intensity or color information.

**Attributes**:
- `points` (list<Point3D>): Array of 3D points, each with (x, y, z) in meters
- `frame_id` (string): Coordinate frame of reference (e.g., "lidar_link")
- `timestamp` (float): ROS 2 timestamp (seconds since epoch)
- `intensity` (list<float>): Optional intensity values (0.0 to 1.0) for each point
- `color` (list<RGB>): Optional RGB color for each point (rare in LiDAR, common in RGB-D)
- `num_points` (int): Total number of points (e.g., 360 for 1° resolution 2D scan)

**Relationships**:
- **Generated by** LiDAR Sensor Plugin
- **Published to** ROS 2 topic as `sensor_msgs/PointCloud2`
- **Visualized in** RViz2 or Unity
- **Processed by** perception algorithms (SLAM, object detection, obstacle avoidance)

**Data Format**:
- **ROS 2 Message**: `sensor_msgs/PointCloud2` (binary point cloud with flexible fields)
- **File Format**: PCD (Point Cloud Data), PLY (Polygon File Format), LAS (LiDAR data exchange)

**Example Point Cloud** (2D LiDAR, 360° scan):
```
num_points: 360
points[0]: {x: 2.5, y: 0.0, z: 0.5, intensity: 0.8}   # Wall at 2.5m ahead
points[90]: {x: 0.0, y: 3.2, z: 0.5, intensity: 0.6}  # Wall at 3.2m to the right
points[180]: {x: -10.0, y: 0.0, z: 0.5, intensity: 0.3} # Far wall at 10m behind
```

**Validation Criteria** (for exercises):
- Point cloud should match environment geometry (±5cm error)
- Number of points should equal `scan_samples` (e.g., 360 for 1° resolution)
- Points beyond max_range should be discarded or marked as invalid (inf values)

---

### 8. Depth Image

**Description**: A 2D array representing the distance from a depth camera to scene surfaces at each pixel, encoded as a grayscale or floating-point image.

**Attributes**:
- `width` (int): Image width in pixels (e.g., 640, 1280)
- `height` (int): Image height in pixels (e.g., 480, 720)
- `depth_values` (matrix<float>): 2D array of distances in meters (or millimeters)
- `min_depth` (float): Minimum valid depth range (e.g., 0.5m)
- `max_depth` (float): Maximum valid depth range (e.g., 5.0m)
- `intrinsics` (CameraIntrinsics): Focal length (fx, fy), principal point (cx, cy)
- `frame_id` (string): Coordinate frame (e.g., "depth_camera_link")
- `timestamp` (float): ROS 2 timestamp

**Relationships**:
- **Generated by** Depth Camera Sensor Plugin
- **Published to** ROS 2 topic as `sensor_msgs/Image` (encoding: 32FC1 or 16UC1)
- **Paired with** RGB image for RGB-D data (optional)
- **Converted to** Point Cloud via intrinsic parameters (for 3D reconstruction)

**Camera Intrinsics**:
- `fx`, `fy` (float): Focal lengths in pixels (x and y directions)
- `cx`, `cy` (float): Principal point (optical center) in pixels
- **Intrinsic Matrix**:
  ```
  K = [fx  0  cx]
      [ 0 fy  cy]
      [ 0  0   1]
  ```

**Data Format**:
- **ROS 2 Message**: `sensor_msgs/Image` with encoding `32FC1` (32-bit float, 1 channel)
- **File Format**: PNG (16-bit grayscale), EXR (OpenEXR for floating-point), TIFF

**Example Depth Image** (640x480, depth in meters):
```
depth_values[240][320] = 2.5  # Center pixel: object at 2.5m
depth_values[100][100] = 1.2  # Top-left: object at 1.2m
depth_values[400][500] = inf  # Bottom-right: no return (beyond max_range)
```

**Noise Model** (educational):
- **Basic**: Gaussian noise σ = k × d² (noise increases quadratically with distance, k ≈ 0.001)
- **Advanced**: Flying pixels (invalid depth at edges), IR interference patterns

**Validation Criteria** (for exercises):
- Depth values should match ground truth distance (±2% error)
- Pixels beyond max_depth should be marked as invalid (inf or 0)
- Depth image should align with RGB image (if RGB-D)

---

### 9. IMU Data

**Description**: Time-series measurements from an Inertial Measurement Unit (IMU) containing linear acceleration (accelerometer) and angular velocity (gyroscope) in the sensor's local frame.

**Attributes**:
- `linear_acceleration` (vector3): Acceleration in m/s² (includes gravity) along x, y, z axes
- `angular_velocity` (vector3): Rotation rate in rad/s around x, y, z axes
- `orientation` (quaternion): Optional estimated orientation (if sensor fusion is included)
- `frame_id` (string): Coordinate frame (e.g., "imu_link")
- `timestamp` (float): ROS 2 timestamp
- `accelerometer_noise` (NoiseModel): Gaussian noise parameters (mean, std_dev)
- `gyroscope_noise` (NoiseModel): Gaussian noise parameters (mean, std_dev)
- `accelerometer_bias` (vector3): Constant offset error in m/s²
- `gyroscope_bias` (vector3): Constant offset error in rad/s

**Relationships**:
- **Generated by** IMU Sensor Plugin
- **Published to** ROS 2 topic as `sensor_msgs/Imu`
- **Attached to** Robot Model link (typically torso or base_link)
- **Used by** state estimation algorithms (EKF, UKF), sensor fusion, odometry

**Data Format**:
- **ROS 2 Message**: `sensor_msgs/Imu`
  ```
  linear_acceleration: {x, y, z}  # m/s²
  angular_velocity: {x, y, z}     # rad/s
  orientation: {x, y, z, w}       # quaternion (optional)
  ```

**Noise Models**:

**Basic (Gaussian White Noise)**:
- Accelerometer: σ = 0.01 m/s²
- Gyroscope: σ = 0.001 rad/s
- Simple constant bias: ±0.05 m/s² (accel), ±0.005 rad/s (gyro)

**Advanced (Allan Variance)**:
- **Angle Random Walk (N)**: Characterizes short-term gyroscope noise
- **Rate Random Walk (K)**: Characterizes long-term drift
- **Bias Instability (B)**: Temperature-dependent drift
- **Source**: Estimated from 15-24 hour stationary datasets, used in Kalman filter Q matrix

**Example IMU Reading** (robot stationary on ground):
```
linear_acceleration: {x: 0.02, y: -0.01, z: 9.81}  # Gravity + noise
angular_velocity: {x: 0.001, y: -0.002, z: 0.0}    # Near-zero rotation + noise
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}      # Identity quaternion (upright)
```

**Validation Criteria** (for exercises):
- Stationary robot: linear_acceleration ≈ [0, 0, 9.81] ± noise
- Constant rotation: angular_velocity should match commanded rotation rate
- Noise statistics: Mean near zero, std dev matches configured parameters

**Educational Importance**: IMU simulation teaches sensor fusion, state estimation, and the critical distinction between ideal (simulation) and real (noisy, biased) sensor behavior.

---

## Entity Relationships Diagram

```
┌─────────────────┐         ┌─────────────────┐
│  Gazebo World   │         │  Unity Scene    │
│  (SDF file)     │         │  (Unity file)   │
└────────┬────────┘         └────────┬────────┘
         │                           │
         │ contains                  │ contains
         ├───────────────────────────┤
         │                           │
         v                           v
┌─────────────────┐         ┌─────────────────┐
│ Physics Engine  │         │  Robot Model    │
│  (ODE, PhysX)   │◄────────┤  (URDF/Articu.) │
└─────────────────┘ applies └────────┬────────┘
                                     │
                            ┌────────┼────────┐
                            │        │        │
                            v        v        v
                    ┌─────────────────┐  ┌─────────────────┐
                    │ Sensor Plugin   │  │Material Props   │
                    │ (LiDAR/IMU/Cam) │  │ (Friction/Tex)  │
                    └────────┬────────┘  └─────────────────┘
                             │
                    ┌────────┼────────┐
                    │        │        │
                    v        v        v
            ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
            │  Point Cloud    │  │  Depth Image    │  │    IMU Data     │
            │ (LiDAR output)  │  │ (Camera output) │  │ (Accel + Gyro)  │
            └─────────────────┘  └─────────────────┘  └─────────────────┘
                    │                    │                    │
                    └────────────────────┴────────────────────┘
                                         │
                                         v
                              ┌─────────────────┐
                              │  ROS 2 Topics   │
                              │ (sensor_msgs)   │
                              └─────────────────┘
```

## Usage in Educational Content

### Chapter 1: Gazebo Physics Fundamentals
**Primary Entities**: Gazebo World, Physics Engine, Robot Model, Material Properties
- Students create worlds, configure physics, import URDFs, debug collisions

### Chapter 2: Unity Visual Environments
**Primary Entities**: Unity Scene, Robot Model (Articulation Body), Material Properties
- Students build scenes, import URDFs, add textures and lighting

### Chapter 3: Sensor Simulation
**Primary Entities**: Sensor Plugin, Point Cloud, Depth Image, IMU Data
- Students configure sensors, collect data, validate accuracy, understand noise

---

**Data Model Version**: 1.0
**Last Updated**: 2025-12-07
**Related Documents**: spec.md, plan.md, research.md
