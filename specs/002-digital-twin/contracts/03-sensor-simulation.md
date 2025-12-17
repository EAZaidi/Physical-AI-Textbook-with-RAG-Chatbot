# Chapter Contract: Chapter 3 - Sensor Simulation and Data Collection

**Chapter**: 03-sensor-simulation.mdx
**Target Length**: 5-6 pages
**Priority**: P1 (Critical for perception algorithm development)
**Prerequisites**: Module 1 (ROS 2, URDF), Chapter 1 (Gazebo), Chapter 2 (Unity)

## Learning Objectives

By the end of this chapter, students will be able to:

1. **Configure** LiDAR sensors in Gazebo (gpu_ray) and Unity (Raytraced Lidar) with realistic parameters
2. **Set up** depth cameras with accurate intrinsic parameters and configurable noise models
3. **Implement** IMU sensors with Gaussian noise models and understand Allan variance (advanced)
4. **Collect** and export synthetic sensor data (PCD point clouds, PNG/EXR depth images, CSV IMU logs)
5. **Validate** sensor accuracy by comparing simulated data to ground truth geometry

## Chapter Outline

### Section 1: Introduction (0.5 pages)

**Purpose**: Motivate sensor simulation and preview data collection workflow

**Content**:
- Why simulate sensors before deploying on hardware:
  - Safe algorithm testing (no risk of robot damage)
  - Perfect ground truth for validation (known environment geometry)
  - Rapid iteration (no hardware setup/teardown)
  - Cost-effective (no need for expensive LiDAR/depth cameras initially)
- Overview of sensors covered: LiDAR (3D point clouds), Depth Camera (2D depth images), IMU (acceleration + gyroscope)
- Preview: Students will configure sensors, collect data, and plot/visualize results
- Reality gap: Simulation is ideal; real sensors have noise, bias, and failures

**Deliverables**:
- 1 paragraph: Value of sensor simulation for algorithm development
- 1 paragraph: Reality gap explanation (simulation limitations)
- 1 screenshot: Final result preview (RViz2 with point cloud + depth image + IMU plot)

---

### Section 2: LiDAR Simulation (1.5 pages)

**Purpose**: Configure GPU-accelerated LiDAR for 3D environment mapping

**Content**:

#### 2.1 Gazebo LiDAR (gpu_ray sensor)

- **Sensor Configuration in URDF**:
  - Add `<sensor type="gpu_ray">` to robot link (typically head or torso)
  - Key parameters:
    - Range: 0.1m (min) to 30m (max) for indoor/outdoor
    - Horizontal scan: 360° coverage, 1° resolution (360 samples)
    - Vertical scan: Single-layer (2D) or multi-layer (3D, e.g., 16/32/64 layers)
    - Update rate: 10-20 Hz (balance between data density and performance)
  - Noise model: Gaussian noise (mean=0, std_dev=0.01m)
- **Launch Gazebo with LiDAR**:
  - Spawn robot with LiDAR sensor in Gazebo world
  - Verify topic: `ros2 topic echo /robot/lidar/points`
  - Visualize in RViz2: Add PointCloud2 display, set topic to `/robot/lidar/points`
- **Validate Accuracy**:
  - Place robot 5m from wall, measure point cloud distance
  - Expected: Points at ~5.0m ± 5cm (within noise tolerance)

**Code Examples**:
```xml
<!-- Inline URDF: LiDAR sensor configuration (25-30 lines) -->
<gazebo reference="head_link">
  <sensor name="lidar" type="gpu_ray">
    <pose>0 0 0.1 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>0.0</min_angle>
          <max_angle>6.28318</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=lidar/points</remapping>
      </ros>
      <output_type>sensor_msgs/PointCloud2</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

#### 2.2 Unity LiDAR (Raytraced Lidar)

- **Unity Robotics Visualizations Package**:
  - Install via Package Manager: `com.unity.robotics.visualizations`
  - Add LidarSensor component to robot GameObject
- **Configuration**:
  - Range: 30m
  - Field of View: 360° horizontal, ±15° vertical (for 3D LiDAR)
  - Angular Resolution: 1° (360 points per scan)
  - Update Rate: 10 Hz
  - Raytraced mode: GPU-accelerated (requires Vulkan on Windows/Linux)
- **Publish to ROS 2**:
  - Use ROS TCP Connector to publish `sensor_msgs/PointCloud2`
  - Verify in RViz2 (same workflow as Gazebo)

**Deliverables**:
- Gazebo LiDAR URDF snippet (inline XML, 30 lines with annotations)
- Unity LiDAR setup steps (numbered list, 4-5 steps)
- 2 screenshots:
  - RViz2 with point cloud from Gazebo (colored by height or intensity)
  - Unity Scene view with LiDAR rays visualized (green lines)

**Validation**:
- [ ] Point cloud publishes to ROS 2 topic at configured rate (verify with `ros2 topic hz`)
- [ ] Point cloud contains expected number of points (e.g., 360 for 1° resolution)
- [ ] Measured distance to wall matches ground truth (±5cm)

---

### Section 3: Depth Camera Simulation (1.5 pages)

**Purpose**: Configure RGB-D cameras for vision-based perception

**Content**:

#### 3.1 Gazebo Depth Camera (depth_camera plugin)

- **Sensor Configuration**:
  - Add `<sensor type="depth_camera">` to robot head link
  - Key parameters:
    - Resolution: 640×480 or 1280×720 (common RGB-D camera resolutions)
    - Field of View: 60-75° (horizontal, matches Intel RealSense/Kinect)
    - Depth Range: 0.5m (min) to 5.0m (max) for indoor scenes
    - Update Rate: 30 Hz (video frame rate)
  - Noise model: Gaussian noise proportional to depth² (σ = 0.001 × d²)
- **Outputs**:
  - RGB image: `sensor_msgs/Image` (8-bit color, encoding: rgb8)
  - Depth image: `sensor_msgs/Image` (32-bit float, encoding: 32FC1)
  - Camera info: `sensor_msgs/CameraInfo` (intrinsic parameters: fx, fy, cx, cy)
- **Visualize in RViz2**:
  - RGB image: Add Image display, topic `/robot/camera/rgb/image_raw`
  - Depth image: Add DepthCloud display, topic `/robot/camera/depth/image_raw`

**Code Examples**:
```xml
<!-- Inline URDF: Depth camera configuration (30-35 lines) -->
<gazebo reference="head_link">
  <sensor name="depth_camera" type="depth_camera">
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.5</near>
        <far>5.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev> <!-- depth-dependent noise -->
      </noise>
    </camera>
    <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/robot/camera</namespace>
        <remapping>~/image_raw:=rgb/image_raw</remapping>
        <remapping>~/depth/image_raw:=depth/image_raw</remapping>
        <remapping>~/camera_info:=camera_info</remapping>
      </ros>
      <frame_name>camera_depth_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

#### 3.2 Unity Depth Camera (Camera Component + Depth Texture)

- **Unity Camera Setup**:
  - Add Camera GameObject as child of robot head
  - Configure: Field of View: 60°, Near: 0.5m, Far: 5.0m
  - Enable depth texture: Camera → Target Texture → Create RenderTexture with Depth format
- **Publish Depth to ROS 2**:
  - Use custom C# script to read depth texture and publish as `sensor_msgs/Image`
  - Optional: Apply noise shader for realistic depth errors

**Deliverables**:
- Gazebo depth camera URDF snippet (inline XML, 35 lines)
- Unity depth camera setup steps (numbered list, 5-6 steps)
- 2 screenshots:
  - RViz2 with side-by-side RGB and depth images
  - Unity Game view with depth visualization (grayscale or color-mapped)

**Validation**:
- [ ] RGB and depth images publish at 30 Hz
- [ ] Depth values match ground truth distance (±2% error)
- [ ] Camera info topic contains correct intrinsic parameters (verify fx, fy, cx, cy)

---

### Section 4: IMU Simulation (1 page)

**Purpose**: Configure inertial sensors for state estimation and sensor fusion

**Content**:

#### 4.1 Gazebo IMU (imu sensor)

- **Sensor Configuration**:
  - Add `<sensor type="imu">` to robot torso or base_link
  - Key parameters:
    - Update rate: 100-200 Hz (typical IMU frequency)
    - Linear acceleration noise: Gaussian (σ = 0.01 m/s²)
    - Angular velocity noise: Gaussian (σ = 0.001 rad/s)
    - Bias: Small constant offset (e.g., ±0.05 m/s² for accel, ±0.005 rad/s for gyro)
  - Outputs: `sensor_msgs/Imu` (linear_acceleration, angular_velocity, orientation)
- **Validate IMU**:
  - Stationary robot: linear_acceleration ≈ [0, 0, 9.81] (gravity only)
  - Rotating robot: angular_velocity matches commanded rotation rate

**Code Examples**:
```xml
<!-- Inline URDF: IMU sensor configuration (20-25 lines) -->
<gazebo reference="torso_link">
  <sensor name="imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.001</stddev>
          </noise>
        </x>
        <!-- y and z similar -->
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </x>
        <!-- y and z similar -->
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/robot</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

#### 4.2 Unity IMU (ArticulationBody + Custom Script)

- **Unity IMU Implementation**:
  - Read `ArticulationBody.velocity` and `ArticulationBody.angularVelocity`
  - Add gravity vector (0, -9.81, 0) to linear acceleration
  - Add Gaussian noise programmatically in C# script
- **Publish to ROS 2**: Use ROS TCP Connector to publish `sensor_msgs/Imu`

**Advanced Topic** (Optional): Allan Variance for Realistic IMU Noise
- Brief explanation: Allan variance characterizes long-term IMU drift
- Reference external tools: `ori-drs/allan_variance_ros` for parameter estimation
- When to use: Advanced sensor fusion projects requiring research-grade accuracy

**Deliverables**:
- Gazebo IMU URDF snippet (inline XML, 25 lines)
- Unity IMU implementation notes (2-3 paragraphs)
- 1 plot: IMU data over 10 seconds (linear acceleration and angular velocity, with noise visible)

**Validation**:
- [ ] IMU publishes at 100+ Hz
- [ ] Stationary robot: linear_acceleration ≈ [0, 0, 9.81] ± noise
- [ ] Noise statistics: Mean ≈ 0, std dev matches configured parameters

---

### Section 5: Data Collection & Export (1 page)

**Purpose**: Teach students to record and process sensor data for offline analysis

**Content**:

#### 5.1 Recording with rosbag2

- **Record Sensor Topics**:
  ```bash
  ros2 bag record /robot/lidar/points /robot/camera/depth/image_raw /robot/imu/data
  ```
- **Playback for Offline Analysis**:
  ```bash
  ros2 bag play <bag_file_name>
  ```
- **Inspect Bag Contents**:
  ```bash
  ros2 bag info <bag_file_name>
  ```

#### 5.2 Exporting Data Formats

- **Point Cloud Export** (PCD format):
  - Use `pcl_ros` or Python script to convert `sensor_msgs/PointCloud2` to .pcd file
  - PCD format: ASCII or binary, readable by CloudCompare, MeshLab, Python Open3D
- **Depth Image Export** (PNG/EXR):
  - PNG: 16-bit grayscale (lossy for >65535mm depth)
  - EXR: OpenEXR 32-bit float (lossless, preserves full depth range)
  - Python example: Use `cv_bridge` to convert ROS Image → OpenCV → save as PNG/EXR
- **IMU Data Export** (CSV):
  - Python script: Subscribe to `/robot/imu/data`, log to CSV with columns: timestamp, ax, ay, az, gx, gy, gz
  - CSV format: Compatible with Excel, MATLAB, Python pandas

**Code Examples**:
```python
# Inline Python: Export point cloud to PCD (15-18 lines)
import rclpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d

def pointcloud_callback(msg):
    # Convert ROS PointCloud2 to Open3D format
    points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points_list = [[p[0], p[1], p[2]] for p in points]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_list)
    o3d.io.write_point_cloud("output.pcd", pcd)
    print("Point cloud saved to output.pcd")

# ROS 2 node setup omitted for brevity (see full example in code repository)
```

```python
# Inline Python: Export IMU to CSV (12-15 lines)
import rclpy
from sensor_msgs.msg import Imu
import csv

csv_file = open('imu_data.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['timestamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])

def imu_callback(msg):
    timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
    gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
    csv_writer.writerow([timestamp, ax, ay, az, gx, gy, gz])

# ROS 2 node setup omitted for brevity
```

**Deliverables**:
- rosbag2 commands (inline bash, 3 commands with explanations)
- Point cloud export script (inline Python, 18 lines)
- IMU CSV export script (inline Python, 15 lines)
- 1 screenshot: Output files (output.pcd, imu_data.csv) in file explorer

**Validation**:
- [ ] rosbag2 successfully records all sensor topics
- [ ] PCD file opens in CloudCompare or Open3D
- [ ] CSV file opens in Excel/pandas with correct column headers
- [ ] Exported data matches real-time ROS 2 topic data (spot-check 5-10 samples)

---

### Section 6: Exercises (0.5 pages)

**Purpose**: Hands-on sensor configuration and data validation

#### Exercise 1: Configure LiDAR and Verify Accuracy

**Task**:
1. Add LiDAR sensor to humanoid robot head (Gazebo or Unity)
2. Place robot 3m from a wall in simulation
3. Collect 100 point cloud scans (10 seconds at 10 Hz)
4. Measure average distance to wall from point cloud data
5. Compare to ground truth (3.0m ± 5cm)

**Expected Output**:
- rosbag2 file with 100 scans
- Python script output: "Average wall distance: 3.02m (error: +0.02m)"
- Screenshot: RViz2 with point cloud visualization

**Success Criteria**:
- [ ] Point cloud publishes at 10 Hz (verify with `ros2 topic hz`)
- [ ] Average measured distance within ±5cm of ground truth
- [ ] Point cloud contains ~360 points per scan

#### Exercise 2: Add Noise to Depth Camera and Observe Effects

**Task**:
1. Configure depth camera in Gazebo with no noise (std_dev=0.0)
2. Capture depth image, measure distance to object at known location
3. Increase noise (std_dev=0.02), capture new depth image
4. Compare: Plot depth values along horizontal centerline for both images
5. Observe: Noisy image has more variance around true depth

**Expected Output**:
- 2 depth images (PNG): noise-free and noisy
- Matplotlib plot: Depth values vs. pixel position (2 curves overlaid)
- Observation: Noisy curve has ±2-5cm variance around ground truth

**Success Criteria**:
- [ ] Noise-free depth image shows consistent depth values (std dev < 1cm)
- [ ] Noisy depth image shows increased variance (std dev ≈ 2cm)
- [ ] Plot clearly visualizes difference between noise levels

---

## Code Examples Summary

**Total Code Examples**: 4 inline snippets (XML/Python) + 3 bash commands

1. **Gazebo LiDAR URDF** (XML, 30 lines) - Section 2
2. **Gazebo Depth Camera URDF** (XML, 35 lines) - Section 3
3. **Gazebo IMU URDF** (XML, 25 lines) - Section 4
4. **rosbag2 commands** (bash, 3 commands) - Section 5
5. **Point cloud to PCD export** (Python, 18 lines) - Section 5
6. **IMU to CSV export** (Python, 15 lines) - Section 5

**Separate Files** (in `docs/docs/module-2-digital-twin/assets/code-examples/`):
- `lidar_config.sdf` - Complete Gazebo LiDAR sensor configuration
- `depth_camera_config.sdf` - Complete Gazebo depth camera configuration
- `imu_config.sdf` - Complete Gazebo IMU configuration
- `collect_sensor_data.py` - Complete ROS 2 node for recording all sensors
- `plot_imu_data.py` - Matplotlib script for visualizing IMU logs
- `README.md` - Usage instructions for all code examples

---

## Screenshots & Diagrams

**Required Screenshots** (7 total):
1. RViz2 with LiDAR point cloud (Gazebo) (Section 2)
2. Unity Scene view with LiDAR rays visualized (Section 2)
3. RViz2 with RGB + depth images side-by-side (Section 3)
4. Unity Game view with depth visualization (Section 3)
5. IMU data plot (10 seconds, accel + gyro) (Section 4)
6. File explorer showing exported files (PCD, CSV) (Section 5)
7. Exercise 1: RViz2 with wall point cloud (Exercise 1)

**Optional Diagrams**:
- Sensor coordinate frames: Camera frame, LiDAR frame, IMU frame (SVG)
- ROS 2 topic architecture: Sensor plugins → Topics → RViz2/rosbag2 (flowchart)

---

## Validation Checklist

**Content Quality**:
- [ ] All sensor configurations tested in Gazebo Fortress and Unity 2022.3 LTS
- [ ] Python scripts tested with ROS 2 Humble (rclpy, sensor_msgs, cv_bridge, open3d)
- [ ] Sensor accuracy validated against ground truth (LiDAR ±5cm, depth camera ±2%)
- [ ] Data export formats verified (PCD opens in CloudCompare, CSV opens in Excel)

**Educational Clarity**:
- [ ] Learning objectives stated at beginning, revisited in summary
- [ ] Sensor parameters explained with physical units and typical values
- [ ] Noise models justified (why Gaussian, when to use Allan variance)
- [ ] Exercises include clear validation steps and expected outputs

**Constitution Compliance**:
- [ ] All claims reference official Gazebo/Unity/ROS 2 documentation
- [ ] Code examples reproducible with documented dependencies (rclpy, open3d, cv_bridge)
- [ ] Page count: 5-6 pages (within target)
- [ ] No hardcoded file paths (use ROS 2 parameters or command-line arguments)

---

## Dependencies

**From Module 1**:
- ROS 2 topic subscription and publishing
- Understanding of sensor_msgs message types (PointCloud2, Image, Imu, CameraInfo)

**From Chapter 1 & 2**:
- Gazebo world with humanoid robot (for sensor attachment)
- Unity scene with humanoid robot (for Unity sensor examples)

**External Documentation**:
- Gazebo Sensors: https://gazebosim.org/docs/fortress/sensors
- Unity Robotics Visualizations: https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/packages/visualizations
- ROS 2 sensor_msgs: https://github.com/ros2/common_interfaces/tree/humble/sensor_msgs

**Tools Required**:
- Python packages: rclpy, sensor_msgs_py, cv_bridge, open3d, matplotlib, pandas
  ```bash
  pip install open3d matplotlib pandas
  sudo apt install ros-humble-cv-bridge  # Ubuntu
  ```
- rosbag2 (included with ROS 2 Humble)
- RViz2 (included with ROS 2 Humble)
- CloudCompare or MeshLab (optional, for PCD visualization)

---

## Follow-Up in Later Modules

**Module 3 (Perception)**: Will use LiDAR and depth camera data for object detection and SLAM

**Module 4 (Motion Planning)**: Will use LiDAR for obstacle avoidance and safe navigation

**Module 5 (Manipulation)**: Will use depth cameras for grasp planning and object pose estimation

---

**Contract Status**: ✅ READY FOR IMPLEMENTATION
**Estimated Writing Time**: 3-5 hours (including sensor configuration testing and data collection examples)
**Last Updated**: 2025-12-07
