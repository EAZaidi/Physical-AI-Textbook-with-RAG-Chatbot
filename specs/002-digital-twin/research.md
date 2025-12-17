# Technical Research: Module 2 - The Digital Twin (Gazebo & Unity)

**Created**: 2025-12-07
**Feature Branch**: `002-digital-twin`
**Purpose**: Research key technical decisions for educational content development

---

## Decision 1: Gazebo Version(s)

### Chosen: Gazebo Fortress (primary) with Garden (optional/advanced)

### Rationale

**Primary: Gazebo Fortress**
- **Stability with ROS 2 Humble**: ROS 2 Humble (Ubuntu 22.04) officially supports Gazebo Fortress with binary packages available through official ROS repositories, making it the most stable and widespread distribution
- **Binary Package Availability**: Fortress has official binary packages, eliminating compilation issues that plague Garden installations
- **Proven Integration**: For production work in 2025 with ROS 2 Humble, Fortress remains the more stable and well-supported option with better integration reliability
- **Community Support**: Larger user base and more extensive troubleshooting resources available

**Optional: Gazebo Garden (for advanced sections)**
- **Future-Proofing**: From Garden onwards, "Ignition Gazebo" was renamed to "Gazebo Sim" due to trademark issues. Students using Fortress will eventually need to update code/SDF files
- **New Features**: Garden includes bug fixes and new functionality not available in Fortress
- **Learning Path**: Introducing Garden in optional/advanced sections prepares students for future migrations

### Alternatives Considered

1. **Gazebo Classic**: Rejected due to deprecated status and poor ROS 2 integration
2. **Gazebo Garden Only**: Rejected due to compilation issues with ros_gz and lack of official binary packages for ROS 2 Humble
3. **Latest Gazebo (Harmonic)**: Too cutting-edge for educational stability; insufficient community resources

### Implementation for Educational Content

- **Chapter 1 (Basic Setup)**: Focus exclusively on Fortress with Docker installation
- **Chapter 2 (Advanced Features)**: Include optional Garden migration guide as advanced topic
- **All Code Examples**: Test primarily on Fortress, note Garden compatibility where applicable

### Sources
- [Gazebo Fortress vs Garden ROS 2 Integration Discussion](https://robotics.stackexchange.com/questions/115514/with-ros2-humble-is-there-any-reason-to-use-gazebo-garden-over-fortress)
- [Migration from Gazebo Classic to Ignition with ROS 2](https://www.blackcoffeerobotics.com/blog/gazebo-simulator-migration-from-classic-to-ignition-with-ros2)
- [ROS 2 Integration - Gazebo Fortress Documentation](https://gazebosim.org/docs/fortress/ros2_integration/)

---

## Decision 2: Unity Version & Packages

### Chosen: Unity 2022.3 LTS with Unity Robotics Hub packages

### Rationale

**Unity 2022.3 LTS**
- **Verified Compatibility**: Unity Robotics Hub officially supports "unity-2020.2+" and has been extensively tested with 2022 LTS releases
- **LTS Support Duration**: Unity 2022.3 LTS receives 2-year support for Personal/Pro users and 3-year support for Enterprise/Industry subscribers (until ~2025-2026)
- **Stability**: LTS releases are recommended for educational use due to stable APIs and minimal breaking changes
- **Robotics-Specific Features**: Contact Modification API (available in 2021.2a12+) and continuous collision detection modes (available in 2020.3.5f1+) are fully supported

**Unity Robotics Hub Package Ecosystem**
- **ROS TCP Endpoint**: ROS node for bidirectional message communication with Unity
- **ROS TCP Connector**: Unity package enabling message exchange and visualization from ROS
- **URDF Importer**: Imports URDF robot definition files directly into Unity scenes
- **Minimum Unity Version**: 2020.2+ officially supported

### Alternatives Considered

1. **Unity 2023 LTS / Unity 6**:
   - **Rejected for primary use**: No explicit testing/validation documentation for Unity Robotics Hub with 2023 LTS
   - Unity 2023 LTS was renamed to Unity 6, which may cause confusion
   - Unity 6.0 LTS supported through October 2026, Unity 6.3 LTS through December 2027
   - Could be considered for future updates after community validation

2. **Unity 2020 LTS**:
   - **Rejected**: End of support approaching; students would be learning on outdated tooling
   - Lacks some robotics-specific improvements introduced in 2021-2022

3. **Unity Tech Stream (non-LTS)**:
   - **Rejected**: Unstable APIs, frequent breaking changes unsuitable for educational content

### Package Version Recommendations

Based on Unity Robotics Hub repository:
- **ROS TCP Connector**: Latest stable release (v0.7.0+)
- **URDF Importer**: Latest stable release compatible with Unity 2022.3 LTS
- **Unity Robotics Visualizations**: Latest stable release for ROS message visualization

**Note**: Specific package versions should be pinned in tutorial documentation and tested Docker/installation scripts to ensure reproducibility.

### Sources
- [Unity Robotics Hub GitHub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [Unity 2022 LTS Overview](https://unity.com/releases/2022-lts)
- [Unity Robotics Hub ROS Unity Integration Tutorial](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md)

---

## Decision 3: Installation Method

### Chosen: Docker for Gazebo, Native for Unity

### Rationale

**Gazebo: Docker Container (Recommended)**
- **Reproducibility**: Docker ensures consistent environments across student machines (Windows, macOS, Linux)
- **Performance**: Minimal performance penalty on native Linux; Docker on native Linux performs nearly as well as bare metal installations
- **Ease of Setup**: Pre-configured containers eliminate dependency hell and version conflicts
- **Educational Value**: Teaches containerization best practices used in robotics industry

**Performance Data**:
- Docker container on Linux bare metal: Minimal 1-2 FPS difference vs. native
- Native Linux with Nvidia drivers: ~62 FPS (optimal performance baseline)
- WSL2 + Docker: ~10 FPS (significant degradation due to WSL 2 lack of hardware acceleration for GUI graphics)

**Gazebo: Native Ubuntu (Alternative for Advanced Users)**
- **When to Use**: Students with native Linux machines who need maximum performance
- **Benefit**: Best possible performance for GPU-intensive simulations
- **Trade-off**: More complex setup, potential dependency conflicts

**Unity: Native Installation Only**
- **Reason**: Docker not practical for GPU-intensive 3D game engine requiring DirectX/Metal/Vulkan access
- **Platform Support**: Native installers for Windows, macOS, Linux
- **GPU Requirement**: Unity benefits significantly from GPU acceleration; containerization adds unnecessary complexity
- **Installation Simplicity**: Unity Hub provides straightforward installation management

### Alternatives Considered

1. **Native Gazebo for All Students**:
   - **Rejected**: Increases setup complexity and troubleshooting burden
   - Platform-specific issues (Windows requires WSL2, macOS has limited support)

2. **Docker for Unity**:
   - **Rejected**: Impractical for GPU-intensive 3D rendering
   - Headless Unity possible but defeats educational visualization purpose

3. **Virtual Machines**:
   - **Rejected**: Significant performance overhead for both Gazebo and Unity
   - GPU passthrough complex and unreliable for educational environments

### Implementation Strategy

**Gazebo Docker Setup (Chapter 1)**:
- Provide Dockerfile based on `osrf/ros:humble-desktop` base image
- Pre-install Gazebo Fortress and required plugins
- Configure X11 forwarding for GUI access (Linux/macOS)
- Windows: Use WSL2 with X server (VcXsrv/X410) or Windows 11 WSLg

**Unity Native Setup (Chapter 2)**:
- Step-by-step Unity Hub installation guide (Windows/macOS/Linux)
- Unity 2022.3 LTS version specification
- Unity Robotics Hub package installation via Package Manager
- Platform-specific troubleshooting (DirectX vs. Vulkan, macOS Rosetta for M1/M2)

### Sources
- [Gazebo Docker Performance Discussion](https://gazebosim.org/docs/fortress/ign_docker_env/)
- [Dockerized Development on Ubuntu - Gazebo Fortress](https://gazebosim.org/docs/fortress/ign_docker_env/)
- [Running ROS and Gazebo on Ubuntu Desktop via Docker](https://github.com/jbnunn/ROSGazeboDesktop)

---

## Decision 4: Sensor Simulation Complexity

### Chosen: Balanced fidelity prioritizing educational clarity over research-grade accuracy

### Rationale

Educational robotics requires balancing **physical accuracy** with **student comprehension**. Over-simplified models fail to prepare students for real-world challenges; over-complex models obscure fundamental concepts.

### LiDAR Simulation: GPU-Accelerated (Preferred)

**Chosen Approach**: GPU-accelerated ray-based simulation (Gazebo `gpu_ray`, Unity Raytraced Lidar)

**Rationale**:
- **Performance**: GPU ray sensors are "more efficient" than CPU-based ray sensors, running in rendering pipeline vs. physics engine
- **Accuracy**: GPU rasterization-based approaches decrease data generation times 100x compared to CPU ray-casting while maintaining accuracy
- **Educational Value**: Students learn about real-world LiDAR characteristics (occlusion, range limits, angular resolution)
- **Scalability**: Enables real-time simulation for multiple robots or complex environments

**Configuration Recommendations**:
- **Range**: 0.1m to 30m (typical indoor/outdoor LiDAR)
- **Angular Resolution**: 0.25° to 1.0° (balance between detail and performance)
- **Update Rate**: 10-20 Hz (realistic for educational scenarios)
- **Noise Model**: Simple Gaussian noise (mean=0, std dev=0.01-0.05m) for basic exercises
- **Advanced**: Introduce intensity-dependent noise and range-dependent accuracy in optional sections

**Platform-Specific**:
- **Gazebo**: Use `gpu_ray` sensor plugin (detects visual shapes, more efficient)
- **Unity**: Use Raytraced Lidar (requires Windows/Linux with Vulkan, most feature-rich and accurate)
  - Alternative: Raster Lidar for complex scenes
  - Fallback: Physics Lidar for weak GPU / strong CPU machines

### Depth Camera Simulation: Balanced Accuracy

**Chosen Approach**: GPU-rendered depth with configurable noise

**Rationale**:
- **Reality Gap**: Simulated depth cameras provide exact depth values without error; students must understand this limitation
- **Educational Focus**: Teach intrinsic/extrinsic calibration, depth range limitations, and pixel-to-point conversion
- **Performance Trade-off**: Accurate depth rendering without excessive ray-marching overhead

**Configuration Recommendations**:
- **Resolution**: 640x480 or 1280x720 (realistic RGB-D camera resolutions)
- **Depth Range**: 0.5m to 5.0m (typical indoor depth camera)
- **Noise Model**:
  - Basic: Gaussian noise proportional to depth² (σ = k × d², k ≈ 0.001)
  - Advanced: Include flying pixels, edge artifacts, IR interference patterns
- **FOV**: 60-75° (matches Intel RealSense, Kinect specifications)

**Platform-Specific**:
- **Gazebo**: Use `depth_camera` plugin with configurable noise parameters
- **Unity**: Use Camera component with depth texture rendering + custom noise shader

**Challenges to Address**:
- Depth cameras in simulation don't capture material reflectivity or IR interference
- Tutorial must explain limitations and when sim-to-real transfer may fail

### IMU Simulation: Tiered Complexity

**Chosen Approach**: Simple Gaussian noise (default) with optional Allan variance modeling (advanced)

**Rationale**:
- **Educational Progression**: Start simple (white noise) → introduce realistic noise (Allan variance) → advanced (full error model)
- **Practical Value**: Gaussian white noise is sufficient for basic Kalman filtering and sensor fusion tutorials
- **Research Readiness**: Allan variance analysis prepares students for real IMU calibration workflows

**Configuration Recommendations**:

**Basic (Chapter 2, Initial Tutorials)**:
- **Accelerometer**: Gaussian white noise, σ = 0.01 m/s²
- **Gyroscope**: Gaussian white noise, σ = 0.001 rad/s
- **Bias**: Small constant offset (e.g., ±0.05 m/s², ±0.005 rad/s)
- **Update Rate**: 100-200 Hz

**Advanced (Optional Section)**:
- **Noise Parameters from Allan Variance**:
  - Angle Random Walk (N): Estimated from stationary gyroscope data
  - Rate Random Walk (K): Characterizes long-term drift
  - Bias Instability (B): Temperature-dependent drift
- **Tools**: Reference `ori-drs/allan_variance_ros` for parameter identification
- **Educational Value**: Links physical noise characteristics to Kalman filter Q matrix

**Rationale for Tiered Approach**:
- Beginning students struggle with Allan variance; introduce after sensor fusion basics
- Allan variance analysis requires 15-24 hour stationary datasets (impractical for intro tutorials)
- Simple Gaussian models are sufficient for 80% of educational use cases
- Advanced students preparing for real hardware can learn proper IMU characterization

### Multi-Sensor Synchronization

**Chosen Approach**: Explicit timestamping and sync validation

**Rationale**:
- **Real-World Skill**: Sensor fusion requires timestamp alignment; students must learn this early
- **Simulation Advantage**: Perfect synchronization possible; students validate algorithms before hardware
- **Configuration**: All sensors publish with ROS 2 timestamps; tutorials include sync verification exercises

### Sources
- [Gazebo GPU Ray vs Ray Sensor](http://answers.gazebosim.org/question/19529/what-is-the-different-between-sdfs-gpu_ray-sensor-and-ray-sensor/)
- [Unity LiDAR Sensor Configuration](https://docs.unity3d.com/Simulation/manual/author/set-up-sensors/configure-a-lidar.html)
- [Robotec GPU Lidar (RGL)](https://www.robotec.ai/automated-future-with-rgl-robotec-gpu-lidar)
- [IMU Noise Model - ETH Kalibr Wiki](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
- [Allan Variance for IMU Noise Analysis](https://daischsensor.com/allan-variance-for-imu-noise-analysis/)
- [IMU Noise Analysis Using Allan Variance - MATLAB](https://www.mathworks.com/help/nav/ug/inertial-sensor-noise-analysis-using-allan-variance.html)
- [Comparative Analysis of ROS-Unity3D and ROS-Gazebo for Mobile Ground Robot Simulation](https://link.springer.com/article/10.1007/s10846-022-01766-2)

---

## Decision 5: Content Structure

### Chosen: 3-chapter structure with inline code examples and progressive complexity

### Rationale

**Chapter Length Targets**: 5-8 pages each, total 15-24 pages (per constitution)

**Progressive Learning**: Each chapter builds on previous concepts, mirrors professional robotics workflow

**Practical Focus**: Code-first approach with immediate visual feedback (see robot move, see sensor data)

### Chapter Organization

#### **Chapter 1: Physics Simulation Fundamentals with Gazebo** (6-7 pages)

**Learning Objectives**:
- Understand physics engine parameters (gravity, friction, damping)
- Create Gazebo worlds and launch humanoid robots
- Debug common physics issues (instability, jitter, explosions)

**Content Structure**:
1. **Introduction** (0.5 page): Why physics simulation matters for robotics
2. **Gazebo Setup** (1 page): Docker installation, launching Gazebo GUI
3. **Creating Your First World** (1.5 pages):
   - Empty world → add ground plane → configure gravity
   - Inline SDF code with annotations
   - Screenshot: Gazebo GUI with world loaded
4. **Importing Humanoid Models** (1.5 pages):
   - Load URDF from Module 1
   - Configure collision geometry and inertia
   - Inline URDF snippet showing inertial properties
   - Screenshot: Humanoid standing (or falling if misconfigured)
5. **Physics Troubleshooting** (1.5 pages):
   - Common issues: jitter, explosions, slow simulation
   - Parameter tuning guide (solver iterations, contact forces)
   - Debugging exercises with expected outputs
6. **Exercises** (0.5 page):
   - Exercise 1: Modify gravity and observe changes
   - Exercise 2: Create environment with obstacles, test collisions

**Code Example Placement**: Inline with explanatory text, max 15-20 lines per snippet

#### **Chapter 2: High-Fidelity Visual Environments with Unity** (6-7 pages)

**Learning Objectives**:
- Set up Unity scenes with realistic lighting and textures
- Import URDF models using Unity Robotics Hub packages
- Create interactive environments for perception testing

**Content Structure**:
1. **Introduction** (0.5 page): Unity's role in robotics (visualization, perception, HRI)
2. **Unity Setup** (1 page): Unity Hub installation, creating new project, installing packages
3. **Building Your First Scene** (1.5 pages):
   - Create floor, walls, obstacles with ProBuilder
   - Lighting setup (directional light, ambient occlusion)
   - Screenshot: Textured environment with shadows
4. **Importing Humanoid Robots** (1.5 pages):
   - URDF Importer workflow
   - Configuring articulation bodies and joints
   - Inline C# snippet (if needed for joint control)
   - Screenshot: Humanoid in Unity scene with visible joints
5. **Interactive Elements** (1 page):
   - Adding physics to movable objects
   - Basic robot-object interaction demo
   - Screenshot: Robot pushing/grasping object
6. **Troubleshooting & Optimization** (1 page):
   - Performance tips (LOD, occlusion culling, shadow resolution)
   - Platform-specific issues (DirectX vs. Vulkan, macOS Metal)
7. **Exercises** (0.5 page):
   - Exercise 1: Create apartment/office environment
   - Exercise 2: Import custom URDF and verify articulation

**Code Example Placement**: Inline with step-by-step instructions, use Unity screenshots for GUI workflows

#### **Chapter 3: Sensor Simulation and Data Collection** (5-6 pages)

**Learning Objectives**:
- Configure LiDAR, depth cameras, and IMUs in Gazebo and Unity
- Collect and visualize synthetic sensor data
- Understand sensor noise models and limitations

**Content Structure**:
1. **Introduction** (0.5 page): Importance of sensor simulation for algorithm development
2. **LiDAR Simulation** (1.5 pages):
   - **Gazebo**: gpu_ray plugin configuration
   - **Unity**: Raytraced Lidar setup
   - Inline sensor configuration (SDF/Unity Inspector)
   - Visualization: Point cloud in RViz2 or Unity
   - Screenshot: LiDAR point cloud overlaid on environment
3. **Depth Camera Simulation** (1.5 pages):
   - **Gazebo**: depth_camera plugin with noise parameters
   - **Unity**: Camera depth texture + noise shader
   - Inline configuration and noise model parameters
   - Visualization: Depth image with color mapping
   - Screenshot: RGB and depth images side-by-side
4. **IMU Simulation** (1 page):
   - **Gazebo**: IMU plugin with accelerometer/gyroscope noise
   - **Unity**: ArticulationBody + custom IMU script
   - Inline noise configuration (Gaussian parameters)
   - Plotting: IMU data over time (matplotlib/Unity visualizations)
5. **Data Collection & Export** (1 page):
   - ROS 2 topic subscription and rosbag recording
   - Export formats: PCD (point clouds), PNG/EXR (depth), CSV (IMU)
   - Code snippet: Python script to record sensor data
6. **Exercises** (0.5 page):
   - Exercise 1: Configure LiDAR, collect 100 scans, verify accuracy
   - Exercise 2: Add noise to depth camera, observe effects on perception

**Code Example Placement**: Inline with sensor configuration, separate Python scripts for data processing

### Code Example Strategy

**Inline vs. Separate Files**:
- **Inline** (preferred for 80% of examples):
  - SDF/URDF snippets showing sensor/physics configuration (< 30 lines)
  - Python scripts for data collection/visualization (< 40 lines)
  - Unity C# snippets for basic sensor setup (< 25 lines)
  - Advantages: Easier to follow, explain line-by-line, copy-paste for students

- **Separate Files** (for complete systems):
  - Full Gazebo world files (`.world`, `.sdf`)
  - Complete Unity scenes (reference GitHub repository)
  - ROS 2 launch files for multi-sensor setups
  - Advantages: Reusable, version-controlled, reduces page clutter

**Code Repository Structure**:
```
docs/module-02-digital-twin/
├── chapter-01-gazebo-physics/
│   ├── examples/
│   │   ├── simple_world.sdf
│   │   ├── humanoid_physics.urdf
│   │   └── launch_gazebo.sh
│   └── screenshots/
├── chapter-02-unity-environments/
│   ├── examples/
│   │   ├── unity_project_setup.md
│   │   └── robot_import_script.cs
│   └── screenshots/
└── chapter-03-sensors/
    ├── examples/
    │   ├── lidar_config.sdf
    │   ├── collect_sensor_data.py
    │   └── plot_imu_data.py
    └── screenshots/
```

### Diagram & Screenshot Requirements

**Screenshots (Mandatory)**:
- Gazebo GUI with world loaded (Chapter 1)
- Humanoid robot in Gazebo, standing and post-collision (Chapter 1)
- Unity scene with textured environment (Chapter 2)
- URDF import workflow in Unity (Chapter 2)
- LiDAR point cloud visualization (Chapter 3)
- Depth image RGB/depth comparison (Chapter 3)
- IMU data plot over time (Chapter 3)

**Diagrams (Optional but Recommended)**:
- Physics engine workflow (collision detection → constraint solving → integration)
- Sensor coordinate frames (camera frame, LiDAR frame, body frame)
- ROS 2 topic architecture for sensor data flow

**Format**: PNG (screenshots), SVG (diagrams for scalability)

### Validation & Expected Outputs

**Every Exercise Must Include**:
1. **Clear Instructions**: Step-by-step with exact commands/GUI actions
2. **Expected Output**: Screenshot, console log, or data sample
3. **Success Criteria**: Measurable (e.g., "Point cloud should contain 360 points per scan")
4. **Troubleshooting**: Common errors and solutions

**Example Exercise Format**:
```markdown
### Exercise 1: Configure LiDAR and Verify Accuracy

**Objective**: Add a LiDAR sensor to your humanoid robot and collect point cloud data.

**Steps**:
1. Open `simple_humanoid.urdf` and add the following sensor block:
   [inline URDF snippet with <sensor> tag]
2. Launch Gazebo: `ros2 launch my_robot gazebo.launch.py`
3. Verify topic: `ros2 topic echo /lidar/points`

**Expected Output**:
- RViz2 should display a point cloud matching the environment geometry
- Point cloud should contain ~360 points per scan (1° angular resolution)
- Screenshot: [show expected RViz2 visualization]

**Success Criteria**:
- [ ] Gazebo launches without errors
- [ ] Point cloud topic publishes at 10 Hz
- [ ] Measured distance to wall matches ground truth (±5cm)

**Troubleshooting**:
- **Issue**: No point cloud visible in RViz2
  - **Solution**: Check topic name, verify frame_id matches robot TF tree
- **Issue**: Point cloud appears noisy or random
  - **Solution**: Verify <range> and <scan> parameters in URDF
```

### Sources
- [Comparative Analysis of ROS-Unity3D and ROS-Gazebo](https://link.springer.com/article/10.1007/s10846-022-01766-2)
- [Learning ROS for Robotics Programming](https://dl.icdst.org/pdfs/files4/423f6ed13ca073890c5ed2f74ecedf95.pdf)
- [Learn Robotics Programming - Packt](https://www.packtpub.com/en-us/product/learn-robotics-programming-9781839218804)

---

## Summary of Decisions

| Decision Area | Chosen Approach | Key Rationale |
|--------------|----------------|---------------|
| **Gazebo Version** | Fortress (primary), Garden (optional) | Stability, official ROS 2 Humble support, binary packages |
| **Unity Version** | Unity 2022.3 LTS | Verified Robotics Hub compatibility, LTS support, stability |
| **Gazebo Installation** | Docker (recommended), Native (alternative) | Reproducibility, minimal performance penalty on Linux |
| **Unity Installation** | Native only | GPU requirements, impractical to containerize |
| **LiDAR Simulation** | GPU-accelerated ray-based | Performance (100x faster), accuracy, real-time capable |
| **Depth Camera** | GPU-rendered with configurable noise | Balance accuracy and performance, teach calibration |
| **IMU Noise Model** | Gaussian (basic), Allan variance (advanced) | Educational progression, simple→realistic→research-grade |
| **Chapter Structure** | 3 chapters, 5-8 pages each, inline code | Progressive complexity, practical focus, immediate feedback |
| **Code Examples** | Inline (< 30 lines), separate files (full systems) | Readability, copy-paste friendly, version-controlled reference |

---

## Next Steps

1. **Planning Phase** (`/sp.plan`):
   - Create detailed chapter outlines with section-by-section breakdown
   - Design exercise templates with validation criteria
   - Define Docker configurations for Gazebo Fortress
   - Specify Unity project setup scripts

2. **Task Generation** (`/sp.tasks`):
   - Break down chapter writing into testable tasks
   - Create sensor configuration templates (SDF, URDF, Unity prefabs)
   - Develop example code repository structure
   - Plan screenshot capture workflow

3. **Implementation**:
   - Write Chapter 1 (Gazebo Physics) with inline examples
   - Write Chapter 2 (Unity Environments) with Unity Hub screenshots
   - Write Chapter 3 (Sensors) with data collection scripts
   - Test all examples in Docker and native environments

4. **Validation**:
   - Peer review by robotics educator with no prior Gazebo/Unity experience
   - Test exercises on clean installations (Docker, fresh Unity project)
   - Verify page count (15-24 pages total)
   - Build in Docusaurus and check for errors

---

## References

### Official Documentation
- [Gazebo Official Documentation](https://gazebosim.org/docs)
- [Unity Robotics Hub GitHub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [Unity URDF Importer](https://github.com/Unity-Technologies/URDF-Importer)
- [ROS 2 Gazebo Integration](https://github.com/ros-simulation/gazebo_ros_pkgs)

### Research & Comparisons
- [Comparative Analysis of ROS-Unity3D and ROS-Gazebo](https://link.springer.com/article/10.1007/s10846-022-01766-2)
- [Simulation of Mobile Robots with Unity and ROS](https://www.diva-portal.org/smash/get/diva2:1334348/FULLTEXT01.pdf)
- [Comparative Study of Gazebo and Unity 3D](https://www.sciencedirect.com/science/article/abs/pii/S1569190X24000091)

### Technical Resources
- [IMU Noise Model - ETH Kalibr](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
- [Allan Variance for IMU Analysis](https://daischsensor.com/allan-variance-for-imu-noise-analysis/)
- [Robotec GPU Lidar](https://www.robotec.ai/automated-future-with-rgl-robotec-gpu-lidar)
- [Unity LiDAR Configuration](https://docs.unity3d.com/Simulation/manual/author/set-up-sensors/configure-a-lidar.html)

### Educational Best Practices
- [Learning ROS for Robotics Programming](https://dl.icdst.org/pdfs/files4/423f6ed13ca073890c5ed2f74ecedf95.pdf)
- [Learn Robotics Programming - Packt](https://www.packtpub.com/en-us/product/learn-robotics-programming-9781839218804)
