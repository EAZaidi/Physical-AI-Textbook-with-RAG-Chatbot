# Chapter Contract: Chapter 1 - Physics Simulation Fundamentals with Gazebo

**Chapter**: 01-gazebo-physics.mdx
**Target Length**: 6-7 pages
**Priority**: P1 (Foundation for all simulation work)
**Prerequisites**: Module 1 (ROS 2, URDF basics)

## Learning Objectives

By the end of this chapter, students will be able to:

1. **Create** Gazebo worlds from scratch with custom physics parameters (gravity, friction, time step)
2. **Import** URDF humanoid models from Module 1 into Gazebo simulations
3. **Configure** collision geometry and inertial properties for stable robot behavior
4. **Debug** common physics issues (jitter, instability, penetration) using solver tuning
5. **Validate** that simulated physics matches expectations through observation and measurement

## Chapter Outline

### Section 1: Introduction (0.5 pages)

**Purpose**: Motivate physics simulation and preview what students will build

**Content**:
- Why physics simulation matters for robotics (safe testing, rapid iteration, algorithm validation)
- Gazebo's role in the ROS 2 ecosystem (de facto standard simulator)
- Preview: By the end, students will have a humanoid robot standing/walking in a custom world
- Brief comparison: Gazebo (physics-first) vs. Unity (visuals-first)

**Deliverables**:
- 1 paragraph: Why simulation before hardware
- 1 paragraph: Gazebo overview and ecosystem position
- 1 screenshot: Final result preview (humanoid robot in Gazebo world)

---

### Section 2: Gazebo Setup with Docker (1 page)

**Purpose**: Get students from zero to running Gazebo GUI in <5 minutes

**Content**:
- Docker installation verification (`docker --version`)
- Pull pre-built Gazebo Fortress + ROS 2 Humble image
- Run container with GUI forwarding (X11 for Linux/macOS, VcXsrv/X410 for Windows)
- Launch Gazebo and verify empty world loads
- Troubleshooting: Common issues (X server connection, permission denied, display not set)

**Code Examples**:
```bash
# Pull Docker image (inline command)
docker pull osrf/ros:humble-desktop

# Run container with GUI (inline command with explanation)
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:humble-desktop \
  gazebo
```

**Deliverables**:
- 3-4 inline bash commands with annotations
- 1 screenshot: Empty Gazebo world running in Docker
- Troubleshooting table (3-5 common errors + solutions)

**Validation**:
- [ ] Student can launch Gazebo GUI from Docker container
- [ ] Empty world loads without errors in terminal
- [ ] Student can rotate camera view and see grid ground plane

---

### Section 3: Creating Your First World (1.5 pages)

**Purpose**: Teach SDF syntax and physics configuration through hands-on world building

**Content**:
- SDF (Simulation Description Format) structure overview
- Create `simple_world.sdf` with:
  - Physics engine selection (ODE recommended for beginners)
  - Gravity configuration (Earth gravity as default, demo with reduced gravity)
  - Ground plane model (infinite plane vs. box geometry)
  - Basic lighting (single directional sun light)
- Save world file and launch with `gazebo simple_world.sdf`
- Experiment: Drop a sphere and observe gravity effect

**Code Examples**:
```xml
<!-- Inline SDF: Complete simple_world.sdf (20-25 lines) -->
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="simple_world">
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

**Deliverables**:
- Complete `simple_world.sdf` file (inline, ~25 lines with comments)
- 1 screenshot: Gazebo GUI with simple_world loaded
- 1 screenshot: Sphere falling under gravity (before/after composite)

**Validation**:
- [ ] World file loads without XML parsing errors
- [ ] Ground plane visible in Gazebo viewport
- [ ] Dropped sphere falls at expected rate (hits ground in ~1.4s from 10m height)

---

### Section 4: Importing Humanoid Models (1.5 pages)

**Purpose**: Bridge Module 1 URDF knowledge to Gazebo simulation

**Content**:
- Review: URDF structure from Module 1 (links, joints, inertia)
- **Critical**: Difference between visual (rendering) and collision (physics) geometry
- Add `<gazebo>` tags to URDF for simulation-specific properties
- Configure inertial properties properly:
  - Mass (kg) for each link
  - Inertia tensor (3x3 matrix in kg·m²)
  - Center of mass offset
- Launch humanoid robot in Gazebo world (`ros2 launch` with robot_state_publisher)
- Observe: Robot should stand (if balanced) or fall (if inertia incorrect)

**Code Examples**:
```xml
<!-- Inline URDF snippet: Torso link with proper inertia (15-20 lines) -->
<link name="torso">
  <inertial>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <mass value="5.0"/>
    <inertia ixx="0.1" iyy="0.1" izz="0.05"
             ixy="0" ixz="0" iyz="0"/>
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
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
</link>

<!-- Gazebo-specific material properties -->
<gazebo reference="torso">
  <material>Gazebo/Blue</material>
</gazebo>
```

```python
# Inline Python: ROS 2 launch file snippet (10-12 lines)
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot',
                      '-file', '/path/to/robot.urdf',
                      '-x', '0', '-y', '0', '-z', '1.0']
        )
    ])
```

**Deliverables**:
- URDF snippet showing proper inertial configuration (inline, 20 lines)
- Launch file snippet for spawning robot (inline, 12 lines)
- 2 screenshots:
  - Humanoid standing (well-configured inertia)
  - Humanoid falling/collapsed (misconfigured inertia for comparison)

**Validation**:
- [ ] Robot spawns in Gazebo at specified position
- [ ] No error messages about missing inertia or mass
- [ ] Robot links move independently when joint commands sent (or falls if uncontrolled)

---

### Section 5: Physics Troubleshooting (1.5 pages)

**Purpose**: Equip students to debug 80% of common simulation issues

**Content**:
- **Issue 1: Jitter/Vibration**
  - Cause: Solver iterations too low, interpenetrating geometry
  - Solution: Increase `<ode><solver><iters>` to 50-100, reduce contact surface layer
  - Demo: Video or GIF showing jitter → fix → stable

- **Issue 2: Robot "Explodes" (flies apart)**
  - Cause: Time step too large, constraint violations
  - Solution: Reduce `max_step_size` from 0.001 to 0.0005, increase solver iterations

- **Issue 3: Slow Simulation (real-time factor < 1.0)**
  - Cause: Complex collision geometry, too many contacts
  - Solution: Simplify collision meshes (use primitives), reduce physics update rate

- **Issue 4: Robot Sinks into Ground**
  - Cause: Incorrect inertia (too light), soft contacts
  - Solution: Verify mass matches real robot, increase contact stiffness

**Code Examples**:
```xml
<!-- Inline SDF: Tuned physics parameters for stable humanoid (10-12 lines) -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

**Deliverables**:
- Troubleshooting table: Issue | Symptoms | Cause | Solution (4 rows minimum)
- Tuned physics configuration snippet (inline, 12 lines)
- 1 before/after screenshot or GIF: Jittering robot → stable robot

**Validation**:
- [ ] Student can identify jitter vs. explosion vs. slow simulation
- [ ] Student can modify solver parameters and observe effect
- [ ] Humanoid robot stands stable for 10 seconds without jitter

---

### Section 6: Exercises (0.5 pages)

**Purpose**: Hands-on practice to solidify concepts

#### Exercise 1: Modify Gravity and Observe

**Task**: Create a copy of `simple_world.sdf`, set gravity to `-4.0` (Mars-like), drop a sphere from 10m, measure time to ground.

**Expected Output**:
- Sphere should take ~√(2×10/4.0) ≈ 2.24 seconds to hit ground
- Screenshot showing sphere mid-fall with Gazebo sim time visible

**Success Criteria**:
- [ ] World file loads without errors
- [ ] Sphere falls slower than Earth gravity
- [ ] Time to impact within ±10% of calculated value

#### Exercise 2: Create Environment with Obstacles

**Task**: Add 3 box obstacles to `simple_world.sdf`, spawn humanoid robot, manually move robot to collide with obstacles (using Gazebo GUI or ROS 2 commands).

**Expected Output**:
- Collision detection triggers (robot stops when contacting obstacles)
- No penetration or pass-through behavior
- Screenshot showing robot in contact with obstacle

**Success Criteria**:
- [ ] Obstacles appear in world with collision geometry
- [ ] Robot cannot pass through obstacles
- [ ] Contact forces prevent robot from sinking into obstacles

---

## Code Examples Summary

**Total Code Examples**: 6 inline snippets + 2 separate files referenced

1. **Docker run command** (bash, 5 lines) - Section 2
2. **simple_world.sdf** (SDF, 25 lines) - Section 3
3. **Torso link URDF** (XML, 20 lines) - Section 4
4. **Spawn robot launch file** (Python, 12 lines) - Section 4
5. **Tuned physics parameters** (SDF, 12 lines) - Section 5
6. **Exercise world file** (SDF, reference to student-created file) - Section 6

**Separate Files** (in `docs/docs/module-2-digital-twin/assets/code-examples/`):
- `simple_world.sdf` - Complete working world file
- `humanoid_physics.urdf` - Properly configured humanoid with inertia
- `launch_gazebo.sh` - One-liner to launch world + robot

---

## Screenshots & Diagrams

**Required Screenshots** (7 total):
1. Empty Gazebo world running in Docker (Section 2)
2. Simple world with ground plane loaded (Section 3)
3. Sphere falling under gravity (before/after composite) (Section 3)
4. Humanoid standing with correct inertia (Section 4)
5. Humanoid collapsed with incorrect inertia (Section 4)
6. Before/after: Jittering robot → stable robot (Section 5)
7. Robot colliding with obstacle (Exercise 2)

**Optional Diagrams**:
- Physics engine workflow: Collision detection → Constraint solving → Integration (SVG)
- Coordinate frames: World frame, link frames, joint axes (SVG)

---

## Validation Checklist

**Content Quality**:
- [ ] All SDF/URDF syntax verified against Gazebo Fortress documentation
- [ ] Docker commands tested on Linux, Windows WSL2, macOS
- [ ] Physics parameters produce stable simulation (10 minutes continuous run)
- [ ] Troubleshooting table addresses most common student issues (validated via pilot testing)

**Educational Clarity**:
- [ ] Learning objectives stated at beginning, revisited in summary
- [ ] Concepts explained before technical implementation (e.g., collision vs. visual geometry explained before URDF code)
- [ ] Common pitfalls highlighted in each section
- [ ] Exercises include clear success criteria and expected outputs

**Constitution Compliance**:
- [ ] All claims reference official Gazebo documentation (gazebosim.org)
- [ ] Code examples reproducible in documented Docker environment
- [ ] Page count: 6-7 pages (within target)
- [ ] No hardcoded file paths (use `package://` URIs or relative paths)

---

## Dependencies

**From Module 1**:
- URDF syntax knowledge (links, joints, visual/collision)
- Basic understanding of coordinate frames and transforms
- Sample `simple_humanoid.urdf` file (reused from Module 1)

**External Documentation**:
- Gazebo Fortress Tutorials: https://gazebosim.org/docs/fortress/tutorials
- SDF Specification: http://sdformat.org/spec
- ODE Solver Parameters: https://gazebosim.org/api/gazebo/6/physics_params.html

**Tools Required**:
- Docker (version 20.10+)
- X server (Linux: native, macOS: XQuartz, Windows: VcXsrv or WSLg)
- Text editor (VS Code, vim, nano)

---

## Follow-Up in Later Chapters

**Chapter 2 (Unity Scenes)**: Will compare Gazebo physics-first approach to Unity's visuals-first workflow

**Chapter 3 (Sensors)**: Will add LiDAR and IMU sensors to the humanoid robot created here

**Module 3 (Perception)**: Will use sensor data from this simulated robot for perception algorithm testing

---

**Contract Status**: ✅ READY FOR IMPLEMENTATION
**Estimated Writing Time**: 4-6 hours (including screenshot capture and testing)
**Last Updated**: 2025-12-07
