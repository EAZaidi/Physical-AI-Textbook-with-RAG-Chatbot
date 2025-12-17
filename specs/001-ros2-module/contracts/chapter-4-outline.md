# Chapter 4 Outline: URDF Basics for Humanoid Robots

**File**: `docs/module-1-ros2/04-urdf-basics.mdx`
**Est. Length**: 5-6 pages
**Learning Time**: ~2 hours
**Priority**: P2 (User Story 3 - Interpreting and Modifying URDF Files)

---

## Learning Objectives

Students will:
1. Understand URDF structure (links, joints, kinematic tree)
2. Read and interpret URDF files for humanoid robots
3. Modify URDF files (add links, adjust joint limits)
4. Visualize URDF models in RViz2
5. Add sensors (cameras, LiDAR) to robot models

**Success Metric** (from spec):
- SC-004: Students fix URDF syntax errors within 15 minutes

---

## Sections

### 1. Introduction to URDF (0.5 pages)
- What is URDF? (Unified Robot Description Format)
- Why robots need URDF (simulation, visualization, planning)
- XML-based format
- URDF vs SDF (brief comparison)

### 2. URDF Structure Overview (1 page)
- **Links**: Rigid bodies (torso, arm, leg)
- **Joints**: Connections between links (revolute, prismatic, fixed)
- **Kinematic Tree**: Parent-child relationships (no cycles)
- **Example structure**:
  ```xml
  <robot name="simple_humanoid">
    <link name="torso">...</link>
    <link name="head">...</link>
    <joint name="neck" type="revolute">
      <parent link="torso"/>
      <child link="head"/>
    </joint>
  </robot>
  ```

### 3. Defining Links (1.5 pages)
- Link anatomy: `<visual>`, `<collision>`, `<inertial>` (optional for basics)
- Geometry types: `<box>`, `<cylinder>`, `<sphere>`, `<mesh>`
- Origin: Position and orientation relative to link frame
- **Example**: Torso link with box geometry
- **Code**: `simple_humanoid.urdf` (link definitions)

### 4. Defining Joints (1.5 pages)
- Joint types: revolute, prismatic, fixed, continuous
- Parent-child relationships
- Axis of rotation/translation
- Joint limits: lower, upper, effort, velocity
- **Example**: Shoulder joint (torso to upper arm)
- **Code**: `simple_humanoid.urdf` (joint definitions)

### 5. Simplified Humanoid URDF (1 page)
- Complete example: 13 links, 12 joints
- Links: head, torso, 2x (upper arm, lower arm, hand, upper leg, lower leg, foot)
- Joints: neck, 2x (shoulder, elbow, wrist, hip, knee, ankle)
- **Code**: Full `simple_humanoid.urdf` (copy-pasteable)
- **Diagram**: Humanoid skeleton with labeled links/joints

### 6. Visualizing URDF in RViz2 (1 page)
- Launch RViz2: `rviz2`
- Load URDF: `ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat simple_humanoid.urdf)"`
- Add RobotModel display
- Manipulate joints with joint_state_publisher_gui
- **Expected output**: 3D humanoid model in RViz2

### 7. Exercise: Adding a Camera Sensor (1 page)
- Task: Add camera link to head
- Define camera link with small box geometry
- Create fixed joint (head to camera)
- **Solution**: `humanoid_with_camera.urdf`
- **Expected output**: Camera visible in RViz2 on head

### 8. Troubleshooting URDF Errors (0.5 pages)
- Common errors: Missing parent link, cycle in kinematic tree, invalid joint type
- Validation: `check_urdf simple_humanoid.urdf`
- Debugging tips

### 9. Summary and Resources (0.5 pages)
- URDF enables simulation and visualization
- Links = rigid bodies, Joints = connections
- Kinematic tree structure (one root, no cycles)
- Next steps: Module 2 (simulation with Gazebo/Unity)

---

## Code Examples

1. `simple_humanoid.urdf` - 13-link humanoid skeleton
2. `humanoid_with_camera.urdf` - Exercise solution (adds camera to head)

---

## Documentation Links

- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [URDF XML Specification](http://wiki.ros.org/urdf/XML)
- [robot_state_publisher](https://docs.ros.org/en/humble/p/robot_state_publisher/)
- [RViz2 User Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)

---

## Validation Checklist

- [ ] URDF examples valid (pass `check_urdf`)
- [ ] URDF visualizes correctly in RViz2
- [ ] Exercise solution provided (`humanoid_with_camera.urdf`)
- [ ] All geometry simple (boxes/cylinders, no complex meshes)
- [ ] Humanoid structure aligns with capstone project
