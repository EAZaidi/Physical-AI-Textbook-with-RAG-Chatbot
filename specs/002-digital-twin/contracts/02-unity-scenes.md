# Chapter Contract: Chapter 2 - High-Fidelity Visual Environments with Unity

**Chapter**: 02-unity-scenes.mdx
**Target Length**: 6-7 pages
**Priority**: P2 (Visual realism for perception and HRI)
**Prerequisites**: Module 1 (ROS 2, URDF), Chapter 1 (physics simulation basics)

## Learning Objectives

By the end of this chapter, students will be able to:

1. **Install** Unity 2022.3 LTS and Unity Robotics Hub packages on their platform (Windows/macOS/Linux)
2. **Create** interactive 3D scenes with realistic lighting, textures, and physics-enabled objects
3. **Import** URDF humanoid models using Unity URDF Importer and configure Articulation Bodies
4. **Build** apartment/office environments for testing humanoid navigation and manipulation
5. **Optimize** Unity scenes for real-time performance (60 FPS target on recommended hardware)

## Chapter Outline

### Section 1: Introduction (0.5 pages)

**Purpose**: Motivate Unity for robotics and preview student deliverables

**Content**:
- Unity's role in robotics: High-fidelity visualization, perception testing, human-robot interaction
- When to use Unity vs. Gazebo:
  - Gazebo: Physics-accurate, faster simulation, headless mode
  - Unity: Photorealistic rendering, VR/AR support, game engine features
- Preview: Students will create a textured indoor environment with interactive objects
- Unity + ROS 2 integration architecture (ROS TCP Connector overview)

**Deliverables**:
- 1 paragraph: Why Unity for robotics (not just game development)
- 1 paragraph: Unity vs. Gazebo decision matrix
- 1 screenshot: Final result preview (humanoid robot in furnished apartment scene)

---

### Section 2: Unity Setup and Project Creation (1 page)

**Purpose**: Get students from zero to empty Unity project in <10 minutes

**Content**:
- **Unity Hub Installation**:
  - Download Unity Hub from https://unity.com/download
  - Platform-specific notes (Windows: DirectX, macOS: Metal/Rosetta for M1/M2, Linux: Vulkan)
- **Unity 2022.3 LTS Installation**:
  - Install via Unity Hub
  - Required modules: Windows Build Support, Linux Build Support (if cross-platform)
- **Create New Project**:
  - Template: 3D (URP - Universal Render Pipeline)
  - Project name: "HumanoidSimulation"
  - Save location: Desktop or Documents folder
- **Verify Installation**:
  - Open project, observe empty scene with camera and directional light
  - Enter Play mode, verify no errors in Console

**Code Examples**:
```bash
# Linux: Install Unity Hub via snap (inline command)
sudo snap install unity-hub --classic

# Verify Unity version (inline command for all platforms)
# Open Unity Hub → Installs → Check 2022.3.XX LTS installed
```

**Deliverables**:
- Step-by-step installation instructions (numbered list, 5-6 steps)
- 1 screenshot: Unity Hub with 2022.3 LTS installed
- 1 screenshot: Empty Unity project (SampleScene with default camera)
- Platform-specific troubleshooting (Windows: DirectX vs. Vulkan, macOS: Rosetta 2 for Intel packages)

**Validation**:
- [ ] Unity 2022.3 LTS launches without errors
- [ ] Empty project created and opens in Unity Editor
- [ ] Play mode runs without console errors

---

### Section 3: Building Your First Scene (1.5 pages)

**Purpose**: Teach Unity fundamentals through hands-on environment creation

**Content**:
- **Scene Hierarchy**: GameObjects, parent-child relationships, transforms
- **Create Floor and Walls** using ProBuilder:
  - Floor: 10m × 10m plane with box collider
  - Walls: 4 vertical planes forming a room perimeter
  - Material: Apply wood texture to floor, white paint to walls
- **Lighting Setup**:
  - Directional Light: Sun simulation (Intensity: 1.0, Color: Warm white)
  - Ambient Occlusion: Enable for realistic shadows in corners
  - Skybox: Default or custom sky material
- **Add Interactive Objects**:
  - Create cube (table), sphere (ball), cylinder (furniture leg)
  - Add Rigidbody component for physics simulation
  - Configure mass and drag for realistic movement
- **Camera Setup**:
  - Main Camera: Free-look perspective (WASD movement, mouse look)
  - Alternative: Cinemachine virtual camera for cinematic views

**Code Examples**:
```csharp
// Inline C#: Simple camera controller script (20-25 lines)
// Save as Assets/Scripts/CameraController.cs
using UnityEngine;

public class CameraController : MonoBehaviour
{
    public float moveSpeed = 5.0f;
    public float lookSpeed = 2.0f;

    void Update()
    {
        // WASD movement
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");
        transform.Translate(new Vector3(horizontal, 0, vertical) * moveSpeed * Time.deltaTime);

        // Mouse look (right-click drag)
        if (Input.GetMouseButton(1))
        {
            float mouseX = Input.GetAxis("Mouse X") * lookSpeed;
            float mouseY = Input.GetAxis("Mouse Y") * lookSpeed;
            transform.Rotate(-mouseY, mouseX, 0);
        }
    }
}
```

**Deliverables**:
- ProBuilder workflow steps (numbered list, 4-5 steps)
- Camera controller script (inline C#, 25 lines with comments)
- 2 screenshots:
  - Scene view: Untextured room with wireframe visible
  - Game view: Textured room with lighting and shadows

**Validation**:
- [ ] Room geometry visible in Scene and Game views
- [ ] Floor has collision (objects don't fall through)
- [ ] Directional light casts shadows
- [ ] Camera controller allows free navigation in Play mode

---

### Section 4: Importing Humanoid Robots (1.5 pages)

**Purpose**: Bridge URDF knowledge to Unity Articulation Bodies

**Content**:
- **Install Unity Robotics Hub Packages**:
  - Open Package Manager (Window → Package Manager)
  - Add package from git URL: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`
  - Verify installation: "URDF Importer" appears in Assets → Import Robot from URDF menu
- **Import URDF Workflow**:
  - Assets → Import Robot from URDF → Select `simple_humanoid.urdf` from Module 1
  - Import settings:
    - Axis conversion: Y-up (Unity) from Z-up (ROS/URDF)
    - Mesh scaling: Auto-detect or manual scale factor
    - Articulation Body: Select "Create Articulation Body" (Unity's joint system)
  - Wait for import (mesh processing, material generation)
- **Configure Articulation Bodies**:
  - Review hierarchy: Each URDF link becomes GameObject with ArticulationBody component
  - Joint parameters: Position limits, velocity limits, damping, friction
  - Immovable base: Set root ArticulationBody to "Immovable" (if humanoid should stand on ground)
- **Test Robot**:
  - Enter Play mode
  - Observe: Robot should stand if balanced (like Gazebo) or fall if top-heavy
  - Use Unity Inspector to manually adjust joint angles (for posing)

**Code Examples**:
```csharp
// Inline C#: Joint controller script for simple arm movement (15-18 lines)
// Save as Assets/Scripts/SimpleJointController.cs
using UnityEngine;

public class SimpleJointController : MonoBehaviour
{
    public ArticulationBody joint;
    public float targetPosition = 0.5f; // radians

    void FixedUpdate()
    {
        if (joint != null && joint.jointType == ArticulationJointType.RevoluteJoint)
        {
            var drive = joint.xDrive;
            drive.target = targetPosition * Mathf.Rad2Deg; // Unity uses degrees
            joint.xDrive = drive;
        }
    }
}
```

**Deliverables**:
- Package Manager installation steps (numbered list with screenshots, 3-4 steps)
- URDF import workflow (numbered list, 5-6 steps)
- Joint controller script (inline C#, 18 lines)
- 2 screenshots:
  - Unity Inspector showing ArticulationBody component on robot link
  - Humanoid robot standing in scene (or posed in specific configuration)

**Validation**:
- [ ] URDF Importer package installed without errors
- [ ] Humanoid URDF imports successfully (all links visible)
- [ ] Articulation Bodies created for all joints
- [ ] Robot appears in scene with correct scale and orientation

---

### Section 5: Interactive Elements (1 page)

**Purpose**: Make scene interactive for manipulation and navigation testing

**Content**:
- **Adding Physics to Objects**:
  - Select object (cube, sphere), add Rigidbody component
  - Configure mass (e.g., 1.0 kg for small objects, 5.0 kg for furniture)
  - Add Box Collider or Sphere Collider for collision detection
- **Robot-Object Interaction Demo**:
  - Create simple push scenario: Robot hand pushes a box across the floor
  - Manual control: Use Inspector to set joint angles for reaching motion
  - Observe: Box should move when contacted by robot hand
- **Grasping Setup** (optional/advanced):
  - Add Fixed Joint component programmatically on contact
  - Detect collision with `OnCollisionEnter`, create joint to "pick up" object
- **Environmental Hazards**:
  - Add trigger zones (e.g., stairs, drop-off) to test navigation safety
  - Use OnTriggerEnter to log when robot enters hazardous areas

**Code Examples**:
```csharp
// Inline C#: Simple object push detector (12-15 lines)
// Save as Assets/Scripts/PushDetector.cs
using UnityEngine;

public class PushDetector : MonoBehaviour
{
    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Robot"))
        {
            Debug.Log($"Robot pushed {gameObject.name}!");
            // Optional: Apply force to simulate impact
            GetComponent<Rigidbody>().AddForce(collision.impulse, ForceMode.Impulse);
        }
    }
}
```

**Deliverables**:
- Instructions for adding Rigidbody and Collider (numbered list, 3-4 steps)
- Push detector script (inline C#, 15 lines)
- 1 screenshot: Robot hand in contact with box, box visibly displaced

**Validation**:
- [ ] Objects with Rigidbody fall under gravity
- [ ] Robot can push objects when contact occurs
- [ ] Collision detection triggers (verified via Debug.Log)

---

### Section 6: Troubleshooting & Optimization (1 page)

**Purpose**: Solve common Unity performance and platform issues

**Content**:
- **Performance Optimization**:
  - **Issue**: Frame rate drops below 30 FPS in complex scenes
  - **Solutions**:
    - Enable GPU Instancing on materials (reduces draw calls)
    - Use LOD (Level of Detail) for robot meshes (simplified mesh at distance)
    - Reduce shadow resolution: Edit → Project Settings → Quality → Shadows: Medium
    - Disable real-time GI (Global Illumination) if not needed

- **Platform-Specific Issues**:
  - **Windows**: DirectX vs. Vulkan selection (Vulkan often faster for physics)
    - Edit → Project Settings → Player → Other Settings → Graphics API
  - **macOS**: Rosetta 2 required for Intel Unity packages on M1/M2
    - Right-click Unity Hub → Get Info → Open using Rosetta
  - **Linux**: Vulkan recommended, verify GPU drivers installed
    - Check: `glxinfo | grep "OpenGL version"`

- **URDF Import Errors**:
  - **Issue**: "Failed to import mesh" errors
  - **Solution**: Verify mesh file paths relative to URDF, convert Collada (.dae) to OBJ if needed
  - **Issue**: Robot appears at wrong scale
  - **Solution**: Adjust "Global Scale Factor" in import settings (typically 1.0 for meters)

**Code Examples**:
```csharp
// Inline C#: Performance profiler snippet (8-10 lines)
// Add to any MonoBehaviour for FPS monitoring
void OnGUI()
{
    float fps = 1.0f / Time.deltaTime;
    GUI.Label(new Rect(10, 10, 200, 30), $"FPS: {fps:F1}");
}
```

**Deliverables**:
- Troubleshooting table: Issue | Symptoms | Solution (5-6 rows)
- Platform-specific setup notes (Windows: 2 bullets, macOS: 2 bullets, Linux: 2 bullets)
- FPS monitoring snippet (inline C#, 10 lines)

**Validation**:
- [ ] Scene runs at 60+ FPS on recommended hardware (GTX 1060 or equivalent)
- [ ] No console errors related to missing meshes or materials
- [ ] Robot import succeeds on student's target platform

---

### Section 7: Exercises (0.5 pages)

**Purpose**: Hands-on practice to solidify Unity skills

#### Exercise 1: Create Apartment/Office Environment

**Task**: Build a 3-room environment (living room, kitchen, bedroom or office layout) with:
- Textured floor and walls (wood, tile, or carpet)
- At least 5 furniture objects (table, chairs, bed, desk)
- Realistic lighting (directional + 2 point lights for accent)
- At least 2 interactive objects (ball, box, book) with Rigidbody

**Expected Output**:
- Scene renders at 60+ FPS in Play mode
- All furniture has collision (robot cannot pass through)
- Interactive objects respond to robot contact
- Screenshot from first-person camera view showing furnished room

**Success Criteria**:
- [ ] Scene contains at least 3 rooms with distinct layouts
- [ ] Lighting creates realistic shadows and ambient occlusion
- [ ] At least 2 objects are interactive (can be pushed or picked up)
- [ ] No console errors in Play mode

#### Exercise 2: Import Custom URDF and Verify Articulation

**Task**: Import a different URDF model (e.g., robotic arm, quadruped, or custom humanoid variation), configure Articulation Bodies, and manually pose the robot in Unity Inspector.

**Expected Output**:
- URDF imports without errors
- All joints appear as ArticulationBody components
- Student can adjust joint angles in Inspector and observe robot pose change
- Screenshot showing robot in custom pose (e.g., arm reaching, leg lifted)

**Success Criteria**:
- [ ] URDF import completes successfully
- [ ] Joint hierarchy matches URDF structure (parent-child relationships correct)
- [ ] Joint angle adjustments produce expected motion
- [ ] Robot meshes visible and correctly scaled

---

## Code Examples Summary

**Total Code Examples**: 5 inline C# scripts + 1 bash command

1. **Unity Hub installation** (bash, 1-2 lines) - Section 2
2. **Camera controller script** (C#, 25 lines) - Section 3
3. **Joint controller script** (C#, 18 lines) - Section 4
4. **Push detector script** (C#, 15 lines) - Section 5
5. **FPS monitoring snippet** (C#, 10 lines) - Section 6
6. **Exercise scripts** (Student-created, not provided)

**Separate Files** (in `docs/docs/module-2-digital-twin/assets/code-examples/`):
- `CameraController.cs` - Complete camera controller (students copy from chapter)
- `SimpleJointController.cs` - Joint control template
- `unity_project_setup.md` - Detailed setup notes for all platforms

---

## Screenshots & Diagrams

**Required Screenshots** (8 total):
1. Unity Hub with 2022.3 LTS installed (Section 2)
2. Empty Unity project (SampleScene) (Section 2)
3. Untextured room in Scene view (Section 3)
4. Textured room in Game view with lighting (Section 3)
5. Unity Package Manager with URDF Importer installed (Section 4)
6. Humanoid robot in scene with ArticulationBody Inspector visible (Section 4)
7. Robot pushing box (contact interaction) (Section 5)
8. Furnished apartment scene (Exercise 1 example)

**Optional Diagrams**:
- Unity Editor interface overview (Scene view, Game view, Inspector, Hierarchy)
- ROS 2 ↔ Unity communication architecture (ROS TCP Connector)

---

## Validation Checklist

**Content Quality**:
- [ ] All Unity instructions verified with Unity 2022.3 LTS (not older versions)
- [ ] C# scripts compile without errors in Unity Editor
- [ ] URDF import tested with sample humanoid from Module 1
- [ ] Performance targets achievable on recommended hardware (GTX 1060, 16GB RAM)

**Educational Clarity**:
- [ ] Learning objectives stated at beginning, revisited in summary
- [ ] GUI workflows explained with screenshots (no assumed prior Unity knowledge)
- [ ] Code comments explain Unity-specific concepts (ArticulationBody, Rigidbody, Collider)
- [ ] Exercises include clear deliverables and validation criteria

**Constitution Compliance**:
- [ ] All claims reference Unity official documentation or Unity Robotics Hub
- [ ] Code examples reproducible with fresh Unity 2022.3 LTS installation
- [ ] Page count: 6-7 pages (within target)
- [ ] No hardcoded file paths (use relative paths or Unity package:// URIs)

---

## Dependencies

**From Module 1**:
- URDF syntax understanding (links, joints)
- Sample `simple_humanoid.urdf` file for import testing

**From Chapter 1**:
- Understanding of collision vs. visual geometry
- Physics concepts (mass, inertia, friction)

**External Documentation**:
- Unity Manual 2022.3 LTS: https://docs.unity3d.com/2022.3/Documentation/Manual/
- Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- URDF Importer Documentation: https://github.com/Unity-Technologies/URDF-Importer

**Tools Required**:
- Unity Hub (latest version)
- Unity 2022.3 LTS (install via Hub)
- Git (for package installation from GitHub)
- Text editor for C# scripts (Unity has built-in, or use VS Code with Unity extension)

---

## Follow-Up in Later Chapters

**Chapter 3 (Sensors)**: Will add depth cameras and LiDAR to Unity scenes for perception testing

**Module 3 (Perception)**: Will use Unity scenes for vision-based algorithm testing (object detection, segmentation)

**Module 6 (VLA Models)**: Will use Unity for human-robot interaction and manipulation demonstrations

---

**Contract Status**: ✅ READY FOR IMPLEMENTATION
**Estimated Writing Time**: 4-6 hours (including Unity scene creation and screenshot capture)
**Last Updated**: 2025-12-07
