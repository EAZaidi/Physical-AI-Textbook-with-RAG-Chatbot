# Unity Project Setup Guide

Complete guide for setting up Unity 2022.3 LTS with Robotics Hub packages for Module 2.

## Prerequisites

- **Unity Hub** 3.5.0 or later
- **Unity 2022.3 LTS** (any patch version, e.g., 2022.3.50f1)
- **Operating System**: Windows 10/11, macOS 12+, or Ubuntu 20.04/22.04
- **Hardware**: GPU with DirectX 11/12 or Metal support

## Step 1: Install Unity 2022.3 LTS

### Via Unity Hub

1. Open Unity Hub
2. Click **Installs** in the left sidebar
3. Click **Install Editor** (top right)
4. Select **Unity 2022.3 LTS** (latest recommended version)
5. Check the following modules:
   - **Microsoft Visual Studio Community** (Windows) or **Visual Studio for Mac** (macOS)
   - **Linux Build Support** (if on Windows/macOS)
   - **Documentation** (optional but recommended)

### Verify Installation

```bash
# Check Unity version
/Applications/Unity/Hub/Editor/2022.3.50f1/Unity.app/Contents/MacOS/Unity -version  # macOS
"C:\Program Files\Unity\Hub\Editor\2022.3.50f1\Editor\Unity.exe" -version           # Windows
```

## Step 2: Create New Project

1. Open Unity Hub
2. Click **New Project**
3. Select **3D (URP)** template (Universal Render Pipeline)
   - Alternatively, use **3D Core** if you prefer Standard Render Pipeline
4. Configure project:
   - **Project Name**: `HumanoidRobotDigitalTwin`
   - **Location**: Choose appropriate directory
   - **Unity Version**: 2022.3.x LTS
5. Click **Create Project**

## Step 3: Install Unity Robotics Hub Packages

### Install via Package Manager (Recommended)

1. In Unity Editor, go to **Window → Package Manager**
2. Click **+** (top left) → **Add package from git URL**
3. Add the following packages one by one:

```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

4. Wait for each package to install and compile

### Install via manifest.json (Alternative)

1. Close Unity Editor
2. Navigate to project folder: `HumanoidRobotDigitalTwin/Packages/`
3. Edit `manifest.json` and add under `dependencies`:

```json
{
  "dependencies": {
    "com.unity.robotics.ros-tcp-connector": "https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector",
    "com.unity.robotics.urdf-importer": "https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer",
    "com.unity.render-pipelines.universal": "14.0.9",
    "com.unity.textmeshpro": "3.0.6"
  }
}
```

4. Reopen Unity Editor (packages will auto-install)

### Verify Package Installation

1. **Window → Package Manager**
2. Switch dropdown from **Packages: In Project** to **Packages: Unity Registry**
3. Verify the following are installed:
   - **ROS TCP Connector** (version 0.7.0+)
   - **URDF Importer** (version 0.5.2+)

## Step 4: Project Configuration

### Configure Physics Settings

1. **Edit → Project Settings → Physics**
2. Set the following parameters:
   - **Default Solver Iterations**: 10
   - **Default Solver Velocity Iterations**: 10
   - **Gravity**: (0, -9.81, 0)
   - **Default Contact Offset**: 0.01
   - **Sleep Threshold**: 0.005

### Configure Time Settings

1. **Edit → Project Settings → Time**
2. Set:
   - **Fixed Timestep**: 0.01 (100 Hz physics update)
   - **Maximum Allowed Timestep**: 0.1

### Configure Quality Settings (Optional)

1. **Edit → Project Settings → Quality**
2. For better physics visualization:
   - **VSync Count**: Don't Sync (for maximum framerate)
   - **Anti Aliasing**: 4x Multi Sampling

## Step 5: Create Project Structure

Create the following folder structure in **Project** window:

```
Assets/
├── Scripts/           # C# scripts (CameraController, SimpleJointController, etc.)
├── Models/            # URDF files and meshes
├── Materials/         # Physics materials
├── Scenes/            # Unity scenes
│   ├── Chapter1_BasicPhysics.unity
│   ├── Chapter2_Visualization.unity
│   └── Chapter3_Sensors.unity
└── Prefabs/           # Reusable robot prefabs
```

To create folders:
1. Right-click in **Project** window → **Create → Folder**
2. Name each folder as shown above

## Step 6: Import Code Examples

### Import C# Scripts

1. Copy the following scripts to `Assets/Scripts/`:
   - `CameraController.cs`
   - `SimpleJointController.cs`
   - `PushDetector.cs`

2. In Unity, verify scripts compile without errors:
   - Check **Console** window (Window → General → Console)
   - Should see "Compilation succeeded" message

### Test Script Attachment

1. Create empty GameObject: **GameObject → Create Empty**
2. Rename to `Main Camera Controller`
3. In **Inspector**, click **Add Component**
4. Search for `CameraController` and add it
5. Verify script shows up with all public fields visible

## Step 7: Create Basic Scene

### Setup Main Camera

1. Select **Main Camera** in **Hierarchy**
2. Set **Transform** to:
   - Position: (0, 2, -5)
   - Rotation: (0, 0, 0)
3. Attach **CameraController** script (if not already attached)

### Create Ground Plane

1. **GameObject → 3D Object → Plane**
2. Rename to `Ground`
3. Set **Transform**:
   - Position: (0, 0, 0)
   - Scale: (10, 1, 10)
4. Create Physics Material:
   - Right-click in **Project** → **Create → Physic Material**
   - Name: `GroundMaterial`
   - Set **Dynamic Friction**: 0.6
   - Set **Static Friction**: 0.6
   - Set **Bounciness**: 0
5. Drag `GroundMaterial` onto Ground plane's **Mesh Collider** component

### Add Lighting

1. **GameObject → Light → Directional Light**
2. Set **Transform Rotation**: (50, -30, 0)
3. Set **Intensity**: 1.5
4. Set **Color**: Slightly warm white (RGB: 255, 244, 230)

## Step 8: Test Project

### Test Camera Controls

1. Press **Play** button (top center)
2. Test controls:
   - **WASD**: Move camera
   - **Mouse**: Look around
   - **Q/E**: Move up/down
   - **Shift**: Sprint
   - **Esc**: Unlock cursor

### Test Physics

1. **GameObject → 3D Object → Cube**
2. Position at (0, 5, 0)
3. Add **Rigidbody** component
4. Attach **PushDetector** script
5. Press **Play**
6. Press **P** to apply push force
7. Observe cube falling and colliding with ground

## Step 9: Configure ROS TCP Connector (For ROS Integration)

### Setup ROS Connection Settings

1. **Robotics → ROS Settings**
2. In **ROS Settings** window:
   - **ROS IP Address**: `127.0.0.1` (localhost) or Docker container IP
   - **ROS Port**: `10000` (default)
   - **Protocol**: `ROS2` (select from dropdown)
3. Click **Generate ROS Messages** if you plan to create custom message types

### Test ROS Connection (Optional)

If you have ROS 2 Humble running:

```bash
# In ROS 2 terminal
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

Then in Unity:
1. **Robotics → ROS Settings**
2. Click **Connect** button
3. Check **Console** for connection status

## Step 10: Save and Version Control

### Save Scene

1. **File → Save As**
2. Name: `Chapter1_BasicPhysics.unity`
3. Save in `Assets/Scenes/`

### Setup .gitignore (Recommended)

Create `.gitignore` in project root:

```gitignore
# Unity generated files
[Ll]ibrary/
[Tt]emp/
[Oo]bj/
[Bb]uild/
[Bb]uilds/
[Ll]ogs/
[Uu]ser[Ss]ettings/

# Visual Studio cache
.vs/
*.csproj
*.unityproj
*.sln
*.suo
*.tmp
*.user
*.userprefs

# Unity specific
*.pidb.meta
*.pdb.meta
*.mdb.meta

# OS generated
.DS_Store
Thumbs.db
```

## Troubleshooting

### Issue: "Package compilation errors"

**Solution**:
1. **Window → Package Manager**
2. Select problematic package
3. Click **Remove** then reinstall
4. Restart Unity Editor

### Issue: "Physics behaves erratically"

**Solution**:
1. Check **Fixed Timestep** is set to 0.01
2. Verify **Rigidbody** mass is reasonable (1-100 kg)
3. Ensure colliders are not interpenetrating at start

### Issue: "Scripts not compiling"

**Solution**:
1. Check **Console** for specific errors
2. Verify Unity version is 2022.3 LTS
3. Ensure `.cs` files are in `Assets/` folder (not Packages/)
4. Try **Assets → Reimport All**

### Issue: "Camera controls not working"

**Solution**:
1. Verify `CameraController.cs` is attached to active camera
2. Check **Inspector** shows no missing references
3. Ensure **Game** window has focus (click on it) when testing

## Next Steps

1. **Import URDF Models**: Follow Chapter 2 to import humanoid robot URDF
2. **Setup Sensors**: Add virtual cameras and raycasters (Chapter 3)
3. **Connect to Gazebo**: Use ROS TCP Connector to sync physics state

## Additional Resources

- [Unity Robotics Hub Documentation](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [URDF Importer Guide](https://github.com/Unity-Technologies/URDF-Importer/blob/main/Documentation~/UrdfImporter.md)
- [ROS TCP Connector API](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [Unity Physics Best Practices](https://docs.unity3d.com/Manual/PhysicsOverview.html)

## Summary Checklist

- [ ] Unity 2022.3 LTS installed
- [ ] Project created with URP template
- [ ] ROS TCP Connector and URDF Importer packages installed
- [ ] Physics and Time settings configured
- [ ] Folder structure created
- [ ] C# scripts imported and compiling
- [ ] Basic scene created with ground, lighting, and camera
- [ ] Camera controls tested successfully
- [ ] Physics simulation tested with test cube
- [ ] ROS connection configured (if applicable)
- [ ] Scene saved and project committed to version control

---

**Project Status**: Ready for Module 2 implementation
**Estimated Setup Time**: 30-45 minutes
**Last Updated**: 2025-12-08
