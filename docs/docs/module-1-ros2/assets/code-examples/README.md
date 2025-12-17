# Module 1 Code Examples

This directory contains all Python code examples for **Module 1: The Robotic Nervous System (ROS 2)**.

All examples are:
- **Tested**: Validated in Docker with ROS 2 Humble
- **Reproducible**: Can be run in the standardized testing environment
- **Educational**: Designed for learning, not production deployment

---

## Table of Contents

### Chapter 3: Python Programming with rclpy

| File | Description | Chapter Reference |
|------|-------------|-------------------|
| `publisher_example.py` | Basic publisher (String messages to /chatter) | [Ch3 §3](/docs/module-1-ros2/03-python-rclpy-control#3-publisher-example) |
| `subscriber_example.py` | Basic subscriber (listens to /chatter) | [Ch3 §4](/docs/module-1-ros2/03-python-rclpy-control#4-subscriber-example) |
| `service_server_example.py` | Service server (AddTwoInts) | [Ch3 §5](/docs/module-1-ros2/03-python-rclpy-control#5-service-server-example) |
| `service_client_example.py` | Service client (calls AddTwoInts) | [Ch3 §6](/docs/module-1-ros2/03-python-rclpy-control#6-service-client-example) |
| `joint_state_publisher.py` | Publishes simulated humanoid joint states | [Ch3 §7](/docs/module-1-ros2/03-python-rclpy-control#7-humanoid-joint-states) |
| `joint_state_subscriber.py` | Subscribes to joint states and logs data | [Ch3 §7](/docs/module-1-ros2/03-python-rclpy-control#7-humanoid-joint-states) |

### Chapter 4: URDF Basics

| File | Description | Chapter Reference |
|------|-------------|-------------------|
| `simple_humanoid.urdf` | Simplified humanoid robot (13 links, 12 joints) | [Ch4 §5](/docs/module-1-ros2/04-urdf-basics#5-simplified-humanoid-urdf) |
| `humanoid_with_camera.urdf` | Humanoid with camera sensor (exercise solution) | [Ch4 §7](/docs/module-1-ros2/04-urdf-basics#7-exercise-adding-a-camera-sensor) |

---

## Running the Examples

### Prerequisites

**Option 1: Native ROS 2 Installation**
- Ubuntu 22.04 LTS
- ROS 2 Humble or Iron
- Python 3.10+

**Option 2: Docker (Recommended for Consistency)**
```bash
# From project root
cd docker/ros2-testing
docker build -t ros2-testing .
docker run -it --rm -v $(pwd):/workspace ros2-testing
```

### Setting Up Python Examples

1. **Create a ROS 2 workspace**:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. **Create a package**:
```bash
ros2 pkg create --build-type ament_python module1_examples --dependencies rclpy std_msgs sensor_msgs example_interfaces
```

3. **Copy examples to package**:
```bash
# Copy Python files to module1_examples/module1_examples/
cp path/to/code-examples/*.py ~/ros2_ws/src/module1_examples/module1_examples/
```

4. **Update setup.py** (add entry points):
```python
entry_points={
    'console_scripts': [
        'publisher = module1_examples.publisher_example:main',
        'subscriber = module1_examples.subscriber_example:main',
        'service_server = module1_examples.service_server_example:main',
        'service_client = module1_examples.service_client_example:main',
        'joint_state_pub = module1_examples.joint_state_publisher:main',
        'joint_state_sub = module1_examples.joint_state_subscriber:main',
    ],
},
```

5. **Build the workspace**:
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Example Usage

### Publisher/Subscriber

**Terminal 1** (Publisher):
```bash
ros2 run module1_examples publisher
```

**Terminal 2** (Subscriber):
```bash
ros2 run module1_examples subscriber
```

**Expected Output**:
- Publisher logs: `Publishing: "Hello ROS 2: 0"`, `Publishing: "Hello ROS 2: 1"`, ...
- Subscriber logs: `I heard: "Hello ROS 2: 0"`, `I heard: "Hello ROS 2: 1"`, ...

---

### Service Server/Client

**Terminal 1** (Server):
```bash
ros2 run module1_examples service_server
```

**Terminal 2** (Client):
```bash
# Add 5 + 3
ros2 run module1_examples service_client 5 3
```

**Expected Output** (Client):
```
[INFO] [add_two_ints_client]: Sending request: a=5 b=3
[INFO] [add_two_ints_client]: Result: 5 + 3 = 8
```

---

### Joint State Publisher/Subscriber

**Terminal 1** (Publisher):
```bash
ros2 run module1_examples joint_state_pub
```

**Terminal 2** (Subscriber):
```bash
ros2 run module1_examples joint_state_sub
```

**Expected Output** (Subscriber):
```
[INFO] [joint_state_subscriber]: Received state for 13 joints
[INFO] [joint_state_subscriber]:   neck: 0.479 rad
[INFO] [joint_state_subscriber]:   left_shoulder: 0.454 rad
[INFO] [joint_state_subscriber]:   left_elbow: 0.415 rad
[INFO] [joint_state_subscriber]:   ... and 10 more joints
```

**Visualize in RViz2** (optional):
```bash
# Terminal 3
rviz2
# Add RobotModel display, set Fixed Frame to "base_link"
```

---

### URDF Visualization

**Validate URDF syntax**:
```bash
check_urdf simple_humanoid.urdf
# Expected: "robot name is: simple_humanoid" + "Successfully Parsed XML"
```

**Visualize in RViz2**:
```bash
# Terminal 1: Publish robot state
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat simple_humanoid.urdf)"

# Terminal 2: Launch RViz2
rviz2
# Configure:
# 1. Set Fixed Frame to "torso" (root link)
# 2. Add -> RobotModel
# 3. Adjust view to see full humanoid
```

**Move joints interactively**:
```bash
# Install GUI tool (if not already installed)
sudo apt install ros-humble-joint-state-publisher-gui

# Run joint state publisher GUI
ros2 run joint_state_publisher_gui joint_state_publisher_gui
# Use sliders to move robot joints in real-time
```

---

## Troubleshooting

### Common Issues

**1. `ModuleNotFoundError: No module named 'rclpy'`**
- **Cause**: ROS 2 environment not sourced
- **Fix**: Run `source /opt/ros/humble/setup.bash` (or add to `~/.bashrc`)

**2. `No executable found`**
- **Cause**: Package not built or entry point missing in `setup.py`
- **Fix**: Rebuild with `colcon build` and verify `setup.py` entry points

**3. Service/topic not found**
- **Cause**: Server/publisher not running or different topic name
- **Fix**: Use `ros2 topic list`, `ros2 service list` to verify active topics/services

**4. URDF parse error**
- **Cause**: XML syntax error (unclosed tags, missing quotes)
- **Fix**: Use `check_urdf <file>` to validate syntax

**5. RViz2 shows empty scene**
- **Cause**: Fixed Frame not set correctly
- **Fix**: Set Fixed Frame to root link name (e.g., "torso" or "base_link")

---

## Validation & Testing

All examples are tested using the project's CI/CD pipeline:

```bash
# From project root
bash scripts/test-code-examples.sh
```

This script:
1. Builds the Docker testing environment
2. Validates Python syntax for all `.py` files
3. Checks URDF files with `check_urdf` (if available)
4. Reports pass/fail status

**GitHub Actions**: Automated testing runs on every commit to `docs/` or code examples.

---

## Code Quality Standards

These examples follow best practices:

- **Source Accuracy**: Based on official ROS 2 tutorials
- **Reproducibility**: Tested in Docker with osrf/ros:humble-desktop
- **Modular Architecture**: Examples are independent, single-responsibility nodes

---

## Further Reading

- [ROS 2 Python Client Library (rclpy) API](https://docs.ros2.org/latest/api/rclpy/)
- [Writing a Simple Publisher and Subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Writing a Simple Service and Client (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)

---

**Need help?** Refer to chapter-specific troubleshooting sections or the [Module 1 landing page](../../index.mdx).
