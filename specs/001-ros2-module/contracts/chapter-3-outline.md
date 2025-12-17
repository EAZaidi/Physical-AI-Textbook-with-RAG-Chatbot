# Chapter 3 Outline: Python-to-ROS Control via rclpy

**File**: `docs/module-1-ros2/03-python-rclpy-control.mdx`
**Est. Length**: 7-8 pages (longest chapter - most hands-on)
**Learning Time**: ~3-3.5 hours
**Priority**: P1 (User Story 2 - Writing ROS 2 Nodes and Publishers)

---

## Learning Objectives

Students will:
1. Write a ROS 2 publisher node using rclpy
2. Write a ROS 2 subscriber node using rclpy
3. Implement service servers and clients
4. Understand node spinning and callback execution
5. Debug common errors (import failures, callback issues, node not found)

**Success Metrics** (from spec):
- SC-002: 90% of students create publisher-subscriber on first attempt
- SC-003: Students write functional node within 30 minutes

---

## Sections

### 1. Setting Up rclpy Environment (1 page)
- Install ROS 2 Humble (Ubuntu 22.04, WSL2, or Docker)
- Source ROS 2 workspace: `source /opt/ros/humble/setup.bash`
- Verify installation: `ros2 --version`
- Create workspace: `mkdir -p ~/ros2_ws/src`

### 2. Tutorial 1: Minimal Publisher (2 pages)
- Step-by-step code walkthrough: `publisher_example.py`
- Import rclpy, create Node class
- Create publisher: `self.create_publisher(String, 'chatter', 10)`
- Timer callback for periodic publishing
- Expected output: Messages logged, visible in `ros2 topic echo /chatter`
- **Code**: Full `publisher_example.py` (copy-pasteable)
- **Troubleshooting**: "ModuleNotFoundError: rclpy", "topic not found"

### 3. Tutorial 2: Minimal Subscriber (2 pages)
- Step-by-step code walkthrough: `subscriber_example.py`
- Create subscriber: `self.create_subscription(String, 'chatter', self.callback, 10)`
- Callback function: Process received messages
- Expected output: Log received messages
- **Code**: Full `subscriber_example.py` (copy-pasteable)
- **Troubleshooting**: "Callback not called", "QoS mismatch"

### 4. Tutorial 3: Service Server (1.5 pages)
- Step-by-step code walkthrough: `service_server_example.py`
- Create service: `self.create_service(AddTwoInts, 'add_two_ints', self.callback)`
- Service callback: Process request, return response
- Expected output: Service available (`ros2 service list`)
- **Code**: Full `service_server_example.py`

### 5. Tutorial 4: Service Client (1 page)
- Step-by-step code walkthrough: `service_client_example.py`
- Create client: `self.create_client(AddTwoInts, 'add_two_ints')`
- Call service asynchronously: `future = self.call_async(request)`
- Expected output: Receive response, log sum
- **Code**: Full `service_client_example.py`

### 6. Advanced: Joint State Publishing (Humanoid Robot Prep) (1 page)
- Example: `joint_state_publisher.py`
- Publish sensor_msgs/JointState (robot joint angles)
- Prepares students for Module 7 (humanoid manipulation)
- **Code**: `joint_state_publisher.py`

### 7. Practice Exercises (1.5 pages)
- Exercise 1: Modify publisher to send custom messages
- Exercise 2: Create subscriber for JointState
- Exercise 3: Implement custom service (e.g., "SetSpeed")

### 8. Summary and Troubleshooting (0.5 pages)
- Common errors and solutions
- Best practices (always source ROS 2, use `rclpy.spin()`, handle shutdown)

---

## Code Examples (Full Tutorials)

1. `publisher_example.py` - Publishes strings at 1 Hz
2. `subscriber_example.py` - Subscribes to `/chatter`
3. `service_server_example.py` - Adds two integers
4. `service_client_example.py` - Calls add_two_ints service
5. `joint_state_publisher.py` - Publishes joint angles
6. `joint_state_subscriber.py` - Subscribes to joint states

All code examples tested in Docker (`osrf/ros:humble-desktop`)

---

## Documentation Links

- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [Writing a Simple Publisher (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Writing a Simple Service (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
