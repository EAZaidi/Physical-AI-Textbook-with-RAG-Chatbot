# Chapter 2 Outline: Nodes, Topics, and Services

**File**: `docs/module-1-ros2/02-nodes-topics-services.mdx`
**Est. Length**: 6-7 pages
**Learning Time**: ~2-2.5 hours
**Priority**: P1 (User Story 1 & 2 - Architecture + Preparation for Coding)

---

## Learning Objectives

Students will:
1. Understand node lifecycle and roles in ROS 2 systems
2. Explain topic-based publish-subscribe communication
3. Differentiate between topics and services (when to use each)
4. Identify message and service types (std_msgs, sensor_msgs, geometry_msgs)
5. Prepare for hands-on rclpy tutorials in Chapter 3

---

## Sections

### 1. Nodes in Depth (1.5 pages)
- Node definition, lifecycle (init, spin, shutdown)
- Node discovery via DDS
- Namespaces and node naming conventions
- **Code snippet**: Minimal node structure (Python preview, full tutorial in Ch3)
- **Diagram**: Node lifecycle state machine

### 2. Topics in Depth (2 pages)
- Publish-subscribe pattern
- Topic naming conventions (/)
- Message types (std_msgs/String, sensor_msgs/JointState, geometry_msgs/Twist)
- QoS profiles for topics (reliable vs best-effort)
- **Code snippet**: Publisher and subscriber (preview, full tutorial in Ch3)
- **Diagram**: Multi-publisher, multi-subscriber topic

### 3. Services in Depth (1.5 pages)
- Request-response pattern
- Service naming conventions
- Service types (std_srvs/SetBool, example_interfaces/AddTwoInts)
- Synchronous vs asynchronous service calls
- **Code snippet**: Service server and client (preview, full tutorial in Ch3)
- **Diagram**: Service call flow (client → server → response)

### 4. Choosing Communication Patterns (0.5 pages)
- Decision matrix: Topics vs Services
- Use cases: Sensor data (topic), Reset command (service), Navigation (action)

### 5. Command-Line Tools (1 page)
- `ros2 topic list`, `ros2 topic echo /topic_name`
- `ros2 service list`, `ros2 service call /service_name`
- `ros2 node list`, `ros2 node info /node_name`
- `rqt_graph` visualization

### 6. Practice Exercises (1 page)
- Exercise: Use command-line tools to explore running ROS 2 system
- Exercise: Identify message types for given scenarios

### 7. Summary and Resources (0.5 pages)

---

## Code Examples (Previews)

- Minimal node (Python snippet)
- Publisher example (snippet)
- Subscriber example (snippet)
- Service server (snippet)
- Service client (snippet)

**Note**: Full tutorials with explanations in Chapter 3

---

## Documentation Links

- [ROS 2 Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
- [ROS 2 Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
- [ROS 2 Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)
