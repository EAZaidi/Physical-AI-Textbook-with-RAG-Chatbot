# Data Model: Module 1 - The Robotic Nervous System (ROS 2)

**Date**: 2025-12-07
**Feature**: Module 1 ROS 2 educational content
**Purpose**: Define ROS 2 entities, concepts, and relationships for educational content

---

## Overview

This data model defines the core ROS 2 entities and concepts that students will learn in Module 1. Unlike traditional software data models (database schemas, API contracts), this model represents **conceptual entities** for educational purposes. Each entity corresponds to a ROS 2 concept that students must understand.

**Note**: This is NOT a database schema or API definition. It's a learning model that structures content organization and RAG retrieval.

---

## Entity: ROS 2 Node

### Definition
A process that performs computation in a ROS 2 system (e.g., sensor processing, control logic, planning). Nodes communicate via topics, services, or actions.

### Attributes (Conceptual)
- **Name**: Unique identifier (e.g., `minimal_publisher`, `camera_driver`)
- **Executable**: Python script or compiled binary
- **Communication interfaces**: Publishers, subscribers, service servers, service clients, action servers, action clients
- **Namespace**: Optional grouping (e.g., `/robot1/camera`, `/robot2/camera`)
- **Parameters**: Configuration values (not covered in Module 1 basics)

### Relationships
- **Has Many** Publishers (0 to N)
- **Has Many** Subscribers (0 to N)
- **Has Many** Service Servers (0 to N)
- **Has Many** Service Clients (0 to N)
- **Participates In** ROS 2 Graph (network of nodes)

### Learning Objectives
- Students understand nodes are independent processes
- Students can create nodes using rclpy
- Students know nodes can have multiple publishers/subscribers
- Students understand node lifecycle (initialization, spinning, shutdown)

### Code Example Context
```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node initialized')

def main():
    rclpy.init()
    node = MinimalNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Chapter Coverage
- **Chapter 1**: Conceptual introduction (what is a node?)
- **Chapter 2**: Node roles in ROS 2 graph
- **Chapter 3**: Creating nodes with rclpy (detailed tutorials)

---

## Entity: Topic

### Definition
A named bus for asynchronous publish-subscribe communication. Topics are typed by message definitions (e.g., `std_msgs/String`, `sensor_msgs/JointState`).

### Attributes (Conceptual)
- **Name**: Unique identifier (e.g., `/chatter`, `/camera/image_raw`)
- **Message Type**: Data structure definition (e.g., `std_msgs/String`)
- **QoS Profile**: Quality of Service settings (reliability, durability, history depth)
- **Publishers**: Nodes publishing to this topic (0 to N)
- **Subscribers**: Nodes subscribing to this topic (0 to N)

### Relationships
- **Belongs To** ROS 2 Graph
- **Has Many** Publishers (Nodes)
- **Has Many** Subscribers (Nodes)
- **Uses** Message Definition (e.g., `std_msgs/String`)

### Learning Objectives
- Students understand topics enable decoupled communication (publishers don't know subscribers)
- Students know topics are many-to-many (N publishers, M subscribers)
- Students can identify appropriate use cases for topics (sensor data streaming, continuous updates)
- Students understand message types define data structure

### Code Example Context
```python
# Publisher
self.publisher_ = self.create_publisher(String, 'chatter', 10)
self.publisher_.publish(msg)

# Subscriber
self.subscription = self.create_subscription(
    String, 'chatter', self.listener_callback, 10)
```

### Chapter Coverage
- **Chapter 1**: Conceptual introduction (pub-sub pattern)
- **Chapter 2**: Topics in detail (when to use, message types)
- **Chapter 3**: Creating publishers and subscribers (rclpy tutorials)

---

## Entity: Service

### Definition
A synchronous request-response communication pattern. Services are defined by service types (e.g., `std_srvs/SetBool`, `example_interfaces/AddTwoInts`).

### Attributes (Conceptual)
- **Name**: Unique identifier (e.g., `/add_two_ints`, `/reset_controller`)
- **Service Type**: Request/response structure definition
- **Server Node**: Node providing the service (1)
- **Client Nodes**: Nodes calling the service (0 to N)

### Relationships
- **Belongs To** ROS 2 Graph
- **Has One** Service Server (Node)
- **Has Many** Service Clients (Nodes)
- **Uses** Service Definition (request + response message types)

### Learning Objectives
- Students understand services are request-response (synchronous, blocking)
- Students know when to use services vs topics (one-time requests vs continuous data)
- Students can implement service servers and clients
- Students understand service calls block until response received

### Code Example Context
```python
# Service Server
self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    return response

# Service Client
self.cli = self.create_client(AddTwoInts, 'add_two_ints')
request = AddTwoInts.Request()
request.a = 1
request.b = 2
future = self.cli.call_async(request)
```

### Chapter Coverage
- **Chapter 1**: Conceptual introduction (request-response pattern)
- **Chapter 2**: Services in detail (when to use, comparison with topics)
- **Chapter 3**: Creating service servers and clients (rclpy tutorials)

---

## Entity: Publisher

### Definition
A component within a node that sends messages on a topic.

### Attributes (Conceptual)
- **Topic Name**: Which topic to publish on
- **Message Type**: Type of messages published
- **Queue Size**: How many messages to buffer
- **Parent Node**: Node containing this publisher

### Relationships
- **Belongs To** Node (1)
- **Publishes On** Topic (1)
- **Sends** Messages (typed data)

### Learning Objectives
- Students understand publishers are node components (not standalone)
- Students know publishers send messages without knowing subscribers
- Students can create publishers with `create_publisher()`
- Students understand queue size affects message buffering

### Code Example Context
```python
self.publisher_ = self.create_publisher(String, 'topic_name', 10)
msg = String()
msg.data = 'Hello ROS 2'
self.publisher_.publish(msg)
```

### Chapter Coverage
- **Chapter 2**: Publisher role in pub-sub
- **Chapter 3**: Creating publishers (detailed tutorial with examples)

---

## Entity: Subscriber

### Definition
A component within a node that receives messages from a topic.

### Attributes (Conceptual)
- **Topic Name**: Which topic to subscribe to
- **Message Type**: Type of messages expected
- **Callback Function**: Function called when message received
- **Queue Size**: How many messages to buffer
- **Parent Node**: Node containing this subscriber

### Relationships
- **Belongs To** Node (1)
- **Subscribes To** Topic (1)
- **Receives** Messages (typed data)

### Learning Objectives
- Students understand subscribers are node components (not standalone)
- Students know subscribers receive messages via callback functions
- Students can create subscribers with `create_subscription()`
- Students understand callback functions must match message type

### Code Example Context
```python
self.subscription = self.create_subscription(
    String, 'topic_name', self.listener_callback, 10)

def listener_callback(self, msg):
    self.get_logger().info(f'Received: {msg.data}')
```

### Chapter Coverage
- **Chapter 2**: Subscriber role in pub-sub
- **Chapter 3**: Creating subscribers (detailed tutorial with examples)

---

## Entity: URDF (Unified Robot Description Format)

### Definition
An XML format for describing a robot's kinematic structure, including links, joints, collision geometries, visual meshes, and sensors.

### Attributes (Conceptual)
- **Robot Name**: Identifier for the robot model
- **Links**: Rigid bodies (list of Link entities)
- **Joints**: Connections between links (list of Joint entities)
- **Sensors**: Optional sensor definitions (cameras, LiDAR, etc.)

### Relationships
- **Has Many** Links (1 to N)
- **Has Many** Joints (0 to N, tree structure requires N-1 joints for N links)
- **Root Link**: One link with no parent (tree root)

### Learning Objectives
- Students understand URDF defines robot structure for simulation and visualization
- Students can read URDF files and identify links, joints, geometries
- Students can modify URDF files (add links, change joint limits)
- Students can visualize URDF in RViz2

### Code Example Context
```xml
<robot name="simple_humanoid">
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
  </link>
  <link name="head">...</link>
  <joint name="neck" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57"/>
  </joint>
</robot>
```

### Chapter Coverage
- **Chapter 4**: URDF basics (syntax, links, joints, visualization)

---

## Entity: Link (URDF)

### Definition
A rigid body in the robot model (e.g., torso, upper arm, head). Links have collision and visual geometries.

### Attributes (Conceptual)
- **Name**: Unique identifier within the robot (e.g., `torso`, `left_upper_arm`)
- **Visual Geometry**: Appearance (box, cylinder, mesh file)
- **Collision Geometry**: Simplified shape for collision detection
- **Inertial Properties**: Mass, center of mass, inertia tensor (optional for Module 1 basics)

### Relationships
- **Belongs To** URDF Robot (1)
- **Connected To** Other Links via Joints (parent or child)
- **Parent Of** Joints (where this link is parent)
- **Child Of** Joint (where this link is child, 0 or 1)

### Learning Objectives
- Students understand links represent rigid bodies
- Students know links have visual (rendering) and collision (physics) geometries
- Students can define links in URDF with simple geometries (box, cylinder)
- Students understand link hierarchy (parent-child relationships via joints)

### Code Example Context
```xml
<link name="left_upper_arm">
  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.03" length="0.3"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.03" length="0.3"/>
    </geometry>
  </collision>
</link>
```

### Chapter Coverage
- **Chapter 4**: Link definitions (visual/collision geometries, simple shapes)

---

## Entity: Joint (URDF)

### Definition
A connection between two links defining relative motion. Joint types include revolute (hinge), prismatic (slider), fixed, continuous, planar, floating.

### Attributes (Conceptual)
- **Name**: Unique identifier (e.g., `left_shoulder`, `neck`)
- **Type**: revolute | prismatic | fixed | continuous | planar | floating
- **Parent Link**: Link this joint connects from
- **Child Link**: Link this joint connects to
- **Axis**: Axis of rotation/translation (for revolute/prismatic)
- **Limits**: Joint limits (lower, upper, effort, velocity)
- **Origin**: Position and orientation of child link relative to parent

### Relationships
- **Belongs To** URDF Robot (1)
- **Connects** Parent Link to Child Link (2 links)
- **Defines** Kinematic Relationship (how child moves relative to parent)

### Learning Objectives
- Students understand joints define how links move relative to each other
- Students know joint types (revolute for rotation, prismatic for linear, fixed for rigid connection)
- Students can define joints with parent, child, axis, limits
- Students understand kinematic tree structure (no cycles, one root link)

### Code Example Context
```xml
<joint name="left_shoulder" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0 0.15 0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
```

### Chapter Coverage
- **Chapter 4**: Joint definitions (types, parent-child, axis, limits)

---

## Relationships Summary

### ROS 2 Communication Graph
```
Node (1) ──has many──> Publisher (N)
Node (1) ──has many──> Subscriber (N)
Node (1) ──has many──> Service Server (N)
Node (1) ──has many──> Service Client (N)

Publisher (N) ──publishes on──> Topic (1)
Subscriber (N) ──subscribes to──> Topic (1)
Service Server (1) ──provides──> Service (1)
Service Client (N) ──calls──> Service (1)
```

### URDF Kinematic Tree
```
URDF Robot (1) ──has many──> Link (N)
URDF Robot (1) ──has many──> Joint (N-1 for N links)

Joint (1) ──connects──> Parent Link (1)
Joint (1) ──connects──> Child Link (1)

Root Link (1) ──has no parent joint
All Other Links (N-1) ──have one parent joint
```

---

## Code Examples Inventory

Based on the data model, Module 1 will include the following code examples:

### Chapter 2-3: ROS 2 Communication Examples
1. **Minimal Publisher** (`publisher_example.py`): Publishes string messages on `/chatter` topic
2. **Minimal Subscriber** (`subscriber_example.py`): Subscribes to `/chatter`, logs received messages
3. **Publisher-Subscriber Pair** (combined example): Demonstrates full pub-sub cycle
4. **Service Server** (`service_server_example.py`): Provides `add_two_ints` service
5. **Service Client** (`service_client_example.py`): Calls `add_two_ints` service
6. **Joint State Publisher** (`joint_state_publisher.py`): Publishes sensor_msgs/JointState (robot-relevant)
7. **Joint State Subscriber** (`joint_state_subscriber.py`): Subscribes to joint states, logs data

### Chapter 4: URDF Examples
8. **Simple Humanoid URDF** (`simple_humanoid.urdf`): 13 links, 12 joints (torso, head, arms, legs)
9. **URDF with Camera Sensor** (`humanoid_with_camera.urdf`): Exercise solution (adds camera to head)

### Test/Validation Scripts
10. **Test All Examples** (`test_examples.sh`): Docker-based validation script (CI/CD)

**Total**: 10 code artifacts (7 Python scripts, 2 URDF files, 1 test script)

---

## Content Organization by Entity

### By Chapter
- **Chapter 1**: Node, Topic, Service (conceptual introduction)
- **Chapter 2**: Node, Topic, Service, Publisher, Subscriber (detailed explanations)
- **Chapter 3**: Node, Publisher, Subscriber, Service Server, Service Client (rclpy tutorials)
- **Chapter 4**: URDF, Link, Joint (robot modeling)

### By Priority (from Spec)
- **P1 (User Story 1 - Architecture)**: Node, Topic, Service (concepts)
- **P1 (User Story 2 - Coding)**: Publisher, Subscriber, Service Server/Client (hands-on)
- **P2 (User Story 3 - URDF)**: URDF, Link, Joint (modeling)

### By RAG Metadata
Each entity will be tagged in RAG chunks:
```json
{
  "metadata": {
    "entity_type": "Node" | "Topic" | "Service" | "Publisher" | "Subscriber" | "URDF" | "Link" | "Joint",
    "chapter": "Chapter 1" | "Chapter 2" | "Chapter 3" | "Chapter 4",
    "content_type": "concept" | "tutorial" | "exercise" | "code_example"
  }
}
```

---

## Validation Rules

### Constitution Compliance
Each entity definition MUST include:
- **Source Reference**: Link to official ROS 2 documentation (e.g., ros.org, design.ros2.org)
- **Code Example**: Working Python or URDF snippet
- **Learning Objective**: What students should understand about this entity

### Testability
Each code example MUST:
- Run in Docker (osrf/ros:humble-desktop)
- Produce verifiable output (e.g., log messages, RViz2 visualization)
- Include "Expected Output" section in chapter

### Reproducibility
Each entity MUST specify:
- ROS 2 version (Humble or Iron)
- Python version (3.10+)
- Required packages (e.g., rclpy, std_msgs, sensor_msgs)

---

## Next Steps

This data model informs:
1. **Chapter outlines** (contracts/): Each chapter covers specific entities
2. **Code examples**: 10 code artifacts mapped to entities
3. **RAG chunking**: Entity metadata for retrieval
4. **Tasks.md**: Implementation tasks organized by entity (e.g., "Write Publisher tutorial", "Create simple_humanoid.urdf")
