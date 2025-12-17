# Chapter 1 Outline: ROS 2 Overview and System Architecture

**File**: `docs/module-1-ros2/01-overview-architecture.mdx`
**Est. Length**: 5-6 pages
**Learning Time**: ~1.5-2 hours
**Priority**: P1 (User Story 1 - Understanding ROS 2 Architecture)

---

## Learning Objectives (1 page)

By the end of this chapter, students will be able to:

1. Explain the key differences between ROS 1 and ROS 2 (architecture, middleware, communication)
2. Describe the DDS (Data Distribution Service) middleware layer and its role in ROS 2
3. Identify the components of the ROS 2 graph (nodes, topics, services, actions)
4. Draw a simple ROS 2 communication diagram for a robot system (e.g., camera → vision processor → actuator)
5. Understand why ROS 2 was designed for distributed, real-time robot systems

**Success Metric**: Student can explain ROS 2 communication patterns in 2-3 sentences each (SC-001 from spec)

---

## Section 1: Introduction to ROS 2 (0.5 pages)

**Content**:
- What is ROS 2? (Robot Operating System 2)
- Why robotics needs middleware (abstraction, modularity, reusability)
- Target audience: AI-driven humanoid robotics students

**Key Points**:
- ROS 2 is NOT an operating system (it's middleware)
- Enables modular robot software (sensor drivers, planning, control can be separate processes)
- Industry standard for robot development (research and commercial)

**Documentation Links**:
- [ROS 2 Overview](https://docs.ros.org/en/humble/index.html)
- [About ROS 2](https://docs.ros.org/en/humble/Concepts/About.html)

---

## Section 2: ROS 1 vs ROS 2 (1 page)

**Content**:
- Historical context: Why ROS 2 was created
- Key differences table:
  | Feature | ROS 1 | ROS 2 |
  |---------|-------|-------|
  | Middleware | Custom TCPROS | DDS (standard) |
  | Real-time support | Limited | Better (DDS QoS) |
  | Multi-robot | Requires rosbridge | Native support |
  | Security | None | DDS security plugins |
  | Python version | Python 2.7 (EOL) | Python 3.x |

**Key Points**:
- ROS 1: Research-focused, single-robot, no real-time guarantees
- ROS 2: Production-ready, multi-robot, real-time capable
- Both share conceptual similarities (nodes, topics, services)
- Migration from ROS 1 to ROS 2 ongoing in industry

**Documentation Links**:
- [ROS 2 Design Overview](https://design.ros2.org/)
- [ROS 1 vs ROS 2 Comparison](https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html)

---

## Section 3: DDS Middleware Layer (1.5 pages)

**Content**:
- What is DDS? (Data Distribution Service - OMG standard)
- How DDS enables peer-to-peer communication (no central broker in ROS 2)
- Quality of Service (QoS) profiles: Reliability, Durability, History
- DDS vendors supported by ROS 2 (Fast DDS, Cyclone DDS, RTI Connext)

**Diagrams**:
- ROS 1 architecture (roscore as central broker)
- ROS 2 architecture (peer-to-peer via DDS)

**Key Points**:
- DDS is publish-subscribe middleware (industry standard, not ROS-specific)
- QoS profiles control message delivery (reliable vs best-effort, volatile vs transient-local)
- No roscore in ROS 2 (nodes discover each other via DDS)
- DDS enables real-time, distributed systems

**Documentation Links**:
- [About DDS and ROS 2](https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html)
- [ROS 2 QoS Policies](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

**Example** (no code, just concept):
```
Camera Node ─[image topic, QoS: reliable]─> Vision Processing Node
             ─[image topic, QoS: best-effort]─> Debug Viewer Node
```

---

## Section 4: The ROS 2 Graph (1.5 pages)

**Content**:
- What is the ROS 2 graph? (network of nodes and their communication links)
- Graph components:
  - **Nodes**: Processes performing computation
  - **Topics**: Named buses for pub-sub communication
  - **Services**: Request-response communication
  - **Actions**: Long-running tasks with feedback (brief mention, not detailed until later modules)
  - **Parameters**: Configuration values (brief mention)

**Diagrams**:
- ROS 2 graph example: Simple robot system
  ```
  Camera Driver Node
      ↓ (publishes) /camera/image_raw
  Vision Processing Node
      ↓ (publishes) /detected_objects
  Motion Planning Node
      ↓ (publishes) /cmd_vel
  Robot Controller Node
  ```

**Key Points**:
- Nodes are independent processes (can run on different computers)
- Topics enable decoupled communication (publishers don't know subscribers)
- Services are for one-time requests (e.g., "reset controller", "save map")
- Graph can be visualized with `rqt_graph` tool

**Documentation Links**:
- [ROS 2 Concepts - Graph](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Graph-Concepts.html)
- [rqt_graph Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

---

## Section 5: When to Use Topics vs Services vs Actions (0.5 pages)

**Content**:
- **Topics**: Continuous data streams (sensor data, robot state)
- **Services**: One-time requests with immediate response (reset, configure)
- **Actions**: Long-running tasks with feedback/cancellation (navigation, manipulation)

**Decision Tree** (simple flowchart):
```
Need continuous data? → Use Topic
Need one-time response? → Use Service
Need long-running task with feedback? → Use Action
```

**Key Points**:
- Topics are many-to-many (N publishers, M subscribers)
- Services are one-to-many (1 server, N clients)
- Actions are similar to services but support cancellation and progress feedback
- Choose communication pattern based on data flow requirements

**Documentation Links**:
- [ROS 2 Communication Patterns](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)

---

## Section 6: Summary and Key Takeaways (0.5 pages)

**Content**:
- ROS 2 is middleware for building modular robot software
- DDS provides peer-to-peer, real-time communication
- ROS 2 graph consists of nodes communicating via topics, services, actions
- Topics for continuous data, services for requests, actions for long tasks
- ROS 2 is production-ready (real-time, multi-robot, secure)

**Self-Check Questions**:
1. What are the three main communication patterns in ROS 2?
2. How does ROS 2's architecture differ from ROS 1 (hint: DDS vs roscore)?
3. When would you use a topic instead of a service?

---

## Section 7: Practice Exercises (1 page)

### Exercise 1: Identify Communication Patterns
Given scenarios, students identify appropriate ROS 2 communication:
- "Stream camera images at 30 FPS" → Topic
- "Request robot to move to waypoint and wait for completion" → Action
- "Ask robot for current battery level" → Service

### Exercise 2: Draw a ROS 2 Graph
Draw a ROS 2 graph for a simple humanoid robot:
- Nodes: Camera driver, Joint state publisher, Motion planner, Motor controller
- Topics: /camera/image, /joint_states, /cmd_vel
- Services: /reset_pose

**Expected Output**: Hand-drawn or digital diagram showing nodes, topics, services

---

## Section 8: Resources and Further Reading (0.5 pages)

**Official Documentation**:
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Design Rationale](https://design.ros2.org/)
- [DDS and ROS 2](https://docs.ros.org/en/humble/Concepts/About-Different-Middleware-Vendors.html)

**Next Chapter Preview**:
- Chapter 2 will dive deeper into nodes, topics, and services
- Students will see code examples and prepare for hands-on tutorials in Chapter 3

---

## Code Examples

**None** (Chapter 1 is conceptual, no code)

Students will see their first code in Chapter 2 (simple publisher/subscriber snippets) and full tutorials in Chapter 3.

---

## Diagrams Required

1. **ROS 1 vs ROS 2 Architecture**: Show roscore (ROS 1) vs peer-to-peer DDS (ROS 2)
2. **ROS 2 Graph Example**: Camera → Vision → Planning → Control
3. **Communication Pattern Decision Tree**: Topics vs Services vs Actions

**Tool**: Diagrams created with Mermaid.js (embeddable in MDX) or exported PNG/SVG

---

## Metadata for RAG Chunking

```json
{
  "module": "Module 1",
  "chapter": "Chapter 1: ROS 2 Overview and System Architecture",
  "sections": [
    {"name": "Introduction to ROS 2", "type": "concept"},
    {"name": "ROS 1 vs ROS 2", "type": "concept"},
    {"name": "DDS Middleware Layer", "type": "concept"},
    {"name": "The ROS 2 Graph", "type": "concept"},
    {"name": "When to Use Topics vs Services vs Actions", "type": "concept"},
    {"name": "Summary and Key Takeaways", "type": "summary"},
    {"name": "Practice Exercises", "type": "exercise"},
    {"name": "Resources and Further Reading", "type": "reference"}
  ]
}
```

---

## Validation Checklist

- [ ] All technical claims link to official ROS 2 documentation
- [ ] Diagrams accurately represent ROS 2 architecture
- [ ] No code examples (intentional - Chapter 1 is conceptual)
- [ ] Exercises align with learning objectives
- [ ] Chapter fits within 5-6 page target
- [ ] Prerequisites: None (this is Module 1, Chapter 1)
- [ ] Connects to Chapter 2: Previews nodes, topics, services in detail
