# Data Model: Module 4 - Vision-Language-Action (VLA)

**Feature**: Module 4 - Vision-Language-Action
**Date**: 2025-12-09
**Status**: Draft

## Overview

This document defines the key data structures (entities) for the VLA system as educational concepts. These are not implementation-specific (no Python classes or ROS 2 message types), but rather conceptual models that students will learn to implement.

## Core Entities

### 1. Voice Command
**Description**: Raw audio input captured from a microphone and processed into transcribed text by Whisper.

**Key Attributes**:
- Audio waveform (time-series data)
- Sample rate (e.g., 16kHz)
- Duration (seconds)
- Language (e.g., English, Spanish, multilingual)
- Transcribed text output
- Confidence score (0.0-1.0)
- Timestamp of capture

**Relationships**:
- Input to **Transcription Node**
- Produces **Transcribed Command**

**State Transitions**:
1. Audio captured → Buffered
2. Buffered → Transcribing (Whisper processing)
3. Transcribing → Transcribed (text available)
4. Transcribed → Published (sent to LLM Planner)

**Validation Rules**:
- Audio duration must be <30 seconds (Whisper limit)
- Sample rate must match Whisper expectations (16kHz recommended)
- Confidence score >0.7 considered valid for robotics commands

---

### 2. Transcription Node
**Description**: ROS 2 node running OpenAI Whisper that listens to microphone input, transcribes speech in real-time, and publishes text commands to a ROS 2 topic.

**Key Attributes**:
- Whisper model size (tiny/base/small/medium/large)
- Input audio topic (e.g., `/audio_input`)
- Output text topic (e.g., `/voice/command`)
- Processing latency (target: <2 seconds)
- GPU acceleration enabled/disabled

**Relationships**:
- Consumes **Voice Command** (audio)
- Publishes **Transcribed Command** (text)
- Triggers **LLM Planner** via topic subscription

**State Transitions**:
1. Idle → Listening (waiting for audio)
2. Listening → Processing (Whisper inference)
3. Processing → Publishing (text command ready)
4. Publishing → Idle

**Validation Rules**:
- Must handle audio buffer overflows gracefully
- Must retry on Whisper inference errors (max 3 retries)
- Must publish empty string if transcription confidence <0.5

---

### 3. Transcribed Command
**Description**: Plain text output from Whisper representing the user's spoken command.

**Key Attributes**:
- Text string (e.g., "move to the kitchen")
- Confidence score
- Timestamp
- Language detected
- Original audio duration

**Relationships**:
- Output of **Transcription Node**
- Input to **LLM Planner**

**Validation Rules**:
- Maximum length: 200 characters (typical robot commands are short)
- Must be UTF-8 encoded
- Empty strings indicate transcription failure

---

### 4. LLM Planner
**Description**: Service that accepts high-level natural language goals and returns structured action sequences (JSON or Python code) using an LLM (GPT-4, Claude, Llama 3).

**Key Attributes**:
- LLM provider (OpenAI / Anthropic / local model)
- Model name (e.g., gpt-4, claude-3-opus, llama-3-70b)
- System prompt template
- Available action primitives (navigate, grasp, release, search, etc.)
- Current robot state (position, held object, battery level)
- Output format (JSON / Python code / natural language)
- API timeout (default: 10 seconds)
- Temperature (creativity parameter, e.g., 0.3 for deterministic planning)

**Relationships**:
- Consumes **Transcribed Command**
- References **Action Primitive** definitions
- May query **Visual Grounding** for object locations
- Produces **Task Plan**

**State Transitions**:
1. Idle → Received Command (new transcription available)
2. Received Command → Planning (querying LLM API)
3. Planning → Validating (checking output syntax)
4. Validating → Publishing (task plan ready) OR Replanning (invalid output)
5. Publishing → Idle

**Validation Rules**:
- LLM response must parse as valid JSON or Python code
- Generated actions must reference only available action primitives
- Action parameters must be within feasible ranges (e.g., navigate coordinates within map bounds)
- Must handle API timeouts with fallback ("I couldn't understand, please try again")

---

### 5. Action Primitive
**Description**: Atomic robot behavior (e.g., `navigate_to(x, y)`, `grasp(object_id)`, `release()`) that can be executed by ROS 2 action servers.

**Key Attributes**:
- Action name (e.g., "navigate_to", "grasp", "search")
- Parameters (e.g., {x: float, y: float} for navigate_to)
- Expected duration (e.g., navigate takes 10-60 seconds)
- Failure modes (e.g., obstacle detected, object not found)
- Prerequisites (e.g., grasp requires object detection first)
- ROS 2 action server topic (e.g., `/navigate_to_pose`, `/grasp_object`)

**Relationships**:
- Defined in **LLM Planner** prompt as available actions
- Executed by **Action Executor**
- May depend on **Visual Grounding** (for object-based actions)

**Validation Rules**:
- Parameters must match expected types (floats for coordinates, strings for object IDs)
- Coordinates must be within map boundaries
- Object IDs must correspond to detected objects
- Each action must have a timeout (default: 60 seconds)

---

### 6. Task Plan
**Description**: Ordered sequence of action primitives with parameters, generated by the LLM and executed by a behavior tree or state machine.

**Key Attributes**:
- List of actions (ordered)
- Each action includes: name, parameters, estimated duration
- Total estimated time
- Generated timestamp
- Original command (text)
- Confidence score (if LLM provides one)

**Relationships**:
- Output of **LLM Planner**
- Consumed by **Execution Monitor**
- References **Action Primitives**

**State Transitions**:
1. Generated → Validated (syntax check passed)
2. Validated → Queued (ready for execution)
3. Queued → Executing (first action started)
4. Executing → Completed (all actions succeeded) OR Failed (action error) OR Replanning (dynamic obstacle)

**Validation Rules**:
- Must contain at least 1 action
- Actions must be sequentially feasible (e.g., can't grasp before navigating to object)
- Total estimated time must be <5 minutes for typical tasks
- Must include error recovery actions (e.g., retry, abort, ask for help)

**Example**:
```json
{
  "command": "fetch the red book from the shelf",
  "actions": [
    {"name": "navigate_to", "params": {"x": 3.5, "y": 2.1}, "duration": 15},
    {"name": "detect_objects", "params": {"query": "red book"}, "duration": 3},
    {"name": "grasp", "params": {"object_id": "book_001"}, "duration": 5},
    {"name": "navigate_to", "params": {"x": 0.0, "y": 0.0}, "duration": 15},
    {"name": "release", "params": {}, "duration": 2}
  ],
  "total_duration": 40,
  "timestamp": "2025-12-09T13:45:00Z"
}
```

---

### 7. Visual Grounding
**Description**: Process of mapping language references (e.g., "the red mug") to detected objects in camera images with bounding boxes and 3D poses.

**Key Attributes**:
- Language query (e.g., "red mug", "book on shelf")
- Detected objects (list of bounding boxes + labels + confidence)
- Best match object ID
- 3D pose (x, y, z, orientation) in robot frame
- Camera frame ID
- Detection timestamp

**Relationships**:
- Triggered by **LLM Planner** when action requires visual grounding (e.g., "grasp red mug")
- Consumes camera images (RGB + depth)
- Publishes detected object poses to ROS 2 topics
- Used by **Action Executor** to target specific objects

**State Transitions**:
1. Idle → Query Received (language reference extracted from plan)
2. Query Received → Detecting (running object detection model)
3. Detecting → Matching (finding best match for query)
4. Matching → Pose Estimation (converting 2D bbox to 3D pose)
5. Pose Estimation → Publishing (object pose available) OR Failed (no match found)

**Validation Rules**:
- Detection confidence must be >0.7 for robot manipulation
- 3D pose must be within robot's reachable workspace
- If multiple matches found, select closest object to robot
- Must publish "object not found" if no detections match query

**Example**:
```json
{
  "query": "red mug",
  "detections": [
    {"label": "mug", "color": "red", "confidence": 0.92, "bbox": [320, 180, 450, 340]},
    {"label": "mug", "color": "blue", "confidence": 0.88, "bbox": [100, 200, 200, 350]}
  ],
  "best_match": {
    "object_id": "mug_001",
    "pose": {"x": 0.85, "y": 0.23, "z": 0.75, "yaw": 1.57},
    "confidence": 0.92
  }
}
```

---

### 8. Execution Monitor
**Description**: ROS 2 node that tracks action progress, detects failures, and triggers replanning when necessary.

**Key Attributes**:
- Current action being executed
- Action status (pending/running/succeeded/failed)
- Retry count (for failed actions)
- Failure reason (e.g., "obstacle detected", "object not found", "timeout")
- Replanning trigger conditions

**Relationships**:
- Consumes **Task Plan**
- Monitors **Action Primitives** execution via ROS 2 action feedback
- Triggers **LLM Planner** for replanning on failures

**State Transitions**:
1. Idle → Executing (task plan received)
2. Executing → Action Running (current action started)
3. Action Running → Action Succeeded (action completed) → Next Action
4. Action Running → Action Failed (error detected) → Retry OR Replan
5. All Actions Succeeded → Task Complete
6. Replanning → New Task Plan → Executing

**Validation Rules**:
- Maximum 3 retries per action before replanning
- Must log all failures for debugging
- Must request user input if replanning fails twice ("I'm stuck, please help")

---

### 9. Capstone Scenario
**Description**: Integrated demo environment (simulated home in Isaac Sim) with multiple rooms, objects, and navigation challenges for testing the full VLA system.

**Key Attributes**:
- Isaac Sim scene file (.usd format)
- Room layout (kitchen, living room, bedroom, etc.)
- Object inventory (mugs, books, chairs, tables, etc.) with poses
- Navigation map (occupancy grid from SLAM)
- Humanoid robot model (from Module 2)
- Test commands (list of voice commands to demonstrate)

**Relationships**:
- Uses **Isaac Sim** from Module 3
- Uses **Nav2** for navigation from Module 3
- Integrates **Whisper** + **LLM Planner** + **Visual Grounding**
- Demonstrates complete VLA pipeline end-to-end

**Test Scenarios**:
1. "Prepare the room for a meeting" → Arrange chairs, clear table
2. "Fetch the book from the shelf" → Navigate, detect, grasp, return
3. "Clean the living room" → Navigate, detect objects, pick up items
4. Handle dynamic obstacles (person enters room mid-task)
5. Recover from failures (object not found, grasp failed)

---

## Entity Relationships Diagram

```
Voice Command (audio)
    ↓
Transcription Node (Whisper)
    ↓
Transcribed Command (text)
    ↓
LLM Planner (GPT-4/Claude/Llama)  ←→  Action Primitives (available actions)
    ↓                                      ↑
Task Plan (action sequence)               |
    ↓                                      |
Execution Monitor                    Visual Grounding
    ↓                                (object detection)
Action Executor (ROS 2 actions)           ↑
    ↓                                      |
Robot Motion (Isaac Sim / Nav2)    ←──────┘
    ↓
Task Complete / Failure → Replanning (back to LLM)
```

## Summary

This data model defines the conceptual entities students will implement across Module 4's 4 chapters:
- **Chapter 1** (Foundations): Introduces all entities and their relationships
- **Chapter 2** (Whisper): Implements Voice Command, Transcription Node, Transcribed Command
- **Chapter 3** (LLM Planning): Implements LLM Planner, Action Primitives, Task Plan, Visual Grounding
- **Chapter 4** (Capstone): Implements Execution Monitor, integrates all entities in Capstone Scenario

All entities are technology-agnostic at this level—implementation details (Python classes, ROS 2 message types, API calls) will be covered in tutorial code.
