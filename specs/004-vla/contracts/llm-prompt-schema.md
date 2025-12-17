# LLM Prompt Schema: VLA Robot Task Planning

**Feature**: Module 4 - Vision-Language-Action
**Date**: 2025-12-09
**Purpose**: Define standard prompt templates for LLM-based robot task planning

## Overview

This document defines the "API contract" between the VLA system and LLMs (GPT-4, Claude, Llama 3). Unlike traditional REST APIs, LLM interactions use natural language prompts with structured outputs.

## Core Prompt Templates

### 1. System Prompt Template

**Purpose**: Define the LLM's role, available actions, and output format

**Schema**:
```python
{
  "role": "system",
  "content": """
You are a robot task planner for a humanoid robot. Your job is to decompose high-level natural language commands into sequences of executable actions.

Available Actions:
{action_definitions}

Current Robot State:
{robot_state}

Output Format:
Generate a JSON list of actions. Each action must have:
- "name": action name (string)
- "params": parameters (dict)
- "duration": estimated time in seconds (int)

Constraints:
- Only use actions from the Available Actions list
- Ensure actions are sequentially feasible
- Include error recovery steps
- Keep plans under 5 minutes total duration

If the command is ambiguous, ask clarifying questions instead of guessing.
"""
}
```

**Variables**:
- `{action_definitions}`: Dynamically injected list of available actions (see Action Primitive Schema below)
- `{robot_state}`: Current robot pose, held objects, battery level, map information

---

### 2. User Command Template

**Purpose**: Send the transcribed voice command to the LLM

**Schema**:
```python
{
  "role": "user",
  "content": "{transcribed_command}"
}
```

**Variables**:
- `{transcribed_command}`: Output from Whisper (e.g., "bring me the red book from the shelf")

**Example**:
```json
{
  "role": "user",
  "content": "prepare the room for a meeting"
}
```

---

### 3. Action Primitive Schema

**Purpose**: Define available robot actions in a format the LLM can understand

**Template**:
```
- navigate_to(x: float, y: float): Move the robot to coordinates (x, y) on the map. Duration: 10-60 seconds.
- grasp(object_id: str): Pick up an object. Requires object_id from visual grounding. Duration: 3-5 seconds.
- release(): Drop the currently held object. Duration: 1-2 seconds.
- detect_objects(query: str): Search for objects matching the query (e.g., "red mug"). Duration: 2-3 seconds.
- rotate(angle: float): Rotate the robot by angle degrees. Duration: 2-5 seconds.
- say(text: str): Speak text using text-to-speech. Duration: 2-5 seconds.
```

**Extensibility**: Additional actions can be added for specific scenarios (e.g., `open_door`, `press_button`, `wait`)

---

### 4. Expected LLM Output Schema

**Purpose**: Validate LLM responses before execution

**JSON Schema**:
```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "command": {
      "type": "string",
      "description": "Original user command (for logging)"
    },
    "actions": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "name": {
            "type": "string",
            "enum": ["navigate_to", "grasp", "release", "detect_objects", "rotate", "say"]
          },
          "params": {
            "type": "object"
          },
          "duration": {
            "type": "integer",
            "minimum": 1,
            "maximum": 120
          }
        },
        "required": ["name", "params", "duration"]
      },
      "minItems": 1
    },
    "total_duration": {
      "type": "integer",
      "description": "Sum of all action durations"
    },
    "notes": {
      "type": "string",
      "description": "Optional explanation of the plan"
    }
  },
  "required": ["command", "actions", "total_duration"]
}
```

**Example Valid Output**:
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
  "notes": "Navigating to bookshelf, detecting target book, grasping it, returning to user, and releasing."
}
```

---

### 5. Error Handling Prompts

**Purpose**: Handle LLM failures, invalid outputs, or ambiguous commands

#### Invalid Output Recovery
```python
{
  "role": "system",
  "content": "Your previous output was invalid. Error: {error_message}. Please regenerate the plan following the correct JSON schema."
}
```

#### Clarification Request
```python
{
  "role": "system",
  "content": "The command '{command}' is ambiguous. Please ask the user a clarifying question to determine the correct action."
}
```

**Example LLM Response**:
```json
{
  "clarification": "Which room should I clean? Kitchen, living room, or bedroom?"
}
```

#### Infeasible Command
```python
{
  "role": "system",
  "content": "The command '{command}' cannot be executed because {reason}. Explain this to the user politely."
}
```

**Example LLM Response**:
```json
{
  "error": "I cannot fly to the ceiling because I don't have flight capabilities. I can only navigate on the ground."
}
```

---

## Prompt Variants by Task Type

### Navigation Task
**Command Example**: "Go to the kitchen"

**System Prompt Addition**:
```
Current Map: {map_bounds}
Known Locations:
- kitchen: (3.5, 2.1)
- living_room: (0.0, 0.0)
- bedroom: (-2.3, 4.8)

Use navigate_to with the coordinates of the target location.
```

---

### Manipulation Task
**Command Example**: "Pick up the red mug"

**System Prompt Addition**:
```
Object Detection: Enabled
Gripper Status: {open/closed}
Held Object: {object_id or None}

Manipulation Workflow:
1. detect_objects(query) to find the target
2. navigate_to the object's location
3. grasp(object_id) to pick it up
```

---

### Multi-Step Task
**Command Example**: "Prepare the room for a meeting"

**System Prompt Addition**:
```
Multi-step tasks require:
- Breaking down the goal into subtasks
- Sequencing actions logically
- Adding verification steps (e.g., detect_objects to confirm task completion)

Example breakdown:
"Prepare the room" → [clear table, arrange chairs, adjust lighting]
```

---

## API Call Pattern (OpenAI/Anthropic)

### OpenAI API (GPT-4)
```python
import openai

response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": transcribed_command}
    ],
    temperature=0.3,  # Low temperature for deterministic planning
    max_tokens=500,
    timeout=10  # seconds
)

task_plan = json.loads(response.choices[0].message.content)
```

### Anthropic API (Claude)
```python
import anthropic

client = anthropic.Anthropic(api_key="...")
message = client.messages.create(
    model="claude-3-opus-20240229",
    max_tokens=500,
    temperature=0.3,
    system=system_prompt,
    messages=[
        {"role": "user", "content": transcribed_command}
    ]
)

task_plan = json.loads(message.content[0].text)
```

---

## Validation Rules

Before executing the LLM-generated task plan, validate:

1. **Schema Compliance**: Output matches JSON schema
2. **Action Existence**: All action names are in the available actions list
3. **Parameter Types**: Parameters match expected types (float for coordinates, str for object IDs)
4. **Feasibility**: Coordinates within map bounds, object IDs correspond to detected objects
5. **Sequential Logic**: Actions are in a logical order (e.g., navigate before grasp)
6. **Duration Constraints**: Total duration <5 minutes (300 seconds)

**Validation Pseudocode**:
```python
def validate_task_plan(plan, available_actions, map_bounds, detected_objects):
    # 1. Schema check
    if not matches_schema(plan, task_plan_schema):
        raise ValidationError("Invalid JSON schema")

    # 2. Action existence
    for action in plan["actions"]:
        if action["name"] not in available_actions:
            raise ValidationError(f"Unknown action: {action['name']}")

    # 3. Parameter types (example: navigate_to)
    for action in plan["actions"]:
        if action["name"] == "navigate_to":
            x, y = action["params"]["x"], action["params"]["y"]
            if not (isinstance(x, float) and isinstance(y, float)):
                raise ValidationError("navigate_to requires float coordinates")

    # 4. Feasibility (example: coordinate bounds)
    for action in plan["actions"]:
        if action["name"] == "navigate_to":
            x, y = action["params"]["x"], action["params"]["y"]
            if not within_bounds(x, y, map_bounds):
                raise ValidationError(f"Coordinates ({x}, {y}) out of map bounds")

    # 5. Duration constraint
    if plan["total_duration"] > 300:
        raise ValidationError("Task plan exceeds 5 minute limit")

    return True  # All checks passed
```

---

## Testing Examples

### Test Case 1: Simple Navigation
**Input Command**: "Move to the kitchen"
**Expected Output**:
```json
{
  "command": "Move to the kitchen",
  "actions": [
    {"name": "navigate_to", "params": {"x": 3.5, "y": 2.1}, "duration": 20}
  ],
  "total_duration": 20,
  "notes": "Navigating to kitchen coordinates."
}
```

### Test Case 2: Object Manipulation
**Input Command**: "Pick up the red mug"
**Expected Output**:
```json
{
  "command": "Pick up the red mug",
  "actions": [
    {"name": "detect_objects", "params": {"query": "red mug"}, "duration": 3},
    {"name": "navigate_to", "params": {"x": 1.2, "y": 0.5}, "duration": 10},
    {"name": "grasp", "params": {"object_id": "mug_001"}, "duration": 5}
  ],
  "total_duration": 18,
  "notes": "Detecting red mug, navigating to it, and grasping."
}
```

### Test Case 3: Ambiguous Command
**Input Command**: "Clean up"
**Expected Output** (Clarification):
```json
{
  "clarification": "Which area should I clean? Kitchen, living room, or bedroom?"
}
```

### Test Case 4: Infeasible Command
**Input Command**: "Fly to the ceiling"
**Expected Output** (Error):
```json
{
  "error": "I cannot fly. I can only navigate on the ground using wheels or legs."
}
```

---

## Summary

This LLM prompt schema defines:
1. **System prompts** that establish the robot's action space and constraints
2. **User prompts** that pass transcribed voice commands
3. **Action primitive definitions** that the LLM can reference
4. **Output schemas** (JSON) for structured task plans
5. **Error handling** for invalid outputs, ambiguities, and infeasible commands
6. **Validation rules** to ensure safe execution

Students will learn to:
- Engineer effective prompts for robotics tasks
- Parse and validate LLM outputs
- Handle common failure modes (API timeouts, syntax errors, infeasible plans)
- Integrate LLM planning with ROS 2 action execution
