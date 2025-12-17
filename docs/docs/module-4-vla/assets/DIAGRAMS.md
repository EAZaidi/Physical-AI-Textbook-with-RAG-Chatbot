# Module 4 Diagram Descriptions

This file contains descriptions for all diagrams referenced in Module 4 chapters.
Diagrams should be created using tools like draw.io, Excalidraw, or generated programmatically.

## Chapter 1: VLA Foundations

### vla-pipeline-diagram.png
**Description**: Flow diagram showing complete VLA pipeline
- Components: Microphone → Whisper (Speech-to-Text) → LLM Planner → Visual Grounding → Action Executor → Robot
- Show data flowing between components with topic names
- Highlight feedback loops (observations back to LLM)

### vla-vs-traditional.png
**Description**: Comparison table diagram
- Two columns: Traditional Control vs VLA
- Rows: Flexibility, Complexity, Adaptability, Programming Required, Zero-Shot Capability
- Use green checkmarks and red X marks

### vla-architectures-comparison.png
**Description**: Architecture comparison diagram for RT-1, PaLM-E, Code as Policies
- Side-by-side blocks showing:
  - RT-1: Vision Transformer → Action Tokens
  - PaLM-E: Image + Text → PaLM → Actions
  - Code as Policies: Text → LLM → Python Code → Robot
- Label key differences (end-to-end learned vs. symbolic)

### react-pattern-diagram.png
**Description**: ReAct loop flow diagram
- Circular flow: Thought → Action → Observation → Thought (repeat)
- Include example at each step:
  - Thought: "I need to navigate to kitchen"
  - Action: navigate_to(5.0, 2.0, "kitchen")
  - Observation: "Navigation successful"
- Show max iterations breakout condition

### vla-failure-modes.png
**Description**: Three failure mode scenarios with illustrations
1. Visual Ambiguity: Multiple similar objects, robot confused
2. Language Grounding Error: "Red mug" when only blue mug exists
3. Action Infeasibility: Target position unreachable due to obstacle
- Use simple icons/diagrams for each scenario

## Chapter 2: Whisper Voice Pipeline

### whisper-ros-flow.png
**Description**: Whisper-ROS 2 integration flow diagram
- Audio Input (Microphone) → VAD (Voice Activity Detection) → Circular Buffer (5s) → Whisper Model → ROS 2 Publisher (/voice/command)
- Show timing: 0.6s for transcription
- Include parameters: sample_rate=16000, channels=1

### whisper-model-comparison.png
**Description**: Table comparing Whisper model sizes
- Columns: Model (tiny/base/small/medium), Parameters, Latency, Accuracy, VRAM, Use Case
- Data:
  - tiny: 39M params, 0.3s, 85%, 1GB, Real-time mobile
  - base: 74M params, 0.6s, 92%, 1.5GB, Balanced (recommended)
  - small: 244M params, 1.2s, 96%, 2.5GB, High accuracy
  - medium: 769M params, 2.8s, 98%, 5GB, Maximum accuracy

## Chapter 3: LLM Cognitive Planning

### llm-planning-flow.png
**Description**: LLM planning flow diagram
- Voice Command → LLM Planner (with system prompt) → JSON Action Sequence → Action Executor
- Show prompt template being loaded
- Include example JSON output
- Show retry loop for invalid JSON

### visual-grounding-pipeline.png
**Description**: Visual grounding pipeline
- RGB Image + Depth Image + Text Query → Grounding DINO → 2D BBox → Depth Lookup → 3D Pose (x,y,z) → TF2 Transform → Robot Frame
- Show example: "red mug" → bbox (100, 150, 200, 250) → depth 1.2m → (0.5, 0.3, 0.8) in base_link
- Timing: 40ms detection + 50ms processing = 90ms total

## Chapter 4: Autonomous Humanoid Capstone

### capstone-system-architecture.png (already in README.md as ASCII, can be converted to diagram)
**Description**: Complete system architecture
- Show all nodes: whisper_node, llm_planner, visual_grounding, action_executor, vla_system
- Show all topics: /voice/command, /planned_actions, /detected_objects_3d, /action_feedback
- Show connections with arrows and topic names

## Creating Diagrams

**Recommended Tools**:
- Draw.io (https://app.diagrams.net/)
- Excalidraw (https://excalidraw.com/)
- PlantUML for flow diagrams
- Python matplotlib for charts/tables

**Style Guidelines**:
- Use consistent colors: Blue for ROS nodes, Green for topics, Orange for external services
- Font: Sans-serif, minimum 14pt for readability
- Export as PNG (1200x800 minimum resolution)
- Use white or light gray background
