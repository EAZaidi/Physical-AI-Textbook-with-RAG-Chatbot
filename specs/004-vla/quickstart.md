# Quickstart Guide: Module 4 - Vision-Language-Action (VLA)

**Feature**: Module 4 - Vision-Language-Action
**Date**: 2025-12-09
**Audience**: Students, instructors, and contributors

## Overview

This quickstart guide provides a fast-path to understanding and implementing Module 4: Vision-Language-Action (VLA) systems for humanoid robots. Follow this guide to go from zero to a working voice-controlled robot in Isaac Sim.

## Prerequisites

**Completed Modules**:
- ✅ Module 1: ROS 2 Fundamentals (nodes, topics, actions)
- ✅ Module 2: Digital Twin (URDF, Gazebo, Isaac Sim basics)
- ✅ Module 3: Isaac AI Brain (VSLAM, Nav2, Isaac ROS perception)

**System Requirements**:
- Ubuntu 22.04 LTS
- NVIDIA GPU (RTX 3060+ with 12GB VRAM recommended)
- ROS 2 Humble Hawksbill
- Isaac Sim 2023.1.1+
- Python 3.10+
- Microphone (USB recommended, e.g., Blue Yeti)
- Internet connection (for LLM API access) OR local LLM setup

**Time Estimate**: 4-6 hours (including installation, tutorials, and capstone demo)

---

## Installation (30 minutes)

### Step 1: Install Whisper Dependencies

```bash
# Install PyTorch with CUDA support
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install faster-whisper (optimized Whisper)
pip install faster-whisper>=0.10.0

# Install audio capture libraries
pip install sounddevice>=0.4.6 scipy webrtcvad>=2.0.10

# Install ROS 2 audio packages
sudo apt install ros-humble-audio-common ros-humble-audio-common-msgs
```

### Step 2: Install LLM Integration Libraries

```bash
# Install LangChain and LLM APIs
pip install langchain>=0.1.0 langchain-openai langchain-anthropic

# Install OpenAI and Anthropic SDKs
pip install openai>=1.0.0 anthropic>=0.18.0

# Optional: Install local LLM support (Llama 3, Mistral)
pip install transformers>=4.30.0 accelerate>=0.20.0
```

### Step 3: Install Grounding DINO (Visual Grounding)

```bash
# Clone Grounding DINO repository
cd ~/ros2_ws/src
git clone https://github.com/IDEA-Research/GroundingDINO.git
cd GroundingDINO

# Install dependencies
pip install -e .

# Download pre-trained model
mkdir -p models
wget -O models/groundingdino_swint_ogc.pth \
  https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
```

### Step 4: Configure API Keys

```bash
# Create .env file for API keys (NEVER commit to git!)
cd ~/ros2_ws
cat > .env << EOF
OPENAI_API_KEY=sk-your-openai-key-here
ANTHROPIC_API_KEY=sk-ant-your-anthropic-key-here
EOF

# Load environment variables
source .env

# Or export directly
export OPENAI_API_KEY="sk-your-key-here"
export ANTHROPIC_API_KEY="sk-ant-your-key-here"
```

**Alternative**: Use local models (no API keys required):
```bash
# Download Llama 3 8B model (requires HuggingFace token)
huggingface-cli login
huggingface-cli download meta-llama/Meta-Llama-3-8B-Instruct
```

### Step 5: Download Module 4 Code Examples

```bash
# Clone textbook repository
cd ~/ros2_ws/src
git clone https://github.com/EAZaidi/Physical-AI-Humanoid-Robotics-Textbook.git

# Navigate to Module 4 code examples
cd Physical-AI-Humanoid-Robotics-Textbook/static/code-examples/module-4

# Install Python dependencies
pip install -r requirements.txt

# Build ROS 2 workspace
cd ~/ros2_ws
colcon build --packages-select vla_system
source install/setup.bash
```

---

## Quick Demo: Voice-Controlled Navigation (15 minutes)

### Step 1: Start Isaac Sim

```bash
# Launch Isaac Sim with humanoid robot scene
cd ~/isaac-sim
./isaac-sim.sh --ros2 --scenario humanoid_warehouse.usd
```

### Step 2: Launch VLA System

```bash
# Terminal 1: Whisper transcription node
ros2 run vla_system whisper_node --model base

# Terminal 2: LLM planner node
ros2 run vla_system llm_planner_node --llm gpt-4o

# Terminal 3: Visual grounding node
ros2 run vla_system grounding_node

# Or launch all at once:
ros2 launch vla_system vla_demo.launch.py
```

### Step 3: Test Voice Commands

```bash
# Monitor transcriptions
ros2 topic echo /voice/command

# Speak into microphone:
"Move to the kitchen"
"Pick up the red mug"
"Search for objects in the living room"
```

### Expected Output

```
[INFO] [whisper_node]: Transcription: "move to the kitchen"
[INFO] [llm_planner]: Action: navigate_to(x=3.5, y=2.1, theta=0.0)
[INFO] [nav2_client]: Navigation started...
[INFO] [nav2_client]: Navigation succeeded
[INFO] [llm_planner]: Task complete
```

---

## Module 4 Learning Path (4 chapters)

### Chapter 1: Foundations of VLA Systems (1 hour)

**Learning Objectives**:
- Understand VLA pipeline: speech → LLM → vision → action
- Compare traditional control vs. cognitive planning
- Explore VLA architectures (RT-1, PaLM-E, Code as Policies)

**Key Concepts**:
- ReAct pattern (Reasoning + Acting)
- Action primitives
- Visual grounding
- Embodied AI

**Hands-On Exercise**: Diagram a VLA pipeline for "fetch the book from the shelf"

---

### Chapter 2: Voice-to-Action Pipeline with Whisper (1.5 hours)

**Learning Objectives**:
- Install and configure OpenAI Whisper
- Create a ROS 2 transcription node
- Handle voice commands with Voice Activity Detection (VAD)
- Integrate with existing ROS 2 systems

**Key Concepts**:
- Speech-to-text (STT)
- faster-whisper optimization
- Audio buffering and VAD
- ROS 2 topic publishing

**Hands-On Exercise**: Build a Whisper node that publishes to `/voice/command`

**Code Example**: `whisper_transcription_node.py`
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from faster_whisper import WhisperModel
import sounddevice as sd

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.model = WhisperModel("base", device="cuda", compute_type="int8")
        self.publisher = self.create_publisher(String, '/voice/command', 10)
        self.get_logger().info("Whisper node ready. Speak a command...")

    def transcribe_audio(self, audio_buffer):
        segments, info = self.model.transcribe(audio_buffer, beam_size=1)
        transcription = " ".join([segment.text for segment in segments])

        msg = String()
        msg.data = transcription
        self.publisher.publish(msg)
        self.get_logger().info(f"Transcription: {transcription}")

# See full implementation in module-4/whisper_transcription_node.py
```

---

### Chapter 3: Cognitive Planning with LLMs for ROS 2 (2 hours)

**Learning Objectives**:
- Engineer prompts for robot task planning
- Implement ReAct agent with LangChain
- Map LLM outputs to ROS 2 action servers
- Handle errors, ambiguities, and replanning

**Key Concepts**:
- Prompt engineering
- ReAct (Reasoning + Acting)
- Function calling (OpenAI/Anthropic)
- Error recovery strategies

**Hands-On Exercise**: Create an LLM planner that generates action sequences for "clean the living room"

**Code Example**: `llm_planner_node.py`
```python
from langchain.agents import create_react_agent, AgentExecutor
from langchain_openai import ChatOpenAI
from langchain.tools import BaseTool

class ROS2NavigateTool(BaseTool):
    name = "navigate_to"
    description = "Navigate robot to (x, y, theta)"

    def _run(self, x: float, y: float, theta: float = 0.0):
        # Call Nav2 action server
        result = self.nav2_client.navigate_to_pose(x, y, theta)
        return f"Navigation {'succeeded' if result.success else 'failed'}"

llm = ChatOpenAI(model="gpt-4o", temperature=0)
tools = [ROS2NavigateTool(...), GraspTool(...), DetectTool(...)]
agent = create_react_agent(llm, tools, prompt_template)
executor = AgentExecutor(agent=agent, tools=tools, max_iterations=10)

# Execute plan
result = executor.invoke({"input": "Go to the kitchen and find the red mug"})
```

**Visual Grounding Integration**: Use Grounding DINO to detect objects referenced in commands

---

### Chapter 4: Capstone - Building the Autonomous Humanoid (1.5 hours)

**Learning Objectives**:
- Integrate Whisper + LLM + Visual Grounding + Nav2 + MoveIt2
- Build a complete autonomous humanoid demo
- Handle dynamic obstacles and failures
- Test multi-step tasks end-to-end

**Key Concepts**:
- System integration
- Error recovery
- Execution monitoring
- Replanning strategies

**Hands-On Exercise**: Complete capstone demo with 3 tasks:
1. "Prepare the room for a meeting" (arrange chairs, clear table)
2. "Fetch the book from the shelf" (navigate, detect, grasp, return)
3. "Clean the living room" (detect objects, pick up items, place in bin)

**Code Example**: `capstone_integration.py`
```python
# Full VLA system integration
class VLASystem:
    def __init__(self):
        self.whisper_node = WhisperNode()
        self.llm_planner = LLMPlannerNode()
        self.grounding_node = GroundingNode()
        self.nav2_client = Nav2Client()
        self.moveit2_client = MoveIt2Client()

    def execute_voice_command(self, audio):
        # 1. Transcribe speech
        command = self.whisper_node.transcribe(audio)

        # 2. Generate plan
        task_plan = self.llm_planner.plan(command)

        # 3. Execute actions
        for action in task_plan["actions"]:
            if action["name"] == "grasp":
                # Visual grounding
                object_pose = self.grounding_node.find_object(action["params"]["query"])
                self.moveit2_client.grasp(object_pose)
            elif action["name"] == "navigate_to":
                self.nav2_client.navigate(action["params"]["x"], action["params"]["y"])

        return {"success": True, "message": "Task completed"}
```

---

## Testing & Validation

### Unit Tests

```bash
# Test Whisper transcription accuracy
ros2 run vla_system test_whisper --audio-file test_commands.wav

# Test LLM planning output
ros2 run vla_system test_llm_planner --command "fetch the red book"

# Test visual grounding
ros2 run vla_system test_grounding --query "red mug" --image-file test_scene.png
```

### Integration Tests

```bash
# Launch full system in Isaac Sim
ros2 launch vla_system capstone_demo.launch.py

# Run automated test suite
ros2 test vla_system
```

### Success Criteria (from spec.md)

- ✅ SC-001: Explain VLA pipeline in <3 minutes with diagram
- ✅ SC-002: 85% of students run Whisper with >80% accuracy
- ✅ SC-003: Write LLM prompt for 3 tasks (navigate, grasp, search)
- ✅ SC-004: LLM plans execute with >70% success rate
- ✅ SC-005: Complete voice-to-action integration with <5 clarifications
- ✅ SC-006: Capstone demo completes 3-step tasks in <5 minutes
- ✅ SC-007: All code runs on Ubuntu 22.04 + ROS 2 Humble + Isaac Sim + Whisper v3
- ✅ SC-008: Identify and fix 2+ VLA failure modes

---

## Troubleshooting

### Whisper Issues

**Problem**: Transcription accuracy <80%
**Solution**:
- Use directional microphone
- Reduce background noise
- Increase `text_threshold` parameter
- Try `small` model instead of `base`

**Problem**: Latency >2 seconds
**Solution**:
- Enable GPU: `device="cuda"`
- Use INT8 quantization: `compute_type="int8"`
- Try `whisper.cpp` for edge devices

### LLM Issues

**Problem**: API timeouts
**Solution**:
- Retry with exponential backoff
- Switch to faster model (GPT-4o-mini, Claude Haiku)
- Use local model (Llama 3 8B)

**Problem**: Invalid JSON outputs
**Solution**:
- Use function calling (structured outputs)
- Add JSON schema validation
- Re-prompt with error message

### Visual Grounding Issues

**Problem**: Object not detected
**Solution**:
- Lower `box_threshold` (0.35 → 0.25)
- Improve lighting
- Move camera closer
- Use more descriptive queries ("red ceramic mug" vs "mug")

---

## Next Steps

**After completing Module 4**:
- Explore Module 5: Manipulation & Grasping (MoveIt2, tactile sensing)
- Experiment with custom action primitives
- Extend to multi-robot coordination
- Deploy on physical hardware (Unitree G1, Boston Dynamics Spot)

**Resources**:
- Module 4 documentation: `/docs/module-4-vla/`
- Code examples: `/static/code-examples/module-4/`
- Prompt templates: `/static/code-examples/module-4/prompts/`
- Research: `/specs/004-vla/research.md`

---

## Cost Estimates

**LLM API Costs** (per capstone demo run):
- OpenAI GPT-4o: $0.15-0.30 per demo (5-10 API calls @ $0.03/call)
- Anthropic Claude Sonnet: $0.10-0.20 per demo
- Local Llama 3: $0 (requires 16GB+ VRAM)

**Recommendation**: Start with GPT-4o-mini ($0.01/call) for development, upgrade to GPT-4o for production quality.

---

**Quickstart Status**: ✅ READY - Follow this guide to build a working voice-controlled robot in 4-6 hours!
