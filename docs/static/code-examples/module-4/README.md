# Module 4: Vision-Language-Action (VLA) - Code Examples

This directory contains all code examples for Module 4: Vision-Language-Action systems for humanoid robots.

## Quick Start

### 1. Install Dependencies

```bash
# Install Python dependencies
pip install -r requirements.txt

# Install ROS 2 Humble (if not already installed)
sudo apt install ros-humble-desktop ros-humble-audio-common

# Install Grounding DINO (for visual grounding)
git clone https://github.com/IDEA-Research/GroundingDINO.git
cd GroundingDINO && pip install -e .

# Download Grounding DINO model
mkdir -p models
wget -O models/groundingdino_swint_ogc.pth \
  https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
```

### 2. Set Up API Keys

```bash
# Create .env file (never commit to git!)
cat > .env << EOF
OPENAI_API_KEY=sk-your-openai-key-here
ANTHROPIC_API_KEY=sk-ant-your-anthropic-key-here
EOF

# Load environment variables
source .env
```

### 3. Run Quick Demo

```bash
# Terminal 1: Launch Isaac Sim
cd ~/isaac-sim
./isaac-sim.sh --ros2

# Terminal 2: Launch VLA system
source /opt/ros/humble/setup.bash
ros2 launch launch/vla_system_launch.py

# Terminal 3: Send voice commands
# Speak into microphone: "Move to the kitchen"
```

## Code Examples Index

### Chapter 2: Whisper Voice Pipeline

| File | Description | Usage |
|------|-------------|-------|
| `whisper_transcription_node.py` | ROS 2 node for real-time speech-to-text | `ros2 run vla_system whisper_node` |
| `test_whisper.py` | Test Whisper accuracy with audio files | `python test_whisper.py --audio test_audio/` |

**Key Features**:
- faster-whisper optimization (0.6s latency)
- Voice Activity Detection (VAD)
- Circular audio buffer (5s window)
- Async inference (non-blocking ROS 2 spin)

### Chapter 3: LLM Cognitive Planning

| File | Description | Usage |
|------|-------------|-------|
| `llm_planner_node.py` | ReAct-based cognitive planner | `ros2 run vla_system llm_planner` |
| `langchain_agent.py` | LangChain integration example | Import as library |
| `action_executor.py` | Execute LLM-generated actions | `ros2 run vla_system action_executor` |
| `visual_grounding_node.py` | Grounding DINO + depth estimation | `ros2 run vla_system grounding_node` |
| `test_llm_planner.py` | Test LLM planning | `python test_llm_planner.py` |

**Key Features**:
- OpenAI, Anthropic, Llama 3 support
- ReAct loop with observation feedback
- JSON output parsing and validation
- Error recovery and replanning
- Visual grounding for object references

### Chapter 4: Autonomous Humanoid Capstone

| File | Description | Usage |
|------|-------------|-------|
| `capstone_integration.py` | Complete VLA system | `python capstone_integration.py` |
| `launch/vla_system_launch.py` | Launch all nodes | `ros2 launch launch/vla_system_launch.py` |

**Key Features**:
- End-to-end voice-to-action pipeline
- Multi-step task execution
- Dynamic replanning
- Error recovery

### Prompt Templates

| File | Description | Use Case |
|------|-------------|----------|
| `prompts/system_prompt.txt` | Base system prompt | All tasks |
| `prompts/navigation_planner.txt` | Navigation-specific | "Go to kitchen" |
| `prompts/manipulation_planner.txt` | Manipulation-specific | "Pick up cup" |
| `prompts/multi_step_task.txt` | Complex tasks | "Prepare room" |

## System Architecture

```
Voice Command (Microphone)
        ↓
Whisper Transcription Node (0.6s)
        ↓
/voice/command (ROS 2 topic)
        ↓
LLM Planner Node (ReAct, 3-5s)
        ↓
Task Plan (JSON actions)
        ↓
┌───────────────┴───────────────┐
↓                               ↓
Visual Grounding Node       Action Executor
(Grounding DINO, 90ms)      (Nav2 + MoveIt2)
        ↓                       ↓
Object Poses            ROS 2 Action Servers
        └───────────┬───────────┘
                    ↓
            Isaac Sim / Robot
```

## Performance Benchmarks

| Component | Target | Actual | Hardware |
|-----------|--------|--------|----------|
| Whisper transcription | <2s | 0.6s | RTX 3060 |
| LLM planning | <5s | 3-4s | GPT-4o API |
| Visual grounding | <100ms | 90ms | RTX 3060 |
| End-to-end | <10s | 4-8s | Combined |

## Common Commands

### Test Individual Components

```bash
# Test Whisper transcription
ros2 run vla_system whisper_node --model base
ros2 topic echo /voice/command

# Test LLM planner
ros2 run vla_system llm_planner --llm gpt-4o
ros2 service call /plan_task vla_msgs/srv/PlanTask "{command: 'go to kitchen'}"

# Test visual grounding
ros2 run vla_system grounding_node
ros2 topic pub /grounding_query std_msgs/String "data: 'red mug'"
ros2 topic echo /detected_objects
```

### Run Complete System

```bash
# Launch all nodes
ros2 launch launch/vla_system_launch.py

# Or manually launch each node:
ros2 run vla_system whisper_node &
ros2 run vla_system llm_planner &
ros2 run vla_system grounding_node &
ros2 run vla_system action_executor &
```

## Troubleshooting

### Whisper Issues

**Problem**: Low transcription accuracy (<80%)
**Solutions**:
- Use directional microphone (Blue Yeti, Audio-Technica)
- Reduce background noise
- Try `small` model instead of `base`
- Increase `text_threshold` parameter

**Problem**: High latency (>2s)
**Solutions**:
- Enable GPU: `device="cuda"`
- Use INT8 quantization: `compute_type="int8"`
- Try whisper.cpp for edge devices

### LLM Issues

**Problem**: API timeouts
**Solutions**:
- Retry with exponential backoff
- Switch to faster model (GPT-4o-mini, Claude Haiku)
- Use local model (Llama 3 8B)

**Problem**: Invalid JSON outputs
**Solutions**:
- Use function calling (structured outputs)
- Add JSON schema validation
- Re-prompt with error message

### Visual Grounding Issues

**Problem**: Object not detected
**Solutions**:
- Lower `box_threshold` (0.35 → 0.25)
- Improve lighting conditions
- Use more descriptive queries ("red ceramic mug" vs "mug")
- Check camera calibration

## API Cost Estimates

**Per Capstone Demo Run** (5-10 LLM calls):
- OpenAI GPT-4o: $0.15-0.30
- Anthropic Claude Sonnet: $0.10-0.20
- Local Llama 3: $0 (requires 16GB+ VRAM)

**Recommendation**: Start with GPT-4o-mini ($0.01/call) for development.

## Hardware Requirements

**Minimum**:
- GPU: NVIDIA GTX 1660 Ti (6GB VRAM)
- CPU: 4 cores
- RAM: 16GB
- Storage: 20GB for models

**Recommended**:
- GPU: NVIDIA RTX 3060 (12GB VRAM)
- CPU: 8 cores
- RAM: 32GB
- Storage: 50GB

## Environment Setup Script

```bash
#!/bin/bash
# setup_env.sh - Complete environment setup

# Install system dependencies
sudo apt update
sudo apt install -y ros-humble-desktop python3-pip portaudio19-dev

# Install Python dependencies
pip install -r requirements.txt

# Install Grounding DINO
if [ ! -d "GroundingDINO" ]; then
    git clone https://github.com/IDEA-Research/GroundingDINO.git
    cd GroundingDINO && pip install -e . && cd ..
fi

# Download models
mkdir -p models
if [ ! -f "models/groundingdino_swint_ogc.pth" ]; then
    wget -O models/groundingdino_swint_ogc.pth \
      https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
fi

# Build ROS 2 workspace
cd ~/ros2_ws
colcon build --packages-select vla_system
source install/setup.bash

echo "✅ Environment setup complete!"
```

## Testing

### Unit Tests

```bash
# Test Whisper accuracy
pytest test_whisper.py -v

# Test LLM planner
pytest test_llm_planner.py -v

# Test visual grounding
pytest test_grounding.py -v
```

### Integration Tests

```bash
# Launch system in test mode
ros2 launch launch/vla_test.launch.py

# Run integration test suite
ros2 test vla_system
```

## Further Resources

- **Module 4 Documentation**: `docs/docs/module-4-vla/`
- **OpenAI Whisper**: https://github.com/openai/whisper
- **LangChain Docs**: https://python.langchain.com/
- **Grounding DINO**: https://github.com/IDEA-Research/GroundingDINO
- **ROS 2 Humble**: https://docs.ros.org/en/humble/
- **Isaac Sim**: https://docs.omniverse.nvidia.com/isaacsim/

## Support

For questions or issues:
1. Check the troubleshooting guide above
2. Review module documentation
3. Post in course discussion forum
4. Open an issue on GitHub

---

**Module 4: Vision-Language-Action (VLA)** | Physical AI & Humanoid Robotics Textbook
