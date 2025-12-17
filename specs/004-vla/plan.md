# Implementation Plan: Module 4 - Vision-Language-Action (VLA)

**Branch**: `004-vla` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-vla/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Module 4 teaches students to build Vision-Language-Action (VLA) systems that enable humanoid robots to understand natural language commands, decompose them using LLMs, and execute actions through ROS 2. The module consists of 4 MDX chapters for Docusaurus covering VLA foundations, OpenAI Whisper voice pipelines, LLM-based cognitive planning, and a capstone autonomous humanoid project. All tutorials integrate with Isaac Sim and previous modules (ROS 2, perception, navigation) and must be reproducible on Ubuntu 22.04 with ROS 2 Humble.

## Technical Context

**Language/Version**: Python 3.10+ (ROS 2 Humble compatibility), MDX (Docusaurus 3.9.2)
**Primary Dependencies**:
- OpenAI Whisper v3 (speech-to-text)
- OpenAI API / Anthropic API (LLM planning) or local alternatives (Llama 3, Mistral)
- ROS 2 Humble Hawksbill (`rclpy`, `std_msgs`, `geometry_msgs`, `nav2_msgs`, `action_msgs`)
- Isaac Sim 2023.1.1+ (from Module 3)
- Python libraries: `openai`, `anthropic`, `whisper`, `sounddevice`, `numpy`, `opencv-python`
- Docusaurus 3.9.2 for content delivery
**Storage**: N/A (educational content, rosbag recordings optional for voice command replays)
**Testing**:
- Manual validation: All code examples tested in Ubuntu 22.04 + ROS 2 Humble + Isaac Sim environment
- Speech transcription tests: Whisper accuracy >80% for robot commands
- LLM output validation: Generated action sequences execute successfully in simulation (>70% task completion)
- Docusaurus build: MDX compilation without errors
**Target Platform**: Ubuntu 22.04 LTS with NVIDIA GPU (minimum RTX 3060 for Whisper + Isaac Sim)
**Project Type**: Educational documentation (Docusaurus static site) + example ROS 2 packages
**Performance Goals**:
- Whisper transcription: <2 seconds latency for voice commands
- LLM API response: <5 seconds for task plan generation (cloud), <10 seconds (local models)
- End-to-end voice-to-action: <10 seconds from speech to robot motion
- Capstone demo: Complete 3-step tasks in <5 minutes
**Constraints**:
- GPU memory: <10GB VRAM for Whisper + Isaac Sim combined
- API costs: Minimize LLM API calls in tutorials (provide cost estimates)
- Tutorial completion: Students complete exercises in <2 hours per chapter
- Dependencies: Must work with free-tier APIs (OpenAI/Anthropic) or local open-source models
**Scale/Scope**:
- 4 chapters (15-20 pages each, ~60-80 pages total)
- ~10-15 code examples (Python ROS 2 nodes, LLM prompt templates, integration scripts)
- 3-4 LLM prompt templates for different task types
- 1 comprehensive capstone project integrating Modules 1-4

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Source Accuracy & Verifiability ✅
- **Status**: PASS
- **Evidence**: All technical claims will be traced to:
  - OpenAI Whisper documentation (github.com/openai/whisper)
  - OpenAI API documentation (platform.openai.com/docs)
  - Anthropic Claude API documentation (docs.anthropic.com)
  - LangChain documentation (python.langchain.com)
  - ROS 2 Humble documentation (docs.ros.org/en/humble)
  - VLA research papers (RT-1, PaLM-E, Code as Policies from Google Research)
- **Action**: Phase 0 research will identify exact documentation URLs, API versions, and validated prompt patterns

### II. Educational Clarity & Student Success ✅
- **Status**: PASS
- **Evidence**: Spec defines 4 prioritized user stories (P1-P3), each independently testable with clear acceptance criteria
- **Learning progression**: VLA Foundations → Voice Pipeline → LLM Planning → Capstone Integration
- **Prerequisites clearly stated**: Modules 1-3 (ROS 2, simulation, Isaac perception/Nav2) must be completed first
- **Action**: Each chapter will include learning objectives, conceptual explanations, step-by-step code, prompt engineering examples, and troubleshooting

### III. Reproducibility & Environment Consistency ✅
- **Status**: PASS
- **Evidence**:
  - Explicit versions: Ubuntu 22.04, ROS 2 Humble, Isaac Sim 2023.1.1+, Whisper v3, Python 3.10+
  - FR-001 mandates installation tutorials with dependency lists
  - FR-010 requires graceful handling of API failures (timeouts, invalid outputs) with fallback behaviors
  - SC-007 validates all code runs on specified environment without errors
- **Action**: Phase 1 will detail pip requirements, API key setup, local model alternatives, validation steps

### IV. Spec-Driven Content Development ✅
- **Status**: PASS
- **Evidence**: Following Spec-Kit Plus workflow:
  - ✅ spec.md created with user scenarios, requirements, success criteria
  - 🔄 plan.md in progress (this file)
  - ⏳ tasks.md will be generated via `/sp.tasks`
- **Action**: Continue with Phase 0 research → Phase 1 design → Phase 2 task breakdown

### V. RAG Chatbot Fidelity ✅
- **Status**: PASS (deferred to deployment phase)
- **Evidence**: Module 4 content will be indexed for RAG retrieval after publication
- **Special consideration**: VLA content includes LLM prompts—ensure chatbot doesn't confuse tutorial prompts with its own instructions
- **Action**: Structure chapter content to clearly separate "example prompts for robotics" from instructional text

### VI. Modular Architecture & Progressive Complexity ✅
- **Status**: PASS
- **Evidence**:
  - Prerequisites explicitly stated: Module 1 (ROS 2 basics), Module 2 (simulation), Module 3 (Isaac perception/Nav2)
  - Module 4 focuses exclusively on VLA cognitive layer (no duplicate perception/navigation content)
  - 4 chapters build progressively: Concepts → Voice → Planning → Integration
  - Out of scope clearly defined: No custom VLA model training, no hardware deployment, no ethics coverage
- **Action**: Reference Module 3 Isaac Sim scenes and Nav2 configs, demonstrate VLA commands triggering existing navigation

### VII. Production-Ready Deployment Standards ✅
- **Status**: PASS (content-level compliance)
- **Evidence**:
  - MDX format compatible with existing Docusaurus 3.9.2 setup
  - SC-007 mandates Docusaurus build success
  - Code examples include error handling and logging per FR-010
- **Action**: Validate MDX syntax during Phase 1, test build in CI/CD pipeline

### Technical Constraints Compliance ✅
- **Content Length**: 60-80 pages (within 120-200 page total target for entire book)
- **Chapter Structure**: Will follow established format (learning objectives, overview, tutorial, exercises, summary)
- **Technology Stack**: Uses approved stack (Docusaurus MDX, ROS 2 Humble, OpenAI/Anthropic APIs or open-source alternatives)
- **Quality Gates**: Will pass source verification, code validation, build success, chatbot integration

### Summary
**GATE RESULT**: ✅ **PASS** - All constitution principles satisfied. No violations requiring justification.

## Project Structure

### Documentation (this feature)

```text
specs/004-vla/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command) - PENDING RESEARCH AGENTS
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command) - LLM prompt schemas
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
docs/
├── docs/
│   └── module-4-vla/                          # NEW: Module 4 content
│       ├── _category_.json                     # Sidebar configuration
│       ├── index.mdx                           # Module overview
│       ├── 01-vla-foundations.mdx              # Chapter 1: VLA System Architecture
│       ├── 02-whisper-voice-pipeline.mdx       # Chapter 2: OpenAI Whisper Integration
│       ├── 03-llm-cognitive-planning.mdx       # Chapter 3: LLM-based Task Decomposition
│       ├── 04-autonomous-humanoid-capstone.mdx # Chapter 4: Full System Integration
│       └── assets/                             # Screenshots, diagrams, architecture images
│           ├── vla-pipeline-diagram.png
│           ├── whisper-ros-flow.png
│           ├── llm-planning-example.png
│           ├── capstone-demo.png
│           └── code-examples/
│               └── README.md                   # Index of all code examples
│
├── static/
│   └── code-examples/
│       └── module-4/                          # NEW: Downloadable code
│           ├── whisper_transcription_node.py  # ROS 2 node for Whisper
│           ├── llm_planner_node.py            # ROS 2 node for LLM planning
│           ├── action_executor.py             # Executes LLM-generated actions
│           ├── visual_grounding_node.py       # Object detection for language refs
│           ├── capstone_integration.py        # Full VLA system launch
│           ├── prompts/                       # LLM prompt templates
│           │   ├── navigation_planner.txt
│           │   ├── manipulation_planner.txt
│           │   └── multi_step_task.txt
│           ├── requirements.txt               # Python dependencies
│           └── launch/
│               └── vla_system_launch.py       # ROS 2 launch file for capstone
│
└── docusaurus.config.js                      # Updated with Module 4 navigation

```

**Structure Decision**: Educational documentation (Docusaurus) with example ROS 2 Python packages. No frontend/backend separation needed—all code is tutorial examples for students to run locally. Follows Module 3 pattern: MDX chapters in `docs/docs/module-4-vla/`, downloadable code in `static/code-examples/module-4/`.

## Complexity Tracking

> **No violations detected** - All constitution gates pass. This section intentionally left minimal per constitution requirements.

## Phase 0: Research & Design Decisions

*CURRENTLY IN PROGRESS - 3 research agents running in parallel*

### Research Tasks
1. **OpenAI Whisper Integration** (Agent ecf23ea5): Model selection, ROS 2 integration patterns, real-time performance, dependencies
2. **LLM Planning Patterns** (Agent f27568a6): Prompt engineering, Code as Policies, action space definition, error handling
3. **Visual Grounding Techniques** (Agent 3d544314): Object detection for language references, depth alignment, ROS 2 vision_msgs

### Additional Research Topics (to be consolidated)
- VLA architecture survey (RT-1, PaLM-E, Code as Policies comparisons)
- ROS 2 action server patterns for LLM-generated commands
- Local LLM alternatives (Llama 3, Mistral) for students without API access
- Speech synthesis (TTS) for robot feedback (optional enhancement)

*Research results will be consolidated into `research.md` below after agents complete.*

---

**STATUS**: Waiting for research agents to complete. Once research.md is finalized, will proceed to Phase 1 (data-model.md, contracts, quickstart.md).
