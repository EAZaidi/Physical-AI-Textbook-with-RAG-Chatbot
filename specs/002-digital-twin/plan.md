# Implementation Plan: Module 2 - The Digital Twin (Gazebo & Unity)

**Branch**: `002-digital-twin` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/002-digital-twin/spec.md`

## Summary

Module 2 teaches students to create digital twins of humanoid robots using Gazebo (physics simulation) and Unity (high-fidelity visualization). Students will learn physics simulation fundamentals, configure realistic sensor simulations (LiDAR, depth cameras, IMUs), and collect synthetic sensor data for algorithm testing. The module builds on Module 1's URDF models and ROS 2 knowledge, providing hands-on experience with industry-standard simulation tools.

**Technical Approach** (from research.md):
- **Gazebo Fortress** (primary) with optional Garden coverage for future-proofing
- **Unity 2022.3 LTS** with Unity Robotics Hub packages (URDF Importer, ROS TCP Connector)
- **Docker installation** for Gazebo (reproducibility), native installation for Unity (GPU requirements)
- **Balanced sensor fidelity**: GPU-accelerated LiDAR, realistic depth cameras, tiered IMU complexity
- **3-chapter structure** (15-24 pages total): Physics → Visualization → Sensors

## Technical Context

**Language/Version**: Python 3.11 (ROS 2 Humble), C# (Unity 2022.3 LTS scripting)
**Primary Dependencies**:
- Gazebo Fortress (osrf/ros:humble-desktop Docker base)
- Unity 2022.3 LTS with Unity Robotics Hub packages (com.unity.robotics.urdf-importer v0.7.0+)
- ROS 2 Humble (ros_gz bridge, sensor plugins)

**Storage**:
- File-based (Gazebo .world/.sdf files, Unity scenes, rosbag2 sensor data)
- Local filesystem for code examples, screenshots, Docker volumes

**Testing**:
- Manual validation (Gazebo GUI, Unity Play mode, RViz2 visualization)
- Automated: Docusaurus build tests, link validation
- Sensor accuracy tests (ground truth comparison for LiDAR ±5cm, depth camera ±2%)

**Target Platform**:
- Docusaurus MDX documentation (cross-platform web)
- Gazebo: Linux (Docker), Windows (WSL2 + Docker), macOS (Docker with X11)
- Unity: Windows/macOS/Linux native installations

**Project Type**: Educational documentation (Docusaurus textbook module)

**Performance Goals**:
- Gazebo simulations: 30+ FPS for basic humanoid robot in simple environment
- Unity scenes: 60 FPS on recommended hardware (GTX 1060 / RX 580 or better)
- LiDAR sensor: 10-20 Hz update rate with 360° coverage
- Build time: < 2 minutes for Docusaurus site generation

**Constraints**:
- Module length: 15-25 pages (constitution limit)
- All examples must be reproducible in Docker (Gazebo) and clean Unity projects
- Sensor simulation must use realistic parameters matching real-world hardware
- Cannot exceed student hardware capabilities (GPU-accelerated but not requiring RTX series)

**Scale/Scope**:
- 3 chapters, ~8 code examples, 15-20 screenshots
- 6-10 exercises with validation criteria
- Supports 50-100 concurrent student users on documentation site

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Source Accuracy & Verifiability ✅ PASS

**Status**: All technical claims traceable to official sources
- Gazebo configuration references: https://gazebosim.org/docs/fortress/
- Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- ROS 2 Gazebo integration: https://github.com/ros-simulation/gazebo_ros_pkgs
- Sensor plugins documented in official Gazebo/Unity docs
- Research.md includes 15+ citations to official documentation

**Action**: Verify all code examples against official tutorials before implementation

### II. Educational Clarity & Student Success ✅ PASS

**Status**: Content designed for progressive learning
- Clear learning objectives in each chapter (spec.md user stories)
- Conceptual explanations precede technical details (Chapter 1: physics theory → Gazebo practice)
- Step-by-step tutorials with expected outputs (exercise format defined in research.md)
- Troubleshooting sections planned for each chapter (FR-009)

**Action**: Include "Common Pitfalls" subsection in each chapter contract

### III. Reproducibility & Environment Consistency ✅ PASS

**Status**: Explicit environment specifications
- Docker configuration for Gazebo Fortress (Decision 3: research.md)
- Unity 2022.3 LTS version pinned (Decision 2: research.md)
- Dependency lists: ROS 2 Humble, gpu_ray plugin, URDF Importer v0.7.0+
- Validation steps planned (SC-006: 10-minute stable simulation test)

**Action**: Create Dockerfile and Unity package manifest in quickstart.md

### IV. Spec-Driven Content Development ✅ PASS

**Status**: Full Spec-Kit Plus workflow followed
- ✅ spec.md created (3 user stories, 12 functional requirements)
- ✅ research.md created (5 key technical decisions documented)
- ⏳ plan.md in progress (this file)
- 🔲 tasks.md pending (next phase: `/sp.tasks`)

**Action**: Continue systematic workflow through task generation and implementation

### V. RAG Chatbot Fidelity ✅ PASS

**Status**: Content structured for RAG indexing
- Each chapter is self-contained MDX file (easy chunking)
- Code examples inline with explanatory text (context preserved)
- Clear section headers for semantic chunking
- No external dependencies for content understanding

**Action**: After content creation, validate RAG retrieval with test queries (e.g., "How do I configure a LiDAR sensor in Gazebo?")

### VI. Modular Architecture & Progressive Complexity ✅ PASS

**Status**: Module 2 builds on Module 1, prepares for Module 3
- **Prerequisite**: Module 1 (ROS 2, URDF) explicitly stated in spec.md
- **Foundation for**: Module 3 (perception with VLMs requires sensor simulation knowledge)
- **Progressive complexity**: Chapter 1 (physics basics) → Chapter 2 (visual environments) → Chapter 3 (advanced sensors)
- **Self-contained**: Can be studied independently after Module 1 completion

**Action**: Cross-reference Module 1 URDF examples in Chapter 1 and 2

### VII. Production-Ready Deployment Standards ✅ PASS

**Status**: Infrastructure requirements aligned
- Docusaurus MDX format (existing CI/CD pipeline)
- Code examples version-locked (Gazebo Fortress, Unity 2022.3 LTS)
- No hardcoded credentials (Docker uses public osrf/ros image)
- Build validation required before merge (existing quality gate)

**Action**: Test Docusaurus build after each chapter completion

**Overall Constitution Compliance**: ✅ 7/7 PASS — Ready to proceed with implementation

## Project Structure

### Documentation (this feature)

```text
specs/002-digital-twin/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification (already created)
├── research.md          # Phase 0 output (already created)
├── data-model.md        # Phase 1 output (entity definitions)
├── quickstart.md        # Phase 1 output (environment setup guide)
├── contracts/           # Phase 1 output (chapter outlines)
│   ├── 01-gazebo-physics.md
│   ├── 02-unity-scenes.md
│   └── 03-sensor-simulation.md
├── checklists/
│   └── requirements.md  # Already validated (14/14 PASS)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
docs/
├── docs/
│   ├── module-1-ros2/          # Existing (Module 1)
│   └── module-2-digital-twin/  # NEW - This module
│       ├── index.mdx           # Module landing page
│       ├── 01-gazebo-physics.mdx
│       ├── 02-unity-scenes.mdx
│       ├── 03-sensor-simulation.mdx
│       └── assets/
│           ├── code-examples/
│           │   ├── simple_world.sdf
│           │   ├── humanoid_physics.urdf
│           │   ├── lidar_config.sdf
│           │   ├── collect_sensor_data.py
│           │   ├── plot_imu_data.py
│           │   └── README.md
│           ├── screenshots/
│           │   ├── gazebo-world-loaded.png
│           │   ├── humanoid-standing.png
│           │   ├── unity-scene-textured.png
│           │   ├── lidar-point-cloud.png
│           │   └── depth-image-comparison.png
│           └── diagrams/
│               ├── physics-engine-workflow.svg
│               └── sensor-coordinate-frames.svg
├── static/
├── docusaurus.config.js
└── package.json

docker/
├── gazebo-fortress/
│   ├── Dockerfile           # NEW - Gazebo Fortress + ROS 2 Humble
│   ├── docker-compose.yml
│   └── entrypoint.sh
└── README.md

scripts/
└── validate-sensor-accuracy.py  # NEW - Test LiDAR/depth camera accuracy
```

**Structure Decision**: This is a **documentation project** (Docusaurus textbook), not a standalone application. Source code exists only as educational examples within `docs/docs/module-2-digital-twin/assets/code-examples/`. The Docker directory provides reproducible Gazebo environments for students. Unity projects are not version-controlled (students create their own following tutorials).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations detected** — Constitution Check passed 7/7. No complexity justifications required.

## Implementation Phases

### Phase 0: Research ✅ COMPLETE

**Output**: `research.md` with 5 key technical decisions
- Decision 1: Gazebo Fortress (primary) + Garden (optional)
- Decision 2: Unity 2022.3 LTS + Robotics Hub packages
- Decision 3: Docker (Gazebo) + Native (Unity) installation
- Decision 4: Balanced sensor fidelity (GPU LiDAR, realistic depth, tiered IMU)
- Decision 5: 3-chapter structure with inline code examples

**Validation**: All decisions include rationale, alternatives considered, and official documentation sources

### Phase 1: Design Artifacts ⏳ IN PROGRESS

**Outputs**:
1. **data-model.md**: Entity definitions for key simulation concepts
2. **contracts/01-gazebo-physics.md**: Chapter 1 section-by-section outline
3. **contracts/02-unity-scenes.md**: Chapter 2 section-by-section outline
4. **contracts/03-sensor-simulation.md**: Chapter 3 section-by-section outline
5. **quickstart.md**: Docker setup + Unity installation guide

**Success Criteria**:
- Each chapter contract specifies learning objectives, sections (with page estimates), code examples, exercises, and screenshots
- quickstart.md provides one-command Docker setup for Gazebo and step-by-step Unity installation
- data-model.md defines 9 entities from spec.md with attributes and relationships

### Phase 2: Task Breakdown 🔲 PENDING

**Command**: `/sp.tasks` (not part of `/sp.plan`)

**Output**: `tasks.md` with testable implementation tasks
- Writing tasks (one per chapter)
- Code example creation tasks (Docker files, SDF/URDF configs, Python scripts)
- Screenshot capture tasks
- Validation tasks (build tests, sensor accuracy tests)

### Phase 3: Implementation 🔲 PENDING

**Command**: `/sp.implement` or manual execution

**Workflow**:
1. Write Chapter 1 (Gazebo Physics) following contract/01-gazebo-physics.md
2. Create code examples and Docker configuration
3. Capture screenshots in Gazebo
4. Write Chapter 2 (Unity Scenes) following contract/02-unity-scenes.md
5. Create Unity setup documentation
6. Capture Unity screenshots
7. Write Chapter 3 (Sensors) following contract/03-sensor-simulation.md
8. Create sensor data collection scripts
9. Validate all exercises and troubleshooting sections
10. Run Docusaurus build and fix any errors

## Architecture Decisions

### AD-001: Gazebo Version Selection

**Decision**: Use Gazebo Fortress as primary platform, with optional Garden coverage

**Rationale**:
- Fortress has official ROS 2 Humble binary packages (no compilation required)
- Garden requires building ros_gz from source (high friction for students)
- Fortress has larger community support and troubleshooting resources
- Garden naming change (Ignition → Gazebo) adds confusion for beginners

**Alternatives Rejected**:
- Gazebo Classic: Deprecated, poor ROS 2 integration
- Garden-only: Compilation issues, lack of binary packages
- Harmonic (latest): Too cutting-edge, insufficient educational resources

**Source**: research.md Decision 1

### AD-002: Unity LTS Version Selection

**Decision**: Unity 2022.3 LTS with verified Robotics Hub compatibility

**Rationale**:
- Unity Robotics Hub officially supports "unity-2020.2+" with testing on 2022 LTS
- 2022.3 LTS receives support until ~2025-2026 (sufficient for course lifetime)
- Stable APIs minimize breaking changes for educational content
- Contact Modification API and continuous collision detection fully supported

**Alternatives Rejected**:
- Unity 2023 LTS / Unity 6: No explicit Robotics Hub validation, potential compatibility issues
- Unity 2020 LTS: Approaching end-of-support, lacks robotics-specific improvements
- Non-LTS versions: Unstable APIs, frequent breaking changes

**Source**: research.md Decision 2

### AD-003: Installation Strategy

**Decision**: Docker for Gazebo, Native for Unity

**Rationale**:
- **Gazebo Docker**: Ensures reproducibility across Windows/macOS/Linux, minimal performance penalty on native Linux (~1-2 FPS), eliminates dependency conflicts
- **Unity Native**: GPU-intensive 3D engine requires DirectX/Metal/Vulkan access, containerization impractical

**Performance Data**:
- Docker on Linux: ~62 FPS (near-native)
- WSL2 + Docker: ~10 FPS (acceptable for basic tutorials, not optimal)
- Native Linux: ~62 FPS (best performance)

**Alternatives Rejected**:
- Native Gazebo for all: Increases setup complexity, platform-specific issues
- Docker for Unity: Impractical for GPU rendering, defeats visualization purpose
- Virtual machines: Significant overhead, unreliable GPU passthrough

**Source**: research.md Decision 3

### AD-004: Sensor Simulation Fidelity

**Decision**: Balanced fidelity prioritizing educational clarity

**LiDAR**: GPU-accelerated ray-based (Gazebo `gpu_ray`, Unity Raytraced Lidar)
- **Rationale**: 100x faster than CPU ray-casting, enables real-time multi-robot simulation, teaches realistic occlusion/range limits

**Depth Camera**: GPU-rendered with configurable Gaussian noise
- **Rationale**: Faster than ray-marching, sufficient for calibration and pixel-to-point tutorials, noise model teaches reality gap

**IMU**: Tiered complexity (Gaussian noise → Allan variance)
- **Rationale**: Beginners struggle with Allan variance; introduce after sensor fusion basics, simple Gaussian sufficient for 80% of use cases

**Alternatives Rejected**:
- CPU-based sensors: Too slow for real-time simulation
- Photorealistic ray-tracing: Overkill for educational purposes, requires high-end GPUs
- Perfect (noise-free) sensors: Fails to prepare students for real-world deployment

**Source**: research.md Decision 4

### AD-005: Content Structure

**Decision**: 3-chapter structure with inline code examples

**Chapter Organization**:
1. **Chapter 1: Gazebo Physics** (6-7 pages): World creation, URDF import, physics troubleshooting
2. **Chapter 2: Unity Scenes** (6-7 pages): Scene building, URDF import, interactive environments
3. **Chapter 3: Sensors** (5-6 pages): LiDAR, depth camera, IMU configuration and data collection

**Code Example Strategy**:
- **Inline** (80% of examples): SDF/URDF snippets (<30 lines), Python scripts (<40 lines), C# snippets (<25 lines)
- **Separate files** (20% of examples): Full world files, complete launch scripts, Unity scenes (GitHub reference)

**Rationale**:
- Inline examples easier to follow, explain line-by-line, and copy-paste
- Separate files reduce page clutter for complex systems
- Total 15-24 pages fits constitution limit
- Progressive complexity mirrors professional robotics workflow

**Alternatives Rejected**:
- Single long chapter: Cognitive overload, violates modular principle
- 5+ chapters: Exceeds page limit, unnecessary granularity for simulation basics
- External code only: Breaks reading flow, harder to explain context

**Source**: research.md Decision 5

## Risk Analysis

### Risk 1: Version Compatibility Drift

**Description**: Gazebo/Unity updates may break tutorials after publication

**Likelihood**: Medium (Gazebo/Unity release every 6-12 months)

**Impact**: High (broken tutorials frustrate students, damage credibility)

**Mitigation**:
- Pin exact versions (Gazebo Fortress, Unity 2022.3 LTS) in all documentation
- Use Docker for Gazebo (immutable environment)
- Test all examples in clean environments before publication
- Add "Version Notes" section warning about compatibility
- Plan annual review and update cycle

**Owner**: Content maintainer

### Risk 2: Student Hardware Limitations

**Description**: Students with low-end GPUs cannot run Unity or Gazebo simulations

**Likelihood**: Medium (15-20% of students may have integrated graphics only)

**Impact**: Medium (students excluded from hands-on exercises)

**Mitigation**:
- Provide Docker Gazebo option (runs on CPU with reduced performance)
- Recommend Unity quality settings adjustments (disable shadows, reduce resolution)
- Offer cloud-based alternatives in troubleshooting section (AWS RoboMaker, Google Colab for Gazebo)
- Set minimum hardware requirements clearly in quickstart.md

**Owner**: Content author + student support

### Risk 3: Cross-Platform Setup Inconsistencies

**Description**: Docker/Unity behave differently on Windows/macOS/Linux

**Likelihood**: High (Windows WSL2 has known GUI limitations)

**Impact**: Medium (increased troubleshooting burden, slower student progress)

**Mitigation**:
- Prioritize Linux workflows (most stable Docker + Gazebo)
- Document platform-specific issues (WSL2 X server, macOS Rosetta for M1/M2)
- Provide platform-specific troubleshooting subsections
- Test on all three platforms before publication

**Owner**: Content author + QA tester

## Next Steps

### Immediate (Phase 1 Completion)

1. **Create data-model.md**: Define 9 entities (Gazebo World, Unity Scene, Physics Engine, Sensor Plugin, Robot Model, Material Properties, Point Cloud, Depth Image, IMU Data)
2. **Create contracts/01-gazebo-physics.md**: Section-by-section outline with page estimates, code examples, exercises
3. **Create contracts/02-unity-scenes.md**: Same structure as above
4. **Create contracts/03-sensor-simulation.md**: Same structure as above
5. **Create quickstart.md**: Docker setup (Dockerfile + docker-compose.yml), Unity installation steps, validation commands

### Phase 2: Task Breakdown

6. **Run `/sp.tasks`**: Generate testable implementation tasks from contracts and data model

### Phase 3: Implementation

7. **Write chapters sequentially**: Follow contracts, create code examples, capture screenshots
8. **Validate continuously**: Test Docker setup, run Docusaurus builds, verify sensor accuracy
9. **Create PHRs**: Document progress and decisions

### Phase 4: Validation

10. **Constitution compliance check**: Re-verify all 7 principles
11. **Peer review**: Robotics educator with no prior Gazebo/Unity experience
12. **Build and deploy**: Merge to main, deploy to GitHub Pages

## Post-Implementation Constitution Check

*To be filled after Phase 1 design completion*

**Status**: ⏳ PENDING — Will re-evaluate after data-model.md and contracts/ are created

**Expected Result**: No violations anticipated, all design decisions align with research and constitution

---

**Plan Status**: ✅ Phase 0 COMPLETE | ⏳ Phase 1 IN PROGRESS | 🔲 Phase 2 PENDING
**Next Action**: Create data-model.md and contracts/ (continuing Phase 1)
