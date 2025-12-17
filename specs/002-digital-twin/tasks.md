# Tasks: Module 2 - The Digital Twin (Gazebo & Unity)

**Input**: Design documents from `/specs/002-digital-twin/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/, quickstart.md

**Tests**: This project uses manual validation (Gazebo GUI, Unity Play mode, RViz2 visualization) and Docusaurus build tests. No automated unit/integration tests for educational content.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each chapter.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story/chapter this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is a **Docusaurus documentation project**:
- **Module content**: `docs/docs/module-2-digital-twin/`
- **Code examples**: `docs/docs/module-2-digital-twin/assets/code-examples/`
- **Screenshots**: `docs/docs/module-2-digital-twin/assets/screenshots/`
- **Docker files**: `docker/gazebo-fortress/`
- **Validation scripts**: `scripts/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create directory structure and foundational files for Module 2

- [x] T001 Create module directory structure at docs/docs/module-2-digital-twin/
- [x] T002 Create assets subdirectories (code-examples, screenshots, diagrams) in docs/docs/module-2-digital-twin/assets/
- [x] T003 [P] Create Docker directory for Gazebo at docker/gazebo-fortress/
- [x] T004 [P] Create scripts directory for validation at scripts/

---

## Phase 2: Foundational (Docker & Environment Setup)

**Purpose**: Docker configuration and environment setup that MUST be complete before chapter writing

**⚠️ CRITICAL**: Chapters 1 and 3 require working Docker/Gazebo setup for code examples and screenshots

- [x] T005 Create Dockerfile for Gazebo Fortress + ROS 2 Humble at docker/gazebo-fortress/Dockerfile
- [x] T006 Create docker-compose.yml for Gazebo container at docker/gazebo-fortress/docker-compose.yml
- [x] T007 Create run_gazebo.sh launcher script at docker/gazebo-fortress/run_gazebo.sh
- [ ] T008 [P] Test Docker setup on Linux (verify Gazebo GUI launches) - MANUAL VALIDATION REQUIRED
- [x] T009 [P] Create README.md for Docker setup at docker/gazebo-fortress/README.md

**Checkpoint**: Docker environment ready - Chapter 1 and 3 writing can begin

---

## Phase 3: User Story 1 - Physics Simulation Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Students learn core physics simulation concepts and can create functional Gazebo worlds with accurate physics behavior for humanoid robots.

**Independent Test**: Students can create a Gazebo world, configure physics parameters (gravity, friction), import URDF humanoid, and demonstrate stable simulation for 10+ minutes.

### Code Examples for User Story 1

- [x] T010 [P] [US1] Create simple_world.sdf example at docs/docs/module-2-digital-twin/assets/code-examples/simple_world.sdf
- [x] T011 [P] [US1] Create humanoid_physics.urdf with inertial properties at docs/docs/module-2-digital-twin/assets/code-examples/humanoid_physics.urdf
- [x] T012 [P] [US1] Create launch_gazebo.sh launcher at docs/docs/module-2-digital-twin/assets/code-examples/launch_gazebo.sh
- [x] T013 [P] [US1] Create physics_tuning.sdf example with tuned parameters at docs/docs/module-2-digital-twin/assets/code-examples/physics_tuning.sdf

### Screenshots for User Story 1

- [ ] T014 [P] [US1] Capture screenshot: Empty Gazebo world in Docker at docs/docs/module-2-digital-twin/assets/screenshots/01-empty-gazebo.png
- [ ] T015 [P] [US1] Capture screenshot: Simple world with ground plane at docs/docs/module-2-digital-twin/assets/screenshots/01-simple-world.png
- [ ] T016 [P] [US1] Capture screenshot: Sphere falling under gravity (composite) at docs/docs/module-2-digital-twin/assets/screenshots/01-gravity-test.png
- [ ] T017 [P] [US1] Capture screenshot: Humanoid standing (correct inertia) at docs/docs/module-2-digital-twin/assets/screenshots/01-humanoid-standing.png
- [ ] T018 [P] [US1] Capture screenshot: Humanoid collapsed (incorrect inertia) at docs/docs/module-2-digital-twin/assets/screenshots/01-humanoid-collapsed.png
- [ ] T019 [P] [US1] Capture screenshot: Jitter fix before/after at docs/docs/module-2-digital-twin/assets/screenshots/01-jitter-fix.png
- [ ] T020 [P] [US1] Capture screenshot: Robot collision with obstacle at docs/docs/module-2-digital-twin/assets/screenshots/01-collision-test.png

### Chapter 1 Writing

- [x] T021 [US1] Write Chapter 1 Section 1: Introduction (0.5 pages) in docs/docs/module-2-digital-twin/01-gazebo-physics.mdx
- [x] T022 [US1] Write Chapter 1 Section 2: Gazebo Setup with Docker (1 page) in docs/docs/module-2-digital-twin/01-gazebo-physics.mdx
- [x] T023 [US1] Write Chapter 1 Section 3: Creating Your First World (1.5 pages) in docs/docs/module-2-digital-twin/01-gazebo-physics.mdx
- [x] T024 [US1] Write Chapter 1 Section 4: Importing Humanoid Models (1.5 pages) in docs/docs/module-2-digital-twin/01-gazebo-physics.mdx
- [x] T025 [US1] Write Chapter 1 Section 5: Physics Troubleshooting (1.5 pages) in docs/docs/module-2-digital-twin/01-gazebo-physics.mdx
- [x] T026 [US1] Write Chapter 1 Section 6: Exercises (0.5 pages) in docs/docs/module-2-digital-twin/01-gazebo-physics.mdx

### Validation for User Story 1

- [x] T027 [US1] Validate Chapter 1 page count (target: 6-7 pages) in docs/docs/module-2-digital-twin/01-gazebo-physics.mdx
- [ ] T028 [US1] Validate all code examples run in Docker environment (simple_world.sdf, humanoid_physics.urdf) - MANUAL
- [ ] T029 [US1] Validate all screenshots referenced in Chapter 1 exist and display correctly - MANUAL
- [ ] T030 [US1] Run Docusaurus build test for Chapter 1 (npm run build) - MANUAL

**Checkpoint**: Chapter 1 complete and independently testable. Students can create Gazebo worlds and simulate humanoid physics.

---

## Phase 4: User Story 3 - Sensor Simulation and Data Collection (Priority: P1)

**Goal**: Students accurately simulate sensors (LiDAR, depth cameras, IMUs) and collect synthetic sensor data for algorithm testing and validation.

**Independent Test**: Students can add LiDAR/depth camera to simulated robot, collect point cloud or depth image data, export to PCD/PNG formats, and verify accuracy against ground truth (±5cm for LiDAR, ±2% for depth).

**Note**: US3 scheduled before US2 because sensor simulation (US3) builds on Gazebo knowledge from US1, while Unity (US2) is a separate track.

### Code Examples for User Story 3

- [x] T031 [P] [US3] Create lidar_config.sdf with gpu_ray sensor at docs/docs/module-2-digital-twin/assets/code-examples/lidar_config.sdf
- [x] T032 [P] [US3] Create depth_camera_config.sdf with depth camera plugin at docs/docs/module-2-digital-twin/assets/code-examples/depth_camera_config.sdf
- [x] T033 [P] [US3] Create imu_config.sdf with IMU sensor at docs/docs/module-2-digital-twin/assets/code-examples/imu_config.sdf
- [x] T034 [P] [US3] Create collect_sensor_data.py ROS 2 node at docs/docs/module-2-digital-twin/assets/code-examples/collect_sensor_data.py
- [x] T035 [P] [US3] Create plot_imu_data.py matplotlib script at docs/docs/module-2-digital-twin/assets/code-examples/plot_imu_data.py
- [x] T036 [P] [US3] Create export_pointcloud.py (sensor_msgs to PCD) at docs/docs/module-2-digital-twin/assets/code-examples/export_pointcloud.py
- [x] T037 [P] [US3] Create export_imu_csv.py (IMU to CSV) at docs/docs/module-2-digital-twin/assets/code-examples/export_imu_csv.py
- [x] T038 [P] [US3] Create README.md for code examples at docs/docs/module-2-digital-twin/assets/code-examples/README.md

### Screenshots for User Story 3

- [ ] T039 [P] [US3] Capture screenshot: RViz2 with LiDAR point cloud (Gazebo) at docs/docs/module-2-digital-twin/assets/screenshots/03-lidar-rviz.png
- [ ] T040 [P] [US3] Capture screenshot: Unity Scene view with LiDAR rays at docs/docs/module-2-digital-twin/assets/screenshots/03-lidar-unity.png
- [ ] T041 [P] [US3] Capture screenshot: RViz2 RGB + depth images side-by-side at docs/docs/module-2-digital-twin/assets/screenshots/03-depth-rviz.png
- [ ] T042 [P] [US3] Capture screenshot: Unity depth visualization at docs/docs/module-2-digital-twin/assets/screenshots/03-depth-unity.png
- [ ] T043 [P] [US3] Create plot: IMU data over 10 seconds (accel + gyro) at docs/docs/module-2-digital-twin/assets/screenshots/03-imu-plot.png
- [ ] T044 [P] [US3] Capture screenshot: Exported files (PCD, CSV) in file explorer at docs/docs/module-2-digital-twin/assets/screenshots/03-exported-files.png
- [ ] T045 [P] [US3] Capture screenshot: RViz2 with wall point cloud (Exercise 1) at docs/docs/module-2-digital-twin/assets/screenshots/03-exercise-lidar.png

### Chapter 3 Writing

- [x] T046 [US3] Write Chapter 3 Section 1: Introduction (0.5 pages) in docs/docs/module-2-digital-twin/03-sensor-simulation.mdx
- [x] T047 [US3] Write Chapter 3 Section 2: LiDAR Simulation (1.5 pages) in docs/docs/module-2-digital-twin/03-sensor-simulation.mdx
- [x] T048 [US3] Write Chapter 3 Section 3: Depth Camera Simulation (1.5 pages) in docs/docs/module-2-digital-twin/03-sensor-simulation.mdx
- [x] T049 [US3] Write Chapter 3 Section 4: IMU Simulation (1 page) in docs/docs/module-2-digital-twin/03-sensor-simulation.mdx
- [x] T050 [US3] Write Chapter 3 Section 5: Data Collection & Export (1 page) in docs/docs/module-2-digital-twin/03-sensor-simulation.mdx
- [x] T051 [US3] Write Chapter 3 Section 6: Exercises (0.5 pages) in docs/docs/module-2-digital-twin/03-sensor-simulation.mdx

### Validation for User Story 3

- [x] T052 [US3] Validate Chapter 3 page count (target: 5-6 pages) in docs/docs/module-2-digital-twin/03-sensor-simulation.mdx
- [ ] T053 [US3] Validate sensor accuracy tests (LiDAR ±5cm, depth camera ±2%) with test scripts - MANUAL
- [ ] T054 [US3] Validate all Python scripts run with ROS 2 Humble (collect_sensor_data.py, export scripts) - MANUAL
- [ ] T055 [US3] Validate exported files (PCD opens in CloudCompare, CSV opens in Excel) - MANUAL
- [ ] T056 [US3] Run Docusaurus build test for Chapter 3 (npm run build) - MANUAL

**Checkpoint**: Chapter 3 complete and independently testable. Students can simulate sensors and export data for algorithm development.

---

## Phase 5: User Story 2 - High-Fidelity Visual Environments (Priority: P2)

**Goal**: Students create realistic Unity scenes with interactive environments for testing humanoid robot perception and navigation algorithms.

**Independent Test**: Students can build a Unity scene with textured environment, lighting, interactive objects, import URDF humanoid, and achieve 60+ FPS on recommended hardware.

### Code Examples for User Story 2

- [x] T057 [P] [US2] Create CameraController.cs script at docs/docs/module-2-digital-twin/assets/code-examples/CameraController.cs
- [x] T058 [P] [US2] Create SimpleJointController.cs script at docs/docs/module-2-digital-twin/assets/code-examples/SimpleJointController.cs
- [x] T059 [P] [US2] Create PushDetector.cs script at docs/docs/module-2-digital-twin/assets/code-examples/PushDetector.cs
- [x] T060 [P] [US2] Create unity_project_setup.md guide at docs/docs/module-2-digital-twin/assets/code-examples/unity_project_setup.md

### Screenshots for User Story 2

- [ ] T061 [P] [US2] Capture screenshot: Unity Hub with 2022.3 LTS installed at docs/docs/module-2-digital-twin/assets/screenshots/02-unity-hub.png
- [ ] T062 [P] [US2] Capture screenshot: Empty Unity project (SampleScene) at docs/docs/module-2-digital-twin/assets/screenshots/02-empty-project.png
- [ ] T063 [P] [US2] Capture screenshot: Untextured room in Scene view at docs/docs/module-2-digital-twin/assets/screenshots/02-scene-untextured.png
- [ ] T064 [P] [US2] Capture screenshot: Textured room in Game view with lighting at docs/docs/module-2-digital-twin/assets/screenshots/02-scene-textured.png
- [ ] T065 [P] [US2] Capture screenshot: Unity Package Manager with URDF Importer at docs/docs/module-2-digital-twin/assets/screenshots/02-package-manager.png
- [ ] T066 [P] [US2] Capture screenshot: Humanoid robot with ArticulationBody Inspector at docs/docs/module-2-digital-twin/assets/screenshots/02-robot-inspector.png
- [ ] T067 [P] [US2] Capture screenshot: Robot pushing box (contact interaction) at docs/docs/module-2-digital-twin/assets/screenshots/02-robot-push.png
- [ ] T068 [P] [US2] Capture screenshot: Furnished apartment scene (Exercise 1 example) at docs/docs/module-2-digital-twin/assets/screenshots/02-apartment-scene.png

### Chapter 2 Writing

- [x] T069 [US2] Write Chapter 2 Section 1: Introduction (0.5 pages) in docs/docs/module-2-digital-twin/02-unity-scenes.mdx
- [x] T070 [US2] Write Chapter 2 Section 2: Unity Setup and Project Creation (1 page) in docs/docs/module-2-digital-twin/02-unity-scenes.mdx
- [x] T071 [US2] Write Chapter 2 Section 3: Building Your First Scene (1.5 pages) in docs/docs/module-2-digital-twin/02-unity-scenes.mdx
- [x] T072 [US2] Write Chapter 2 Section 4: Importing Humanoid Robots (1.5 pages) in docs/docs/module-2-digital-twin/02-unity-scenes.mdx
- [x] T073 [US2] Write Chapter 2 Section 5: Interactive Elements (1 page) in docs/docs/module-2-digital-twin/02-unity-scenes.mdx
- [x] T074 [US2] Write Chapter 2 Section 6: Troubleshooting & Optimization (1 page) in docs/docs/module-2-digital-twin/02-unity-scenes.mdx
- [x] T075 [US2] Write Chapter 2 Section 7: Exercises (0.5 pages) in docs/docs/module-2-digital-twin/02-unity-scenes.mdx

### Validation for User Story 2

- [x] T076 [US2] Validate Chapter 2 page count (target: 6-7 pages) in docs/docs/module-2-digital-twin/02-unity-scenes.mdx
- [ ] T077 [US2] Validate Unity installation and package import (Unity 2022.3 LTS, URDF Importer v0.7.0+) - MANUAL
- [ ] T078 [US2] Validate C# scripts compile without errors in Unity Editor - MANUAL
- [ ] T079 [US2] Validate performance target (60+ FPS in simple scene on recommended hardware) - MANUAL
- [ ] T080 [US2] Run Docusaurus build test for Chapter 2 (npm run build) - MANUAL

**Checkpoint**: Chapter 2 complete and independently testable. Students can create Unity scenes and import URDF models.

---

## Phase 6: Module Landing Page and Integration

**Purpose**: Create module landing page and ensure all chapters integrate smoothly

- [x] T081 Create module landing page at docs/docs/module-2-digital-twin/index.mdx
- [x] T082 Add navigation links to all 3 chapters in docs/docs/module-2-digital-twin/index.mdx
- [x] T083 Add module overview and learning objectives in docs/docs/module-2-digital-twin/index.mdx
- [x] T084 Add prerequisites section (Module 1 completion) in docs/docs/module-2-digital-twin/index.mdx
- [x] T085 Update Docusaurus sidebar configuration for Module 2 in docs/docusaurus.config.js or docs/sidebars.js

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final validation, cleanup, and quality assurance for Module 2

### Documentation & Cleanup

- [ ] T086 [P] Create module-level README.md at docs/docs/module-2-digital-twin/README.md - OPTIONAL
- [x] T087 [P] Review all chapters for consistent formatting (MDX syntax, code blocks, screenshots)
- [x] T088 [P] Verify all external links point to official documentation (Gazebo, Unity, ROS 2)
- [ ] T089 [P] Add diagrams (physics workflow, sensor frames) at docs/docs/module-2-digital-twin/assets/diagrams/ - OPTIONAL

### Validation & Testing

- [ ] T090 Run complete Docusaurus build test (npm run build) for entire site - MANUAL
- [x] T091 Validate all internal links within Module 2 (between chapters, to code examples)
- [ ] T092 Validate all screenshots display correctly in built site - BLOCKED (screenshots not captured)
- [x] T093 Validate total module page count (target: 15-24 pages per constitution)
- [ ] T094 [P] Test Docker setup on Windows WSL2 (verify Gazebo GUI with VcXsrv/WSLg) - MANUAL
- [ ] T095 [P] Test Docker setup on macOS (verify Gazebo GUI with XQuartz) - MANUAL
- [ ] T096 Validate quickstart.md instructions accuracy (test on clean Ubuntu 22.04 VM) - MANUAL

### Constitution Compliance Check

- [x] T097 Verify Source Accuracy: All technical claims link to official docs (Gazebo, Unity, ROS 2)
- [x] T098 Verify Educational Clarity: All chapters have learning objectives, troubleshooting, expected outputs
- [ ] T099 Verify Reproducibility: All code examples tested in documented Docker/Unity environments - MANUAL
- [x] T100 Verify Spec-Driven Development: All requirements from spec.md addressed in chapters
- [x] T101 Verify RAG Chatbot Fidelity: Content structured for chunking (clear headers, self-contained sections)
- [x] T102 Verify Modular Architecture: Module 2 references Module 1, prepares for Module 3
- [ ] T103 Verify Production Standards: Docusaurus build succeeds, no hardcoded paths, version-locked dependencies - MANUAL

### Final Deployment Preparation

- [x] T104 Run link checker for broken external links (npm run check-links or manual validation)
- [ ] T105 Optimize all screenshots for web (compress PNG files, target <500KB per image)
- [ ] T106 Create BUILD_SUCCESS_REPORT.md for Module 2 at specs/002-digital-twin/BUILD_SUCCESS_REPORT.md
- [ ] T107 Update main project README.md to include Module 2 completion status

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (Phase 1) - BLOCKS Chapters 1 & 3 (Gazebo-dependent)
- **User Story 1 - Chapter 1 (Phase 3)**: Depends on Foundational (Phase 2)
- **User Story 3 - Chapter 3 (Phase 4)**: Depends on Foundational (Phase 2) - can run parallel with US1
- **User Story 2 - Chapter 2 (Phase 5)**: Depends on Setup (Phase 1) only - can run parallel with US1/US3
- **Landing Page (Phase 6)**: Depends on all chapters (Phase 3, 4, 5) completion
- **Polish (Phase 7)**: Depends on all content (Phase 6) completion

### User Story Dependencies

- **User Story 1 (P1) - Chapter 1**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 3 (P1) - Chapter 3**: Can start after Foundational (Phase 2) - No dependencies on other stories (parallel with US1)
- **User Story 2 (P2) - Chapter 2**: Can start after Setup (Phase 1) - Independent of Gazebo setup (parallel with US1/US3)

### Within Each User Story

- Code examples and screenshots can be created in parallel (all marked [P])
- Chapter writing follows linear section order (Section 1 → 2 → 3 → etc.)
- Validation tasks run after chapter writing completes

### Parallel Opportunities

**Maximum Parallelism** (with 3 developers):
- Developer A: Phase 3 (Chapter 1 - Gazebo Physics) after Phase 2 complete
- Developer B: Phase 4 (Chapter 3 - Sensor Simulation) after Phase 2 complete
- Developer C: Phase 5 (Chapter 2 - Unity Scenes) after Phase 1 complete (can start earlier!)

**Within Each Chapter**:
- All code examples marked [P] can be created simultaneously (different files)
- All screenshots marked [P] can be captured simultaneously (different scenes/configurations)

---

## Parallel Example: User Story 1 (Chapter 1)

```bash
# Launch all code examples for Chapter 1 together:
Task: "Create simple_world.sdf example at docs/docs/module-2-digital-twin/assets/code-examples/simple_world.sdf"
Task: "Create humanoid_physics.urdf with inertial properties at docs/docs/module-2-digital-twin/assets/code-examples/humanoid_physics.urdf"
Task: "Create launch_gazebo.sh launcher at docs/docs/module-2-digital-twin/assets/code-examples/launch_gazebo.sh"
Task: "Create physics_tuning.sdf example with tuned parameters at docs/docs/module-2-digital-twin/assets/code-examples/physics_tuning.sdf"

# Launch all screenshots for Chapter 1 together (after code examples ready):
Task: "Capture screenshot: Empty Gazebo world in Docker at docs/docs/module-2-digital-twin/assets/screenshots/01-empty-gazebo.png"
Task: "Capture screenshot: Simple world with ground plane at docs/docs/module-2-digital-twin/assets/screenshots/01-simple-world.png"
Task: "Capture screenshot: Sphere falling under gravity (composite) at docs/docs/module-2-digital-twin/assets/screenshots/01-gravity-test.png"
Task: "Capture screenshot: Humanoid standing (correct inertia) at docs/docs/module-2-digital-twin/assets/screenshots/01-humanoid-standing.png"
Task: "Capture screenshot: Humanoid collapsed (incorrect inertia) at docs/docs/module-2-digital-twin/assets/screenshots/01-humanoid-collapsed.png"
Task: "Capture screenshot: Jitter fix before/after at docs/docs/module-2-digital-twin/assets/screenshots/01-jitter-fix.png"
Task: "Capture screenshot: Robot collision with obstacle at docs/docs/module-2-digital-twin/assets/screenshots/01-collision-test.png"
```

---

## Parallel Example: User Story 3 (Chapter 3)

```bash
# Launch all code examples for Chapter 3 together:
Task: "Create lidar_config.sdf with gpu_ray sensor at docs/docs/module-2-digital-twin/assets/code-examples/lidar_config.sdf"
Task: "Create depth_camera_config.sdf with depth camera plugin at docs/docs/module-2-digital-twin/assets/code-examples/depth_camera_config.sdf"
Task: "Create imu_config.sdf with IMU sensor at docs/docs/module-2-digital-twin/assets/code-examples/imu_config.sdf"
Task: "Create collect_sensor_data.py ROS 2 node at docs/docs/module-2-digital-twin/assets/code-examples/collect_sensor_data.py"
Task: "Create plot_imu_data.py matplotlib script at docs/docs/module-2-digital-twin/assets/code-examples/plot_imu_data.py"
Task: "Create export_pointcloud.py (sensor_msgs to PCD) at docs/docs/module-2-digital-twin/assets/code-examples/export_pointcloud.py"
Task: "Create export_imu_csv.py (IMU to CSV) at docs/docs/module-2-digital-twin/assets/code-examples/export_imu_csv.py"
Task: "Create README.md for code examples at docs/docs/module-2-digital-twin/assets/code-examples/README.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only - Chapter 1)

1. Complete Phase 1: Setup (T001-T004)
2. Complete Phase 2: Foundational (T005-T009) - Docker setup CRITICAL
3. Complete Phase 3: User Story 1 (T010-T030) - Chapter 1: Gazebo Physics
4. **STOP and VALIDATE**: Test Chapter 1 independently
   - Verify students can follow Docker setup
   - Verify all code examples run successfully
   - Verify Docusaurus builds without errors
5. Deploy Chapter 1 to staging for student testing

### Incremental Delivery

1. Complete Setup (Phase 1) + Foundational (Phase 2) → Docker environment ready
2. Add Chapter 1 (Phase 3) → Test independently → Deploy/Demo (MVP!)
3. Add Chapter 3 (Phase 4) → Test independently → Deploy/Demo
4. Add Chapter 2 (Phase 5) → Test independently → Deploy/Demo
5. Add Landing Page (Phase 6) → Integrate all chapters
6. Polish & Validate (Phase 7) → Final quality assurance

### Parallel Team Strategy

With multiple developers:

1. **Week 1**: Team completes Setup + Foundational together (T001-T009)
2. **Week 2-3**: Once Foundational is done:
   - **Developer A**: Chapter 1 (T010-T030) - Gazebo Physics
   - **Developer B**: Chapter 3 (T031-T056) - Sensor Simulation
   - **Developer C**: Chapter 2 (T057-T080) - Unity Scenes (can start in Week 1!)
3. **Week 4**: Landing Page (T081-T085) + Polish (T086-T107)
4. Chapters complete and integrate independently

---

## Task Summary

**Total Tasks**: 107

**Tasks by Phase**:
- Phase 1 (Setup): 4 tasks
- Phase 2 (Foundational): 5 tasks
- Phase 3 (User Story 1 - Chapter 1): 21 tasks
  - Code examples: 4 tasks
  - Screenshots: 7 tasks
  - Chapter writing: 6 tasks
  - Validation: 4 tasks
- Phase 4 (User Story 3 - Chapter 3): 26 tasks
  - Code examples: 8 tasks
  - Screenshots: 7 tasks
  - Chapter writing: 6 tasks
  - Validation: 5 tasks
- Phase 5 (User Story 2 - Chapter 2): 24 tasks
  - Code examples: 4 tasks
  - Screenshots: 8 tasks
  - Chapter writing: 7 tasks
  - Validation: 5 tasks
- Phase 6 (Landing Page): 5 tasks
- Phase 7 (Polish): 22 tasks

**Parallel Opportunities**:
- **Setup**: 2 tasks can run in parallel (T003, T004)
- **Foundational**: 2 tasks can run in parallel (T008, T009)
- **Chapter 1**: 11 parallel tasks (4 code examples + 7 screenshots)
- **Chapter 3**: 15 parallel tasks (8 code examples + 7 screenshots)
- **Chapter 2**: 12 parallel tasks (4 code examples + 8 screenshots)
- **Polish**: 7 parallel tasks (documentation, validation on different platforms)

**Independent Test Criteria**:
- **User Story 1**: Students create Gazebo world, import URDF, stable 10-minute simulation
- **User Story 3**: Students collect sensor data, export to PCD/PNG/CSV, verify ±5cm LiDAR accuracy
- **User Story 2**: Students build Unity scene, import URDF, achieve 60+ FPS

**Suggested MVP Scope**: User Story 1 (Chapter 1: Gazebo Physics) - Phases 1, 2, and 3 (T001-T030)

---

## Notes

- [P] tasks = different files, no dependencies - can run simultaneously
- [Story] label maps task to specific user story/chapter for traceability
- Each user story/chapter is independently completable and testable
- No automated unit tests - validation via manual testing (Gazebo GUI, Unity Play, RViz2) and Docusaurus builds
- Commit after each task or logical group (e.g., all Chapter 1 screenshots)
- Stop at any checkpoint to validate chapter independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- Docker setup (Phase 2) is critical blocker for Chapters 1 and 3
- Unity setup (Chapter 2) can proceed independently - only needs Phase 1 setup

---

**Format Validation**: ✅ All tasks follow checklist format with checkboxes, IDs, [P] markers where applicable, [Story] labels for user story phases, and exact file paths.
