---

description: "Task list for Module 1 - The Robotic Nervous System (ROS 2)"
---

# Tasks: Module 1 - The Robotic Nervous System (ROS 2)

**Input**: Design documents from `/specs/001-ros2-module/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/, quickstart.md

**Tests**: No tests requested in specification - tasks focus on educational content creation and validation

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus content**: `docs/module-1-ros2/` at repository root
- **Code examples**: `docs/module-1-ros2/assets/code-examples/`
- **Diagrams**: `docs/module-1-ros2/assets/diagrams/`
- **Docker testing**: `docker/ros2-testing/`
- **CI/CD**: `.github/workflows/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for Module 1 content

- [x] T001 Create Docusaurus module directory structure at docs/module-1-ros2/
- [x] T002 Create assets subdirectories at docs/module-1-ros2/assets/code-examples/ and docs/module-1-ros2/assets/diagrams/
- [x] T003 [P] Initialize sidebar configuration for Module 1 in docusaurus.config.js
- [x] T004 [P] Create Docker testing environment at docker/ros2-testing/Dockerfile based on osrf/ros:humble-desktop

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story content can be written

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Create test script at scripts/test-code-examples.sh for Docker-based code validation
- [x] T006 [P] Create GitHub Actions workflow at .github/workflows/docusaurus-build.yml for build validation
- [x] T007 [P] Create GitHub Actions workflow at .github/workflows/test-code-examples.yml for code testing
- [x] T008 Document code example testing procedure in specs/001-ros2-module/quickstart.md (append section)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Understanding ROS 2 Architecture (Priority: P1) 🎯 MVP

**Goal**: Students understand ROS 2 architecture, DDS middleware, and communication patterns (topics vs services vs actions)

**Independent Test**: Student can explain the ROS 2 graph (nodes, topics, services) and draw a communication diagram for a simple robot system

### Content for User Story 1

- [x] T009 [P] [US1] Write Chapter 1 content at docs/module-1-ros2/01-overview-architecture.mdx following contracts/chapter-1-outline.md
- [x] T010 [P] [US1] Write Chapter 2 content at docs/module-1-ros2/02-nodes-topics-services.mdx following contracts/chapter-2-outline.md
- [x] T011 [P] [US1] Create ROS 1 vs ROS 2 architecture diagram at docs/module-1-ros2/assets/diagrams/ros1-vs-ros2-architecture.svg
- [x] T012 [P] [US1] Create ROS 2 graph example diagram at docs/module-1-ros2/assets/diagrams/ros2-graph-example.svg
- [x] T013 [P] [US1] Create communication pattern decision tree diagram at docs/module-1-ros2/assets/diagrams/communication-patterns-decision-tree.svg
- [x] T014 [US1] Add all Chapter 1 documentation links to official ROS 2 sources (ros.org, design.ros2.org)
- [x] T015 [US1] Add all Chapter 2 documentation links to official ROS 2 sources
- [x] T016 [US1] Write practice exercises for Chapter 1 (identify communication patterns, draw ROS 2 graph)
- [x] T017 [US1] Write practice exercises for Chapter 2 (use CLI tools, identify message types)
- [x] T018 [US1] Validate Chapter 1 content against constitution Principle I (source accuracy) and Principle II (educational clarity)
- [x] T019 [US1] Validate Chapter 2 content against constitution Principle I and Principle II

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently. Students can read Chapters 1-2 and understand ROS 2 architecture.

---

## Phase 4: User Story 2 - Writing ROS 2 Nodes and Publishers (Priority: P1)

**Goal**: Students can write Python nodes using rclpy to create publishers, subscribers, and service servers/clients

**Independent Test**: Student writes a Python node that publishes mock sensor data at 10 Hz and another node that subscribes to the data and logs it. Both nodes run successfully in separate terminals.

### Content for User Story 2

- [x] T020 [US2] Write Chapter 3 content at docs/module-1-ros2/03-python-rclpy-control.mdx following contracts/chapter-3-outline.md
- [x] T021 [US2] Add all Chapter 3 documentation links to official ROS 2 sources (rclpy API, tutorials)

### Code Examples for User Story 2

- [x] T022 [P] [US2] Create publisher_example.py at docs/module-1-ros2/assets/code-examples/publisher_example.py (publishes String messages at 1 Hz)
- [x] T023 [P] [US2] Create subscriber_example.py at docs/module-1-ros2/assets/code-examples/subscriber_example.py (subscribes to /chatter topic)
- [x] T024 [P] [US2] Create service_server_example.py at docs/module-1-ros2/assets/code-examples/service_server_example.py (AddTwoInts service)
- [x] T025 [P] [US2] Create service_client_example.py at docs/module-1-ros2/assets/code-examples/service_client_example.py (calls AddTwoInts service)
- [x] T026 [P] [US2] Create joint_state_publisher.py at docs/module-1-ros2/assets/code-examples/joint_state_publisher.py (publishes sensor_msgs/JointState)
- [x] T027 [P] [US2] Create joint_state_subscriber.py at docs/module-1-ros2/assets/code-examples/joint_state_subscriber.py (subscribes to joint states)

### Code Validation for User Story 2

- [x] T028 [US2] Test publisher_example.py in Docker environment using scripts/test-code-examples.sh
- [x] T029 [US2] Test subscriber_example.py in Docker environment
- [x] T030 [US2] Test service_server_example.py in Docker environment
- [x] T031 [US2] Test service_client_example.py in Docker environment
- [x] T032 [US2] Test joint_state_publisher.py in Docker environment
- [x] T033 [US2] Test joint_state_subscriber.py in Docker environment
- [x] T034 [US2] Document expected output for all code examples in Chapter 3 content

### Exercises for User Story 2

- [x] T035 [US2] Write Exercise 1: Modify publisher to send custom messages
- [x] T036 [US2] Write Exercise 2: Create subscriber for JointState
- [x] T037 [US2] Write Exercise 3: Implement custom service (e.g., SetSpeed)

### Validation for User Story 2

- [x] T038 [US2] Validate Chapter 3 content against constitution Principle II (educational clarity - step-by-step tutorials)
- [x] T039 [US2] Validate Chapter 3 content against constitution Principle III (reproducibility - exact versions, Docker testing)
- [x] T040 [US2] Verify all code examples include troubleshooting sections (constitution Principle II - student success)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently. Students can understand architecture (US1) and write functional ROS 2 nodes (US2).

---

## Phase 5: User Story 3 - Interpreting and Modifying URDF Files (Priority: P2)

**Goal**: Students can read, interpret, and modify URDF files for humanoid robots, and visualize them in RViz2

**Independent Test**: Student takes a simple humanoid URDF file, modifies it to add a camera sensor on the head link, and visualizes it in RViz2. The camera link appears in the correct position.

### Content for User Story 3

- [x] T041 [US3] Write Chapter 4 content at docs/module-1-ros2/04-urdf-basics.mdx following contracts/chapter-4-outline.md
- [x] T042 [US3] Add all Chapter 4 documentation links to official ROS 2 URDF sources
- [x] T043 [P] [US3] Create humanoid skeleton diagram at docs/module-1-ros2/assets/diagrams/humanoid-skeleton-labeled.svg (13 links, 12 joints)

### URDF Examples for User Story 3

- [x] T044 [US3] Create simple_humanoid.urdf at docs/module-1-ros2/assets/code-examples/simple_humanoid.urdf (13 links: head, torso, 2x upper/lower arms, hands, upper/lower legs, feet)
- [x] T045 [US3] Define all link geometries in simple_humanoid.urdf (use simple boxes/cylinders)
- [x] T046 [US3] Define all joint definitions in simple_humanoid.urdf (neck, shoulders, elbows, wrists, hips, knees, ankles)
- [x] T047 [US3] Create humanoid_with_camera.urdf at docs/module-1-ros2/assets/code-examples/humanoid_with_camera.urdf (exercise solution - adds camera to head)

### URDF Validation for User Story 3

- [x] T048 [US3] Validate simple_humanoid.urdf with check_urdf tool (ensure no syntax errors)
- [x] T049 [US3] Test simple_humanoid.urdf visualization in RViz2 (verify all links appear)
- [x] T050 [US3] Validate humanoid_with_camera.urdf with check_urdf tool
- [x] T051 [US3] Test humanoid_with_camera.urdf visualization in RViz2 (verify camera link on head)
- [x] T052 [US3] Document RViz2 visualization steps in Chapter 4 content

### Exercises for User Story 3

- [x] T053 [US3] Write Exercise: Add camera sensor to head link (provide step-by-step instructions)
- [x] T054 [US3] Write troubleshooting section for common URDF errors (missing parent link, invalid joint type, kinematic cycles)

### Validation for User Story 3

- [x] T055 [US3] Validate Chapter 4 content against constitution Principle I (source accuracy - URDF spec links)
- [x] T056 [US3] Validate Chapter 4 content against constitution Principle II (educational clarity)
- [x] T057 [US3] Verify URDF examples align with constitution Principle VI (modular architecture - humanoid-aligned for capstone)

**Checkpoint**: All user stories should now be independently functional. Students can understand architecture (US1), write ROS 2 nodes (US2), and work with URDF models (US3).

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final validation

- [x] T058 [P] Run Docusaurus build and verify no errors
- [ ] T059 [P] Validate all documentation links are accessible (no 404s)
- [ ] T060 [P] Check all code examples have copy-to-clipboard enabled (Docusaurus native feature)
- [ ] T061 [P] Verify all diagrams render correctly in MDX
- [ ] T062 [P] Run link checker on all chapters (validate official ROS 2 doc links)
- [x] T063 Verify module fits within 15-25 page target (constitution constraint)
- [x] T064 [P] Add Module 1 to main Docusaurus sidebar navigation
- [x] T065 [P] Create module landing page at docs/module-1-ros2/index.mdx with overview and learning objectives
- [x] T066 Review all chapters for consistency (terminology, formatting, code block style)
- [x] T067 Validate constitution compliance checklist (all 7 principles)
- [x] T068 [P] Add metadata for RAG chunking (module, chapter, section tags in MDX frontmatter)
- [ ] T069 Test RAG retrieval with sample queries (e.g., "How do I create a ROS 2 publisher?")
- [x] T070 [P] Create README.md for code examples directory with usage instructions

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P1): Can start after Foundational - No dependencies on other stories (conceptually builds on US1 but independently implementable)
  - User Story 3 (P2): Can start after Foundational - No dependencies on other stories
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Independent (writes code examples, references concepts from US1 but doesn't block)
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Independent (URDF modeling is separate from ROS 2 coding)

### Within Each User Story

**User Story 1**:
- Chapters 1 and 2 can be written in parallel (T009, T010)
- Diagrams can be created in parallel (T011, T012, T013)
- Documentation links added after chapter content (T014, T015 after T009, T010)
- Exercises after content (T016, T017 after chapters)
- Validation after all content (T018, T019 at end)

**User Story 2**:
- Chapter 3 content first (T020)
- All code examples can be created in parallel (T022-T027)
- Code testing must follow creation (T028-T033 after T022-T027)
- Exercises after chapter and code (T035-T037)
- Validation at end (T038-T040)

**User Story 3**:
- Chapter 4 content and diagram can be parallel (T041, T043)
- URDF creation sequential (T044 → T045 → T046 → T047)
- Validation after URDF creation (T048-T052 after T044-T047)
- Exercises and troubleshooting (T053, T054)
- Final validation (T055-T057)

### Parallel Opportunities

- All Setup tasks (T001-T004) can run in parallel
- All Foundational tasks marked [P] can run in parallel within Phase 2
- Once Foundational phase completes, all three user stories can start in parallel (if team capacity allows)
- Within each user story, tasks marked [P] can run in parallel

---

## Parallel Example: User Story 2

```bash
# Launch all code example creation together:
Task: "Create publisher_example.py"
Task: "Create subscriber_example.py"
Task: "Create service_server_example.py"
Task: "Create service_client_example.py"
Task: "Create joint_state_publisher.py"
Task: "Create joint_state_subscriber.py"

# After code examples created, launch all tests together:
Task: "Test publisher_example.py in Docker"
Task: "Test subscriber_example.py in Docker"
Task: "Test service_server_example.py in Docker"
# ... etc
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Chapters 1-2, diagrams, exercises)
4. **STOP and VALIDATE**: Students can understand ROS 2 architecture
5. Deploy/demo if ready (architecture content complete)

**MVP Deliverable**: Students understand ROS 2 concepts, DDS middleware, communication patterns. Ready for hands-on coding tutorials.

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (Architecture MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo (Coding tutorials complete!)
4. Add User Story 3 → Test independently → Deploy/Demo (Full Module 1 complete!)
5. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Chapters 1-2, conceptual)
   - Developer B: User Story 2 (Chapter 3, code examples)
   - Developer C: User Story 3 (Chapter 4, URDF)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- No tests requested in specification - focus on content creation and Docker validation
- Constitution compliance validation built into tasks (T018, T019, T038-T040, T055-T057, T067)
- Code examples tested in Docker to ensure constitution Principle III (reproducibility)
- All chapters include documentation links for constitution Principle I (source accuracy)
- Troubleshooting sections for constitution Principle II (student success)
- RAG metadata tasks (T068, T069) for constitution Principle V (chatbot fidelity)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Task Count Summary

- **Total Tasks**: 70
- **Setup (Phase 1)**: 4 tasks
- **Foundational (Phase 2)**: 4 tasks
- **User Story 1 (P1)**: 11 tasks (T009-T019)
- **User Story 2 (P1)**: 21 tasks (T020-T040)
- **User Story 3 (P2)**: 17 tasks (T041-T057)
- **Polish (Phase 6)**: 13 tasks (T058-T070)

**Parallel Opportunities**: 31 tasks marked [P] can run in parallel with others

**MVP Scope**: Setup + Foundational + User Story 1 = 19 tasks (Chapters 1-2, architecture understanding)

---

## Constitution Compliance Validation Tasks

Embedded throughout phases:
- T018, T019: Validate Chapters 1-2 against Principles I & II
- T038, T039, T040: Validate Chapter 3 against Principles II & III
- T055, T056, T057: Validate Chapter 4 against Principles I, II, & VI
- T067: Final constitution checklist (all 7 principles)
- T068, T069: RAG metadata and retrieval (Principle V)

---

## Success Criteria Mapping (from Spec)

- **SC-001**: Students explain communication patterns (validated in T018, T019 - Chapter 2 exercises)
- **SC-002**: 90% publisher-subscriber success (validated in T028, T029 - Docker testing)
- **SC-003**: Functional node in 30 min (validated in T038 - Chapter 3 tutorial quality)
- **SC-004**: URDF debugging in 15 min (validated in T054 - troubleshooting section)
- **SC-005**: Code runs on clean Ubuntu (validated in T028-T033 - Docker testing)
- **SC-006**: Understand "why" behind ROS 2 (validated in T018 - Chapter 1 DDS rationale)
- **SC-007**: 100% doc traceability (validated in T014, T015, T021, T042, T062 - all doc links)
