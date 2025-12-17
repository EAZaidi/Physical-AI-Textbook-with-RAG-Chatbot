<!--
Sync Impact Report:
- Version change: [initial] → 1.0.0
- Modified principles: N/A (initial version)
- Added sections: All core sections established
- Removed sections: None
- Templates requiring updates:
  ✅ plan-template.md - reviewed, Constitution Check section aligns
  ✅ spec-template.md - reviewed, requirements sections align
  ✅ tasks-template.md - reviewed, task categorization aligns
- Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook + RAG Chatbot Constitution

## Core Principles

### I. Source Accuracy & Verifiability

Every technical claim, API reference, simulation workflow, and robotics concept MUST be traceable to official documentation from:
- ROS 2 (ros.org, design.ros2.org)
- Gazebo (gazebosim.org)
- Unity Robotics (github.com/Unity-Technologies/Unity-Robotics-Hub)
- NVIDIA Isaac Sim & Isaac Lab (docs.nvidia.com)
- Vision-Language-Action (VLA) model papers and official implementations

**Rationale**: Robotics and AI frameworks evolve rapidly. Unverified or outdated information leads to broken tutorials, frustrated learners, and loss of credibility. Zero hallucination is non-negotiable.

### II. Educational Clarity & Student Success

All explanations, code examples, and tutorials MUST be designed for AI/robotics students with varied backgrounds. Each module MUST include:
- Clear learning goals stated upfront
- Conceptual explanations before technical details
- Step-by-step reproducible code with expected outputs
- Common pitfalls and troubleshooting guidance

**Rationale**: The textbook serves as both learning resource and reference. Students must be able to follow tutorials independently and understand both the "what" and the "why" behind robotics concepts.

### III. Reproducibility & Environment Consistency

All code, simulations, launch files, and deployment instructions MUST be reproducible across target environments. This includes:
- Explicit version specifications for ROS 2, Gazebo, Unity, Isaac Sim
- Complete dependency lists (apt packages, pip requirements, Unity packages)
- Environment setup scripts or Docker configurations
- Validation steps to confirm successful setup

**Rationale**: "Works on my machine" is unacceptable in educational robotics. Students waste hours on environment issues rather than learning. Reproducibility builds confidence and enables focus on concepts.

### IV. Spec-Driven Content Development

All book content, tutorials, and RAG chatbot features MUST follow the Spec-Kit Plus workflow:
1. Feature specification (`spec.md`) defining learning objectives and requirements
2. Implementation plan (`plan.md`) detailing content structure and dependencies
3. Task breakdown (`tasks.md`) with testable acceptance criteria
4. Incremental development with validation checkpoints

**Rationale**: Systematic development prevents scope creep, ensures completeness, and maintains quality. Spec-driven workflow enables iterative refinement and clear progress tracking.

### V. RAG Chatbot Fidelity

The RAG chatbot MUST answer questions strictly from book content with zero hallucination:
- Retrieve only from indexed book chapters and code examples
- Support "selected-text-only" mode for precise context queries
- Cite specific chapters/sections in responses
- Explicitly state "Not covered in this book" when information is unavailable

**Rationale**: A hallucinating chatbot undermines the entire textbook's credibility. Students rely on accurate, source-grounded answers. The chatbot must be a trustworthy study companion, not a source of misinformation.

### VI. Modular Architecture & Progressive Complexity

Content MUST be organized in self-contained modules with clear dependencies:
- **Module 1**: ROS 2 fundamentals (nodes, topics, services, actions)
- **Module 2**: Simulation environments (Gazebo, Unity, Isaac Sim)
- **Module 3**: Robot perception (cameras, LiDAR, depth, VLMs)
- **Module 4**: Motion planning and navigation
- **Module 5**: Manipulation and grasping
- **Module 6**: Vision-Language-Action (VLA) models
- **Module 7**: Capstone humanoid project integrating all skills

Each module builds on previous ones but can be understood independently with appropriate prerequisites stated.

**Rationale**: Progressive learning prevents cognitive overload. Students master fundamentals before tackling integration. Modular structure enables flexible learning paths and targeted study.

### VII. Production-Ready Deployment Standards

All infrastructure (Docusaurus site, RAG chatbot API, vector database) MUST meet production standards:
- Automated CI/CD for book builds and deployments
- Dockerized chatbot backend (FastAPI + Qdrant + Neon PostgreSQL)
- Secure API key management (environment variables, no hardcoding)
- Health checks, logging, and error handling
- GitHub Pages deployment for static site

**Rationale**: The textbook is a public-facing educational resource. Broken deployments, slow chatbot responses, or security vulnerabilities damage reputation and user trust. Production-grade infrastructure ensures reliability.

## Technical Constraints

### Content Scope & Length

- **Target length**: 120-200 pages (excluding code appendices)
- **Chapter structure**: Consistent format across all modules
  - Learning objectives (1 page)
  - Conceptual overview (3-5 pages)
  - Hands-on tutorial with code (5-10 pages)
  - Practice exercises (1-2 pages)
  - Summary and resources (1 page)
- **Code examples**: Fully annotated, tested, and version-locked

### Technology Stack

- **Book platform**: Docusaurus (MDX for interactive content)
- **ROS version**: ROS 2 Humble or Iron (LTS releases only)
- **Simulation**: Gazebo Fortress/Garden, Unity 2022 LTS, Isaac Sim 2023+
- **VLA frameworks**: OpenVLA, RT-1/RT-2, or equivalent open models
- **RAG backend**: FastAPI (Python 3.11+), OpenAI Agents/ChatKit
- **Vector store**: Qdrant (cloud or self-hosted)
- **Database**: Neon PostgreSQL (serverless)
- **Deployment**: GitHub Pages (static site), Docker (chatbot API)

### Quality Gates

Before any content module is marked complete, it MUST pass:
1. **Source verification**: All claims linked to official docs
2. **Code validation**: All tutorials tested in clean environments
3. **Build success**: Docusaurus build completes without errors
4. **Chatbot integration**: Content indexed and retrievable via RAG
5. **Peer review**: At least one subject matter expert review

## Development Workflow

### Content Creation Process

1. **Specification**: Define module learning objectives and outline (`spec.md`)
2. **Research**: Gather official documentation links and verify accuracy
3. **Planning**: Design content structure and code examples (`plan.md`)
4. **Task breakdown**: Create testable tasks for writing and validation (`tasks.md`)
5. **Implementation**: Write content incrementally, validate tutorials continuously
6. **Integration**: Index content for RAG, deploy updates to staging
7. **Validation**: Run quality gates, collect feedback, iterate

### Chatbot Development Process

1. **Specification**: Define query capabilities and constraints (`spec.md`)
2. **Architecture**: Design RAG pipeline (chunking, embedding, retrieval) (`plan.md`)
3. **Tasks**: Break down API endpoints, database schema, indexing (`tasks.md`)
4. **Implementation**: Build FastAPI backend, integrate Qdrant and Neon
5. **Testing**: Validate retrieval accuracy, test hallucination prevention
6. **Deployment**: Dockerize, deploy to cloud, monitor performance

### Version Control & Branching

- **Main branch**: Production-ready content only
- **Feature branches**: `###-module-name` or `###-chatbot-feature`
- **Commit messages**: Follow conventional commits (feat, fix, docs, test)
- **Pull requests**: Require passing quality gates before merge

## Governance

### Constitution Authority

This constitution supersedes all other development practices for the Physical AI & Humanoid Robotics Textbook project. All contributors, AI agents, and automated workflows MUST verify compliance.

### Amendment Procedure

Constitution changes require:
1. Documented rationale (why current principle is insufficient)
2. Approval from project maintainer
3. Migration plan for affected content and workflows
4. Updated `CONSTITUTION_VERSION` following semantic versioning

### Compliance Review

All pull requests MUST include a checklist confirming:
- [ ] All technical claims verified against official documentation
- [ ] Code examples tested in specified environments
- [ ] Tutorials include reproducibility instructions
- [ ] Spec-driven workflow followed (spec → plan → tasks → implementation)
- [ ] RAG chatbot content indexed (if content changes)
- [ ] No hardcoded secrets or credentials
- [ ] Docusaurus build succeeds

### Complexity Justification

Any violation of simplicity or scope constraints (e.g., adding non-robotics content, exceeding 200 pages, using experimental frameworks) MUST be justified in the implementation plan with:
- Specific educational need
- Alternative approaches considered and rejected
- Impact on reproducibility and student success

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
