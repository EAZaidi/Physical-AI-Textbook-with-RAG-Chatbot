---
id: 001
title: Robotics Textbook Initial Constitution
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Essa Zaidi
command: /sp.constitution
labels: ["constitution", "robotics", "education", "rag-chatbot", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A (constitution document)
---

## Prompt

```text
Project: Physical AI & Humanoid Robotics Textbook + RAG Chatbot

## Core Principles
- Accuracy from official ROS 2, Gazebo, Unity, NVIDIA Isaac, and VLA sources
- Clarity for AI/robotics students
- Reproducibility of all code, simulations, and deployments
- Spec-driven workflow (Spec-Kit Plus + Claude Code)
- Zero hallucination; all facts verifiable

## Key Standards
- All claims traceable to authoritative robotics/AI documentation
- Docusaurus MDX formatting, consistent structure across modules
- Each chapter: learning goals, explanations, code, tutorials
- Capstone: voice → plan → navigate → perceive → manipulate
- RAG chatbot: OpenAI Agents/ChatKit + FastAPI + Neon + Qdrant
- Chatbot answers strictly from book content; supports "selected-text-only" mode

## Constraints
- Book size: 120-200 pages
- Output: full Docusaurus book deployed to GitHub Pages
- Scope: ROS 2, Gazebo/Unity, Isaac, VLA modules + capstone humanoid project
- No unverified or speculative robotics content

## Success Criteria
- All content accurate and source-verified
- Book fully generated from specs and builds cleanly
- Tutorials and code reproducible by students
- RAG chatbot embedded and hallucination-free
```

## Response snapshot

Created initial constitution (v1.0.0) for Physical AI & Humanoid Robotics Textbook project with seven core principles:

1. **Source Accuracy & Verifiability**: All technical claims traceable to official ROS 2, Gazebo, Unity, Isaac, VLA documentation
2. **Educational Clarity & Student Success**: Clear learning goals, step-by-step tutorials, troubleshooting guidance
3. **Reproducibility & Environment Consistency**: Version-locked dependencies, Docker configs, validation steps
4. **Spec-Driven Content Development**: Spec-Kit Plus workflow (spec → plan → tasks → implementation)
5. **RAG Chatbot Fidelity**: Zero hallucination, book-content-only retrieval, citation of sources
6. **Modular Architecture & Progressive Complexity**: 7 modules from ROS 2 fundamentals to capstone humanoid project
7. **Production-Ready Deployment Standards**: CI/CD, Dockerized backend, secure secrets management

Established technical constraints (Docusaurus, ROS 2 Humble/Iron, FastAPI + Qdrant + Neon stack) and quality gates (source verification, code validation, build success, chatbot integration, peer review).

## Outcome

- ✅ Impact: Established project governance and development standards for robotics textbook and RAG chatbot
- 🧪 Tests: N/A (constitution document, no code tests required)
- 📁 Files: Created .specify/memory/constitution.md with complete principles and governance
- 🔁 Next prompts: Create first module specification (e.g., ROS 2 fundamentals), or specify RAG chatbot feature
- 🧠 Reflection: Constitution emphasizes zero hallucination and reproducibility, critical for educational robotics content

## Evaluation notes (flywheel)

- Failure modes observed: None (initial creation)
- Graders run and results (PASS/FAIL): N/A (manual review required for constitution)
- Prompt variant (if applicable): Standard /sp.constitution command with detailed project principles
- Next experiment (smallest change to try): Validate constitution alignment when creating first feature spec
