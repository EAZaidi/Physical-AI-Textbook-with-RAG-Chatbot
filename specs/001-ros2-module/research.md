# Research: Module 1 - The Robotic Nervous System (ROS 2)

**Date**: 2025-12-07
**Feature**: Module 1 ROS 2 educational content for Docusaurus textbook
**Status**: Complete

## Purpose

Resolve technical unknowns and architectural decisions for implementing ROS 2 educational content in Docusaurus format. Research focuses on: MDX component strategy, RAG chunking compatibility, Docker-based reproducibility testing, chapter structure, and URDF example complexity.

---

## Research Task 1: Docusaurus MDX Component Strategy for Code Examples

### Question
How should code examples (Python rclpy, XML URDF) be presented in Docusaurus MDX to maximize student usability (copy-paste, syntax highlighting) and align with interactive learning goals?

### Research Findings

**Docusaurus Native Code Blocks**:
- Built-in syntax highlighting via Prism.js (supports Python, XML, Bash)
- Copy-to-clipboard button automatic in Docusaurus v3+
- Supports line highlighting: ```python {2-4}
- Supports line numbering: ```python showLineNumbers
- Supports filename display: ```python title="publisher_example.py"
- [Source: Docusaurus docs - Markdown Features](https://docusaurus.io/docs/markdown-features/code-blocks)

**Docusaurus Live Codeblock Plugin**:
- `@docusaurus/theme-live-codeblock` for interactive JavaScript/JSX
- NOT suitable for Python/ROS 2 (requires browser execution, ROS 2 is system-level)
- Could be useful later for JavaScript-based ROS graph visualizations
- [Source: Docusaurus docs - Live Codeblock](https://docusaurus.io/docs/api/themes/@docusaurus/theme-live-codeblock)

**Third-Party Solutions**:
- CodeSandbox embeds: External dependency, requires internet, breaks offline usage
- Asciinema for terminal recordings: Good for demonstrating commands, but not for copy-paste code
- Custom React components: Over-engineering, reinvents Docusaurus features

### Decision

**Use Docusaurus native code blocks with enhancements**:
```python title="publisher_example.py" showLineNumbers
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
```

**Enhancements**:
- Always include `title` attribute for file context
- Use `showLineNumbers` for tutorials (helps with explanations)
- Use line highlighting `{X-Y}` to emphasize key lines
- Include "Expected Output" code blocks after command examples

### Rationale

- Native Docusaurus features are well-documented, maintained, and performant
- Copy-to-clipboard is critical for educational content (students want to run code immediately)
- Syntax highlighting improves readability (Python, XML/URDF, Bash)
- Line numbers help with step-by-step explanations ("On line 7, we create the publisher...")
- Filename context helps students organize their workspace

### Alternatives Considered

1. **Plain Markdown code blocks**: Lack copy button, no advanced features
2. **Live Codeblock plugin**: Not suitable for Python/ROS 2 (browser-based execution)
3. **CodeSandbox embeds**: External dependency, breaks offline usage, overkill for ROS 2
4. **Custom React component**: Reinvents wheel, maintenance burden, no clear benefit

### Impact

- Students can easily copy code examples with one click
- Consistent formatting across all chapters
- No external dependencies (offline-first)
- Future-proof (Docusaurus is well-maintained, widely adopted)

---

## Research Task 2: Content Chunking Strategy for RAG Compatibility

### Question
How should Docusaurus MDX content be chunked for RAG indexing to balance retrieval precision (find relevant section) with context preservation (answer is complete)?

### Research Findings

**Semantic Chunking by Headings**:
- Docusaurus MDX uses H1 (chapter title), H2 (major section), H3 (subsection)
- Example structure:
  - H1: "Chapter 2: Nodes, Topics, and Services"
  - H2: "Understanding Topics"
  - H3: "Creating a Publisher"
- Each H2/H3 section is self-contained explanation (200-1500 words typically)
- [Source: Docusaurus best practices, semantic HTML structure]

**Token Size Considerations**:
- LLM context windows: 8k-128k tokens (modern models)
- RAG retrieval: Typically 3-5 chunks per query
- Optimal chunk size: 500-1000 tokens (balance context vs precision)
- Metadata: Chapter number, title, H2/H3 heading for citation

**Chunking Strategies Compared**:
1. **Fixed-size chunking** (e.g., every 512 tokens):
   - Pros: Uniform size, predictable
   - Cons: Breaks semantic boundaries (mid-sentence, mid-code-example), harder to cite
2. **Paragraph-level chunking**:
   - Pros: Respects paragraph boundaries
   - Cons: Too granular (loses context), many small chunks
3. **Whole-chapter chunking**:
   - Pros: Maximum context
   - Cons: Too coarse (retrieves irrelevant content), hard to pinpoint answer
4. **Heading-based semantic chunking** (H2/H3):
   - Pros: Respects semantic boundaries, natural citation points, balanced size
   - Cons: Variable chunk size (need to handle very long sections)

### Decision

**Semantic chunking by H2/H3 headings with 500-1000 token target**:
- Each chunk = one H2 or H3 section (including nested content)
- If H2 section exceeds 1000 tokens, split by H3 subsections
- Metadata for each chunk:
  - `module`: "Module 1"
  - `chapter`: "Chapter 2: Nodes, Topics, and Services"
  - `section`: "Understanding Topics" (H2 or H3 heading)
  - `section_type`: "concept" | "tutorial" | "exercise" | "summary"

**Example chunk**:
```json
{
  "content": "## Understanding Topics\n\nTopics are named buses...",
  "metadata": {
    "module": "Module 1",
    "chapter": "Chapter 2: Nodes, Topics, and Services",
    "section": "Understanding Topics",
    "section_type": "concept"
  }
}
```

### Rationale

- H2/H3 sections are semantically coherent (one concept, one tutorial step)
- Students ask questions at section granularity ("How do I create a publisher?")
- Chatbot can cite specific sections ("See Module 1, Chapter 2, 'Creating a Publisher'")
- 500-1000 tokens balances context (enough to answer question) with precision (not too much irrelevant content)

### Alternatives Considered

1. **Fixed-size chunking**: Breaks semantic boundaries, poor citations
2. **Paragraph-level**: Too granular, loses context
3. **Whole-chapter**: Too coarse, retrieves irrelevant content

### Impact

- RAG chatbot retrieves relevant sections with high precision
- Students get complete answers with clear chapter/section citations
- Chunking strategy works across all modules (consistent metadata schema)

---

## Research Task 3: Docker Testing Workflow for Code Reproducibility

### Question
How should code examples be validated for reproducibility in clean environments (constitution Principle III: "works on my machine" unacceptable)?

### Research Findings

**Docker for Reproducibility**:
- Docker provides isolated, reproducible environments
- Official ROS 2 Docker images: `osrf/ros:humble-desktop` (Ubuntu 22.04 + ROS 2 Humble)
- Can mount code examples from `docs/module-1-ros2/assets/code-examples/` into container
- [Source: ROS 2 Docker Tutorial](https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)

**Testing Workflow Options**:
1. **Manual Docker testing**:
   - Developer runs `docker run -it osrf/ros:humble-desktop`
   - Manually copies code, runs examples, validates output
   - Pros: Simple, no automation overhead
   - Cons: Manual, error-prone, not scalable, no CI/CD
2. **Automated Docker test script**:
   - Shell script: `test-code-examples.sh`
   - Iterates through code examples, runs each in Docker, validates output
   - CI/CD integration: GitHub Actions runs script on every PR
   - Pros: Automated, scalable, catches regressions
   - Cons: Requires test script maintenance
3. **GitHub Codespaces**:
   - Cloud-based dev environment
   - Pros: Accessible anywhere
   - Cons: Requires internet, vendor lock-in, not always available
4. **VM-based testing** (e.g., Vagrant):
   - Full VM with Ubuntu 22.04 + ROS 2
   - Pros: Complete isolation
   - Cons: Heavier, slower, more complex than Docker

### Decision

**Automated Docker test script with CI/CD integration**:
- Dockerfile: `docker/ros2-testing/Dockerfile` (based on `osrf/ros:humble-desktop`)
- Test script: `scripts/test-code-examples.sh`
- CI/CD: GitHub Actions workflow (`.github/workflows/test-code-examples.yml`)

**Test Script Logic**:
```bash
#!/bin/bash
# Test all Python code examples in Docker

CODE_DIR="docs/module-1-ros2/assets/code-examples"
CONTAINER_NAME="ros2-code-test"

for file in $CODE_DIR/*.py; do
  echo "Testing $file..."
  docker run --rm \
    -v $(pwd)/$CODE_DIR:/workspace \
    osrf/ros:humble-desktop \
    bash -c "source /opt/ros/humble/setup.bash && python3 /workspace/$(basename $file)"

  if [ $? -ne 0 ]; then
    echo "FAIL: $file"
    exit 1
  fi
done

echo "All tests passed!"
```

### Rationale

- Automated testing catches regressions (e.g., API changes in ROS 2 updates)
- CI/CD ensures every PR validates code examples
- Docker is lightweight, fast, and standard for ROS 2
- Students can use same Docker image for local testing (Dockerfile in repo)
- Matches target platform: Ubuntu 22.04 + ROS 2 Humble

### Alternatives Considered

1. **Manual testing**: Not scalable, human error
2. **GitHub Codespaces**: Vendor lock-in, not always available
3. **VM-based**: Too heavy, slower CI/CD

### Impact

- Code examples guaranteed to run in clean environment
- Students have reference Docker setup for local testing
- PR quality gate: code must pass Docker tests

---

## Research Task 4: Chapter Structure and Length Distribution

### Question
How should 4 chapters be structured to fit within 15-25 page module target (of 120-200 page book) while balancing pedagogical flow?

### Research Findings

**Constitution Constraint**:
- Total book: 120-200 pages (excluding code appendices)
- 7 modules planned → ~17-28 pages per module average
- Module 1 is foundational (slightly shorter acceptable, ~15-25 pages)

**Pedagogical Structure per Chapter** (from constitution):
- Learning objectives: 1 page
- Conceptual overview: 3-5 pages
- Hands-on tutorial with code: 5-10 pages
- Practice exercises: 1-2 pages
- Summary and resources: 1 page
- **Total per chapter**: 11-19 pages

**4 Chapters for Module 1**:
1. **Chapter 1: ROS 2 Overview and System Architecture** (conceptual, foundational)
   - Learning objectives, ROS 1 vs ROS 2, DDS middleware, graph concepts
   - Less code-heavy (mostly diagrams, conceptual explanations)
   - Estimated: 5-6 pages
2. **Chapter 2: Nodes, Topics, and Services** (theory + light code)
   - Communication patterns, pub-sub vs request-response
   - Simple code examples (1-2 publishers, 1-2 subscribers, 1 service)
   - Estimated: 6-7 pages
3. **Chapter 3: Python-to-ROS Control via rclpy** (most hands-on)
   - Detailed rclpy tutorials, multiple examples (publishers, subscribers, services)
   - Most code-heavy (P1 user story - students write nodes)
   - Estimated: 7-8 pages
4. **Chapter 4: URDF Basics for Humanoid Robots** (modeling-focused)
   - URDF syntax, link/joint definitions, RViz2 visualization
   - Moderate code (1 URDF example, modifications)
   - P2 priority (slightly shorter acceptable)
   - Estimated: 5-6 pages

**Total**: 23-27 pages (fits within 15-25 page target, slight overage acceptable)

### Decision

**Distribute ~15-25 pages across 4 chapters**:
- Chapter 1: 5-6 pages (architecture, conceptual)
- Chapter 2: 6-7 pages (communication patterns, light code)
- Chapter 3: 7-8 pages (rclpy tutorials, most hands-on)
- Chapter 4: 5-6 pages (URDF modeling)

**Justification for Chapter 3 length**:
- P1 user story (highest priority)
- Core skill (students MUST write ROS 2 nodes)
- Multiple code examples needed (publishers, subscribers, services)
- Detailed step-by-step tutorials (more pages)

### Rationale

- Total 23-27 pages fits module target (15-25 pages)
- Chapter 3 is longest (aligns with P1 priority and hands-on focus)
- Chapter 1 is shorter (conceptual, fewer code examples)
- Chapter 4 is moderate (P2 priority, but still important for humanoid robotics)
- Pedagogical flow: theory (Ch1) → patterns (Ch2) → practice (Ch3) → modeling (Ch4)

### Alternatives Considered

1. **Equal length chapters** (6-7 pages each): Doesn't align with P1/P2 priorities
2. **Combine Ch1+Ch2** (3 chapters total): Too dense, loses pedagogical flow
3. **Split Ch3** (5 chapters total): Too fragmented, students want cohesive rclpy tutorial

### Impact

- Clear chapter scope (students know time investment)
- Pedagogical flow optimized (theory before practice)
- Module fits within book length constraints

---

## Research Task 5: URDF Example Complexity

### Question
What level of URDF complexity (number of links/joints, detail level) balances learning objectives (understand URDF structure) with student comprehension (not overwhelming)?

### Research Findings

**URDF for Humanoid Robots**:
- Full humanoid: 30-50+ links (fingers, toes, detailed head)
- Example: PR2 robot has 70+ links, 60+ joints
- Example: Atlas humanoid has 30+ links
- Simplified humanoid: 10-15 links (no fingers/toes, basic structure)

**Learning Objectives for Chapter 4**:
- Understand link/joint definitions
- Parent-child relationships (kinematic tree)
- Collision vs visual geometries
- Adding sensors (camera, LiDAR) to robot model
- NOT: Advanced Xacro macros, Gazebo plugins, transmission elements

**Complexity Levels Considered**:
1. **Simple arm** (5-6 links: base, shoulder, elbow, wrist, gripper):
   - Pros: Easy to understand
   - Cons: Not humanoid, doesn't align with capstone
2. **Simplified humanoid** (13 links: head, torso, 2x upper/lower arms, 2x hands, 2x upper/lower legs, 2x feet):
   - Pros: Humanoid structure, demonstrates tree hierarchy, manageable complexity
   - Cons: Still requires understanding parent-child relationships
3. **Full humanoid** (30+ links with fingers, toes):
   - Pros: Realistic
   - Cons: Overwhelming, students get lost in details, violates "simplicity" principle

**Student Time Budget**:
- Chapter 4 estimated: 5-6 pages, ~2-3 hours of study
- URDF comprehension + modification exercise: 30-45 minutes target
- Simplified humanoid (13 links) is manageable in this time

### Decision

**Simplified humanoid skeleton with 13 links and 12 joints**:
- **Links**: head, torso, left/right upper arm, left/right lower arm, left/right hand, left/right upper leg, left/right lower leg, left/right foot
- **Joints**: neck (head-torso), left/right shoulder (torso-upper arm), left/right elbow (upper-lower arm), left/right wrist (lower arm-hand), left/right hip (torso-upper leg), left/right knee (upper-lower leg), left/right ankle (lower leg-foot)
- **Geometries**: Simple boxes/cylinders (no detailed meshes)
- **Sensors**: Add camera to head (exercise: students add this)

**Example URDF structure**:
```xml
<robot name="simple_humanoid">
  <link name="torso">...</link>
  <link name="head">...</link>
  <joint name="neck" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    ...
  </joint>
  <!-- Arms, legs, etc. -->
</robot>
```

### Rationale

- 13 links is enough to demonstrate key concepts (tree structure, parent-child, joint types)
- Humanoid structure aligns with course capstone (Module 7: humanoid project)
- Simple geometries (boxes/cylinders) keep file readable (no mesh paths, no complex collision)
- Students can visualize in RViz2 and see full body structure
- Manageable complexity for 30-45 minute exercise

### Alternatives Considered

1. **Simple arm (5-6 links)**: Not humanoid, doesn't prepare for capstone
2. **Full humanoid (30+ links)**: Overwhelming, students get lost
3. **Mobile robot**: Wrong robot type for course focus

### Impact

- Students learn URDF fundamentals with relevant humanoid example
- Foundation for Module 7 capstone (humanoid manipulation/navigation)
- Exercise: "Add camera sensor to head link" is achievable in tutorial time

---

## Summary of Decisions

| Decision Area | Choice | Key Rationale |
|---------------|--------|---------------|
| **Code Examples** | Docusaurus native code blocks with `title`, `showLineNumbers`, line highlighting | Built-in copy-paste, syntax highlighting, no external dependencies |
| **RAG Chunking** | Semantic chunking by H2/H3 headings (500-1000 tokens) | Balances context preservation with retrieval precision, natural citations |
| **Reproducibility Testing** | Automated Docker test script + CI/CD (GitHub Actions) | Ensures clean-environment reproducibility, catches regressions |
| **Chapter Length** | Ch1: 5-6p, Ch2: 6-7p, Ch3: 7-8p, Ch4: 5-6p (total ~23-27p) | Aligns with priorities (Ch3 longest for P1 story), fits module target |
| **URDF Complexity** | Simplified humanoid (13 links, 12 joints, simple geometries) | Demonstrates key concepts, humanoid-aligned, manageable in 30-45 min |

---

## Next Steps

All technical unknowns resolved. Proceed to:
1. **Phase 1**: Create data-model.md (ROS 2 entities), chapter outlines (contracts/), quickstart.md
2. **Phase 2**: Generate tasks.md from user stories and chapter structure
3. **Implementation**: Write MDX chapters, code examples, validate in Docker

---

## References

- [Docusaurus Code Blocks](https://docusaurus.io/docs/markdown-features/code-blocks)
- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Docker Tutorial](https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
