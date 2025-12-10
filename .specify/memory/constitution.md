<!-- Sync Impact Report:
Version change: 0.1.0 -> 0.1.1 (Patch - Reaffirmation of principles)
Modified principles: None
Added sections: None
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
# AI/Spec-Driven Book + RAG Chatbot on “Physical AI & Humanoid Robotics” Constitution

## Core Principles

### Technical Accuracy
All content, code, and explanations must be technically accurate, specifically referencing ROS 2, Gazebo, Unity, Isaac, and VLA where applicable.

### Consistency & Maintainability
Maintain consistency with Spec-Kit Plus guidelines and Docusaurus framework for project structure and documentation. Ensure verified code using ROS 2, rclpy, Python, FastAPI, and OpenAI Agents/ChatKit.

### Clarity & Educational Value
Provide clear, structured explanations tailored for AI/robotics students. Content must be free of hallucinations and strictly follow official documentation.

### Content Structure & Referencing
Each chapter must include clear objectives, relevant diagrams, verified code examples, and defined workflows. Use IEEE citations where necessary.

### RAG Chatbot Fidelity
The RAG chatbot must strictly answer questions based on the content provided within the book, ensuring zero external knowledge interference.

### Latest SDK & Tooling
All integrations and examples must utilize the latest stable SDK versions for all relevant tools and libraries.

## Constraints

- Book must be fully Spec-Kit Plus compatible and deployable via GitHub Pages.
- RAG system will use Qdrant Cloud for vector storage and Neon Postgres for metadata.
- Content coverage is limited to syllabus modules:
  1) ROS 2 fundamentals + URDF
  2) Gazebo/Unity digital twin
  3) NVIDIA Isaac Sim/Isaac ROS/Nav2
  4) VLA: Whisper + LLM planning

## Success Criteria

- The book must build in Docusaurus without any errors.
- The RAG chatbot must function correctly end-to-end, including retrieval capabilities.

## Governance

- All Pull Requests and code reviews must explicitly verify compliance with these constitutional principles.
- Any increase in system complexity must be thoroughly justified with clear rationale and documented tradeoffs.
- Refer to specific guidance files (e.g., `README.md`, `docs/quickstart.md`) for runtime development procedures and best practices.

**Version**: 0.1.1 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09
