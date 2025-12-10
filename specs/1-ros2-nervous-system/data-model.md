## Data Model: AI/Spec-Driven Book + RAG Chatbot

### Entity: Book
- **Description**: The overall Docusaurus-based book.
- **Fields**:
  - `title`: String (e.g., "Physical AI & Humanoid Robotics")
  - `architecture_sketch`: Object (folders, pages, sidebar structure)
  - `chapter_outline`: Array of Chapter entities
  - `content_workflow`: Object (Spec-Kit Plus + Claude Code generation)
  - `quality_checklist`: Object (accuracy, code correctness, build success)
  - `versioning_strategy`: String (e.g., GitHub Pages)
  - `deployment_strategy`: String (e.g., GitHub Pages)

### Entity: Chapter
- **Description**: A section or module within the book.
- **Fields**:
  - `title`: String
  - `sections`: Array of Section entities
  - `objectives`: Array of Strings
  - `diagrams`: Array of Diagram entities
  - `code_examples`: Array of CodeExample entities
  - `workflows`: Array of Strings

### Entity: Section
- **Description**: A subsection within a chapter.
- **Fields**:
  - `title`: String
  - `content`: Markdown text

### Entity: Diagram
- **Description**: Visual representation within the book.
- **Fields**:
  - `style`: String (e.g., Markdown/ASCII)
  - `content`: String (diagram definition)

### Entity: CodeExample
- **Description**: Verified code snippets.
- **Fields**:
  - `format`: String (e.g., Python/ROS 2)
  - `code`: String (actual code)
  - `test_status`: Boolean (must run without errors)

### Entity: RAGChatbot
- **Description**: The Retrieval Augmented Generation chatbot integrated with the book.
- **Fields**:
  - `integration_method`: String (how it connects to the book)
  - `knowledge_base_source`: String (the book content)
  - `vector_storage`: String (e.g., Qdrant Cloud)
  - `metadata_storage`: String (e.g., Neon Postgres)
  - `response_time_goal`: String (e.g., 1-2 seconds total)
  - `fidelity_constraint`: String (strictly answer based on book content)
