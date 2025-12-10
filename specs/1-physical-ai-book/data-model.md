# Data Model for Physical AI & Humanoid Robotics Book

This document outlines the conceptual data model for the Docusaurus-based book content on Physical AI & Humanoid Robotics. It describes the entities, their fields, and relationships, focusing on how content is structured for both presentation in the book and future integration with RAG systems.

## 1. Book Module Entity

Represents a main module within the Physical AI & Humanoid Robotics book.

*   **`id`** (string): Unique identifier (e.g., auto-generated slug from title).
*   **`name`** (string): Title of the module (e.g., "ROS 2 Foundations", "Digital Twin Simulation", "AI-Robot Brain", "VLA Integration").
*   **`outline`** (string): High-level summary or bullet points of sections covered.
*   **`objectives`** (array of strings): Key learning goals for the module.
*   **`content_path`** (string): Relative path to the main Markdown file for this module (e.g., `docs/ros2-foundations/index.md`).
*   **`chapters`** (array of strings): List of chapter IDs that belong to this module.
*   **`target_audience`** (string): Description of the intended audience (e.g., "Intermediate to advanced AI/robotics students").
*   **`status`** (enum: "Outline", "Drafting", "Review", "Completed"): Current development status.
*   **`last_updated`** (timestamp): Date and time of the last modification.

## 2. Chapter Entity

Represents a chapter within a book module.

*   **`id`** (string): Unique identifier for the chapter.
*   **`module_id`** (string): ID of the parent module this chapter belongs to.
*   **`title`** (string): Title of the chapter (e.g., "Foundations of the Robotic Nervous System", "Gazebo Physics", "Isaac Sim", "Voice-to-Action").
*   **`sections`** (array of strings): List of section IDs that belong to this chapter.
*   **`objectives`** (array of strings): Learning objectives for this chapter.
*   **`content_path`** (string): Relative path to the Markdown file for this chapter.
*   **`prerequisites`** (array of strings): Prerequisites needed before reading this chapter.
*   **`related_topics`** (array of strings): Related topics or chapters.
*   **`status`** (enum: "Outline", "Drafting", "Review", "Completed"): Current development status.
*   **`last_updated`** (timestamp): Date and time of the last modification.

## 3. Section Entity

Represents a section within a chapter.

*   **`id`** (string): Unique identifier for the section.
*   **`chapter_id`** (string): ID of the parent chapter this section belongs to.
*   **`title`** (string): Title of the section.
*   **`content`** (string): The actual text content of the section.
*   **`diagrams_references`** (array of strings): List of relative paths or identifiers to associated diagrams.
*   **`code_examples_references`** (array of strings): List of relative paths or identifiers to code snippets.
*   **`learning_outcomes`** (array of strings): Specific outcomes students should achieve after reading this section.
*   **`difficulty_level`** (enum: "Beginner", "Intermediate", "Advanced"): Complexity level of the section.
*   **`status`** (enum: "Outline", "Drafting", "Review", "Completed"): Current development status.
*   **`last_updated`** (timestamp): Date and time of the last modification.

## 4. Code Example Entity

Represents a verified code snippet within the book.

*   **`id`** (string): Unique identifier for the code example.
*   **`section_id`** (string): ID of the section this code example belongs to.
*   **`language`** (string): Programming language (e.g., "Python", "C++").
*   **`framework`** (string): Framework or tool (e.g., "ROS 2", "Gazebo", "Unity", "Isaac ROS").
*   **`code`** (string): The actual code content.
*   **`description`** (string): Explanation of what the code does.
*   **`test_status`** (boolean): Whether the code has been tested and verified to run.
*   **`test_environment`** (string): Environment where the code was tested (e.g., "ROS 2 Humble", "Isaac Sim 2023.1").
*   **`last_verified`** (timestamp): Date and time when the code was last verified.
*   **`status`** (enum: "Draft", "Verified", "Deprecated"): Current status of the code example.

## 5. Diagram Entity

Represents a visual representation within the book.

*   **`id`** (string): Unique identifier for the diagram.
*   **`section_id`** (string): ID of the section this diagram belongs to.
*   **`title`** (string): Title or caption for the diagram.
*   **`description`** (string): Detailed explanation of the diagram.
*   **`style`** (string): Format of the diagram (e.g., "Mermaid", "Markdown", "Static Image").
*   **`content`** (string): The actual diagram definition (e.g., Mermaid code, image path).
*   **`alt_text`** (string): Alternative text for accessibility.
*   **`status`** (enum: "Draft", "Completed"): Current status of the diagram.
*   **`last_updated`** (timestamp): Date and time of the last modification.

## 6. RAG Document Chunk Entity

Represents a smaller, retrievable chunk of content from the book, optimized for the RAG chatbot system.

*   **`chunk_id`** (string): Unique identifier for the content chunk.
*   **`text_content`** (string): The actual text content extracted from a book section.
*   **`embedding`** (vector): Numerical vector representation of `text_content` generated by an embedding model (stored in Qdrant).
*   **`metadata`** (object): Additional contextual information for retrieval and display.
    *   **`module_id`** (string): ID of the parent module.
    *   **`chapter_id`** (string): ID of the parent chapter.
    *   **`section_id`** (string): ID of the parent section.
    *   **`section_title`** (string): Title of the specific section this chunk belongs to.
    *   **`source_path`** (string): Original Docusaurus Markdown file path (e.g., `docs/ros2-foundations/foundations.md`).
    *   **`page_url`** (string): Deployed URL of the Docusaurus page where this chunk originates.
    *   **`keywords`** (array of strings): Auto-generated or manually added keywords for enhanced search.
    *   **`last_updated`** (timestamp): Timestamp of when the original content was last modified.
    *   **`content_type`** (enum: "text", "code_snippet", "diagram_description"): Type of content in the chunk.
*   **`relationships`** (array of strings): List of `chunk_id`s or concept tags for related content.

## 7. RAG Query/Response Entity (Conceptual)

Describes the interaction flow with the RAG chatbot. These are transient entities during a user session.

*   **`query_id`** (string): Unique identifier for a user query.
*   **`user_query`** (string): The natural language query submitted by the user.
*   **`retrieved_chunk_ids`** (array of strings): List of `chunk_id`s from the `RAG Document Chunk` entity that were retrieved based on the `user_query`.
*   **`generated_response`** (string): The LLM-generated answer based on the retrieved chunks and strictly from book content.
*   **`response_timestamp`** (timestamp): Date and time when the response was generated.
*   **`feedback`** (object): Optional user feedback on the response quality.
    *   **`rating`** (integer): e.g., 1-5 stars.
    *   **`comment`** (string): User's textual feedback.

## 8. Learning Path Entity

Represents the recommended progression through the book content.

*   **`id`** (string): Unique identifier for the learning path.
*   **`name`** (string): Name of the path (e.g., "Complete Physical AI Course", "ROS 2 to VLA Pipeline").
*   **`description`** (string): Explanation of the learning path.
*   **`modules_order`** (array of strings): Ordered list of module IDs in the learning sequence.
*   **`estimated_duration`** (string): Estimated time to complete the path.
*   **`prerequisites`** (array of strings): Prerequisites for starting the path.
*   **`status`** (enum: "Draft", "Published", "Deprecated"): Current status of the learning path.

## Relationships

*   **One-to-Many**: A `Book Module` can contain multiple `Chapter` entities.
*   **One-to-Many**: A `Chapter` can contain multiple `Section` entities.
*   **One-to-Many**: A `Section` can contain multiple `Code Example` and `Diagram` entities.
*   **One-to-Many**: A `Book Module` can contain multiple `RAG Document Chunk` entities.
*   **Many-to-Many (Conceptual)**: `RAG Document Chunk` entities can have relationships with other chunks or broader concepts.
*   **One-to-Many**: A `Learning Path` can reference multiple `Book Module` entities in a specific order.

This data model provides a foundation for organizing the Physical AI & Humanoid Robotics book content, enabling efficient retrieval for the RAG chatbot, and structuring the educational material in a hierarchical and accessible manner.