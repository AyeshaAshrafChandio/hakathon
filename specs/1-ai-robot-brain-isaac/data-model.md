# Data Model for AI-Robot Brain (NVIDIA Isaac)

This document outlines the conceptual data model for the Docusaurus-based book content on AI-Robot Brain (NVIDIA Isaac). It describes the entities, their fields, and relationships, focusing on how content is structured for both presentation in the book and future integration with RAG systems.

## 1. Book Module Entity

Represents the main AI-Robot Brain module within the Docusaurus book.

*   **`id`** (string): Unique identifier (e.g., auto-generated slug from title).
*   **`name`** (string): Title of the module (e.g., "AI-Robot Brain (NVIDIA Isaac)").
*   **`outline`** (string): High-level summary or bullet points of sections covered.
*   **`objectives`** (array of strings): Key learning goals for the module.
*   **`content_path`** (string): Relative path to the main Markdown file for this module (e.g., `docs/ai-robot-brain/index.md`).
*   **`chapters`** (array of strings): List of chapter IDs that belong to this module.
*   **`target_audience`** (string): Description of the intended audience (e.g., "Robotics students").
*   **`status`** (enum: "Outline", "Drafting", "Review", "Completed"): Current development status.
*   **`last_updated`** (timestamp): Date and time of the last modification.

## 2. Chapter Entity

Represents a chapter within the AI-Robot Brain module.

*   **`id`** (string): Unique identifier for the chapter.
*   **`module_id`** (string): ID of the parent module this chapter belongs to.
*   **`title`** (string): Title of the chapter (e.g., "Isaac Sim", "Isaac ROS", "Nav2 Navigation").
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

Represents a high-level workflow or code snippet within the book.

*   **`id`** (string): Unique identifier for the code example.
*   **`section_id`** (string): ID of the section this code example belongs to.
*   **`language`** (string): Programming language (e.g., "Python", "C++").
*   **`framework`** (string): Framework or tool (e.g., "ROS 2", "Isaac ROS", "Nav2").
*   **`code`** (string): The actual code or workflow content (high-level as per constraints).
*   **`description`** (string): Explanation of what the code/workflow does.
*   **`test_status`** (boolean): Whether the code has been verified for conceptual accuracy.
*   **`test_environment`** (string): Environment where the concept would be applied (e.g., "ROS 2 Humble", "Isaac Sim 2023.1").
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

## 6. Training Data Entity

Represents synthetic training data concepts discussed in the Isaac Sim chapter.

*   **`id`** (string): Unique identifier for the training data concept.
*   **`section_id`** (string): ID of the section this concept belongs to.
*   **`name`** (string): Name of the training data type (e.g., "Synthetic LiDAR Data", "Photorealistic RGB Images").
*   **`description`** (string): Explanation of the training data concept.
*   **`use_case`** (string): How this training data is used in perception systems.
*   **`generation_method`** (string): How the synthetic data is generated in Isaac Sim.
*   **`status`** (enum: "Concept", "Detailed", "Example"): Level of detail provided.

## 7. RAG Document Chunk Entity (Future Integration)

Represents a smaller, retrievable chunk of content from the book, optimized for future RAG system integration.

*   **`chunk_id`** (string): Unique identifier for the content chunk.
*   **`text_content`** (string): The actual text content extracted from a book section.
*   **`embedding`** (vector): Numerical vector representation of `text_content` (for future use).
*   **`metadata`** (object): Additional contextual information for retrieval and display.
    *   **`module_id`** (string): ID of the parent module.
    *   **`chapter_id`** (string): ID of the parent chapter.
    *   **`section_id`** (string): ID of the parent section.
    *   **`section_title`** (string): Title of the specific section this chunk belongs to.
    *   **`source_path`** (string): Original Docusaurus Markdown file path.
    *   **`page_url`** (string): Deployed URL of the Docusaurus page where this chunk originates.
    *   **`keywords`** (array of strings): Auto-generated or manually added keywords for enhanced search.
    *   **`last_updated`** (timestamp): Timestamp of when the original content was last modified.
    *   **`content_type`** (enum: "text", "code_snippet", "diagram_description", "training_concept"): Type of content in the chunk.
*   **`relationships`** (array of strings): List of `chunk_id`s or concept tags for related content.

## Relationships

*   **One-to-Many**: A `Book Module` can contain multiple `Chapter` entities.
*   **One-to-Many**: A `Chapter` can contain multiple `Section` entities.
*   **One-to-Many**: A `Section` can contain multiple `Code Example`, `Diagram`, and `Training Data` entities.
*   **One-to-Many**: A `Book Module` can contain multiple `RAG Document Chunk` entities (for future integration).
*   **Many-to-Many (Conceptual)**: `RAG Document Chunk` entities can have relationships with other chunks or broader concepts.

This data model provides a foundation for organizing the AI-Robot Brain book content, enabling efficient future retrieval for RAG systems, and structuring the educational material in a hierarchical and accessible manner.