# Module 6: RAG Chatbot Stack

## Overview
This module defines the specification for a Retrieval-Augmented Generation (RAG) Chatbot. The chatbot will be capable of answering user questions by leveraging information extracted exclusively from user-selected text, ensuring responses are grounded in provided context. The stack will utilize FastAPI for the backend, Qdrant Cloud Free Tier for vector storage, Neon Serverless Postgres for metadata persistence, and OpenAI ChatKit for conversational interfaces.

## Components

### 1. Backend (FastAPI)
- **API Endpoints:** RESTful API for text ingestion, vectorization, question answering, and conversational management.
- **Text Processing:** Handling of user-selected text, including chunking and embedding generation.
- **Integration Layer:** Orchestration of interactions between Qdrant, Neon, and OpenAI ChatKit.
- **Security:** Basic API key authentication for endpoint access.

### 2. Vector Database (Qdrant Cloud Free Tier)
- **Vector Storage:** Storing text embeddings generated from user-selected content.
- **Similarity Search:** Performing efficient nearest-neighbor searches to retrieve relevant text chunks based on user queries.
- **Scalability:** Leveraging Qdrant's capabilities for vector indexing and querying.

### 3. Metadata Database (Neon Serverless Postgres)
- **Metadata Storage:** Storing metadata associated with ingested text chunks (e.g., source document, user ID, timestamps).
- **Scalability and Cost-Efficiency:** Utilizing Neon's serverless architecture for on-demand scaling and cost optimization.
- **Data Integrity:** Ensuring consistency and reliability of metadata.

### 4. Conversational Interface (OpenAI ChatKit)
- **User Interface:** Providing a frontend for users to input text, ask questions, and receive answers.
- **Context Management:** Handling conversational turns and maintaining context for coherent interactions.
- **Response Generation:** Utilizing OpenAI models via ChatKit to generate natural language responses based on retrieved information and user queries.
- **Strict Context Adherence:** Ensuring the LLM only answers questions based on the provided user-selected text and explicitly states if information is not found in the given context.

## Project Goal
Develop a fully functional RAG Chatbot that precisely answers user questions using only the context from user-provided text, demonstrating the seamless integration of FastAPI, Qdrant, Neon, and OpenAI ChatKit.

### Key Features to Implement
- **Text Ingestion:** API to upload and process user-selected text for embedding and storage.
- **Contextual Question Answering:** Users can ask questions, and the chatbot provides answers strictly from the ingested text.
- **Source Attribution:** Each answer includes references to the specific text chunks from which the information was retrieved.
- **Conversational Flow:** Maintaining chat history and context for follow-up questions.
- **Error Handling:** Robust handling of cases where questions cannot be answered from the provided context.

### Deliverables
- Deployed FastAPI backend.
- Configured Qdrant and Neon instances.
- Functional OpenAI ChatKit frontend.
- Comprehensive API documentation.
- Project documentation including architecture, design choices, and deployment instructions.

### Evaluation Criteria
- Accuracy of answers based solely on provided context.
- Efficiency of retrieval and response generation.
- Robustness of the overall system.
- User experience of the chatbot interface.
- Adherence to the specified technology stack.
