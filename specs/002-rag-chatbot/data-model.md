# Data Model: RAG Chatbot System

## Entity Models

### ChatSession
Represents a conversation between a user and the chatbot, maintaining context and history.

**Fields:**
- `id` (UUID): Unique identifier for the session
- `user_id` (UUID, optional): Identifier for authenticated users, null for anonymous
- `created_at` (DateTime): Timestamp when session was created
- `updated_at` (DateTime): Timestamp of last activity
- `metadata` (JSON): Additional session metadata (user agent, page context, etc.)
- `is_active` (Boolean): Whether the session is currently active

**Relationships:**
- One-to-many with UserQuery (has many queries in the session)
- One-to-many with GeneratedResponse (has many responses in the session)

### DocumentChunk
Represents a segment of book content that has been processed and stored in the vector database.

**Fields:**
- `id` (UUID): Unique identifier for the chunk
- `document_id` (UUID): Reference to the source document
- `chunk_index` (Integer): Position of the chunk within the document
- `content` (Text): The actual text content of the chunk
- `embedding_id` (String): ID reference in the vector database
- `token_count` (Integer): Number of tokens in the chunk
- `metadata` (JSON): Additional metadata (headings, section, etc.)
- `created_at` (DateTime): When the chunk was created

**Relationships:**
- Many-to-one with Document (belongs to a document)
- Zero-to-many with UserQuery (referenced by queries through vector similarity)

### UserQuery
Represents a question or statement submitted by the user for processing.

**Fields:**
- `id` (UUID): Unique identifier for the query
- `session_id` (UUID): Reference to the chat session
- `content` (Text): The actual question/query text
- `query_embedding` (Vector): Embedding vector of the query
- `context_info` (JSON): Additional context (selected text, current page, etc.)
- `created_at` (DateTime): When the query was submitted

**Relationships:**
- Many-to-one with ChatSession (belongs to a session)
- Many-to-many with DocumentChunk (references relevant chunks through retrieval)

### RetrievedContext
Represents relevant book content retrieved from the vector database to answer a query.

**Fields:**
- `id` (UUID): Unique identifier for the retrieved context
- `query_id` (UUID): Reference to the original user query
- `chunk_id` (UUID): Reference to the document chunk
- `similarity_score` (Float): Similarity score from vector search
- `rank` (Integer): Rank of this chunk in the retrieval results
- `content` (Text): The content that was retrieved
- `created_at` (DateTime): When the context was retrieved

**Relationships:**
- Many-to-one with UserQuery (belongs to a query)
- Many-to-one with DocumentChunk (references the chunk)

### GeneratedResponse
Represents the AI-generated answer provided to the user, including citations to source material.

**Fields:**
- `id` (UUID): Unique identifier for the response
- `session_id` (UUID): Reference to the chat session
- `query_id` (UUID): Reference to the original user query
- `content` (Text): The generated response text
- `sources` (JSON): List of source citations (document IDs, chunk IDs, etc.)
- `model_used` (String): Name of the LLM model used
- `created_at` (DateTime): When the response was generated
- `token_usage` (JSON): Information about token usage (input, output, total)

**Relationships:**
- Many-to-one with ChatSession (belongs to a session)
- Many-to-one with UserQuery (responds to a query)
- Many-to-many with RetrievedContext (cites multiple context pieces)

### Document
Represents a source document (book chapter, page, etc.) that has been ingested into the system.

**Fields:**
- `id` (UUID): Unique identifier for the document
- `filename` (String): Original filename
- `path` (Text): Path where the document is stored
- `title` (String): Document title
- `version` (String): Version of the document
- `checksum` (String): Checksum to detect changes
- `ingestion_status` (String): Status of ingestion ('pending', 'processing', 'completed', 'failed')
- `metadata` (JSON): Additional document metadata (author, date, etc.)
- `created_at` (DateTime): When the document was added
- `updated_at` (DateTime): When the document was last updated

**Relationships:**
- One-to-many with DocumentChunk (has many chunks)
- Many-to-many with RetrievedContext (referenced by contexts)

## Validation Rules

### DocumentChunk Validation
- Content must not be empty
- Token count must be within reasonable limits (max 1000 tokens per chunk)
- Chunk index must be non-negative
- Document ID must reference an existing document

### UserQuery Validation
- Content must not be empty
- Session ID must reference an existing active session
- Query embedding must be properly formatted vector

### GeneratedResponse Validation
- Content must not be empty
- Session ID and Query ID must reference existing records
- Sources must be properly formatted JSON

## State Transitions

### Document Ingestion Flow
1. `pending` → `processing`: When ingestion job starts
2. `processing` → `completed`: When all chunks are successfully stored
3. `processing` → `failed`: When ingestion encounters an error
4. `completed` → `processing`: When document is updated and needs reprocessing

### ChatSession Lifecycle
1. Created when user first interacts with chatbot
2. Remains `active` while user continues conversation
3. Becomes inactive after period of inactivity
4. Can be explicitly closed by user or system

## Indexes and Performance Considerations

### Database Indexes
- Index on `DocumentChunk.document_id` for fast document lookup
- Index on `UserQuery.session_id` for session-based queries
- Index on `GeneratedResponse.query_id` for response retrieval
- Composite index on `RetrievedContext.query_id` and `rank` for ordered retrieval

### Vector Database Considerations
- Use appropriate similarity metric (cosine similarity)
- Implement efficient vector search with filtering capabilities
- Consider using sparse and dense vector combinations for better retrieval