# Research: RAG Chatbot Implementation

## Overall System Architecture

### Architecture Components
- **Frontend**: Embeddable React widget for Docusaurus pages
- **Backend**: FastAPI server handling API requests and business logic
- **Vector Database**: Qdrant Cloud for storing document embeddings
- **Metadata Database**: Neon Serverless Postgres for document metadata
- **LLM Service**: OpenRouter API for chat completions
- **Embedding Service**: Qwen models for generating document and query embeddings

### Architecture Pattern
The system follows a standard RAG (Retrieval-Augmented Generation) architecture:
1. Content ingestion pipeline processes documentation and stores embeddings
2. User query is embedded and matched against document embeddings
3. Relevant document chunks are retrieved and sent to LLM
4. LLM generates contextual response based on retrieved context

## Book Content Ingestion Pipeline

### Markdown Processing
- Parse Markdown files from Docusaurus documentation
- Extract text content while preserving structural information
- Split content into semantic chunks (paragraphs, sections)
- Maintain metadata (source file, headings, hierarchy)

### Chunking Strategy
- Use semantic chunking rather than fixed-length splitting
- Target chunk sizes of 500-1000 tokens for optimal context
- Preserve document structure and relationships between chunks
- Include overlap between chunks to maintain context continuity

## Qwen Embedding Generation Workflow

### Embedding Model Selection
- Use Qwen embedding models via API for consistency
- Batch processing for efficiency during ingestion
- Store embedding dimensions and metadata for retrieval

### Processing Workflow
1. Preprocess text (clean, normalize, segment)
2. Generate embeddings for each chunk
3. Store embeddings in Qdrant with metadata
4. Maintain mapping between chunks and source documents in Postgres

## Qdrant Cloud Vector Indexing Strategy

### Collection Design
- Create collection with appropriate vector dimensions for Qwen embeddings
- Use cosine similarity for semantic search
- Include payload fields for document metadata and source tracking

### Indexing Strategy
- Upsert vectors with document chunk IDs as point IDs
- Include metadata in payload (source file, section, headings)
- Implement efficient batch indexing for large document sets
- Use sparse and dense vector combinations if needed

## Neon Postgres Schema

### Document Tracking
- Store document metadata (filename, path, version, checksum)
- Track ingestion status and timestamps
- Maintain relationships between documents and chunks

### Schema Design
```sql
-- Documents table
documents (
  id UUID PRIMARY KEY,
  filename VARCHAR(255),
  path TEXT,
  version VARCHAR(50),
  checksum VARCHAR(64),
  created_at TIMESTAMP,
  updated_at TIMESTAMP,
  ingestion_status VARCHAR(20) -- 'pending', 'processing', 'completed', 'failed'
)

-- Document chunks table
document_chunks (
  id UUID PRIMARY KEY,
  document_id UUID REFERENCES documents(id),
  chunk_index INTEGER,
  content TEXT,
  metadata JSONB,
  created_at TIMESTAMP
)
```

## FastAPI Endpoints

### Chat Endpoints
- `POST /api/chat`: Process user questions against full knowledge base
- `POST /api/chat/selection`: Process questions based on user-selected text
- `GET /api/health`: Health check for the service

### Ingestion Endpoints
- `POST /api/ingest`: Ingest new documents into the knowledge base
- `GET /api/ingest/status`: Check ingestion status
- `DELETE /api/ingest/{doc_id}`: Remove documents from knowledge base

## OpenRouter API Usage

### Model Selection
- Use appropriate models from OpenRouter that support context windows
- Implement proper rate limiting and error handling
- Configure temperature and other parameters for optimal responses

### Integration Pattern
- Send retrieved context along with user query
- Handle streaming responses for better UX
- Implement retry logic for API failures

## Docusaurus Frontend Integration

### Embedding Strategy
- Create standalone JavaScript widget that can be embedded via script tag
- Use shadow DOM to avoid CSS conflicts with host page
- Implement responsive design for different screen sizes

### User Experience
- Floating widget that expands on interaction
- Visual indication of processing state
- Clear attribution of sources in responses

## Deployment Flow

### Backend Deployment
- Containerized deployment using Docker
- Environment variable configuration for different environments
- Health checks and monitoring setup

### Frontend Deployment
- Build static assets for the chat widget
- CDN distribution for global access
- Versioning strategy for updates

## Data Flow from User Question to Final Answer

### Complete Data Flow
1. **User Input**: User types question in the Docusaurus page widget
2. **Frontend Processing**: Widget sends question to backend API
3. **Query Embedding**: Backend generates embedding for the user question using Qwen
4. **Vector Search**: Embedding is sent to Qdrant Cloud to find similar document chunks
5. **Context Retrieval**: Relevant document chunks are retrieved with their content and metadata
6. **LLM Prompt**: Retrieved context is combined with the original question and sent to OpenRouter API
7. **Response Generation**: LLM generates contextual answer based on the provided context
8. **Response Formatting**: Backend formats response with source citations
9. **Frontend Display**: Widget displays answer with source attribution to the user

### Error Handling Flow
- If no relevant documents found: Return appropriate message
- If API errors occur: Fallback responses and error logging
- If rate limits hit: Queue requests or return temporary unavailability message

## Key Decisions

### Decision: Use of Qwen Embeddings
- **Rationale**: Consistent with system requirements and provides good semantic understanding
- **Alternatives considered**: Sentence Transformers, OpenAI embeddings
- **Chosen**: Qwen embeddings to match system requirements

### Decision: Qdrant Cloud for Vector Storage
- **Rationale**: Managed service reduces operational overhead
- **Alternatives considered**: Self-hosted Qdrant, Pinecone, Weaviate
- **Chosen**: Qdrant Cloud for alignment with requirements

### Decision: OpenRouter for LLM Inference
- **Rationale**: Provides access to multiple models with good pricing
- **Alternatives considered**: Direct OpenAI API, Anthropic API
- **Chosen**: OpenRouter to meet system requirements

## Security Considerations

- API key management and rotation
- Rate limiting to prevent abuse
- Input sanitization to prevent injection attacks
- Proper authentication if needed for private documents