# Quickstart Guide: RAG Chatbot for Docusaurus

## Overview
This guide will help you set up and run the RAG chatbot system for your Docusaurus documentation site. The system consists of a FastAPI backend and a React-based frontend widget that can be embedded in your Docusaurus pages.

## Prerequisites
- Python 3.11+
- Node.js 18+ (for frontend development)
- Docker (optional, for containerized deployment)
- Access to OpenRouter API
- Qdrant Cloud account
- Neon Serverless Postgres account

## Backend Setup

### 1. Environment Configuration
Create a `.env` file in the backend directory with the following variables:

```bash
OPENROUTER_API_KEY=your_openrouter_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_cluster_url
NEON_DATABASE_URL=your_neon_postgres_connection_string
EMBEDDING_MODEL=qwen/qwen-7b-chat # or your preferred Qwen model
LLM_MODEL=your_preferred_model_on_openrouter
```

### 2. Install Dependencies
```bash
cd backend
pip install -r requirements.txt
```

### 3. Run Migrations
```bash
cd src
python -m scripts.database_migrations
```

### 4. Start the Backend Server
```bash
cd src
python -m api.main
```

The backend server will start on `http://localhost:8000`

## Frontend Setup

### 1. Install Dependencies
```bash
cd frontend
npm install
```

### 2. Configure Environment
Create a `.env` file in the frontend directory:

```bash
REACT_APP_BACKEND_URL=http://localhost:8000
```

### 3. Build the Widget
```bash
npm run build
```

### 4. Development Server
```bash
npm start
```

## Document Ingestion

### 1. Prepare Documents
Ensure your documents are in Markdown format and organized in a directory structure.

### 2. Ingest Documents
Use the ingestion API to add documents to your knowledge base:

```bash
curl -X POST http://localhost:8000/api/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "documents": [
      {
        "title": "Getting Started Guide",
        "content": "# Getting Started\nThis is the content...",
        "source_url": "/docs/getting-started"
      }
    ]
  }'
```

### 3. Monitor Ingestion
Check the status of your ingestion job:

```bash
curl http://localhost:8000/api/ingest/status/{job_id}
```

## Docusaurus Integration

### 1. Copy the Widget
Copy the built frontend files to your Docusaurus `static` directory:

```bash
cp -r frontend/build/* docs/static/chatbot/
```

### 2. Add Script to Docusaurus
Add the following script to your Docusaurus page or layout:

```html
<script src="/chatbot/chatbot-embed.js"></script>
```

### 3. Initialize the Widget
The widget will automatically initialize when the page loads. You can customize its appearance and behavior using data attributes:

```html
<div
  id="chatbot-container"
  data-backend-url="http://localhost:8000"
  data-position="bottom-right"
  data-initially-open="false">
</div>
```

## API Endpoints

### Chat Endpoints
- `POST /api/chat` - Process user questions
- `POST /api/chat/selection` - Process questions with selected text context
- `GET /api/health` - Health check

### Ingestion Endpoints
- `POST /api/ingest` - Ingest new documents
- `GET /api/ingest/status/{job_id}` - Check ingestion status
- `GET /api/documents` - List all documents
- `DELETE /api/ingest/{document_id}` - Remove documents

## Testing the System

### 1. Verify Backend Health
```bash
curl http://localhost:8000/api/health
```

### 2. Test a Query
```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is the main concept of this documentation?",
    "session_id": "test-session"
  }'
```

### 3. Check Document List
```bash
curl http://localhost:8000/api/documents
```

## Deployment

### Backend Deployment
1. Build the backend for production
2. Deploy to your preferred cloud platform (AWS, GCP, Azure, etc.)
3. Configure environment variables securely
4. Set up monitoring and logging

### Frontend Deployment
1. Build static assets: `npm run build`
2. Deploy to CDN or serve from your Docusaurus site
3. Update the script URL in your Docusaurus configuration

## Troubleshooting

### Common Issues
- **API Keys**: Ensure all API keys are correctly configured
- **Network**: Verify the backend is accessible from the frontend
- **Documents**: Confirm documents are properly formatted and ingested
- **Rate Limits**: Monitor API usage to avoid rate limiting

### Logs
Check backend logs for errors and performance metrics:
- API request logs
- Error logs
- Performance metrics
- Database query logs