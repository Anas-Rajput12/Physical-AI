# Implementation Plan: RAG Chatbot for Docusaurus Documentation

**Branch**: `002-rag-chatbot` | **Date**: 2025-12-18 | **Spec**: [specs/002-rag-chatbot/spec.md](specs/002-rag-chatbot/spec.md)
**Input**: Feature specification from `/specs/002-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a Retrieval-Augmented Generation (RAG) chatbot system for Docusaurus-based technical documentation. The system will enable users to ask questions about book content and receive accurate, contextual answers. The architecture includes a FastAPI backend with Qwen embeddings, Qdrant vector database, Neon Postgres for metadata, and OpenRouter for LLM inference. The frontend will be an embeddable widget for Docusaurus pages.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/TypeScript for frontend
**Primary Dependencies**: FastAPI, Qwen embedding models, Qdrant client, Neon Postgres, OpenRouter API, React
**Storage**: Qdrant Cloud (vector database), Neon Serverless Postgres (metadata), Docusaurus static files
**Testing**: pytest for backend, Jest for frontend, integration tests for end-to-end functionality
**Target Platform**: Linux server (backend), Web browser (frontend widget)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <5 second response time for 95% of queries, support 100 concurrent users
**Constraints**: <200ms p95 for internal API calls, <5s for end-to-end question answering, open-source tools preferred
**Scale/Scope**: Support for 10k+ documentation pages, 1M+ words of content, 100 concurrent users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Open Source & Accessibility**: All components use open-source tools (FastAPI, Qdrant, Neon Postgres) as specified in constitution
- **Technical Accuracy & Practical Relevance**: Implementation uses industry-standard RAG architecture with proven technologies
- **Modular & Extensible Content**: Architecture separates concerns with clear API boundaries between components
- **Evidence-Based Approach**: All technical decisions backed by research and proven implementations

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── chat_session.py
│   │   ├── document_chunk.py
│   │   ├── user_query.py
│   │   └── generated_response.py
│   ├── services/
│   │   ├── embedding_service.py
│   │   ├── vector_db_service.py
│   │   ├── llm_service.py
│   │   ├── content_ingestion_service.py
│   │   └── rag_service.py
│   ├── api/
│   │   ├── main.py
│   │   ├── chat_endpoints.py
│   │   └── ingestion_endpoints.py
│   └── config/
│       ├── settings.py
│       └── database.py
└── tests/
    ├── unit/
    ├── integration/
    └── contract/

frontend/
├── src/
│   ├── components/
│   │   ├── ChatWidget.jsx
│   │   ├── Message.jsx
│   │   ├── ChatInput.jsx
│   │   └── ChatHistory.jsx
│   ├── services/
│   │   ├── api_client.js
│   │   └── context_extractor.js
│   └── styles/
│       └── chatbot.css
├── public/
└── tests/
    ├── unit/
    └── integration/

docs/
└── embed/
    └── chatbot-embed.js  # Standalone script for Docusaurus integration
```

**Structure Decision**: Selected web application structure with separate backend and frontend to allow independent deployment and scaling. Backend uses FastAPI for the API server, while frontend is a React-based widget that can be embedded in Docusaurus pages.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple repositories | Separation of concerns and independent deployment | Tightly coupled system would make maintenance harder |
| Third-party APIs | OpenRouter and Qdrant Cloud provide essential capabilities | Self-hosted alternatives would require significant infrastructure investment |
