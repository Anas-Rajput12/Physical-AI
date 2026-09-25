from typing import List, Dict, Any, Optional
from sqlalchemy.orm import Session

from ..services.embedding_service import embedding_service
from ..services.vector_db_service import vector_db_service
from ..services.llm_service import llm_service, LLMResponse

from ..models.user_query import UserQuery
from ..models.generated_response import GeneratedResponse

import traceback


class RAGService:

    def __init__(self):
        self.top_k = 5
        self.min_similarity_score = 0.3

    # ========================================================
    # QUERY KNOWLEDGE BASE
    # ========================================================

    def query_knowledge_base(
        self,
        db: Session,
        question: str,
        session_id: Optional[str] = None,
        page_context: Optional[Dict[str, Any]] = None,
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:

        try:
            print("========================================", flush=True)
            print("🔥 RAG QUERY STARTED", flush=True)
            print("QUESTION:", question, flush=True)

            # ------------------------------------------------
            # Validate DB
            # ------------------------------------------------

            if db is None:
                raise ValueError(
                    "Database session is None"
                )

            print("✅ DATABASE SESSION EXISTS", flush=True)

            # ------------------------------------------------
            # STEP 1: EMBEDDING
            # ------------------------------------------------

            print("🔥 STEP 1: Generating embedding", flush=True)

            query_embedding = (
                embedding_service.generate_embedding(question)
            )

            print(
                "✅ EMBEDDING GENERATED",
                flush=True
            )

            # ------------------------------------------------
            # STEP 2: VECTOR SEARCH
            # ------------------------------------------------

            print("🔥 STEP 2: Searching Qdrant", flush=True)

            if (
                page_context
                and page_context.get("document_id")
            ):

                similar_chunks = (
                    vector_db_service.search_similar(
                        query_embedding=query_embedding,
                        limit=self.top_k,
                        document_id_filter=(
                            page_context.get("document_id")
                        )
                    )
                )

            else:

                similar_chunks = (
                    vector_db_service.search_similar(
                        query_embedding=query_embedding,
                        limit=self.top_k
                    )
                )

            print(
                "✅ QDRANT SEARCH COMPLETE",
                flush=True
            )

            print(
                "CHUNKS FOUND:",
                len(similar_chunks),
                flush=True
            )

            # ------------------------------------------------
            # STEP 3: FILTER RESULTS
            # ------------------------------------------------

            relevant_chunks = [
                chunk
                for chunk in similar_chunks
                if chunk.get("similarity_score", 0)
                >= self.min_similarity_score
            ]

            print(
                "✅ RELEVANT CHUNKS:",
                len(relevant_chunks),
                flush=True
            )

            # ------------------------------------------------
            # STEP 4: LLM
            # ------------------------------------------------

            print(
                "🔥 STEP 4: Calling LLM",
                flush=True
            )

            llm_response: LLMResponse = (
                llm_service.generate_response(
                    prompt=question,
                    context=relevant_chunks,
                    selected_text=selected_text
                )
            )

            print(
                "✅ LLM RESPONSE GENERATED",
                flush=True
            )

            # ------------------------------------------------
            # STEP 5: CREATE USER QUERY
            # ------------------------------------------------

            print(
                "🔥 STEP 5: Creating UserQuery",
                flush=True
            )

            user_query = UserQuery(
                session_id=session_id,
                content=question,
                query_embedding=(
                    str(query_embedding)[:100]
                ),
                context_info={
                    "page_context": page_context or {},
                    "selected_text": selected_text or "",
                    "relevant_chunks_count": len(
                        relevant_chunks
                    )
                }
            )

            db.add(user_query)

            # Get generated ID
            db.flush()

            print(
                "✅ USER QUERY CREATED:",
                user_query.id,
                flush=True
            )

            # ------------------------------------------------
            # BUILD SOURCES
            # ------------------------------------------------

            sources = [
                {
                    "document_id": chunk.get(
                        "document_id"
                    ),
                    "chunk_id": chunk.get("id"),
                    "content_snippet": (
                        chunk.get("content", "")[:200]
                        + "..."
                        if len(
                            chunk.get("content", "")
                        ) > 200
                        else chunk.get("content", "")
                    ),
                    "similarity_score": chunk.get(
                        "similarity_score",
                        0
                    )
                }
                for chunk in relevant_chunks
            ]

            # ------------------------------------------------
            # TOKEN USAGE
            # ------------------------------------------------

            token_usage = (
                llm_response.token_usage
            )

            if hasattr(
                token_usage,
                "model_dump"
            ):
                token_usage_data = (
                    token_usage.model_dump()
                )

            elif hasattr(
                token_usage,
                "dict"
            ):
                token_usage_data = (
                    token_usage.dict()
                )

            elif isinstance(
                token_usage,
                dict
            ):
                token_usage_data = (
                    token_usage
                )

            else:
                token_usage_data = {}

            # ------------------------------------------------
            # STEP 6: GENERATED RESPONSE
            # ------------------------------------------------

            print(
                "🔥 STEP 6: Creating GeneratedResponse",
                flush=True
            )

            generated_response = GeneratedResponse(
                session_id=session_id,
                query_id=str(user_query.id),
                content=llm_response.content,
                sources=sources,
                model_used=llm_response.model_used,
                token_usage=token_usage_data
            )

            db.add(generated_response)

            # ------------------------------------------------
            # STEP 7: COMMIT
            # ------------------------------------------------

            print(
                "🔥 STEP 7: Committing database",
                flush=True
            )

            db.commit()

            print(
                "✅ DATABASE COMMIT SUCCESS",
                flush=True
            )

            # ------------------------------------------------
            # FINAL RESPONSE
            # ------------------------------------------------

            result = {
                "response_id": str(
                    generated_response.id
                ),
                "answer": llm_response.content,
                "sources": sources,
                "session_id": session_id,
                "relevant_chunks_count": len(
                    relevant_chunks
                ),
                "model_used": llm_response.model_used
            }

            print(
                "🎉 RAG QUERY COMPLETED",
                flush=True
            )

            return result

        except Exception as e:

            print(
                "========================================",
                flush=True
            )

            print(
                "🔥🔥🔥 RAG ERROR",
                flush=True
            )

            print(
                "ERROR TYPE:",
                type(e).__name__,
                flush=True
            )

            print(
                "ERROR:",
                repr(e),
                flush=True
            )

            print(
                "TRACEBACK:",
                flush=True
            )

            traceback.print_exc()

            print(
                "========================================",
                flush=True
            )

            # Rollback DB transaction
            try:
                if db is not None:
                    db.rollback()
                    print(
                        "✅ DATABASE ROLLBACK COMPLETE",
                        flush=True
                    )
            except Exception as rollback_error:

                print(
                    "🔥 ROLLBACK ERROR:",
                    repr(rollback_error),
                    flush=True
                )

            raise

    # ========================================================
    # SELECTED TEXT ONLY
    # ========================================================

    def query_selected_text_only(
        self,
        db: Session,
        question: str,
        selected_text: str,
        session_id: Optional[str] = None,
        page_context: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:

        try:

            print(
                "🔥 SELECTED TEXT QUERY STARTED",
                flush=True
            )

            if db is None:
                raise ValueError(
                    "Database session is None"
                )

            # ------------------------------------------------
            # Context
            # ------------------------------------------------

            context = [
                {
                    "content": selected_text,
                    "document_id": "selected_text",
                    "id": "selected_text_chunk",
                    "similarity_score": 1.0
                }
            ]

            # ------------------------------------------------
            # LLM
            # ------------------------------------------------

            print(
                "🔥 Calling LLM with selected text",
                flush=True
            )

            llm_response: LLMResponse = (
                llm_service.generate_response(
                    prompt=question,
                    context=context,
                    selected_text=selected_text
                )
            )

            print(
                "✅ LLM RESPONSE GENERATED",
                flush=True
            )

            # ------------------------------------------------
            # User Query
            # ------------------------------------------------

            user_query = UserQuery(
                session_id=session_id,
                content=question,
                context_info={
                    "page_context": page_context or {},
                    "selected_text": selected_text,
                    "query_type": "selected_text_only"
                }
            )

            db.add(user_query)
            db.flush()

            # ------------------------------------------------
            # Token usage
            # ------------------------------------------------

            token_usage = (
                llm_response.token_usage
            )

            if hasattr(
                token_usage,
                "model_dump"
            ):
                token_usage_data = (
                    token_usage.model_dump()
                )

            elif hasattr(
                token_usage,
                "dict"
            ):
                token_usage_data = (
                    token_usage.dict()
                )

            elif isinstance(
                token_usage,
                dict
            ):
                token_usage_data = (
                    token_usage
                )

            else:
                token_usage_data = {}

            # ------------------------------------------------
            # Sources
            # ------------------------------------------------

            sources = [
                {
                    "document_id": "selected_text",
                    "chunk_id": "selected_text_chunk",
                    "content_snippet": (
                        selected_text[:200]
                        + "..."
                        if len(selected_text) > 200
                        else selected_text
                    ),
                    "similarity_score": 1.0
                }
            ]

            # ------------------------------------------------
            # Generated Response
            # ------------------------------------------------

            generated_response = GeneratedResponse(
                session_id=session_id,
                query_id=str(user_query.id),
                content=llm_response.content,
                sources=sources,
                model_used=llm_response.model_used,
                token_usage=token_usage_data
            )

            db.add(generated_response)

            # ------------------------------------------------
            # Commit
            # ------------------------------------------------

            db.commit()

            print(
                "✅ SELECTED TEXT QUERY COMPLETE",
                flush=True
            )

            return {
                "response_id": str(
                    generated_response.id
                ),
                "answer": llm_response.content,
                "sources": sources,
                "session_id": session_id,
                "model_used": llm_response.model_used
            }

        except Exception as e:

            print(
                "🔥🔥 SELECTED TEXT RAG ERROR",
                flush=True
            )

            print(
                "ERROR TYPE:",
                type(e).__name__,
                flush=True
            )

            print(
                "ERROR:",
                repr(e),
                flush=True
            )

            traceback.print_exc()

            try:
                if db is not None:
                    db.rollback()
            except Exception:
                pass

            raise


# ============================================================
# SINGLETON
# ============================================================

rag_service = RAGService()
