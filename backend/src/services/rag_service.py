from typing import List, Dict, Any, Optional
from ..services.embedding_service import embedding_service
from ..services.vector_db_service import vector_db_service
from ..services.llm_service import llm_service, LLMResponse
from sqlalchemy.orm import Session
from ..models.user_query import UserQuery
from ..models.generated_response import GeneratedResponse
import uuid


class RAGService:
    def __init__(self):
        self.top_k = 5  # Number of chunks to retrieve
        self.min_similarity_score = 0.3  # Minimum similarity score for relevance

    def query_knowledge_base(
        self,
        db: Session,
        question: str,
        session_id: Optional[str] = None,
        page_context: Optional[Dict[str, Any]] = None,
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Process a query against the knowledge base using RAG.

        Args:
            db: Database session
            question: The user's question
            session_id: Optional session ID for conversation context
            page_context: Optional context about the current page
            selected_text: Optional selected text for context-aware responses

        Returns:
            Dictionary containing the answer, sources, and metadata
        """
        # Generate embedding for the question
        query_embedding = embedding_service.generate_embedding(question)

        # Search for relevant chunks in the vector database
        if page_context and page_context.get("document_id"):
            # If we have a specific document context, search within that document only
            similar_chunks = vector_db_service.search_similar(
                query_embedding=query_embedding,
                limit=self.top_k,
                document_id_filter=page_context.get("document_id")
            )
        else:
            # Otherwise, search across all documents
            similar_chunks = vector_db_service.search_similar(
                query_embedding=query_embedding,
                limit=self.top_k
            )

        # Filter chunks based on minimum similarity score
        relevant_chunks = [
            chunk for chunk in similar_chunks
            if chunk["similarity_score"] >= self.min_similarity_score
        ]

        # Generate response using LLM
        llm_response: LLMResponse = llm_service.generate_response(
            prompt=question,
            context=relevant_chunks,
            selected_text=selected_text
        )

        # Create UserQuery record
        user_query = UserQuery(
            session_id=session_id,
            content=question,
            query_embedding=str(query_embedding)[:100],  # Store first 100 chars as reference
            context_info={
                "page_context": page_context or {},
                "selected_text": selected_text or "",
                "relevant_chunks_count": len(relevant_chunks)
            }
        )
        db.add(user_query)
        db.flush()  # Get the ID without committing

        # Create GeneratedResponse record
        generated_response = GeneratedResponse(
            session_id=session_id,
            query_id=str(user_query.id),
            content=llm_response.content,
            sources=[
                {
                    "document_id": chunk["document_id"],
                    "chunk_id": chunk["id"],
                    "content_snippet": chunk["content"][:200] + "..." if len(chunk["content"]) > 200 else chunk["content"],
                    "similarity_score": chunk["similarity_score"]
                }
                for chunk in relevant_chunks
            ],
            model_used=llm_response.model_used,
            token_usage=llm_response.token_usage.model_dump() if hasattr(llm_response.token_usage, 'model_dump') else llm_response.token_usage.dict() if hasattr(llm_response.token_usage, 'dict') else dict(llm_response.token_usage)
        )
        db.add(generated_response)
        db.commit()

        return {
            "response_id": str(generated_response.id),
            "answer": llm_response.content,
            "sources": [
                {
                    "document_id": chunk["document_id"],
                    "chunk_id": chunk["id"],
                    "content_snippet": chunk["content"][:200] + "..." if len(chunk["content"]) > 200 else chunk["content"],
                    "similarity_score": chunk["similarity_score"]
                }
                for chunk in relevant_chunks
            ],
            "session_id": session_id,
            "relevant_chunks_count": len(relevant_chunks),
            "model_used": llm_response.model_used
        }

    def query_selected_text_only(
        self,
        db: Session,
        question: str,
        selected_text: str,
        session_id: Optional[str] = None,
        page_context: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Process a query based only on the selected text.

        Args:
            db: Database session
            question: The user's question
            selected_text: The text selected by the user
            session_id: Optional session ID for conversation context
            page_context: Optional context about the current page

        Returns:
            Dictionary containing the answer and metadata
        """
        # For selected text only, we'll create a simple context with just the selected text
        context = [{
            "content": selected_text,
            "document_id": "selected_text",
            "id": "selected_text_chunk"
        }]

        # Generate response using LLM with only the selected text as context
        llm_response: LLMResponse = llm_service.generate_response(
            prompt=question,
            context=context,
            selected_text=selected_text
        )

        # Create UserQuery record
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
        db.flush()  # Get the ID without committing

        # Create GeneratedResponse record
        generated_response = GeneratedResponse(
            session_id=session_id,
            query_id=str(user_query.id),
            content=llm_response.content,
            sources=[{
                "document_id": "selected_text",
                "chunk_id": "selected_text_chunk",
                "content_snippet": selected_text[:200] + "..." if len(selected_text) > 200 else selected_text,
                "similarity_score": 1.0  # Perfect match for selected text
            }],
            model_used=llm_response.model_used,
            token_usage=llm_response.token_usage.model_dump() if hasattr(llm_response.token_usage, 'model_dump') else llm_response.token_usage.dict() if hasattr(llm_response.token_usage, 'dict') else dict(llm_response.token_usage)
        )
        db.add(generated_response)
        db.commit()

        return {
            "response_id": str(generated_response.id),
            "answer": llm_response.content,
            "sources": [{
                "document_id": "selected_text",
                "chunk_id": "selected_text_chunk",
                "content_snippet": selected_text[:200] + "..." if len(selected_text) > 200 else selected_text,
                "similarity_score": 1.0
            }],
            "session_id": session_id,
            "model_used": llm_response.model_used
        }


# Singleton instance
rag_service = RAGService()