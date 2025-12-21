from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from ..config.database import get_db
from ..services.rag_service import rag_service


router = APIRouter()


class ChatRequest(BaseModel):
    question: str
    session_id: Optional[str] = None
    page_context: Optional[Dict[str, Any]] = None


class ChatSelectionRequest(BaseModel):
    question: str
    selected_text: str
    session_id: Optional[str] = None
    page_context: Optional[Dict[str, Any]] = None


class ChatResponse(BaseModel):
    response_id: str
    answer: str
    sources: List[Dict[str, Any]]
    session_id: Optional[str] = None
    timestamp: float


@router.post("/ask", response_model=ChatResponse)
async def ask_question(
    request: ChatRequest,
    db: Session = Depends(get_db)
):
    """
    Process a user question against the full knowledge base and return a contextual answer.
    """
    try:
        # Process the query using RAG service
        result = rag_service.query_knowledge_base(
            db=db,
            question=request.question,
            session_id=request.session_id,
            page_context=request.page_context
        )

        return ChatResponse(
            response_id=result["response_id"],
            answer=result["answer"],
            sources=result["sources"],
            session_id=result["session_id"],
            timestamp=result.get("timestamp", __import__('time').time())
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing question: {str(e)}")


@router.post("/ask-selected", response_model=ChatResponse)
async def ask_with_selected_text(
    request: ChatSelectionRequest,
    db: Session = Depends(get_db)
):
    """
    Process a question based on user-selected text from the current page.
    """
    try:
        # Process the query using RAG service with selected text only
        result = rag_service.query_selected_text_only(
            db=db,
            question=request.question,
            selected_text=request.selected_text,
            session_id=request.session_id,
            page_context=request.page_context
        )

        return ChatResponse(
            response_id=result["response_id"],
            answer=result["answer"],
            sources=result["sources"],
            session_id=result["session_id"],
            timestamp=result.get("timestamp", __import__('time').time())
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing question with selected text: {str(e)}")


# Keep the original endpoints for backward compatibility
@router.post("/chat", response_model=ChatResponse)
async def ask_question_legacy(
    request: ChatRequest,
    db: Session = Depends(get_db)
):
    """
    LEGACY: Process a user question against the full knowledge base and return a contextual answer.
    """
    try:
        # Process the query using RAG service
        result = rag_service.query_knowledge_base(
            db=db,
            question=request.question,
            session_id=request.session_id,
            page_context=request.page_context
        )

        return ChatResponse(
            response_id=result["response_id"],
            answer=result["answer"],
            sources=result["sources"],
            session_id=result["session_id"],
            timestamp=result.get("timestamp", __import__('time').time())
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing question: {str(e)}")


@router.post("/chat/selection", response_model=ChatResponse)
async def ask_with_selected_text_legacy(
    request: ChatSelectionRequest,
    db: Session = Depends(get_db)
):
    """
    LEGACY: Process a question based on user-selected text from the current page.
    """
    try:
        # Process the query using RAG service with selected text only
        result = rag_service.query_selected_text_only(
            db=db,
            question=request.question,
            selected_text=request.selected_text,
            session_id=request.session_id,
            page_context=request.page_context
        )

        return ChatResponse(
            response_id=result["response_id"],
            answer=result["answer"],
            sources=result["sources"],
            session_id=result["session_id"],
            timestamp=result.get("timestamp", __import__('time').time())
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing question with selected text: {str(e)}")