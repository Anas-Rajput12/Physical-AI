from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import Optional, List, Dict, Any
import time
import traceback

from ..config.database import get_db
from ..services.rag_service import rag_service


router = APIRouter()


# ============================================================
# REQUEST MODELS
# ============================================================

class ChatRequest(BaseModel):
    question: str
    session_id: Optional[str] = None
    page_context: Optional[Dict[str, Any]] = None


class ChatSelectionRequest(BaseModel):
    question: str
    selected_text: str
    session_id: Optional[str] = None
    page_context: Optional[Dict[str, Any]] = None


# ============================================================
# RESPONSE MODEL
# ============================================================

class ChatResponse(BaseModel):
    response_id: str
    answer: str
    sources: List[Dict[str, Any]]
    session_id: Optional[str] = None
    timestamp: float


# ============================================================
# ASK QUESTION
# ============================================================

@router.post("/ask", response_model=ChatResponse)
async def ask_question(
    request: ChatRequest,
    db: Session = Depends(get_db)
):
    """
    Process a user question against the full knowledge base
    and return a contextual answer.
    """

    try:
        print("========================================")
        print("CHAT /ASK REQUEST")
        print("Question:", request.question)
        print("Session ID:", request.session_id)
        print("========================================")

        result = rag_service.query_knowledge_base(
            db=db,
            question=request.question,
            session_id=request.session_id,
            page_context=request.page_context
        )

        print("RAG RESULT SUCCESS")

        return ChatResponse(
            response_id=result["response_id"],
            answer=result["answer"],
            sources=result["sources"],
            session_id=result.get("session_id"),
            timestamp=result.get("timestamp", time.time())
        )

    except Exception as e:
        print("========================================")
        print("🔥 ERROR IN /ASK")
        print("ERROR TYPE:", type(e).__name__)
        print("ERROR:", str(e))
        print("TRACEBACK:")
        traceback.print_exc()
        print("========================================")

        raise HTTPException(
            status_code=500,
            detail=f"Error processing question: {str(e)}"
        )


# ============================================================
# ASK WITH SELECTED TEXT
# ============================================================

@router.post("/ask-selected", response_model=ChatResponse)
async def ask_with_selected_text(
    request: ChatSelectionRequest,
    db: Session = Depends(get_db)
):
    """
    Process a question based on user-selected text
    from the current page.
    """

    try:
        print("========================================")
        print("CHAT /ASK-SELECTED REQUEST")
        print("Question:", request.question)
        print("Selected text length:", len(request.selected_text))
        print("Session ID:", request.session_id)
        print("========================================")

        result = rag_service.query_selected_text_only(
            db=db,
            question=request.question,
            selected_text=request.selected_text,
            session_id=request.session_id,
            page_context=request.page_context
        )

        print("SELECTED TEXT RAG RESULT SUCCESS")

        return ChatResponse(
            response_id=result["response_id"],
            answer=result["answer"],
            sources=result["sources"],
            session_id=result.get("session_id"),
            timestamp=result.get("timestamp", time.time())
        )

    except Exception as e:
        print("========================================")
        print("🔥 ERROR IN /ASK-SELECTED")
        print("ERROR TYPE:", type(e).__name__)
        print("ERROR:", str(e))
        print("TRACEBACK:")
        traceback.print_exc()
        print("========================================")

        raise HTTPException(
            status_code=500,
            detail=f"Error processing question with selected text: {str(e)}"
        )


# ============================================================
# LEGACY /CHAT ENDPOINT
# ============================================================

@router.post("/chat", response_model=ChatResponse)
async def ask_question_legacy(
    request: ChatRequest,
    db: Session = Depends(get_db)
):
    """
    LEGACY:
    Process a user question against the full knowledge base
    and return a contextual answer.
    """

    try:
        print("========================================")
        print("🔥 LEGACY /CHAT REQUEST")
        print("Question:", request.question)
        print("Session ID:", request.session_id)
        print("Page Context:", request.page_context)
        print("========================================")

        result = rag_service.query_knowledge_base(
            db=db,
            question=request.question,
            session_id=request.session_id,
            page_context=request.page_context
        )

        print("LEGACY /CHAT RAG RESULT SUCCESS")

        return ChatResponse(
            response_id=result["response_id"],
            answer=result["answer"],
            sources=result["sources"],
            session_id=result.get("session_id"),
            timestamp=result.get("timestamp", time.time())
        )

    except Exception as e:
        print("========================================")
        print("🔥🔥🔥 ERROR IN LEGACY /CHAT")
        print("ERROR TYPE:", type(e).__name__)
        print("ERROR:", str(e))
        print("TRACEBACK:")
        traceback.print_exc()
        print("========================================")

        raise HTTPException(
            status_code=500,
            detail=f"Error processing question: {str(e)}"
        )


# ============================================================
# LEGACY /CHAT/SELECTION ENDPOINT
# ============================================================

@router.post("/chat/selection", response_model=ChatResponse)
async def ask_with_selected_text_legacy(
    request: ChatSelectionRequest,
    db: Session = Depends(get_db)
):
    """
    LEGACY:
    Process a question based on user-selected text only.
    """

    try:
        print("========================================")
        print("LEGACY /CHAT/SELECTION REQUEST")
        print("Question:", request.question)
        print("Selected text length:", len(request.selected_text))
        print("Session ID:", request.session_id)
        print("========================================")

        result = rag_service.query_selected_text_only(
            db=db,
            question=request.question,
            selected_text=request.selected_text,
            session_id=request.session_id,
            page_context=request.page_context
        )

        print("LEGACY SELECTION RAG RESULT SUCCESS")

        return ChatResponse(
            response_id=result["response_id"],
            answer=result["answer"],
            sources=result["sources"],
            session_id=result.get("session_id"),
            timestamp=result.get("timestamp", time.time())
        )

    except Exception as e:
        print("========================================")
        print("🔥 ERROR IN LEGACY /CHAT/SELECTION")
        print("ERROR TYPE:", type(e).__name__)
        print("ERROR:", str(e))
        print("TRACEBACK:")
        traceback.print_exc()
        print("========================================")

        raise HTTPException(
            status_code=500,
            detail=f"Error processing question with selected text: {str(e)}"
        )
