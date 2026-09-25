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
# /ASK
# ============================================================

@router.post("/ask", response_model=ChatResponse)
async def ask_question(
    request: ChatRequest,
    db: Session = Depends(get_db)
):
    try:
        print("========================================", flush=True)
        print("🔥 /api/ask STARTED", flush=True)
        print("QUESTION:", request.question, flush=True)
        print("SESSION:", request.session_id, flush=True)
        print("========================================", flush=True)

        print("🔥 BEFORE RAG", flush=True)

        result = rag_service.query_knowledge_base(
            db=db,
            question=request.question,
            session_id=request.session_id,
            page_context=request.page_context
        )

        print("🔥 AFTER RAG", flush=True)
        print("RESULT TYPE:", type(result).__name__, flush=True)

        return ChatResponse(
            response_id=result["response_id"],
            answer=result["answer"],
            sources=result.get("sources", []),
            session_id=result.get("session_id"),
            timestamp=result.get("timestamp", time.time())
        )

    except Exception as e:
        print("========================================", flush=True)
        print("🔥🔥 ERROR /api/ask", flush=True)
        print("ERROR TYPE:", type(e).__name__, flush=True)
        print("ERROR:", repr(e), flush=True)
        traceback.print_exc()
        print("========================================", flush=True)

        raise HTTPException(
            status_code=500,
            detail=f"{type(e).__name__}: {str(e)}"
        )


# ============================================================
# /ASK-SELECTED
# ============================================================

@router.post("/ask-selected", response_model=ChatResponse)
async def ask_with_selected_text(
    request: ChatSelectionRequest,
    db: Session = Depends(get_db)
):
    try:
        print("========================================", flush=True)
        print("🔥 /api/ask-selected STARTED", flush=True)
        print("QUESTION:", request.question, flush=True)
        print(
            "SELECTED TEXT LENGTH:",
            len(request.selected_text),
            flush=True
        )
        print("========================================", flush=True)

        print("🔥 BEFORE SELECTED RAG", flush=True)

        result = rag_service.query_selected_text_only(
            db=db,
            question=request.question,
            selected_text=request.selected_text,
            session_id=request.session_id,
            page_context=request.page_context
        )

        print("🔥 AFTER SELECTED RAG", flush=True)

        return ChatResponse(
            response_id=result["response_id"],
            answer=result["answer"],
            sources=result.get("sources", []),
            session_id=result.get("session_id"),
            timestamp=result.get("timestamp", time.time())
        )

    except Exception as e:
        print("========================================", flush=True)
        print("🔥🔥 ERROR /api/ask-selected", flush=True)
        print("ERROR TYPE:", type(e).__name__, flush=True)
        print("ERROR:", repr(e), flush=True)
        traceback.print_exc()
        print("========================================", flush=True)

        raise HTTPException(
            status_code=500,
            detail=f"{type(e).__name__}: {str(e)}"
        )


# ============================================================
# LEGACY /CHAT
# ============================================================

@router.post("/chat", response_model=ChatResponse)
async def ask_question_legacy(
    request: ChatRequest,
    db: Session = Depends(get_db)
):
    try:
        print("🔥 /api/chat ENTERED", flush=True)
        print("QUESTION:", request.question, flush=True)

        result = rag_service.query_knowledge_base(
            db=db,
            question=request.question,
            session_id=request.session_id,
            page_context=request.page_context
        )

        print("🔥 RAG COMPLETED", flush=True)

        return ChatResponse(
            response_id=result["response_id"],
            answer=result["answer"],
            sources=result.get("sources", []),
            session_id=result.get("session_id"),
            timestamp=time.time()
        )

    except Exception as e:
        print("🔥🔥 CHAT ERROR", flush=True)
        print("TYPE:", type(e).__name__, flush=True)
        print("ERROR:", repr(e), flush=True)
        traceback.print_exc()

        raise HTTPException(
            status_code=500,
            detail=f"{type(e).__name__}: {str(e)}"
        )


# ============================================================
# LEGACY /CHAT/SELECTION
# ============================================================

@router.post("/chat/selection", response_model=ChatResponse)
async def ask_with_selected_text_legacy(
    request: ChatSelectionRequest,
    db: Session = Depends(get_db)
):
    try:
        print("========================================", flush=True)
        print("🔥 /api/chat/selection STARTED", flush=True)
        print("QUESTION:", request.question, flush=True)
        print(
            "SELECTED TEXT LENGTH:",
            len(request.selected_text),
            flush=True
        )
        print("========================================", flush=True)

        print("🔥 BEFORE SELECTED RAG", flush=True)

        result = rag_service.query_selected_text_only(
            db=db,
            question=request.question,
            selected_text=request.selected_text,
            session_id=request.session_id,
            page_context=request.page_context
        )

        print("🔥 AFTER SELECTED RAG", flush=True)

        if not isinstance(result, dict):
            raise ValueError(
                f"RAG service returned {type(result).__name__}, expected dict"
            )

        return ChatResponse(
            response_id=result["response_id"],
            answer=result["answer"],
            sources=result.get("sources", []),
            session_id=result.get("session_id"),
            timestamp=result.get("timestamp", time.time())
        )

    except Exception as e:
        print("========================================", flush=True)
        print("🔥🔥 ERROR /api/chat/selection", flush=True)
        print("ERROR TYPE:", type(e).__name__, flush=True)
        print("ERROR:", repr(e), flush=True)
        print("TRACEBACK BELOW:", flush=True)

        traceback.print_exc()

        print("========================================", flush=True)

        raise HTTPException(
            status_code=500,
            detail=(
                "Error processing selected text: "
                f"{type(e).__name__}: {str(e)}"
            )
        )
