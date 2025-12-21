from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
from ..config.database import get_db
from ..services.content_ingestion_service import content_ingestion_service


router = APIRouter()


class Document(BaseModel):
    id: Optional[str] = None
    title: str
    content: str
    source_url: Optional[str] = None
    doc_metadata: Optional[Dict[str, Any]] = {}


class IngestionRequest(BaseModel):
    documents: List[Document]
    options: Optional[Dict[str, Any]] = {"chunk_size": 500, "overlap": 50, "reprocess": False}


class IngestionResponse(BaseModel):
    job_id: str
    status: str
    documents_processed: int
    estimated_completion: float
    timestamp: float


class IngestionStatusResponse(BaseModel):
    job_id: str
    status: str
    progress: Dict[str, Any]
    timestamp: float
    details: Optional[str] = None


class DocumentListResponse(BaseModel):
    documents: List[Dict[str, Any]]
    total_count: int
    timestamp: float


@router.post("/ingest", response_model=IngestionResponse)
async def ingest_documents(
    request: IngestionRequest,
    db: Session = Depends(get_db)
):
    """
    Ingest new documents into the knowledge base.
    """
    try:
        # Process each document
        processed_count = 0
        for doc in request.documents:
            # In a real implementation, we would track the job and process asynchronously
            # For now, we'll process synchronously
            content_ingestion_service.ingest_document(
                db=db,
                title=doc.title,
                content=doc.content,
                path=doc.source_url or "",
                filename=doc.title.replace(" ", "_").lower() + ".md",
                metadata=doc.doc_metadata
            )
            processed_count += 1

        import time
        return IngestionResponse(
            job_id="sync_job_" + str(int(time.time())),
            status="completed",
            documents_processed=processed_count,
            estimated_completion=time.time(),
            timestamp=time.time()
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error ingesting documents: {str(e)}")


@router.get("/ingest/status/{job_id}", response_model=IngestionStatusResponse)
async def get_ingestion_status(
    job_id: str,
    db: Session = Depends(get_db)
):
    """
    Check the status of an ingestion job.
    """
    # In a real implementation, this would track actual job status
    # For now, we'll return a mock response
    import time
    return IngestionStatusResponse(
        job_id=job_id,
        status="completed",
        progress={
            "total_documents": 1,
            "processed_documents": 1,
            "failed_documents": 0,
            "percentage": 100.0
        },
        timestamp=time.time(),
        details="Synchronous processing completed"
    )


@router.get("/documents", response_model=DocumentListResponse)
async def list_documents(
    db: Session = Depends(get_db)
):
    """
    List all documents in the knowledge base.
    """
    try:
        from ..models.document import Document as DocumentModel
        documents = db.query(DocumentModel).all()

        doc_list = []
        for doc in documents:
            doc_list.append({
                "id": str(doc.id),
                "title": doc.title,
                "source_url": doc.path,
                "chunk_count": 0,  # In a real implementation, we'd count related chunks
                "created_at": doc.created_at.isoformat() if doc.created_at else None,
                "updated_at": doc.updated_at.isoformat() if doc.updated_at else None,
                "ingestion_status": doc.ingestion_status
            })

        import time
        return DocumentListResponse(
            documents=doc_list,
            total_count=len(doc_list),
            timestamp=time.time()
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error listing documents: {str(e)}")


@router.delete("/ingest/{document_id}", response_model=Dict[str, str])
async def remove_document(
    document_id: str,
    db: Session = Depends(get_db)
):
    """
    Remove a document from the knowledge base.
    """
    try:
        from ..models.document import Document as DocumentModel
        from ..models.document_chunk import DocumentChunk as DocumentChunkModel

        # Find the document
        document = db.query(DocumentModel).filter(DocumentModel.id == document_id).first()
        if not document:
            raise HTTPException(status_code=404, detail="Document not found")

        # Remove from vector database
        from ..services.vector_db_service import vector_db_service
        vector_db_service.delete_document_chunks(document_id)

        # Remove chunks from Postgres
        db.query(DocumentChunkModel).filter(DocumentChunkModel.document_id == document_id).delete()

        # Remove document from Postgres
        db.delete(document)
        db.commit()

        return {
            "document_id": document_id,
            "status": "removed"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error removing document: {str(e)}")