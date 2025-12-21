from sqlalchemy import Column, String, Text, DateTime, JSON
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
from ..config.database import Base
import uuid


class Document(Base):
    __tablename__ = "documents"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    filename = Column(String, nullable=False)  # Original filename
    path = Column(Text, nullable=False)  # Path where the document is stored
    title = Column(String, nullable=False)  # Document title
    version = Column(String, default="1.0")  # Version of the document
    checksum = Column(String, nullable=True)  # Checksum to detect changes
    ingestion_status = Column(String, default="pending")  # Status of ingestion ('pending', 'processing', 'completed', 'failed')
    doc_metadata = Column(JSON, default=dict)  # Additional document metadata (author, date, etc.)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())