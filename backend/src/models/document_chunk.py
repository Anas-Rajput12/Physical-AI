from sqlalchemy import Column, String, Integer, Text, DateTime, JSON
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
from ..config.database import Base
import uuid


class DocumentChunk(Base):
    __tablename__ = "document_chunks"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    document_id = Column(UUID(as_uuid=True), nullable=False)  # Reference to the source document
    chunk_index = Column(Integer, nullable=False)  # Position of the chunk within the document
    content = Column(Text, nullable=False)  # The actual text content of the chunk
    embedding_id = Column(String, nullable=True)  # ID reference in the vector database
    token_count = Column(Integer, nullable=False)  # Number of tokens in the chunk
    chunk_metadata = Column(JSON, default=dict)  # Additional metadata (headings, section, etc.)
    created_at = Column(DateTime(timezone=True), server_default=func.now())