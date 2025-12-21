from sqlalchemy import Column, String, Text, DateTime, JSON
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
from ..config.database import Base
import uuid


class UserQuery(Base):
    __tablename__ = "user_queries"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), nullable=True)  # Reference to the chat session (optional)
    content = Column(Text, nullable=False)  # The actual question/query text
    query_embedding = Column(String, nullable=True)  # Embedding vector of the query (stored as string)
    context_info = Column(JSON, default=dict)  # Additional context (selected text, current page, etc.)
    created_at = Column(DateTime(timezone=True), server_default=func.now())