from sqlalchemy import Column, String, Text, DateTime, JSON
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
from ..config.database import Base
import uuid


class GeneratedResponse(Base):
    __tablename__ = "generated_responses"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), nullable=True)  # Reference to the chat session (optional)
    query_id = Column(UUID(as_uuid=True), nullable=False)  # Reference to the original user query
    content = Column(Text, nullable=False)  # The generated response text
    sources = Column(JSON, default=list)  # List of source citations (document IDs, chunk IDs, etc.)
    model_used = Column(String, nullable=False)  # Name of the LLM model used
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    token_usage = Column(JSON, default=dict)  # Information about token usage (input, output, total)