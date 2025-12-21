from pydantic_settings import BaseSettings
from typing import Optional


class Settings(BaseSettings):
    # API Keys and URLs
    openrouter_api_key: str
    qdrant_api_key: Optional[str] = None
    qdrant_url: str
    neon_database_url: str

    # Model configurations
    embedding_model: str = "openai/text-embedding-3-small"
    llm_model: str = "anthropic/claude-sonnet-4.5"

    # Application settings
    app_name: str = "RAG Chatbot API"
    debug: bool = False
    host: str = "0.0.0.0"
    port: int = 8000

    # Qdrant settings
    qdrant_collection_name: str = "document_chunks"

    # Vector settings
    embedding_dimensions: int = 1536  # Updated for openai/text-embedding-3-small model

    class Config:
        env_file = ".env"


settings = Settings()