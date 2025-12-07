from pydantic_settings import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    cohere_api_key: str
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection: str = "docusaurus_book"
    postgres_url: Optional[str] = None

    class Config:
        env_file = ".env"
        extra = "ignore"

settings = Settings()
