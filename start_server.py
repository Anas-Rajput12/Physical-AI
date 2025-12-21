#!/usr/bin/env python3
"""
Script to start the backend server with proper error handling
"""
import os
import sys
from pathlib import Path
import logging

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def start_server():
    # Add backend/src to the Python path
    project_root = Path(__file__).parent
    backend_src = project_root / "backend" / "src"
    sys.path.insert(0, str(backend_src))

    # Load environment variables
    from dotenv import load_dotenv
    backend_env_file = project_root / "backend" / ".env"
    load_dotenv(str(backend_env_file))

    # Import and check settings
    from src.config.settings import settings
    logger.info(f"Using embedding model: {settings.embedding_model}")
    logger.info(f"Using LLM model: {settings.llm_model}")
    logger.info(f"Using embedding dimensions: {settings.embedding_dimensions}")

    # Test the embedding service
    from src.services.embedding_service import EmbeddingService
    try:
        embedding_service = EmbeddingService()
        logger.info(f"Embedding service initialized with model: {embedding_service.model}")

        # Test a simple embedding
        test_embedding = embedding_service.generate_embedding("test")
        logger.info(f"Test embedding successful, dimensions: {len(test_embedding)}")
    except Exception as e:
        logger.error(f"Embedding service test failed: {e}")
        return False

    # Import and start the app
    from src.api.main import app
    import uvicorn

    logger.info("Starting server on http://127.0.0.1:8001")

    try:
        uvicorn.run(
            app,
            host="127.0.0.1",
            port=8001,
            reload=False
        )
        return True
    except Exception as e:
        logger.error(f"Failed to start server: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = start_server()
    if not success:
        logger.error("Server startup failed")
        sys.exit(1)