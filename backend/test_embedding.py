#!/usr/bin/env python3
"""
Test script to verify the embedding service works with the new model
"""
import os
import sys
from pathlib import Path

# Add the project root to the Python path so we can import properly
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

# Add backend/src to the Python path
backend_src = project_root / "backend" / "src"
sys.path.insert(0, str(backend_src))

from dotenv import load_dotenv

# Load environment variables from the backend .env file
backend_env_path = project_root / "backend" / ".env"
load_dotenv(str(backend_env_path))

# Now import the modules (using absolute imports since we modified sys.path)
from backend.src.services.embedding_service import EmbeddingService
from backend.src.config.settings import settings

def test_embedding():
    print(f"Using embedding model: {settings.embedding_model}")
    print(f"Using embedding dimensions: {settings.embedding_dimensions}")

    # Create embedding service instance
    embedding_service = EmbeddingService()

    # Test embedding generation
    test_text = "This is a test sentence for embedding."

    try:
        print(f"Generating embedding for: '{test_text}'")
        embedding = embedding_service.generate_embedding(test_text)
        print(f"Successfully generated embedding with {len(embedding)} dimensions")
        print(f"First 5 values: {embedding[:5]}")
        print("[SUCCESS] Embedding service is working correctly!")
        return True
    except Exception as e:
        print(f"[ERROR] Error generating embedding: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_embedding()
    if not success:
        sys.exit(1)
    else:
        print("All tests passed!")