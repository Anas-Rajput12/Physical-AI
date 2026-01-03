import os
import sys
from pathlib import Path

# Add the project root and backend/src to the Python path so we can import properly
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
from backend.src.config.settings import settings

print("Configuration Check:")
print(f"Embedding model: {settings.embedding_model}")
print(f"LLM model: {settings.llm_model}")
print(f"Embedding dimensions: {settings.embedding_dimensions}")

# Test embedding service
from backend.src.services.embedding_service import EmbeddingService
try:
    embedding_service = EmbeddingService()
    print(f"Embedding service model: {embedding_service.model}")

    # Test a simple embedding
    test_text = "test"
    embedding = embedding_service.generate_embedding(test_text)
    print(f"Test embedding successful: {len(embedding)} dimensions")
except Exception as e:
    print(f"Embedding test failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Import and run the app
from backend.src.api.main import app
import uvicorn

print("Starting server on http://127.0.0.1:8001...")
uvicorn.run(app, host="127.0.0.1", port=8001, reload=False)