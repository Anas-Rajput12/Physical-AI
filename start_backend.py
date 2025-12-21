#!/usr/bin/env python3
"""
Backend startup script for the RAG Chatbot System
"""
import os
import sys
from pathlib import Path

# Load environment variables from .env file
try:
    from dotenv import load_dotenv
    backend_dir = Path(__file__).parent / "backend"
    env_path = backend_dir / ".env"
    load_dotenv(env_path)
except ImportError:
    print("[WARNING] python-dotenv not found. Environment variables must be set manually.")
    pass

def check_environment_variables():
    """Check if all required environment variables are set"""
    required_vars = [
        'OPENROUTER_API_KEY',
        'QDRANT_API_KEY',
        'QDRANT_URL',
        'NEON_DATABASE_URL'
    ]

    missing_vars = []
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        print("[ERROR] Missing required environment variables:")
        for var in missing_vars:
            print(f"  - {var}")
        print("\nPlease set these environment variables before running the backend.")
        print("You can create a .env file in the backend directory with these values.")
        return False

    print("[SUCCESS] All required environment variables are set")
    return True

def start_backend():
    """Start the FastAPI backend server"""
    if not check_environment_variables():
        return False

    # Change to the backend directory to ensure proper imports
    original_cwd = os.getcwd()
    backend_dir = Path(__file__).parent / "backend"
    os.chdir(backend_dir)

    # Add the backend/src directory to the Python path
    backend_src = backend_dir / "src"
    sys.path.insert(0, str(backend_src))

    try:
        import uvicorn

        print("[STARTING] Starting RAG Chatbot Backend Server...")
        print("Listening on http://0.0.0.0:8000")
        print("Press Ctrl+C to stop the server")

        # Import the app to ensure it works
        from src.api.main import app

        uvicorn.run(
            app,
            host="0.0.0.0",
            port=8000,
            reload=True
        )
        return True
    except ImportError as e:
        print(f"[ERROR] Import error: {e}")
        print("Make sure you have installed the required dependencies:")
        print("cd backend && pip install -r requirements.txt")
        return False
    except Exception as e:
        print(f"[ERROR] Error starting backend: {e}")
        return False
    finally:
        # Restore original working directory
        os.chdir(original_cwd)

if __name__ == "__main__":
    success = start_backend()
    if not success:
        sys.exit(1)