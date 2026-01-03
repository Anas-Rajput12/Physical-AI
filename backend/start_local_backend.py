#!/usr/bin/env python3
"""
Local backend startup script for the RAG Chatbot System
"""
import os
import sys
from pathlib import Path
import uvicorn

def start_local_backend():
    """Start the FastAPI backend server locally"""
    # Add the backend/src directory to the Python path
    backend_dir = Path(__file__).parent / "backend"
    src_dir = backend_dir / "src"
    sys.path.insert(0, str(src_dir))

    # Change to the backend directory
    original_cwd = os.getcwd()
    os.chdir(backend_dir)

    try:
        # Import the app after setting up the path
        from api.main import app
        print("[STARTING] Starting RAG Chatbot Backend Server locally...")
        print("Listening on http://127.0.0.1:8000")
        print("Press Ctrl+C to stop the server")

        uvicorn.run(
            app,
            host="127.0.0.1",  # Use localhost instead of 0.0.0.0 for local dev
            port=8000,
            reload=True
        )
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
    start_local_backend()