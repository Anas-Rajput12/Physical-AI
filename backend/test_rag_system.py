#!/usr/bin/env python3
"""
Test script to verify the complete RAG system functionality
"""
import sys
import os
import time
import requests
import json
from pathlib import Path

# Add the backend src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent / "backend" / "src"))

def test_backend_health():
    """Test the backend health endpoint"""
    print("Testing backend health endpoint...")
    try:
        response = requests.get("http://localhost:8000/api/health", timeout=10)
        if response.status_code == 200:
            health_data = response.json()
            print(f"✓ Backend health check passed: {health_data['status']}")
            return True
        else:
            print(f"✗ Backend health check failed with status {response.status_code}")
            return False
    except requests.exceptions.ConnectionError:
        print("✗ Backend is not running. Please start the backend server first.")
        return False
    except Exception as e:
        print(f"✗ Error during health check: {str(e)}")
        return False

def test_ask_endpoint():
    """Test the /ask endpoint"""
    print("\nTesting /ask endpoint...")
    try:
        payload = {
            "question": "What is this book about?",
            "session_id": "test_session_123",
            "page_context": {
                "url": "http://localhost:3000/test",
                "title": "Test Page",
                "section": "Introduction"
            }
        }

        response = requests.post(
            "http://localhost:8000/api/ask",
            json=payload,
            timeout=30
        )

        if response.status_code == 200:
            data = response.json()
            print(f"✓ /ask endpoint test passed")
            print(f"  Response ID: {data.get('response_id', 'N/A')}")
            print(f"  Answer length: {len(data.get('answer', ''))} characters")
            print(f"  Sources found: {len(data.get('sources', []))}")
            return True
        else:
            print(f"✗ /ask endpoint test failed with status {response.status_code}")
            print(f"  Response: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error during /ask test: {str(e)}")
        return False

def test_ask_selected_endpoint():
    """Test the /ask-selected endpoint"""
    print("\nTesting /ask-selected endpoint...")
    try:
        payload = {
            "question": "What does this selected text mean?",
            "selected_text": "This is some sample text that was selected by the user.",
            "session_id": "test_session_123",
            "page_context": {
                "url": "http://localhost:3000/test",
                "title": "Test Page",
                "section": "Introduction"
            }
        }

        response = requests.post(
            "http://localhost:8000/api/ask-selected",
            json=payload,
            timeout=30
        )

        if response.status_code == 200:
            data = response.json()
            print(f"✓ /ask-selected endpoint test passed")
            print(f"  Response ID: {data.get('response_id', 'N/A')}")
            print(f"  Answer length: {len(data.get('answer', ''))} characters")
            print(f"  Sources found: {len(data.get('sources', []))}")
            return True
        else:
            print(f"✗ /ask-selected endpoint test failed with status {response.status_code}")
            print(f"  Response: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error during /ask-selected test: {str(e)}")
        return False

def test_ingestion():
    """Test the ingestion process with sample documents"""
    print("\nTesting document ingestion...")
    try:
        from backend.scripts.ingest_documents import main as ingest_main
        from backend.src.config.database import SessionLocal

        # Create a temporary directory with sample markdown files
        import tempfile
        import shutil

        with tempfile.TemporaryDirectory() as temp_dir:
            # Create sample markdown files
            docs_dir = Path(temp_dir) / "docs"
            docs_dir.mkdir()

            # Create a sample markdown file
            sample_file = docs_dir / "sample.md"
            sample_file.write_text("""
# Sample Document

This is a sample document for testing the RAG system.

## Section 1

This section contains information about the RAG system implementation.

## Section 2

This section explains how the system works with Docusaurus.

The system uses Qwen embeddings and OpenRouter for LLM responses.
            """)

            # Run ingestion
            db = SessionLocal()
            try:
                # Import the service here to avoid issues with path
                from backend.src.services.content_ingestion_service import content_ingestion_service
                document_ids = content_ingestion_service.ingest_documents_from_directory(db, str(docs_dir))
                print(f"✓ Ingestion test passed. Ingested {len(document_ids)} documents")
                return True
            finally:
                db.close()

    except Exception as e:
        print(f"✗ Error during ingestion test: {str(e)}")
        return False

def main():
    """Main test function"""
    print("🔍 Testing Complete RAG System Functionality")
    print("=" * 50)

    tests = [
        ("Backend Health", test_backend_health),
        ("Ask Endpoint", test_ask_endpoint),
        ("Ask Selected Endpoint", test_ask_selected_endpoint),
        ("Document Ingestion", test_ingestion),
    ]

    results = []
    for test_name, test_func in tests:
        print(f"\n🧪 Running {test_name} test...")
        result = test_func()
        results.append((test_name, result))

    print("\n" + "=" * 50)
    print("📊 Test Results Summary:")
    print("=" * 50)

    passed = 0
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} - {test_name}")
        if result:
            passed += 1

    print(f"\nOverall: {passed}/{len(results)} tests passed")

    if passed == len(results):
        print("🎉 All tests passed! The RAG system is working correctly.")
        return True
    else:
        print("⚠️  Some tests failed. Please check the implementation.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)