#!/usr/bin/env python3
"""
Script to load Markdown files from docs/ directory, chunk them,
generate embeddings, and store in vector and metadata databases.
"""
import sys
import os
from pathlib import Path

# Add the src directory to the path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from sqlalchemy.orm import Session
from src.config.database import SessionLocal
from src.services.content_ingestion_service import content_ingestion_service


def get_database_session() -> Session:
    """Get a database session."""
    return SessionLocal()


def main(docs_path: str = "./docs"):
    """Main function to ingest documents from a directory."""
    print(f"Starting document ingestion from: {docs_path}")

    # Create database session
    db = get_database_session()

    try:
        # Ingest documents from the specified directory
        document_ids = content_ingestion_service.ingest_documents_from_directory(
            db=db,
            docs_path=docs_path
        )

        print(f"Successfully ingested {len(document_ids)} documents:")
        for doc_id in document_ids:
            print(f"  - {doc_id}")

    except Exception as e:
        print(f"Error during document ingestion: {str(e)}")
        raise
    finally:
        db.close()


if __name__ == "__main__":
    # Use command line argument if provided, otherwise default to ./docs
    docs_dir = sys.argv[1] if len(sys.argv) > 1 else "./docs"
    main(docs_dir)