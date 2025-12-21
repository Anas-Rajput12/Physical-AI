import os
import re
import uuid
from typing import List, Dict, Any, Tuple
from pathlib import Path
from ..models.document import Document
from ..models.document_chunk import DocumentChunk
from ..services.embedding_service import embedding_service
from ..services.vector_db_service import vector_db_service
from sqlalchemy.orm import Session
from ..config.settings import settings
import tiktoken


class ContentIngestionService:
    def __init__(self):
        self.chunk_size = 500  # Target chunk size in tokens
        self.overlap = 50  # Overlap between chunks in tokens
        self.encoding = tiktoken.encoding_for_model("gpt-3.5-turbo")  # Use for token counting

    def count_tokens(self, text: str) -> int:
        """Count the number of tokens in a text."""
        return len(self.encoding.encode(text))

    def chunk_text(self, text: str, max_chunk_size: int = 500, overlap: int = 50) -> List[Dict[str, Any]]:
        """
        Split text into semantic chunks with overlap.

        Args:
            text: The text to chunk
            max_chunk_size: Maximum size of each chunk in tokens
            overlap: Number of tokens to overlap between chunks

        Returns:
            List of chunks with content and metadata
        """
        # Split the text into sentences
        sentences = re.split(r'(?<=[.!?])\s+', text)

        chunks = []
        current_chunk = ""
        current_token_count = 0

        for sentence in sentences:
            sentence_token_count = self.count_tokens(sentence)

            # If adding this sentence would exceed the chunk size
            if current_token_count + sentence_token_count > max_chunk_size and current_chunk:
                # Save the current chunk
                chunks.append({
                    "content": current_chunk.strip(),
                    "token_count": current_token_count
                })

                # Start a new chunk with overlap
                if overlap > 0:
                    # Find overlap content from the end of current chunk
                    overlap_tokens = self.encoding.encode(current_chunk)
                    if len(overlap_tokens) > overlap:
                        overlap_start = len(overlap_tokens) - overlap
                        overlap_text = self.encoding.decode(overlap_tokens[overlap_start:])
                        current_chunk = overlap_text + " " + sentence
                    else:
                        current_chunk = current_chunk + " " + sentence
                    current_token_count = self.count_tokens(current_chunk)
                else:
                    current_chunk = sentence
                    current_token_count = sentence_token_count
            else:
                # Add sentence to current chunk
                current_chunk += " " + sentence if current_chunk else sentence
                current_token_count += sentence_token_count

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append({
                "content": current_chunk.strip(),
                "token_count": current_token_count
            })

        return chunks

    def load_markdown_files(self, docs_path: str) -> List[Dict[str, Any]]:
        """
        Load all Markdown files from the docs directory.

        Args:
            docs_path: Path to the docs directory

        Returns:
            List of documents with their content and metadata
        """
        documents = []
        docs_dir = Path(docs_path)

        for md_file in docs_dir.rglob("*.md"):
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()

                # Extract title from first heading if available
                title_match = re.search(r'^#\s+(.+)', content, re.MULTILINE)
                title = title_match.group(1) if title_match else md_file.stem

                documents.append({
                    "id": str(uuid.uuid4()),
                    "title": title,
                    "content": content,
                    "path": str(md_file.relative_to(docs_dir)),
                    "filename": md_file.name,
                    "checksum": self._calculate_checksum(content)
                })

        return documents

    def _calculate_checksum(self, content: str) -> str:
        """Calculate a simple checksum for content comparison."""
        import hashlib
        return hashlib.md5(content.encode()).hexdigest()

    def ingest_document(
        self,
        db: Session,
        title: str,
        content: str,
        path: str = "",
        filename: str = "",
        metadata: Dict[str, Any] = None
    ) -> str:
        """
        Ingest a single document into the system.

        Args:
            db: Database session
            title: Title of the document
            content: Content of the document
            path: Path where the document is stored
            filename: Original filename
            metadata: Additional metadata for the document

        Returns:
            Document ID of the ingested document
        """
        if metadata is None:
            metadata = {}

        # Create document record in Postgres
        document = Document(
            title=title,
            path=path,
            filename=filename,
            checksum=self._calculate_checksum(content),
            ingestion_status="processing",
            doc_metadata=metadata
        )
        db.add(document)
        db.flush()  # Get the ID without committing

        document_id = str(document.id)

        try:
            # Update status to processing
            document.ingestion_status = "processing"
            db.commit()

            # Chunk the content
            chunks = self.chunk_text(content)

            # Process each chunk
            for i, chunk_data in enumerate(chunks):
                chunk_content = chunk_data["content"]
                token_count = chunk_data["token_count"]

                # Generate embedding for the chunk
                embedding = embedding_service.generate_embedding(chunk_content)

                # Store in vector database
                chunk_id = str(uuid.uuid4())
                vector_db_service.store_embedding(
                    vector_id=chunk_id,
                    embedding=embedding,
                    document_id=document_id,
                    chunk_content=chunk_content,
                    chunk_metadata={"chunk_index": i, "token_count": token_count}
                )

                # Store chunk metadata in Postgres
                chunk = DocumentChunk(
                    document_id=document_id,
                    chunk_index=i,
                    content=chunk_content,
                    embedding_id=chunk_id,
                    token_count=token_count,
                    chunk_metadata={"chunk_index": i, "token_count": token_count}
                )
                db.add(chunk)

            # Update document status to completed
            document.ingestion_status = "completed"
            db.commit()

            return document_id

        except Exception as e:
            # Update document status to failed
            document.ingestion_status = "failed"
            db.commit()
            raise e

    def ingest_documents_from_directory(self, db: Session, docs_path: str) -> List[str]:
        """
        Ingest all documents from a directory.

        Args:
            db: Database session
            docs_path: Path to the directory containing documents

        Returns:
            List of document IDs that were ingested
        """
        # Load all documents from the directory
        documents = self.load_markdown_files(docs_path)

        ingested_document_ids = []
        for doc_data in documents:
            doc_id = self.ingest_document(
                db=db,
                title=doc_data["title"],
                content=doc_data["content"],
                path=doc_data["path"],
                filename=doc_data["filename"]
            )
            ingested_document_ids.append(doc_id)

        return ingested_document_ids


# Singleton instance
content_ingestion_service = ContentIngestionService()