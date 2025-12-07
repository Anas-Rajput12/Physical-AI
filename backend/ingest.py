import os
import glob
import uuid
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models
import cohere
import time

# ----------------------------
# Load environment variables
# ----------------------------
load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if not all([COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY]):
    raise ValueError("Please set COHERE_API_KEY, QDRANT_URL, and QDRANT_API_KEY in your .env file.")

# ----------------------------
# Helpers
# ----------------------------

def get_text_from_docs(path: str):
    """Yield the content of all Markdown files recursively in a folder."""
    if not os.path.isdir(path):
        print(f"Error: Directory '{path}' not found.")
        return
        
    all_md_files = glob.glob(os.path.join(path, "**/*.md"), recursive=True)
    if not all_md_files:
        print(f"No Markdown files found in '{path}'")
    else:
        print(f"Found {len(all_md_files)} markdown files to ingest.")
        
    for file in all_md_files:
        with open(file, "r", encoding="utf-8") as f:
            yield f.read()

def chunk_text(text: str, chunk_size: int = 512, chunk_overlap: int = 100):
    """Split text into smaller chunks with overlap."""
    if not isinstance(text, str) or not text.strip():
        return []
    chunks = []
    for i in range(0, len(text), chunk_size - chunk_overlap):
        chunk = text[i:i + chunk_size]
        if chunk.strip():
            chunks.append(chunk)
    return chunks

def get_embeddings(texts: list, client, batch_size: int = 96):
    """Get embeddings from Cohere for a list of texts in batches."""
    if not texts:
        return []
    
    print(f"Getting embeddings for {len(texts)} chunks...")
    all_embeddings = []
    for i in range(0, len(texts), batch_size):
        batch = texts[i:i + batch_size]
        try:
            response = client.embed(
                texts=batch,
                model="embed-english-v3.0",
                input_type="search_document"
            )
            all_embeddings.extend(response.embeddings)
            print(f"  - Embedded batch {i//batch_size + 1}/{(len(texts) + batch_size - 1)//batch_size}")
            time.sleep(1) # Respect rate limits
        except cohere.core.api_error.ApiError as e:
            print(f"Error embedding batch: {e}")
            # Pad with None for failed embeddings to maintain alignment
            all_embeddings.extend([None] * len(batch))
            
    return all_embeddings

# ----------------------------
# Main ingestion
# ----------------------------

def ingest_docs(docs_path, collection_name="docusaurus_book"):
    print("Starting ingestion process...")
    cohere_client = cohere.Client(COHERE_API_KEY)
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

    # Check if collection exists, otherwise create
    try:
        collections = qdrant_client.get_collections()
        if collection_name not in [c.name for c in collections.collections]:
            qdrant_client.recreate_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE)
            )
            print(f"Created collection: {collection_name}")
        else:
            print(f"Collection already exists: {collection_name}")
    except Exception as e:
        print(f"Error checking/creating collection: {e}")
        return

    points = []
    for doc in get_text_from_docs(docs_path):
        chunks = chunk_text(doc)
        if not chunks:
            print(f"No chunks found in document snippet: {doc[:50]}...")
            continue
            
        embeddings = get_embeddings(chunks, cohere_client)
        
        for chunk, embedding in zip(chunks, embeddings):
            if chunk and embedding:
                points.append(
                    models.PointStruct(
                        id=str(uuid.uuid4()),
                        vector=embedding,
                        payload={"text": chunk}
                    )
                )

    if points:
        print(f"Upserting {len(points)} points to Qdrant...")
        qdrant_client.upsert(
            collection_name=collection_name,
            points=points,
            wait=True
        )
        print(f"Ingestion complete. Total points: {len(points)}")
    else:
        print("No new points to upsert. Check your docs folder and chunking logic.")

# ----------------------------
# Run script
# ----------------------------

if __name__ == "__main__":
    docs_path = os.path.join(os.path.dirname(__file__), '..', 'docs')
    ingest_docs(docs_path=docs_path)