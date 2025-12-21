from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from ..config.settings import settings


class VectorDBService:
    def __init__(self):
        # Initialize Qdrant client
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False
        )
        self.collection_name = settings.qdrant_collection_name
        self.embedding_size = settings.embedding_dimensions

        # Create collection if it doesn't exist
        self._create_collection_if_not_exists()

    def _create_collection_if_not_exists(self):
        """Create the Qdrant collection if it doesn't exist or recreate if dimensions don't match."""
        try:
            # Try to get collection info to see if it exists
            collection_info = self.client.get_collection(self.collection_name)

            # Check if the vector size matches our expected size
            if collection_info.config.params.vectors.size != self.embedding_size:
                print(f"Vector dimension mismatch detected: expected {self.embedding_size}, got {collection_info.config.params.vectors.size}")
                print("Recreating collection with correct dimensions...")

                # Delete the existing collection with wrong dimensions
                self.client.delete_collection(self.collection_name)

                # Create new collection with correct dimensions
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=self.embedding_size,
                        distance=models.Distance.COSINE
                    )
                )

                # Create payload index for faster filtering
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="document_id",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                print("Collection recreated successfully with correct dimensions.")
            else:
                print(f"Collection exists with correct dimensions: {self.embedding_size}")

        except:
            # Collection doesn't exist, create it
            print("Creating new collection...")
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=self.embedding_size,
                    distance=models.Distance.COSINE
                )
            )

            # Create payload index for faster filtering
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="document_id",
                field_schema=models.PayloadSchemaType.KEYWORD
            )
            print("New collection created successfully.")

    def store_embedding(
        self,
        vector_id: str,
        embedding: List[float],
        document_id: str,
        chunk_content: str,
        chunk_metadata: Dict[str, Any]
    ):
        """
        Store a single embedding in Qdrant.

        Args:
            vector_id: Unique identifier for the vector
            embedding: The embedding vector
            document_id: ID of the source document
            chunk_content: The content of the text chunk
            chunk_metadata: Additional metadata for the chunk
        """
        # Prepare the payload
        payload = {
            "document_id": document_id,
            "content": chunk_content,
            "metadata": chunk_metadata
        }

        # Upsert the vector
        self.client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=vector_id,
                    vector=embedding,
                    payload=payload
                )
            ]
        )

    def search_similar(
        self,
        query_embedding: List[float],
        limit: int = 5,
        document_id_filter: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings in the vector database.

        Args:
            query_embedding: The embedding to search for
            limit: Maximum number of results to return
            document_id_filter: Optional filter to search within a specific document

        Returns:
            List of similar chunks with their content and metadata
        """
        # Prepare filters if document_id is provided
        filters = None
        if document_id_filter:
            filters = models.Filter(
                must=[
                    models.FieldCondition(
                        key="document_id",
                        match=models.MatchValue(value=document_id_filter)
                    )
                ]
            )

        # Perform the search
        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            query_filter=filters,
            limit=limit,
            with_payload=True
        )

        # Format results
        results = []
        for hit in search_results:
            results.append({
                "id": hit.id,
                "document_id": hit.payload.get("document_id"),
                "content": hit.payload.get("content"),
                "metadata": hit.payload.get("metadata", {}),
                "similarity_score": hit.score
            })

        return results

    def delete_document_chunks(self, document_id: str):
        """
        Delete all chunks associated with a document.

        Args:
            document_id: ID of the document to delete
        """
        # Delete points where document_id matches
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.FilterSelector(
                filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="document_id",
                            match=models.MatchValue(value=document_id)
                        )
                    ]
                )
            )
        )

    def clear_collection(self):
        """Clear all vectors from the collection."""
        # Get all point IDs and delete them
        all_points = self.client.scroll(
            collection_name=self.collection_name,
            limit=10000  # Adjust based on your needs
        )[0]

        if all_points:
            point_ids = [point.id for point in all_points]
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=point_ids)
            )


# Singleton instance
vector_db_service = VectorDBService()