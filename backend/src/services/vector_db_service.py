from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.exceptions import UnexpectedResponse
from typing import List, Dict, Any, Optional
from ..config.settings import settings


class VectorDBService:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False,
        )

        self.collection_name = settings.qdrant_collection_name
        self.embedding_size = settings.embedding_dimensions
        self._initialized = False

    def _ensure_initialized(self):
        """Initialize Qdrant collection when it is actually needed."""

        if self._initialized:
            return

        try:
            collection_info = self.client.get_collection(
                self.collection_name
            )

            existing_size = collection_info.config.params.vectors.size

            if existing_size != self.embedding_size:
                print(
                    f"Vector dimension mismatch: "
                    f"expected {self.embedding_size}, "
                    f"got {existing_size}"
                )

                self.client.delete_collection(
                    self.collection_name
                )

                self._create_collection()

            else:
                print(
                    f"Collection exists with correct dimensions: "
                    f"{self.embedding_size}"
                )

        except UnexpectedResponse as e:
            # Collection probably doesn't exist.
            print(
                f"Qdrant collection not found. Creating "
                f"'{self.collection_name}'..."
            )

            self._create_collection()

        except Exception as e:
            # Connection/authentication/configuration errors
            # should NOT be treated as a missing collection.
            print(f"Qdrant connection error: {e}")
            raise

        self._initialized = True

    def _create_collection(self):
        """Create the Qdrant collection and payload index."""

        self.client.create_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(
                size=self.embedding_size,
                distance=models.Distance.COSINE,
            ),
        )

        self.client.create_payload_index(
            collection_name=self.collection_name,
            field_name="document_id",
            field_schema=models.PayloadSchemaType.KEYWORD,
        )

        print(
            f"Qdrant collection '{self.collection_name}' "
            f"created successfully."
        )

    def store_embedding(
        self,
        vector_id: str,
        embedding: List[float],
        document_id: str,
        chunk_content: str,
        chunk_metadata: Dict[str, Any],
    ):
        self._ensure_initialized()

        payload = {
            "document_id": document_id,
            "content": chunk_content,
            "metadata": chunk_metadata,
        }

        self.client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=vector_id,
                    vector=embedding,
                    payload=payload,
                )
            ],
        )

    def search_similar(
        self,
        query_embedding: List[float],
        limit: int = 5,
        document_id_filter: Optional[str] = None,
    ) -> List[Dict[str, Any]]:

        self._ensure_initialized()

        filters = None

        if document_id_filter:
            filters = models.Filter(
                must=[
                    models.FieldCondition(
                        key="document_id",
                        match=models.MatchValue(
                            value=document_id_filter
                        ),
                    )
                ]
            )

        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            query_filter=filters,
            limit=limit,
            with_payload=True,
        )

        results = []

        for hit in search_results:
            results.append(
                {
                    "id": hit.id,
                    "document_id": hit.payload.get(
                        "document_id"
                    ),
                    "content": hit.payload.get("content"),
                    "metadata": hit.payload.get(
                        "metadata", {}
                    ),
                    "similarity_score": hit.score,
                }
            )

        return results

    def delete_document_chunks(
        self,
        document_id: str,
    ):
        self._ensure_initialized()

        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.FilterSelector(
                filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="document_id",
                            match=models.MatchValue(
                                value=document_id
                            ),
                        )
                    ]
                )
            ),
        )

    def clear_collection(self):
        self._ensure_initialized()

        all_points = self.client.scroll(
            collection_name=self.collection_name,
            limit=10000,
        )[0]

        if all_points:
            point_ids = [
                point.id for point in all_points
            ]

            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=point_ids
                ),
            )


# Singleton instance
vector_db_service = VectorDBService()
