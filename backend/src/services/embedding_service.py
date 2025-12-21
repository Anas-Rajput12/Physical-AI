import os
import requests
import numpy as np
from typing import List, Union
from ..config.settings import settings


class EmbeddingService:
    def __init__(self):
        self.api_key = settings.openrouter_api_key
        self.model = settings.embedding_model
        self.base_url = "https://openrouter.ai/api/v1/embeddings"

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text using OpenRouter API.

        Args:
            text: The text to embed

        Returns:
            List of floats representing the embedding vector
        """
        payload = {
            "model": self.model,
            "input": text
        }

        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }

        try:
            response = requests.post(self.base_url, json=payload, headers=headers)
            response.raise_for_status()

            result = response.json()
            embedding = result['data'][0]['embedding']
            return embedding
        except requests.exceptions.RequestException as e:
            raise Exception(f"Error calling embedding API: {str(e)}")
        except (KeyError, IndexError) as e:
            raise Exception(f"Unexpected response format from embedding API: {str(e)}, response: {result if 'result' in locals() else response.text}")

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts.

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        embeddings = []
        for text in texts:
            embedding = self.generate_embedding(text)
            embeddings.append(embedding)
        return embeddings


# Singleton instance
embedding_service = EmbeddingService()