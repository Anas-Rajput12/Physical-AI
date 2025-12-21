import os
import requests
import json
from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from ..config.settings import settings


class TokenUsage(BaseModel):
    prompt_tokens: Optional[int] = None
    completion_tokens: Optional[int] = None
    total_tokens: Optional[int] = None
    # Additional fields that might be returned by the API
    cost: Optional[float] = None
    prompt_tokens_details: Optional[Dict[str, Any]] = None
    completion_tokens_details: Optional[Dict[str, Any]] = None
    cost_details: Optional[Dict[str, Any]] = None


class LLMResponse(BaseModel):
    content: str
    model_used: str
    token_usage: TokenUsage


class LLMService:
    def __init__(self):
        self.api_key = settings.openrouter_api_key
        self.model = settings.llm_model
        self.base_url = "https://openrouter.ai/api/v1/chat/completions"

    def generate_response(
        self,
        prompt: str,
        context: Optional[List[Dict[str, str]]] = None,
        selected_text: Optional[str] = None
    ) -> LLMResponse:
        """
        Generate a response from the LLM based on the prompt and context.

        Args:
            prompt: The user's question
            context: List of relevant document chunks as context
            selected_text: Optional selected text for context-aware responses

        Returns:
            LLMResponse containing the generated content, model used, and token usage
        """
        # Build the system message with context
        system_content = (
            "You are a helpful assistant that answers questions based on provided documentation. "
            "Only use information from the provided context to answer questions. "
            "If the answer is not in the context, clearly state that the information is not available in the documentation."
        )

        # Add selected text context if provided
        if selected_text:
            system_content += f"\n\nThe user has selected this text for context: '{selected_text}'"

        # Build the message history
        messages = [
            {"role": "system", "content": system_content}
        ]

        # Add context if available
        if context:
            context_str = "\n\nRelevant context from documentation:\n"
            for i, chunk in enumerate(context):
                context_str += f"Document {i+1}: {chunk.get('content', '')}\n"
            messages.append({"role": "system", "content": context_str})

        # Add the user's question
        messages.append({"role": "user", "content": prompt})

        # Prepare the request payload
        payload = {
            "model": self.model,
            "messages": messages,
            "temperature": 0.7,
            "max_tokens": 1000
        }

        # Make the API request
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }

        try:
            response = requests.post(self.base_url, json=payload, headers=headers)
            response.raise_for_status()

            result = response.json()

            # Extract the content and token usage
            content = result['choices'][0]['message']['content']
            raw_token_usage = result.get('usage', {})

            # Create TokenUsage object from the raw data
            token_usage = TokenUsage(
                prompt_tokens=raw_token_usage.get('prompt_tokens'),
                completion_tokens=raw_token_usage.get('completion_tokens'),
                total_tokens=raw_token_usage.get('total_tokens'),
                cost=raw_token_usage.get('cost'),
                prompt_tokens_details=raw_token_usage.get('prompt_tokens_details'),
                completion_tokens_details=raw_token_usage.get('completion_tokens_details'),
                cost_details=raw_token_usage.get('cost_details')
            )

            return LLMResponse(
                content=content,
                model_used=result.get('model', self.model),
                token_usage=token_usage
            )
        except requests.exceptions.RequestException as e:
            raise Exception(f"Error calling LLM API: {str(e)}")
        except (KeyError, IndexError) as e:
            raise Exception(f"Unexpected response format from LLM API: {str(e)}, response: {result if 'result' in locals() else response.text}")


# Singleton instance
llm_service = LLMService()