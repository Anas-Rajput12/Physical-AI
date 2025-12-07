from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, List
import cohere
# Qdrant client is completely removed for now

from config import settings

app = FastAPI(title="Docusaurus RAG API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

cohere_client = cohere.Client(settings.cohere_api_key)

class AskRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None

class AskResponse(BaseModel):
    answer: str

def get_embedding(text: str, input_type: str):
    # This function is still needed for embedding, even if Qdrant search is bypassed
    response = cohere_client.embed(
        texts=[text],
        model="embed-english-v3.0",
        input_type=input_type
    )
    return response.embeddings[0]

def build_rag_prompt(user_query: str, documents: List[dict]):
    return f"""
You are a helpful tutor for a textbook about Physical AI & Humanoid Robotics.

Answer the user's question using ONLY the context below. 
If the answer is not in the context, say you don't see it in the book.

Context:
{documents}

Question:
{user_query}

Answer in clear, structured English suitable for a student.
"""



@app.post("/ask", response_model=AskResponse)
def ask(req: AskRequest):
    # Qdrant search is completely removed for now.
    # The chatbot will directly use Cohere for answering.
    
    try:
        response = cohere_client.chat(
            message=f"Answer the question: '{req.query}'.",
            model="command-r-plus",
            temperature=0.3
        )
        answer = response.text
    except Exception as e:
        answer = f"Error generating answer with Cohere (Qdrant functionality removed): {str(e)}"
    
    return AskResponse(answer=answer)

@app.get("/")
def read_root():
    return {"message": "Docusaurus RAG API is running (Qdrant functionality removed)."}
