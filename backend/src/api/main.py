from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session
from ..config.database import get_db
from ..config.settings import settings
from .chat_endpoints import router as chat_router
from .ingestion_endpoints import router as ingestion_router
import time
import logging


# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title=settings.app_name,
    description="API for RAG Chatbot System",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat_router, prefix="/api", tags=["chat"])
app.include_router(ingestion_router, prefix="/api", tags=["ingestion"])


@app.middleware("http")
async def add_process_time_header(request: Request, call_next):
    """Middleware to add processing time to response headers."""
    start_time = time.time()
    response = await call_next(request)
    process_time = time.time() - start_time
    response.headers["X-Process-Time"] = str(process_time)
    return response


@app.get("/api/health")
async def health_check(db: Session = Depends(get_db)):
    """Health check endpoint to verify service availability."""
    try:
        # Check if we can connect to the database
        from sqlalchemy import text
        db.execute(text("SELECT 1"))

        # In a real implementation, you might want to check:
        # - Vector database connectivity
        # - LLM API connectivity
        # - Embedding service connectivity

        return {
            "status": "healthy",
            "timestamp": time.time(),
            "services": {
                "postgres": "available",
                "vector_db": "assumed_available",  # Would require actual connection check
                "llm_api": "assumed_available",    # Would require actual connection check
            }
        }
    except Exception as e:
        raise HTTPException(status_code=503, detail=f"Service unavailable: {str(e)}")


@app.on_event("startup")
async def startup_event():
    """Actions to perform when the application starts."""
    logger.info(f"Starting {settings.app_name} on {settings.host}:{settings.port}")


@app.on_event("shutdown")
async def shutdown_event():
    """Actions to perform when the application shuts down."""
    logger.info(f"Shutting down {settings.app_name}")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "api.main:app",
        host=settings.host,
        port=settings.port,
        reload=settings.debug
    )