from fastapi import FastAPI, HTTPException, Depends, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from sqlalchemy.orm import Session
from sqlalchemy import text

from ..config.database import get_db
from ..config.settings import settings
from .chat_endpoints import router as chat_router
from .ingestion_endpoints import router as ingestion_router

import time
import logging
import traceback


# ============================================================
# LOGGING
# ============================================================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(name)s - %(message)s"
)

logger = logging.getLogger(__name__)


# ============================================================
# CREATE FASTAPI APP
# ============================================================

app = FastAPI(
    title=settings.app_name,
    description="API for RAG Chatbot System",
    version="1.0.0"
)


# ============================================================
# CORS
# ============================================================

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ============================================================
# GLOBAL EXCEPTION HANDLER
# ============================================================

@app.exception_handler(Exception)
async def global_exception_handler(
    request: Request,
    exc: Exception
):
    print("========================================", flush=True)
    print("🔥🔥 GLOBAL EXCEPTION", flush=True)
    print("METHOD:", request.method, flush=True)
    print("PATH:", request.url.path, flush=True)
    print("ERROR TYPE:", type(exc).__name__, flush=True)
    print("ERROR:", repr(exc), flush=True)
    print("TRACEBACK:", flush=True)

    traceback.print_exc()

    print("========================================", flush=True)

    return JSONResponse(
        status_code=500,
        content={
            "detail": f"{type(exc).__name__}: {str(exc)}",
            "path": request.url.path
        }
    )


# ============================================================
# INCLUDE ROUTERS
# ============================================================

app.include_router(
    chat_router,
    prefix="/api",
    tags=["chat"]
)

app.include_router(
    ingestion_router,
    prefix="/api",
    tags=["ingestion"]
)


# ============================================================
# PROCESS TIME MIDDLEWARE
# ============================================================

@app.middleware("http")
async def add_process_time_header(
    request: Request,
    call_next
):
    start_time = time.time()

    print(
        f"➡️ REQUEST {request.method} {request.url.path}",
        flush=True
    )

    try:
        response = await call_next(request)

        process_time = time.time() - start_time

        response.headers["X-Process-Time"] = str(
            process_time
        )

        print(
            f"⬅️ RESPONSE {response.status_code} "
            f"{request.method} {request.url.path} "
            f"({process_time:.4f}s)",
            flush=True
        )

        return response

    except Exception as e:
        process_time = time.time() - start_time

        print("========================================", flush=True)
        print("🔥 MIDDLEWARE ERROR", flush=True)
        print("METHOD:", request.method, flush=True)
        print("PATH:", request.url.path, flush=True)
        print("ERROR TYPE:", type(e).__name__, flush=True)
        print("ERROR:", repr(e), flush=True)
        print("TIME:", process_time, flush=True)
        print("TRACEBACK:", flush=True)

        traceback.print_exc()

        print("========================================", flush=True)

        raise


# ============================================================
# ROOT
# ============================================================

@app.get("/")
async def root():
    return {
        "status": "ok",
        "message": "RAG Chatbot API is running",
        "version": "1.0.0"
    }


# ============================================================
# HEALTH CHECK
# ============================================================

@app.get("/api/health")
async def health_check(
    db: Session = Depends(get_db)
):
    try:
        print("🔥 HEALTH CHECK STARTED", flush=True)

        db.execute(text("SELECT 1"))

        print("✅ DATABASE CONNECTION OK", flush=True)

        return {
            "status": "healthy",
            "timestamp": time.time(),
            "services": {
                "postgres": "available",
                "vector_db": "assumed_available",
                "llm_api": "assumed_available"
            }
        }

    except Exception as e:
        print("========================================", flush=True)
        print("🔥 HEALTH CHECK ERROR", flush=True)
        print("ERROR TYPE:", type(e).__name__, flush=True)
        print("ERROR:", repr(e), flush=True)
        traceback.print_exc()
        print("========================================", flush=True)

        raise HTTPException(
            status_code=503,
            detail=f"Service unavailable: {type(e).__name__}: {str(e)}"
        )


# ============================================================
# STARTUP
# ============================================================

@app.on_event("startup")
async def startup_event():

    print("========================================", flush=True)
    print("🚀 APPLICATION STARTING", flush=True)
    print(
        f"APP NAME: {settings.app_name}",
        flush=True
    )
    print(
        f"HOST: {settings.host}",
        flush=True
    )
    print(
        f"PORT: {settings.port}",
        flush=True
    )
    print("========================================", flush=True)

    logger.info(
        f"Starting {settings.app_name} "
        f"on {settings.host}:{settings.port}"
    )


# ============================================================
# SHUTDOWN
# ============================================================

@app.on_event("shutdown")
async def shutdown_event():

    print("========================================", flush=True)
    print("🛑 APPLICATION SHUTTING DOWN", flush=True)
    print("========================================", flush=True)

    logger.info(
        f"Shutting down {settings.app_name}"
    )


# ============================================================
# LOCAL DEVELOPMENT
# ============================================================

if __name__ == "__main__":

    import uvicorn

    uvicorn.run(
        "src.api.main:app",
        host=settings.host,
        port=settings.port,
        reload=settings.debug
    )
