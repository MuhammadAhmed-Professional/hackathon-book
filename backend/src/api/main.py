"""
FastAPI application main module with CORS configuration and error handling.
"""
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from contextlib import asynccontextmanager
import logging

from src.config import config
from src.db.connection import init_db_pool, close_db_pool
from src.api.endpoints.ping import router as ping_router
from src.api.endpoints.ingest import router as ingest_router
from src.api.endpoints.ask import router as ask_router
from src.api.endpoints.ask_selected import router as ask_selected_router
from src.api.endpoints.skills import router as skills_router
from src.api.endpoints.auth import router as auth_router
from src.api.endpoints.personalization import router as personalization_router
from src.api.endpoints.translate import router as translate_router

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handler for database initialization and cleanup.
    """
    # Startup: Initialize Neon Postgres connection pool
    logger.info("Initializing Neon Postgres connection pool...")
    try:
        init_db_pool()
        logger.info("Database connection pool initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize database pool: {e}")
        # Don't raise - allow app to start for health checks

    yield

    # Shutdown: Close database connections
    logger.info("Closing Neon Postgres connection pool...")
    close_db_pool()
    logger.info("Database connection pool closed")


# Create FastAPI app instance
app = FastAPI(
    title="Physical AI Textbook API",
    description="RAG-based chatbot API for Physical AI & Humanoid Robotics Textbook with Neon Postgres conversation persistence",
    version="2.0.0",
    lifespan=lifespan
)

# Configure CORS middleware
# When allow_credentials=True, you cannot use "*" for allow_origins
# Must specify exact origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Local development
        "http://localhost:3001",  # Alternative local port
        "https://demolinator.github.io",  # GitHub Pages production
        "https://talal.github.io",  # Legacy GitHub Pages (if still in use)
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
    expose_headers=["*"],
)


@app.exception_handler(Exception)
async def qdrant_error_handler(request: Request, exc: Exception):
    """
    Handle Qdrant-related errors gracefully.

    Args:
        request: FastAPI request object
        exc: Exception that was raised

    Returns:
        JSONResponse with error message
    """
    logger.error(f"Qdrant error: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": "Vector database error",
            "message": "An error occurred while accessing the vector database. Please try again later.",
            "detail": str(exc) if config.validate() else "Configuration error",
        },
    )


@app.exception_handler(ValueError)
async def openai_error_handler(request: Request, exc: ValueError):
    """
    Handle OpenAI API errors gracefully.

    Args:
        request: FastAPI request object
        exc: ValueError that was raised

    Returns:
        JSONResponse with error message
    """
    logger.error(f"OpenAI error: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": "AI service error",
            "message": "An error occurred while processing your request. Please try again later.",
            "detail": str(exc),
        },
    )


@app.get("/")
async def root():
    """Root endpoint for health check."""
    return {"message": "Hackathon RAG API", "status": "running"}


# Register API routers
app.include_router(ping_router, tags=["Health"])
app.include_router(ingest_router, tags=["Ingestion"])
app.include_router(ask_router, tags=["Chatbot"])
app.include_router(ask_selected_router, tags=["Chatbot"])
app.include_router(skills_router, tags=["Skills"])
app.include_router(auth_router, tags=["Authentication"])
app.include_router(personalization_router, tags=["Personalization"])
app.include_router(translate_router, tags=["Translation"])

