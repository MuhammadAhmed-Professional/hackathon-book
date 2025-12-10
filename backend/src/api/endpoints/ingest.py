"""
Ingestion endpoint for processing book content.
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Dict, Any
import logging

from src.services.ingestion_service import ingest_book

logger = logging.getLogger(__name__)

router = APIRouter()


class IngestResponse(BaseModel):
    """Response model for ingestion endpoint."""
    status: str
    chapters_processed: int
    total_chunks: int
    message: str


@router.post("/ingest", response_model=IngestResponse)
async def ingest():
    """
    Ingest all book content into Qdrant.
    
    Reads markdown files from frontend/docs, chunks them, creates embeddings,
    and stores them in Qdrant Cloud.
    """
    try:
        result = ingest_book()
        
        return IngestResponse(
            status="success",
            chapters_processed=result["chapters_processed"],
            total_chunks=result["total_chunks"],
            message=f"Successfully ingested {result['total_chunks']} chunks from {result['chapters_processed']} chapters"
        )
    
    except FileNotFoundError as e:
        logger.error(f"File not found: {str(e)}")
        raise HTTPException(status_code=404, detail=f"Docs directory not found: {str(e)}")
    
    except Exception as e:
        logger.error(f"Error during ingestion: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error ingesting book content: {str(e)}")

