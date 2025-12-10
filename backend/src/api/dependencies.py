"""
FastAPI dependencies for authentication and authorization.
Feature: 002-physical-ai-book
"""

from fastapi import Header, HTTPException
from jose import JWTError
from typing import Dict
from ..services.auth_service import verify_token


async def get_current_user(authorization: str = Header(None)) -> Dict:
    """
    Dependency to extract and verify JWT token from Authorization header.

    Args:
        authorization: Authorization header with format "Bearer <token>"

    Returns:
        Dict with user_id and email

    Raises:
        HTTPException: 401 if token is missing, invalid, or expired
    """
    if not authorization:
        raise HTTPException(
            status_code=401,
            detail="Authorization header missing",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Extract token from "Bearer <token>" format
    parts = authorization.split()
    if len(parts) != 2 or parts[0].lower() != "bearer":
        raise HTTPException(
            status_code=401,
            detail="Invalid authorization header format. Expected 'Bearer <token>'",
            headers={"WWW-Authenticate": "Bearer"},
        )

    token = parts[1]

    try:
        payload = verify_token(token)
        return payload  # Contains user_id and email
    except JWTError as e:
        raise HTTPException(
            status_code=401,
            detail=f"Invalid or expired token: {str(e)}",
            headers={"WWW-Authenticate": "Bearer"},
        )
