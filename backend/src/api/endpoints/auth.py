"""
Authentication endpoints for Physical AI Textbook (Better-auth with user background).
Feature: 002-physical-ai-book
"""
from fastapi import APIRouter, HTTPException, Depends, Header
from pydantic import BaseModel, EmailStr
from typing import Optional
import logging
import re

from src.services.auth_service import hash_password, verify_password, create_access_token, verify_token
from src.services.db_service import create_user, get_user_by_email, update_last_login
from src.models.user import UserCreate, SoftwareBackground, HardwareBackground

logger = logging.getLogger(__name__)

router = APIRouter()


class SignupRequest(BaseModel):
    """Request model for user signup."""
    email: EmailStr
    password: str
    software_background: SoftwareBackground
    hardware_background: HardwareBackground


class SigninRequest(BaseModel):
    """Request model for user signin."""
    email: EmailStr
    password: str


class AuthResponse(BaseModel):
    """Response model for authentication endpoints."""
    user_id: int
    email: str
    token: str


class UserProfile(BaseModel):
    """User profile response model."""
    user_id: int
    email: str
    software_background: Optional[dict] = None
    hardware_background: Optional[dict] = None
    created_at: str
    last_login: Optional[str] = None


def get_current_user(authorization: Optional[str] = Header(None)) -> dict:
    """
    Dependency to extract and verify JWT token from Authorization header.

    Args:
        authorization: Authorization header (Bearer <token>)

    Returns:
        Dict with user_id and email

    Raises:
        HTTPException: If token is missing or invalid
    """
    if not authorization:
        raise HTTPException(status_code=401, detail="Authorization header missing")

    if not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Invalid authorization format. Expected: Bearer <token>")

    token = authorization[7:]  # Remove "Bearer " prefix

    try:
        payload = verify_token(token)
        return payload
    except Exception as e:
        logger.error(f"Token verification failed: {str(e)}")
        raise HTTPException(status_code=401, detail="Invalid or expired token")


@router.post("/auth/signup", response_model=AuthResponse)
async def signup(request: SignupRequest):
    """
    Sign up a new user with email/password and background information (T078).

    Args:
        request: SignupRequest with email, password, software_background, hardware_background

    Returns:
        AuthResponse with user_id, email, and JWT token
    """
    try:
        # Validate email format (already done by EmailStr type)
        # Validate password strength (minimum 8 characters, maximum 200 characters)
        if len(request.password) < 8:
            raise HTTPException(
                status_code=400,
                detail="Password must be at least 8 characters long"
            )
        if len(request.password) > 200:
            raise HTTPException(
                status_code=400,
                detail="Password must be no more than 200 characters long"
            )

        # Check email uniqueness
        existing_user = get_user_by_email(request.email)
        if existing_user:
            raise HTTPException(
                status_code=400,
                detail="Email already registered"
            )

        # Hash password
        password_hash = hash_password(request.password)

        # Create user in database
        # Note: password_hash is passed separately to create_user, so password field is optional
        user_data = UserCreate(
            email=request.email,
            software_background=request.software_background,
            hardware_background=request.hardware_background
        )

        user = create_user(user_data, password_hash)

        # Generate JWT token
        token = create_access_token(user.id, user.email)

        logger.info(f"User signed up: {user.email} (ID: {user.id})")

        return AuthResponse(
            user_id=user.id,
            email=user.email,
            token=token
        )

    except HTTPException:
        raise
    except Exception as e:
        import traceback
        logger.error(f"Signup error: {str(e)}")
        logger.error(f"Full traceback: {traceback.format_exc()}")
        raise HTTPException(
            status_code=500,
            detail=f"Signup failed: {str(e)}"
        )


@router.post("/auth/signin", response_model=AuthResponse)
async def signin(request: SigninRequest):
    """
    Sign in an existing user with email/password (T079).

    Args:
        request: SigninRequest with email and password

    Returns:
        AuthResponse with user_id, email, and JWT token
    """
    try:
        # Get user by email
        user_data = get_user_by_email(request.email)

        if not user_data:
            raise HTTPException(
                status_code=401,
                detail="Invalid email or password"
            )

        # Verify password
        if not verify_password(request.password, user_data["password_hash"]):
            raise HTTPException(
                status_code=401,
                detail="Invalid email or password"
            )

        # Update last_login timestamp
        try:
            update_last_login(user_data["id"])
        except Exception as db_error:
            logger.warning(f"Failed to update last_login: {str(db_error)}")
            # Don't fail signin if last_login update fails

        # Generate JWT token
        token = create_access_token(user_data["id"], user_data["email"])

        logger.info(f"User signed in: {user_data['email']} (ID: {user_data['id']})")

        return AuthResponse(
            user_id=user_data["id"],
            email=user_data["email"],
            token=token
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Signin error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Signin failed: {str(e)}"
        )


@router.get("/auth/me", response_model=UserProfile)
async def get_me(current_user: dict = Depends(get_current_user)):
    """
    Get current authenticated user's profile (T080).

    Args:
        current_user: User data from JWT token (injected by get_current_user dependency)

    Returns:
        UserProfile with user details and background information
    """
    try:
        user_id = current_user["user_id"]

        # Get user profile from database
        user_data = get_user_by_email(current_user["email"])

        if not user_data:
            raise HTTPException(status_code=404, detail="User not found")

        return UserProfile(
            user_id=user_data["id"],
            email=user_data["email"],
            software_background=user_data.get("software_background"),
            hardware_background=user_data.get("hardware_background"),
            created_at=user_data["created_at"].isoformat() if user_data.get("created_at") else None,
            last_login=user_data["last_login"].isoformat() if user_data.get("last_login") else None
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Get user profile error: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to retrieve user profile: {str(e)}"
        )
