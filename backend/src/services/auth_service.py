"""
Authentication service for Physical AI Textbook (Better-auth integration).
Feature: 002-physical-ai-book
"""

import bcrypt
from jose import JWTError, jwt
from datetime import datetime, timedelta
import os
import hashlib
from typing import Optional

# Bcrypt has a 72-byte limit, so we pre-hash longer passwords with SHA-256
BCRYPT_MAX_LENGTH = 72

# JWT configuration
SECRET_KEY = os.getenv("BETTER_AUTH_SECRET")
if not SECRET_KEY:
    raise ValueError("BETTER_AUTH_SECRET environment variable not set")

ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 60 * 24  # 24 hours


def _preprocess_password(password: str) -> str:
    """
    Preprocess password for bcrypt (which has a 72-byte limit).
    If password is longer than 72 bytes, hash it with SHA-256 first.
    
    Args:
        password: Plain password string
        
    Returns:
        Password (or SHA-256 hash if original was > 72 bytes)
    """
    password_bytes = password.encode('utf-8')
    password_length = len(password_bytes)
    
    if password_length > BCRYPT_MAX_LENGTH:
        # Hash with SHA-256 to get a fixed 32-byte output (64 hex chars = 64 bytes)
        sha256_hash = hashlib.sha256(password_bytes).hexdigest()
        # Double-check the hash is under 72 bytes when encoded
        hash_bytes = sha256_hash.encode('utf-8')
        if len(hash_bytes) > BCRYPT_MAX_LENGTH:
            # Should never happen, but truncate if it does
            return sha256_hash[:BCRYPT_MAX_LENGTH]
        return sha256_hash
    
    # Even if under 72 bytes, ensure it's safe by checking encoding
    if password_length > BCRYPT_MAX_LENGTH - 10:  # Leave some margin
        # For passwords close to the limit, also hash to be safe
        return hashlib.sha256(password_bytes).hexdigest()
    
    return password


def hash_password(plain_password: str) -> str:
    """
    Hash a plain password using bcrypt.
    Handles passwords longer than 72 bytes by pre-hashing with SHA-256.
    
    Args:
        plain_password: Plain text password
        
    Returns:
        Bcrypt hashed password
    """
    import logging
    logger = logging.getLogger(__name__)
    
    # Preprocess password to ensure it's always safe for bcrypt
    original_bytes = plain_password.encode('utf-8')
    original_length = len(original_bytes)
    
    # If password is > 72 bytes, always use SHA-256 hash (64 bytes when encoded)
    if original_length > BCRYPT_MAX_LENGTH:
        logger.info(f"Password length {original_length} bytes exceeds bcrypt limit, using SHA-256 pre-hash")
        # Use SHA-256 hash which is always 64 bytes when encoded
        processed_password = hashlib.sha256(original_bytes).hexdigest()
    else:
        processed_password = plain_password
    
    # Final byte-level check and truncation if needed (shouldn't be needed, but be safe)
    processed_bytes = processed_password.encode('utf-8')
    processed_length = len(processed_bytes)
    
    if processed_length >= BCRYPT_MAX_LENGTH:
        logger.warning(f"Processed password {processed_length} bytes still >= {BCRYPT_MAX_LENGTH}, truncating at byte level")
        # Truncate at byte level, leaving margin
        processed_bytes = processed_bytes[:BCRYPT_MAX_LENGTH - 5]  # Leave 5 byte margin for safety
        processed_password = processed_bytes.decode('utf-8', errors='ignore')
        processed_length = len(processed_password.encode('utf-8'))
        logger.info(f"After truncation: {processed_length} bytes")
    
    # Verify final length is safe
    final_bytes = processed_password.encode('utf-8')
    if len(final_bytes) >= BCRYPT_MAX_LENGTH:
        # This should never happen, but if it does, use a minimal safe hash
        logger.error(f"Final check failed: {len(final_bytes)} bytes. Using minimal 32-byte hash.")
        minimal_hash = hashlib.sha256(original_bytes).hexdigest()[:32]  # 32 chars = 32 bytes
        processed_password = minimal_hash
    
    try:
        # Use bcrypt directly to avoid passlib initialization issues
        password_bytes = processed_password.encode('utf-8')
        # Ensure password is under 72 bytes
        if len(password_bytes) >= BCRYPT_MAX_LENGTH:
            password_bytes = password_bytes[:BCRYPT_MAX_LENGTH - 1]
        salt = bcrypt.gensalt()
        hashed = bcrypt.hashpw(password_bytes, salt)
        return hashed.decode('utf-8')
    except Exception as e:
        error_msg = str(e)
        logger.error(f"Password hashing failed. Original: {original_length} bytes, Processed: {len(processed_password.encode('utf-8'))} bytes. Error: {error_msg}")
        # Re-raise with more context
        raise ValueError(f"Failed to hash password: {error_msg}")


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a plain password against a hashed password.
    Handles passwords longer than 72 bytes by pre-hashing with SHA-256.
    
    Args:
        plain_password: Plain text password to verify
        hashed_password: Bcrypt hashed password from database
        
    Returns:
        True if password matches, False otherwise
    """
    processed_password = _preprocess_password(plain_password)
    # Use bcrypt directly to avoid passlib initialization issues
    password_bytes = processed_password.encode('utf-8')
    # Ensure password is under 72 bytes
    if len(password_bytes) >= BCRYPT_MAX_LENGTH:
        password_bytes = password_bytes[:BCRYPT_MAX_LENGTH - 1]
    hash_bytes = hashed_password.encode('utf-8')
    return bcrypt.checkpw(password_bytes, hash_bytes)


def create_access_token(user_id: int, email: str) -> str:
    """
    Create a JWT access token for authenticated user.

    Args:
        user_id: User's database ID
        email: User's email address

    Returns:
        JWT token string
    """
    expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode = {
        "sub": str(user_id),  # Subject (user ID)
        "email": email,
        "exp": expire
    }
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


def verify_token(token: str) -> dict:
    """
    Verify and decode a JWT token.

    Args:
        token: JWT token string

    Returns:
        Decoded token payload with user_id and email

    Raises:
        JWTError: If token is invalid or expired
    """
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        user_id: str = payload.get("sub")
        email: str = payload.get("email")

        if user_id is None or email is None:
            raise JWTError("Invalid token payload")

        return {
            "user_id": int(user_id),
            "email": email
        }
    except JWTError as e:
        raise JWTError(f"Token verification failed: {str(e)}")
