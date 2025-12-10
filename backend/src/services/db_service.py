"""
Database service for Physical AI Textbook (Neon Postgres operations).
Feature: 002-physical-ai-book
"""

import json
import logging
import time
from typing import Optional, List, Dict
from datetime import datetime
import psycopg2
from psycopg2 import OperationalError, InterfaceError
from ..db.connection import get_db_connection, release_db_connection
from ..models.user import User, UserCreate, SoftwareBackground, HardwareBackground
from ..models.conversation import Conversation, ConversationCreate, Citation

logger = logging.getLogger(__name__)


def create_user(user_data: UserCreate, password_hash: str) -> User:
    """
    Create a new user in the database.

    Args:
        user_data: User signup data
        password_hash: Bcrypt-hashed password

    Returns:
        Created User object
    """
    import logging
    logger = logging.getLogger(__name__)
    
    conn = get_db_connection()
    try:
        with conn.cursor() as cur:
            # Convert Pydantic models to dicts, then to JSON strings
            try:
                software_bg_dict = user_data.software_background.dict() if hasattr(user_data.software_background, 'dict') else user_data.software_background
                hardware_bg_dict = user_data.hardware_background.dict() if hasattr(user_data.hardware_background, 'dict') else user_data.hardware_background
            except Exception as e:
                logger.error(f"Error converting background to dict: {str(e)}")
                raise
            
            # Ensure we have dicts before JSON encoding
            if not isinstance(software_bg_dict, dict):
                logger.error(f"software_bg_dict is not a dict, type: {type(software_bg_dict)}")
                raise ValueError(f"software_background must be a dict or Pydantic model, got {type(software_bg_dict)}")
            if not isinstance(hardware_bg_dict, dict):
                logger.error(f"hardware_bg_dict is not a dict, type: {type(hardware_bg_dict)}")
                raise ValueError(f"hardware_background must be a dict or Pydantic model, got {type(hardware_bg_dict)}")
            
            try:
                software_bg_json = json.dumps(software_bg_dict)
                hardware_bg_json = json.dumps(hardware_bg_dict)
            except Exception as e:
                logger.error(f"Error encoding JSON: {str(e)}")
                raise
            
            cur.execute(
                """
                INSERT INTO users (email, password_hash, software_background, hardware_background)
                VALUES (%s, %s, %s, %s)
                RETURNING id, email, software_background, hardware_background, created_at, last_login
                """,
                (
                    user_data.email,
                    password_hash,
                    software_bg_json,
                    hardware_bg_json
                )
            )
            row = cur.fetchone()
            conn.commit()

            # Handle JSON fields - PostgreSQL may return as dict or string
            def parse_json_field(field):
                if field is None:
                    return None
                # If already a dict, return it directly
                if isinstance(field, dict):
                    return field
                # If it's a string, parse it
                if isinstance(field, (str, bytes, bytearray)):
                    try:
                        return json.loads(field)
                    except (json.JSONDecodeError, TypeError):
                        # If parsing fails, return as-is (might already be parsed)
                        return field
                # For any other type, return as-is
                return field
            
            # Parse JSON fields safely
            software_bg_parsed = parse_json_field(row[2])
            hardware_bg_parsed = parse_json_field(row[3])
            
            return User(
                id=row[0],
                email=row[1],
                software_background=SoftwareBackground(**software_bg_parsed) if software_bg_parsed else None,
                hardware_background=HardwareBackground(**hardware_bg_parsed) if hardware_bg_parsed else None,
                created_at=row[4],
                last_login=row[5]
            )
    except Exception as e:
        conn.rollback()
        raise
    finally:
        release_db_connection(conn)


def get_user_by_email(email: str) -> Optional[Dict]:
    """
    Get user by email (including password_hash for authentication).

    Args:
        email: User's email address

    Returns:
        Dict with user data including password_hash, or None if not found
    """
    max_retries = 3
    for attempt in range(max_retries):
        conn = None
        try:
            conn = get_db_connection()
            with conn.cursor() as cur:
                cur.execute(
                    """
                    SELECT id, email, password_hash, software_background, hardware_background, created_at, last_login
                    FROM users WHERE email = %s
                    """,
                    (email,)
                )
                row = cur.fetchone()

                if not row:
                    return None

                # Helper function to parse JSON fields safely
                def parse_json_field(field):
                    if field is None:
                        return None
                    if isinstance(field, dict):
                        return field
                    if isinstance(field, (str, bytes, bytearray)):
                        try:
                            return json.loads(field)
                        except (json.JSONDecodeError, TypeError):
                            return field
                    return field
                
                result = {
                    "id": row[0],
                    "email": row[1],
                    "password_hash": row[2],
                    "software_background": parse_json_field(row[3]),
                    "hardware_background": parse_json_field(row[4]),
                    "created_at": row[5],
                    "last_login": row[6]
                }
                release_db_connection(conn)
                return result
        except (OperationalError, InterfaceError) as e:
            logger.warning(f"Database connection error (attempt {attempt + 1}/{max_retries}): {str(e)}")
            if conn:
                try:
                    conn.rollback()
                    release_db_connection(conn)
                except:
                    pass
            if attempt < max_retries - 1:
                time.sleep(0.5 * (attempt + 1))
                continue
            else:
                logger.error(f"Failed to get user after {max_retries} attempts")
                raise
        except Exception as e:
            if conn:
                try:
                    conn.rollback()
                    release_db_connection(conn)
                except:
                    pass
            raise


def get_user_by_id(user_id: int) -> Optional[Dict]:
    """
    Get user by ID (excluding password_hash for security).

    Args:
        user_id: User's ID

    Returns:
        Dict with user data (no password_hash), or None if not found
    """
    conn = get_db_connection()
    try:
        with conn.cursor() as cur:
            cur.execute(
                """
                SELECT id, email, software_background, hardware_background, created_at, last_login
                FROM users WHERE id = %s
                """,
                (user_id,)
            )
            row = cur.fetchone()

            if not row:
                return None

            # Helper function to parse JSON fields safely
            def parse_json_field(field):
                if field is None:
                    return None
                if isinstance(field, dict):
                    return field
                if isinstance(field, (str, bytes, bytearray)):
                    try:
                        return json.loads(field)
                    except (json.JSONDecodeError, TypeError):
                        return field
                return field
            
            return {
                "id": row[0],
                "email": row[1],
                "software_background": parse_json_field(row[2]),
                "hardware_background": parse_json_field(row[3]),
                "created_at": row[4],
                "last_login": row[5]
            }
    finally:
        release_db_connection(conn)


def update_last_login(user_id: int):
    """Update user's last_login timestamp."""
    conn = get_db_connection()
    try:
        with conn.cursor() as cur:
            cur.execute(
                "UPDATE users SET last_login = NOW() WHERE id = %s",
                (user_id,)
            )
            conn.commit()
    except Exception as e:
        conn.rollback()
        raise
    finally:
        release_db_connection(conn)


def save_conversation(conv_data: ConversationCreate) -> Conversation:
    """
    Save a conversation to the database.

    Args:
        conv_data: Conversation data to persist

    Returns:
        Created Conversation object
    """
    conn = get_db_connection()
    try:
        with conn.cursor() as cur:
            # Convert citations to JSON
            citations_json = json.dumps([c.dict() for c in conv_data.citations])

            cur.execute(
                """
                INSERT INTO conversations
                (user_id, session_id, question, answer, citations, question_type)
                VALUES (%s, %s, %s, %s, %s, %s)
                RETURNING id, user_id, session_id, question, answer, citations, question_type, timestamp
                """,
                (
                    conv_data.user_id,
                    conv_data.session_id,
                    conv_data.question,
                    conv_data.answer,
                    citations_json,
                    conv_data.question_type
                )
            )
            row = cur.fetchone()
            conn.commit()

            return Conversation(
                id=row[0],
                user_id=row[1],
                session_id=row[2],
                question=row[3],
                answer=row[4],
                citations=[Citation(**c) for c in json.loads(row[5])] if row[5] else [],
                question_type=row[6],
                timestamp=row[7]
            )
    except Exception as e:
        conn.rollback()
        raise
    finally:
        release_db_connection(conn)


def get_user_conversations(user_id: int, limit: int = 50) -> List[Conversation]:
    """
    Get recent conversations for a logged-in user.

    Args:
        user_id: User's database ID
        limit: Maximum number of conversations to return

    Returns:
        List of Conversation objects, most recent first
    """
    conn = get_db_connection()
    try:
        with conn.cursor() as cur:
            cur.execute(
                """
                SELECT id, user_id, session_id, question, answer, citations, question_type, timestamp
                FROM conversations
                WHERE user_id = %s
                ORDER BY timestamp DESC
                LIMIT %s
                """,
                (user_id, limit)
            )
            rows = cur.fetchall()

            return [
                Conversation(
                    id=row[0],
                    user_id=row[1],
                    session_id=row[2],
                    question=row[3],
                    answer=row[4],
                    citations=[Citation(**c) for c in json.loads(row[5])] if row[5] else [],
                    question_type=row[6],
                    timestamp=row[7]
                )
                for row in rows
            ]
    finally:
        release_db_connection(conn)


def get_session_conversations(session_id: str, limit: int = 50) -> List[Conversation]:
    """
    Get recent conversations for an anonymous session.

    Args:
        session_id: Anonymous session UUID
        limit: Maximum number of conversations to return

    Returns:
        List of Conversation objects, most recent first
    """
    conn = get_db_connection()
    try:
        with conn.cursor() as cur:
            cur.execute(
                """
                SELECT id, user_id, session_id, question, answer, citations, question_type, timestamp
                FROM conversations
                WHERE session_id = %s
                ORDER BY timestamp DESC
                LIMIT %s
                """,
                (session_id, limit)
            )
            rows = cur.fetchall()

            return [
                Conversation(
                    id=row[0],
                    user_id=row[1],
                    session_id=row[2],
                    question=row[3],
                    answer=row[4],
                    citations=[Citation(**c) for c in json.loads(row[5])] if row[5] else [],
                    question_type=row[6],
                    timestamp=row[7]
                )
                for row in rows
            ]
    finally:
        release_db_connection(conn)
