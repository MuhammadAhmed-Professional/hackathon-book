"""
Neon Serverless Postgres connection pool.
Feature: 002-physical-ai-book
"""

import psycopg2
from psycopg2 import pool, OperationalError
from typing import Optional
import os
import logging
import time

logger = logging.getLogger(__name__)


# Global connection pool instance
_connection_pool: Optional[pool.SimpleConnectionPool] = None


def init_db_pool():
    """Initialize Neon Postgres connection pool on application startup."""
    global _connection_pool

    database_url = os.getenv("NEON_DATABASE_URL")
    if not database_url:
        raise ValueError("NEON_DATABASE_URL environment variable not set")

    try:
        # For Neon serverless, use smaller pool and configure for serverless
        _connection_pool = psycopg2.pool.SimpleConnectionPool(
            minconn=1,
            maxconn=5,  # Reduced for serverless
            dsn=database_url,
            # Add connection parameters for better serverless compatibility
            connect_timeout=10,
            keepalives=1,
            keepalives_idle=30,
            keepalives_interval=10,
            keepalives_count=5
        )
        logger.info("✓ Neon Postgres connection pool initialized")
        print("✓ Neon Postgres connection pool initialized")
    except Exception as e:
        logger.error(f"✗ Failed to initialize database pool: {e}")
        print(f"✗ Failed to initialize database pool: {e}")
        raise


def get_db_connection(max_retries=3):
    """
    Get a connection from the pool with retry logic for Neon serverless.
    
    Args:
        max_retries: Maximum number of retry attempts
        
    Returns:
        Database connection
    """
    if _connection_pool is None:
        raise RuntimeError("Database pool not initialized. Call init_db_pool() first.")
    
    for attempt in range(max_retries):
        try:
            conn = _connection_pool.getconn()
            # Check if connection is closed
            if conn.closed:
                # Connection is closed, return it to pool and try again
                if attempt < max_retries - 1:
                    try:
                        _connection_pool.putconn(conn, close=True)
                    except:
                        pass
                    time.sleep(0.2 * (attempt + 1))
                    continue
                else:
                    raise OperationalError("All connection attempts returned closed connections")
            return conn
        except (OperationalError, psycopg2.InterfaceError) as e:
            logger.warning(f"Connection attempt {attempt + 1} failed: {str(e)}")
            if attempt < max_retries - 1:
                time.sleep(0.5 * (attempt + 1))  # Exponential backoff
                continue
            else:
                logger.error(f"Failed to get database connection after {max_retries} attempts")
                raise


def release_db_connection(conn):
    """Return a connection to the pool."""
    if _connection_pool is None:
        raise RuntimeError("Database pool not initialized")
    _connection_pool.putconn(conn)


def close_db_pool():
    """Close all connections in the pool (on application shutdown)."""
    global _connection_pool
    if _connection_pool:
        _connection_pool.closeall()
        _connection_pool = None
        print("✓ Neon Postgres connection pool closed")
