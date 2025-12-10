"""
Configuration module for loading environment variables and application settings.
"""
import os
from typing import Optional
from dotenv import load_dotenv
import logging

# Load environment variables from .env file
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class Config:
    """Application configuration loaded from environment variables."""

    # OpenAI Configuration
    OPENAI_API_KEY: Optional[str] = os.getenv("OPENAI_API_KEY")

    # Qdrant Configuration
    QDRANT_URL: Optional[str] = os.getenv("QDRANT_URL")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")

    # Neon Serverless Postgres Configuration
    NEON_DATABASE_URL: Optional[str] = os.getenv("NEON_DATABASE_URL")

    # Better-auth Configuration
    BETTER_AUTH_SECRET: Optional[str] = os.getenv("BETTER_AUTH_SECRET")

    # API Configuration
    API_BASE_URL: str = os.getenv("API_BASE_URL", "http://localhost:8000")
    
    @classmethod
    def validate(cls) -> bool:
        """
        Validate that all required configuration values are present.
        
        Returns:
            bool: True if all required values are present, False otherwise
        """
        missing = []
        
        if not cls.OPENAI_API_KEY:
            missing.append("OPENAI_API_KEY")
        if not cls.QDRANT_URL:
            missing.append("QDRANT_URL")
        if not cls.QDRANT_API_KEY:
            missing.append("QDRANT_API_KEY")
        if not cls.NEON_DATABASE_URL:
            missing.append("NEON_DATABASE_URL")
        if not cls.BETTER_AUTH_SECRET:
            missing.append("BETTER_AUTH_SECRET")
        
        if missing:
            logger.error(f"Missing required environment variables: {', '.join(missing)}")
            return False
        
        logger.info("Configuration validated successfully")
        return True


# Global config instance
config = Config()

