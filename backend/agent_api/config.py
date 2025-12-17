"""Configuration management for RAG Agent API.

Loads and validates environment variables, provides constants and settings.
"""

import os
from pathlib import Path
from typing import Optional

from dotenv import load_dotenv
from pydantic import field_validator
from pydantic_settings import BaseSettings

# Load environment variables from .env file
load_dotenv()


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # OpenAI Configuration
    openai_api_key: str
    openai_model: str = "gpt-4o-mini"
    openai_embedding_model: str = "text-embedding-3-small"

    # Qdrant Configuration
    qdrant_url: str = "http://localhost:6333"
    qdrant_api_key: Optional[str] = None
    qdrant_collection: str = "textbook_chunks"

    # PostgreSQL Configuration
    database_url: str

    # API Configuration
    api_host: str = "0.0.0.0"
    api_port: int = 8000
    api_rate_limit: int = 100  # requests per minute per API key
    log_level: str = "INFO"

    # Agent Configuration
    agent_system_prompt_path: str = "agent_api/prompts/system.txt"
    confidence_threshold_high: float = 0.85
    confidence_threshold_medium: float = 0.75
    confidence_threshold_low: float = 0.60
    top_k_default: int = 5
    similarity_threshold_default: float = 0.7

    # Session Configuration
    session_expiry_hours: int = 1
    max_messages_per_session: int = 50

    # Constants
    MAX_QUERY_LENGTH: int = 1000
    MAX_RESPONSE_TOKENS: int = 1000
    RATE_LIMIT: int = 100

    @field_validator("openai_api_key")
    @classmethod
    def validate_openai_api_key(cls, v: str) -> str:
        """Validate OpenAI API key is not empty."""
        if not v or v.startswith("sk-proj-..."):
            raise ValueError(
                "OPENAI_API_KEY must be set to a valid OpenAI API key. "
                "Get one from https://platform.openai.com/api-keys"
            )
        return v

    @field_validator("database_url")
    @classmethod
    def validate_database_url(cls, v: str) -> str:
        """Validate database URL is properly formatted."""
        if not v.startswith("postgresql://"):
            raise ValueError("DATABASE_URL must be a valid PostgreSQL connection string")
        return v

    @field_validator("confidence_threshold_high", "confidence_threshold_medium", "confidence_threshold_low")
    @classmethod
    def validate_confidence_threshold(cls, v: float) -> float:
        """Validate confidence thresholds are between 0 and 1."""
        if not 0.0 <= v <= 1.0:
            raise ValueError("Confidence thresholds must be between 0.0 and 1.0")
        return v

    @field_validator("top_k_default")
    @classmethod
    def validate_top_k(cls, v: int) -> int:
        """Validate top_k is within reasonable range."""
        if not 1 <= v <= 10:
            raise ValueError("top_k_default must be between 1 and 10")
        return v

    @field_validator("similarity_threshold_default")
    @classmethod
    def validate_similarity_threshold(cls, v: float) -> float:
        """Validate similarity threshold is between 0 and 1."""
        if not 0.0 <= v <= 1.0:
            raise ValueError("similarity_threshold_default must be between 0.0 and 1.0")
        return v

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False
        extra = "ignore"  # Ignore extra env vars (e.g., from ingestion pipeline)


def load_system_prompt() -> str:
    """Load the system prompt from file."""
    prompt_path = Path(settings.agent_system_prompt_path)
    if not prompt_path.exists():
        raise FileNotFoundError(
            f"System prompt file not found: {prompt_path}. "
            f"Create the file with strict RAG instructions."
        )
    return prompt_path.read_text(encoding="utf-8")


# Global settings instance
settings = Settings()

# Export commonly used constants
MAX_QUERY_LENGTH = settings.MAX_QUERY_LENGTH
MAX_RESPONSE_TOKENS = settings.MAX_RESPONSE_TOKENS
RATE_LIMIT = settings.RATE_LIMIT
TOP_K_DEFAULT = settings.top_k_default
SIMILARITY_THRESHOLD = settings.similarity_threshold_default
