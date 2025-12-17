"""Structured logging configuration for RAG Agent API.

Provides JSON-formatted logging with request context for production monitoring.
"""

import logging
import json
import sys
from datetime import datetime
from typing import Dict, Any, Optional
from contextvars import ContextVar

from agent_api.config import settings

# Context variable for request tracking
request_id_var: ContextVar[Optional[str]] = ContextVar("request_id", default=None)
session_id_var: ContextVar[Optional[str]] = ContextVar("session_id", default=None)


class StructuredFormatter(logging.Formatter):
    """JSON formatter for structured logging (T051).
    
    Outputs logs in JSON format with:
    - Standard fields (timestamp, level, logger, message)
    - Request context (request_id, session_id)
    - Error details (stack trace if exception)
    """
    
    def format(self, record: logging.LogRecord) -> str:
        """Format log record as JSON.
        
        Args:
            record: Log record to format
            
        Returns:
            JSON string with structured log data
        """
        log_data: Dict[str, Any] = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "level": record.levelname,
            "logger": record.name,
            "message": record.getMessage(),
        }
        
        # Add request context if available
        request_id = request_id_var.get()
        if request_id:
            log_data["request_id"] = request_id
        
        session_id = session_id_var.get()
        if session_id:
            log_data["session_id"] = session_id
        
        # Add extra fields
        if hasattr(record, "extra_fields"):
            log_data.update(record.extra_fields)
        
        # Add exception info if present
        if record.exc_info:
            log_data["exception"] = {
                "type": record.exc_info[0].__name__,
                "message": str(record.exc_info[1]),
                "traceback": self.formatException(record.exc_info)
            }
        
        # Add file and line info for errors
        if record.levelno >= logging.ERROR:
            log_data["location"] = {
                "file": record.pathname,
                "line": record.lineno,
                "function": record.funcName
            }
        
        return json.dumps(log_data)


def setup_structured_logging():
    """Configure structured logging for the application.
    
    Sets up JSON-formatted logging to stdout with the configured log level.
    Call this during application startup.
    """
    # Create root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(getattr(logging, settings.log_level.upper()))
    
    # Remove default handlers
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)
    
    # Add structured JSON handler
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(StructuredFormatter())
    root_logger.addHandler(handler)
    
    # Set specific logger levels
    logging.getLogger("uvicorn").setLevel(logging.WARNING)
    logging.getLogger("httpx").setLevel(logging.WARNING)
    logging.getLogger("httpcore").setLevel(logging.WARNING)
    
    root_logger.info("Structured logging configured", extra={
        "extra_fields": {"log_level": settings.log_level}
    })


def get_logger(name: str) -> logging.Logger:
    """Get a logger instance with the given name.
    
    Args:
        name: Logger name (typically __name__)
        
    Returns:
        Configured logger instance
    """
    return logging.getLogger(name)


def set_request_context(request_id: str = None, session_id: str = None):
    """Set request context for structured logging.
    
    Args:
        request_id: Unique request identifier
        session_id: Session/conversation identifier
    """
    if request_id:
        request_id_var.set(request_id)
    if session_id:
        session_id_var.set(session_id)


def clear_request_context():
    """Clear request context after processing."""
    request_id_var.set(None)
    session_id_var.set(None)
