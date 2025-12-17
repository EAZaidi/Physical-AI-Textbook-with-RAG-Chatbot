"""Exponential backoff retry decorator for transient failures.

Uses tenacity library for robust retry logic with exponential backoff.
Designed for OpenAI API rate limits and transient network errors.
"""

import logging
from functools import wraps
from typing import Callable, Any

from tenacity import (
    retry,
    stop_after_attempt,
    wait_exponential,
    retry_if_exception_type,
    before_sleep_log,
    RetryError
)
from openai import RateLimitError, APIError, APITimeoutError

logger = logging.getLogger(__name__)


# Retry decorator for OpenAI API calls (T042)
openai_retry = retry(
    stop=stop_after_attempt(5),
    wait=wait_exponential(multiplier=1, min=1, max=16),
    retry=retry_if_exception_type((RateLimitError, APITimeoutError)),
    before_sleep=before_sleep_log(logger, logging.WARNING),
    reraise=True
)


def retry_on_openai_error(func: Callable) -> Callable:
    """Decorator to retry OpenAI API calls with exponential backoff.
    
    Retries up to 5 times with exponential backoff (1s, 2s, 4s, 8s, 16s).
    Only retries on rate limit and timeout errors.
    
    Args:
        func: Async or sync function to wrap
        
    Returns:
        Wrapped function with retry logic
        
    Example:
        @retry_on_openai_error
        async def call_openai():
            return await client.embeddings.create(...)
    """
    @wraps(func)
    async def async_wrapper(*args, **kwargs):
        try:
            return await openai_retry(func)(*args, **kwargs)
        except RetryError as e:
            logger.error(f"Max retries exceeded for {func.__name__}: {e}")
            raise e.last_attempt.exception()
    
    @wraps(func)
    def sync_wrapper(*args, **kwargs):
        try:
            return openai_retry(func)(*args, **kwargs)
        except RetryError as e:
            logger.error(f"Max retries exceeded for {func.__name__}: {e}")
            raise e.last_attempt.exception()
    
    # Return appropriate wrapper based on function type
    import asyncio
    if asyncio.iscoroutinefunction(func):
        return async_wrapper
    else:
        return sync_wrapper


def log_retry_attempt(retry_state):
    """Log retry attempts for debugging.
    
    Args:
        retry_state: Tenacity retry state object
    """
    logger.warning(
        f"Retry attempt {retry_state.attempt_number} after "
        f"{retry_state.seconds_since_start:.1f}s due to "
        f"{retry_state.outcome.exception().__class__.__name__}"
    )
