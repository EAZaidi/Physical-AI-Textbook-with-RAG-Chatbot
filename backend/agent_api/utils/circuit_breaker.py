"""Circuit Breaker pattern for protecting against cascading failures.

Prevents repeated calls to failing external services (Qdrant, OpenAI).
Opens circuit after consecutive failures, closes after timeout.
"""

import time
import logging
from typing import Callable, Any
from enum import Enum
from functools import wraps

from fastapi import HTTPException, status

logger = logging.getLogger(__name__)


class CircuitState(Enum):
    """Circuit breaker states."""
    CLOSED = "closed"      # Normal operation, requests flow through
    OPEN = "open"          # Failures detected, block all requests
    HALF_OPEN = "half_open"  # Testing if service recovered


class CircuitBreaker:
    """Circuit breaker for external service calls (T043).
    
    Opens circuit after consecutive failures, preventing cascading failures.
    Automatically attempts recovery after timeout period.
    
    Attributes:
        failure_threshold: Number of consecutive failures before opening (default: 5)
        timeout: Seconds to wait before attempting recovery (default: 60)
        success_threshold: Successes needed in half-open to close circuit (default: 2)
    """
    
    def __init__(
        self,
        failure_threshold: int = 5,
        timeout: int = 60,
        success_threshold: int = 2,
        name: str = "CircuitBreaker"
    ):
        """Initialize circuit breaker.
        
        Args:
            failure_threshold: Consecutive failures before opening
            timeout: Seconds to wait before half-open attempt
            success_threshold: Successes needed to close from half-open
            name: Identifier for logging
        """
        self.failure_threshold = failure_threshold
        self.timeout = timeout
        self.success_threshold = success_threshold
        self.name = name
        
        # State tracking
        self.state = CircuitState.CLOSED
        self.failure_count = 0
        self.success_count = 0
        self.last_failure_time = None
        
        logger.info(
            f"{self.name} initialized: "
            f"failure_threshold={failure_threshold}, timeout={timeout}s"
        )
    
    def call(self, func: Callable, *args, **kwargs) -> Any:
        """Execute function with circuit breaker protection.
        
        Args:
            func: Function to call
            *args: Positional arguments for func
            **kwargs: Keyword arguments for func
            
        Returns:
            Result from func
            
        Raises:
            HTTPException: 503 if circuit is open
        """
        # Check if circuit should transition to half-open
        if self.state == CircuitState.OPEN:
            if self._should_attempt_reset():
                logger.info(f"{self.name}: Transitioning to HALF_OPEN (timeout expired)")
                self.state = CircuitState.HALF_OPEN
                self.success_count = 0
            else:
                # Circuit still open, reject request
                logger.warning(f"{self.name}: Circuit OPEN, rejecting request")
                raise HTTPException(
                    status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                    detail={
                        "error_code": "CIRCUIT_BREAKER_OPEN",
                        "message": f"{self.name}: Service temporarily unavailable",
                        "suggested_action": f"Retry after {self._time_until_retry():.0f} seconds"
                    }
                )
        
        # Attempt to call function
        try:
            result = func(*args, **kwargs)
            self._on_success()
            return result
        except Exception as e:
            self._on_failure()
            raise
    
    def _on_success(self):
        """Handle successful call."""
        if self.state == CircuitState.HALF_OPEN:
            self.success_count += 1
            logger.info(
                f"{self.name}: HALF_OPEN success "
                f"({self.success_count}/{self.success_threshold})"
            )
            
            if self.success_count >= self.success_threshold:
                # Close circuit
                logger.info(f"{self.name}: Closing circuit (recovery successful)")
                self.state = CircuitState.CLOSED
                self.failure_count = 0
                self.success_count = 0
                self.last_failure_time = None
        elif self.state == CircuitState.CLOSED:
            # Reset failure count on success
            if self.failure_count > 0:
                logger.debug(f"{self.name}: Resetting failure count after success")
                self.failure_count = 0
    
    def _on_failure(self):
        """Handle failed call."""
        self.failure_count += 1
        self.last_failure_time = time.time()
        
        if self.state == CircuitState.HALF_OPEN:
            # Failure during recovery attempt, reopen circuit
            logger.warning(f"{self.name}: HALF_OPEN failure, reopening circuit")
            self.state = CircuitState.OPEN
            self.success_count = 0
        elif self.state == CircuitState.CLOSED:
            logger.warning(
                f"{self.name}: Failure {self.failure_count}/{self.failure_threshold}"
            )
            
            if self.failure_count >= self.failure_threshold:
                # Open circuit
                logger.error(
                    f"{self.name}: Opening circuit "
                    f"(threshold {self.failure_threshold} reached)"
                )
                self.state = CircuitState.OPEN
    
    def _should_attempt_reset(self) -> bool:
        """Check if enough time has passed to attempt recovery."""
        if self.last_failure_time is None:
            return False
        return (time.time() - self.last_failure_time) >= self.timeout
    
    def _time_until_retry(self) -> float:
        """Calculate seconds until retry is allowed."""
        if self.last_failure_time is None:
            return 0.0
        elapsed = time.time() - self.last_failure_time
        return max(0.0, self.timeout - elapsed)
    
    def reset(self):
        """Manually reset circuit breaker to closed state."""
        logger.info(f"{self.name}: Manual reset")
        self.state = CircuitState.CLOSED
        self.failure_count = 0
        self.success_count = 0
        self.last_failure_time = None


def circuit_breaker(
    failure_threshold: int = 5,
    timeout: int = 60,
    name: str = None
):
    """Decorator to apply circuit breaker to a function.
    
    Args:
        failure_threshold: Consecutive failures before opening
        timeout: Seconds to wait before recovery attempt
        name: Circuit breaker identifier
        
    Example:
        @circuit_breaker(failure_threshold=3, timeout=30, name="Qdrant")
        def search_qdrant(query):
            return qdrant_client.search(...)
    """
    # Create circuit breaker instance
    cb = CircuitBreaker(
        failure_threshold=failure_threshold,
        timeout=timeout,
        name=name or "CircuitBreaker"
    )
    
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs):
            return cb.call(func, *args, **kwargs)
        
        @wraps(func)
        async def async_wrapper(*args, **kwargs):
            return cb.call(func, *args, **kwargs)
        
        # Return appropriate wrapper
        import asyncio
        if asyncio.iscoroutinefunction(func):
            return async_wrapper
        else:
            return wrapper
    
    return decorator
