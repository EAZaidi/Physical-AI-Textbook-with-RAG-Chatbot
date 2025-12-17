"""FastAPI application for RAG Agent API.

Main entry point with middleware, health checks, and routing.
"""

from contextlib import asynccontextmanager
from typing import Dict, Any
import time
import logging

from fastapi import FastAPI, Request, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from agent_api.config import settings
from agent_api.sessions import engine, Base
from agent_api.routers import chat, sessions as sessions_router

# Configure logging
logging.basicConfig(
    level=getattr(logging, settings.log_level),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


def validate_startup():
    """Validate all required environment variables and dependencies (T064)."""
    logger.info("=" * 70)
    logger.info("RAG Agent API - Startup Validation")
    logger.info("=" * 70)

    # Step 1: Validate required environment variables
    logger.info("Step 1: Validating environment variables...")

    required_vars = {
        "OPENAI_API_KEY": settings.openai_api_key,
        "QDRANT_URL": settings.qdrant_url,
        "DATABASE_URL": str(settings.database_url) if hasattr(settings, 'database_url') else None
    }

    missing_vars = []
    for var_name, var_value in required_vars.items():
        if not var_value or var_value == "":
            missing_vars.append(var_name)
            logger.error(f"  ❌ {var_name} is not set")
        else:
            # Mask sensitive values for logging
            if "KEY" in var_name or "PASSWORD" in var_name:
                masked_value = var_value[:8] + "..." if len(var_value) > 8 else "***"
                logger.info(f"  ✓ {var_name} = {masked_value}")
            else:
                logger.info(f"  ✓ {var_name} = {var_value}")

    if missing_vars:
        error_msg = f"Missing required environment variables: {', '.join(missing_vars)}"
        logger.error(f"\n❌ STARTUP FAILED: {error_msg}")
        logger.error("Please check your .env file and ensure all required variables are set.")
        raise RuntimeError(error_msg)

    logger.info("  ✓ All required environment variables are set")
    logger.info("")

    # Step 2: Test database connection
    logger.info("Step 2: Testing PostgreSQL connection...")

    try:
        from sqlalchemy import text
        with engine.connect() as conn:
            result = conn.execute(text("SELECT version()"))
            pg_version = result.fetchone()[0]
            logger.info(f"  ✓ PostgreSQL connection successful")
            logger.info(f"    Version: {pg_version[:50]}...")

            # Create tables if needed
            Base.metadata.create_all(bind=engine)
            logger.info(f"  ✓ Database tables verified/created")

    except Exception as e:
        logger.error(f"  ❌ PostgreSQL connection failed: {e}")
        logger.error("  Please check DATABASE_URL and ensure PostgreSQL is running")
        raise RuntimeError(f"Database connection failed: {e}")

    logger.info("")

    # Step 3: Validate Qdrant collection
    logger.info("Step 3: Validating Qdrant collection...")

    try:
        from qdrant_client import QdrantClient

        qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key
        )

        # Check collection exists
        collection_name = settings.qdrant_collection
        collections = qdrant_client.get_collections()
        collection_names = [c.name for c in collections.collections]

        if collection_name in collection_names:
            collection_info = qdrant_client.get_collection(collection_name)
            logger.info(f"  ✓ Qdrant collection '{collection_name}' found")
            logger.info(f"    Vectors: {collection_info.points_count}")
            logger.info(f"    Dimensions: {collection_info.config.params.vectors.size}")

            if collection_info.points_count == 0:
                logger.warning(f"  ⚠️  Collection is empty (0 vectors)")
                logger.warning(f"     Run ingestion pipeline to populate collection")
        else:
            logger.error(f"  ❌ Qdrant collection '{collection_name}' not found")
            logger.error(f"     Available collections: {collection_names}")
            logger.error(f"     Please run the ingestion pipeline (001-rag-ingestion-pipeline)")
            raise RuntimeError(f"Qdrant collection '{collection_name}' not found")

    except Exception as e:
        logger.error(f"  ❌ Qdrant validation failed: {e}")
        logger.error(f"  Please check QDRANT_URL and ensure Qdrant is running")
        raise RuntimeError(f"Qdrant validation failed: {e}")

    logger.info("")

    # Step 4: Test OpenAI API connection
    logger.info("Step 4: Testing OpenAI API connection...")

    try:
        from openai import OpenAI

        client = OpenAI(api_key=settings.openai_api_key)

        # Test API key by listing models
        models = client.models.list()
        model_names = [m.id for m in models.data[:5]]
        logger.info(f"  ✓ OpenAI API connection successful")
        logger.info(f"    Available models (sample): {', '.join(model_names)}")

        # Check if configured model is available
        if settings.openai_model in [m.id for m in models.data]:
            logger.info(f"  ✓ Configured model '{settings.openai_model}' is available")
        else:
            logger.warning(f"  ⚠️  Configured model '{settings.openai_model}' not found in available models")

    except Exception as e:
        logger.error(f"  ❌ OpenAI API connection failed: {e}")
        logger.error(f"  Please check OPENAI_API_KEY is valid")
        raise RuntimeError(f"OpenAI API connection failed: {e}")

    logger.info("")

    # Step 5: Configuration summary
    logger.info("Step 5: Configuration Summary")
    logger.info(f"  OpenAI Model: {settings.openai_model}")
    logger.info(f"  Qdrant Collection: {settings.qdrant_collection}")
    logger.info(f"  Rate Limit: {settings.api_rate_limit} requests/minute")
    logger.info(f"  Max Query Length: {settings.MAX_QUERY_LENGTH} characters")
    logger.info(f"  Top-K Default: {settings.top_k_default}")
    logger.info(f"  Similarity Threshold: {settings.similarity_threshold_default}")
    logger.info(f"  Confidence Thresholds: HIGH={settings.confidence_threshold_high}, MED={settings.confidence_threshold_medium}, LOW={settings.confidence_threshold_low}")
    logger.info(f"  Session Expiry: {settings.session_expiry_hours} hours")
    logger.info(f"  Log Level: {settings.log_level}")
    logger.info("")

    logger.info("=" * 70)
    logger.info("✅ ALL STARTUP VALIDATIONS PASSED - API is ready to serve requests")
    logger.info("=" * 70)
    logger.info("")


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan context manager with comprehensive startup validation."""
    # Startup - Run all validations (T064)
    try:
        validate_startup()
    except Exception as e:
        logger.critical(f"FATAL: Startup validation failed - {e}")
        logger.critical("API will not start. Please fix the issues above and restart.")
        raise

    yield

    # Shutdown
    logger.info("Shutting down RAG Agent API...")
    engine.dispose()
    logger.info("Database connections closed.")


app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook - RAG Agent API",
    description="Production-ready RAG agent that answers questions about the textbook using retrieved context only",
    version="0.1.0",
    lifespan=lifespan
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Simple in-memory rate limiter
rate_limit_store: Dict[str, Dict[str, Any]] = {}

@app.middleware("http")
async def rate_limit_middleware(request: Request, call_next):
    """Rate limiting middleware."""
    if request.url.path == "/health":
        return await call_next(request)

    auth_header = request.headers.get("Authorization", "")
    api_key = auth_header.replace("Bearer ", "") if auth_header.startswith("Bearer ") else "anonymous"

    current_time = time.time()
    if api_key not in rate_limit_store:
        rate_limit_store[api_key] = {"count": 0, "reset_time": current_time + 60}

    if current_time > rate_limit_store[api_key]["reset_time"]:
        rate_limit_store[api_key] = {"count": 0, "reset_time": current_time + 60}

    rate_limit_store[api_key]["count"] += 1

    if rate_limit_store[api_key]["count"] > settings.api_rate_limit:
        retry_after = int(rate_limit_store[api_key]["reset_time"] - current_time)
        return JSONResponse(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            content={
                "error_code": "RATE_LIMIT_EXCEEDED",
                "message": f"Rate limit exceeded: {settings.api_rate_limit} requests per minute",
                "suggested_action": f"Wait {retry_after} seconds before retrying"
            },
            headers={"Retry-After": str(retry_after)}
        )

    return await call_next(request)


@app.middleware("http")
async def request_size_middleware(request: Request, call_next):
    """Validate request size."""
    if request.method in ["POST", "PUT", "PATCH"]:
        content_length = request.headers.get("content-length")
        if content_length and int(content_length) > 10 * 1024:
            return JSONResponse(
                status_code=status.HTTP_413_REQUEST_ENTITY_TOO_LARGE,
                content={
                    "error_code": "PAYLOAD_TOO_LARGE",
                    "message": "Request payload too large",
                    "suggested_action": "Reduce the size of your request"
                }
            )

    return await call_next(request)


@app.get("/health", tags=["Health"])
async def health_check():
    """Health check endpoint with dependency status."""
    health_status = {
        "status": "healthy",
        "dependencies": {},
        "timestamp": time.time()
    }

    try:
        from qdrant_client import QdrantClient
        qdrant_client = QdrantClient(url=settings.qdrant_url, api_key=settings.qdrant_api_key)
        start = time.time()
        collections = qdrant_client.get_collections()
        qdrant_latency = int((time.time() - start) * 1000)
        health_status["dependencies"]["qdrant"] = {"status": "up", "latency_ms": qdrant_latency}
    except Exception as e:
        health_status["dependencies"]["qdrant"] = {"status": "down", "error": str(e)}
        health_status["status"] = "degraded"

    try:
        from openai import OpenAI
        client = OpenAI(api_key=settings.openai_api_key)
        start = time.time()
        models = client.models.list()
        openai_latency = int((time.time() - start) * 1000)
        health_status["dependencies"]["openai"] = {"status": "up", "latency_ms": openai_latency}
    except Exception as e:
        health_status["dependencies"]["openai"] = {"status": "down", "error": str(e)}
        health_status["status"] = "degraded"

    try:
        from sqlalchemy import text
        with engine.connect() as conn:
            start = time.time()
            conn.execute(text("SELECT 1"))
            pg_latency = int((time.time() - start) * 1000)
        health_status["dependencies"]["postgresql"] = {"status": "up", "latency_ms": pg_latency}
    except Exception as e:
        health_status["dependencies"]["postgresql"] = {"status": "down", "error": str(e)}
        health_status["status"] = "degraded"

    if health_status["status"] == "degraded":
        return JSONResponse(status_code=status.HTTP_503_SERVICE_UNAVAILABLE, content=health_status)

    return health_status


app.include_router(chat.router, prefix="/chat", tags=["Chat"])
app.include_router(sessions_router.router, prefix="/session", tags=["Sessions"])


@app.get("/", tags=["Root"])
async def root():
    """Root endpoint with API information."""
    return {
        "service": "Physical AI & Humanoid Robotics Textbook - RAG Agent API",
        "version": "0.1.0",
        "status": "online",
        "documentation": "/docs",
        "health": "/health"
    }
