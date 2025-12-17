# Configuration Reference: RAG Agent API

Complete reference for all environment variables and configuration options.

## Overview

The RAG Agent API is configured entirely through environment variables loaded from a `.env` file. All settings have sensible defaults optimized for development and production use.

## Environment File Setup

Create a `.env` file in the `backend/` directory:

```bash
cp backend/.env.example backend/.env
```

Then edit with your actual credentials.

## Required Environment Variables

These variables **MUST** be set for the API to function:

### OpenAI Configuration

```env
# OpenAI API key (required)
# Get one at: https://platform.openai.com/api-keys
OPENAI_API_KEY=sk-...

# OpenAI model to use for chat completions (required)
# Default: gpt-4o-mini
# Options: gpt-4o-mini, gpt-4o, gpt-4-turbo, gpt-3.5-turbo
OPENAI_MODEL=gpt-4o-mini
```

**Notes:**
- `gpt-4o-mini` is recommended for cost-efficiency and speed
- `gpt-4o` provides better reasoning for complex questions
- Embedding model is fixed: `text-embedding-3-small` (1536 dimensions)

### Qdrant Configuration

```env
# Qdrant vector database URL (required)
# Local: http://localhost:6333
# Cloud: https://your-cluster.cloud.qdrant.io
QDRANT_URL=http://localhost:6333

# Qdrant API key (optional for local, required for cloud)
QDRANT_API_KEY=your-qdrant-api-key

# Qdrant collection name (optional, default: textbook_chunks)
QDRANT_COLLECTION=textbook_chunks
```

**Notes:**
- Collection must exist before starting the API
- Run ingestion pipeline (`001-rag-ingestion-pipeline`) to create collection
- Collection should contain vectors with 1536 dimensions (OpenAI embeddings)

### PostgreSQL Configuration

```env
# PostgreSQL connection string (required)
# Format: postgresql://user:password@host:port/database
DATABASE_URL=postgresql://agent_user:agent_pass@localhost:5432/agent_db
```

**Notes:**
- Database must exist before starting the API
- Run `python -m agent_api.db.init_db` to create tables
- Connection pool size: 20 connections (configurable via `DB_POOL_SIZE`)

## Optional Environment Variables

### API Configuration

```env
# Maximum query length in characters (default: 1000)
# Requests exceeding this will return HTTP 400
MAX_QUERY_LENGTH=1000

# Maximum response tokens from OpenAI (default: 1000)
# Limits how long agent responses can be
MAX_RESPONSE_TOKENS=1000

# Rate limiting: requests per minute per API key (default: 100)
# Set to 0 to disable rate limiting (not recommended)
API_RATE_LIMIT=100

# API key for authentication (optional, for development)
# If set, all requests must include X-API-Key header
API_KEY=optional-development-key
```

### Confidence Thresholds

Multi-metric confidence scoring uses these thresholds:

```env
# High confidence threshold (default: 0.85)
# Avg similarity >= 0.85, min >= 0.75, count >= 3
# Agent answers directly without disclaimer
CONFIDENCE_THRESHOLD_HIGH=0.85

# Medium confidence threshold (default: 0.75)
# Avg similarity >= 0.75, min >= 0.65, count >= 2
# Agent answers but includes disclaimer
CONFIDENCE_THRESHOLD_MEDIUM=0.75

# Low confidence threshold (default: 0.60)
# Avg similarity >= 0.60, count >= 1
# Agent REFUSES to answer below this threshold
CONFIDENCE_THRESHOLD_LOW=0.60
```

**Decision Logic:**
- **High**: Average ≥ 0.85 AND Min ≥ 0.75 AND Count ≥ 3 → Answer directly
- **Medium**: Average ≥ 0.75 AND Min ≥ 0.65 AND Count ≥ 2 → Answer with disclaimer
- **Low**: Average ≥ 0.60 AND Count ≥ 1 → Answer with disclaimer
- **Insufficient**: Average < 0.60 OR Count < 1 → REFUSE

### Session Management

```env
# Agent cache expiry in hours (default: 1)
# Cached agents are evicted after this many hours of inactivity
SESSION_EXPIRY_HOURS=1

# Maximum conversation messages to keep in memory (default: 50)
# Older messages are archived to prevent database bloat
MAX_CONVERSATION_MESSAGES=50

# Conversation summarization threshold (default: 10)
# Generate topic_summary after this many exchanges
SUMMARIZATION_THRESHOLD=10
```

### Retrieval Parameters

```env
# Default top-K for vector search (default: 5)
# Number of chunks to retrieve from Qdrant
TOP_K_DEFAULT=5

# Default similarity threshold (default: 0.7)
# Minimum cosine similarity for chunk retrieval
SIMILARITY_THRESHOLD_DEFAULT=0.7

# Maximum top-K allowed in API requests (default: 10)
# Prevents excessive chunk retrieval
MAX_TOP_K=10
```

### Database Connection Pool

```env
# Connection pool size (default: 20)
DB_POOL_SIZE=20

# Maximum overflow connections (default: 40)
DB_MAX_OVERFLOW=40

# Enable pool pre-ping (default: true)
# Tests connections before using them
DB_POOL_PRE_PING=true
```

### Logging

```env
# Logging level (default: INFO)
# Options: DEBUG, INFO, WARNING, ERROR, CRITICAL
LOG_LEVEL=INFO

# Enable structured JSON logging (default: false)
# Useful for log aggregation in production
LOG_JSON=false

# Log file path (optional, default: stdout only)
LOG_FILE=/var/log/agent_api/app.log
```

### Performance & Resilience

```env
# OpenAI API retry attempts (default: 5)
OPENAI_RETRY_ATTEMPTS=5

# Exponential backoff multiplier (default: 1)
RETRY_BACKOFF_MULTIPLIER=1

# Minimum backoff delay in seconds (default: 1)
RETRY_MIN_DELAY=1

# Maximum backoff delay in seconds (default: 16)
RETRY_MAX_DELAY=16

# Circuit breaker failure threshold (default: 5)
# Open circuit after this many consecutive failures
CIRCUIT_BREAKER_THRESHOLD=5

# Circuit breaker timeout in seconds (default: 60)
# How long to keep circuit open before retry
CIRCUIT_BREAKER_TIMEOUT=60
```

## Environment-Specific Configurations

### Development

```env
# Development .env
OPENAI_API_KEY=sk-...
OPENAI_MODEL=gpt-4o-mini
QDRANT_URL=http://localhost:6333
DATABASE_URL=postgresql://agent_user:agent_pass@localhost:5432/agent_db
LOG_LEVEL=DEBUG
API_RATE_LIMIT=1000  # Higher for development
```

### Staging

```env
# Staging .env
OPENAI_API_KEY=sk-...
OPENAI_MODEL=gpt-4o-mini
QDRANT_URL=https://staging-qdrant.cloud.qdrant.io
QDRANT_API_KEY=...
DATABASE_URL=postgresql://agent_user:password@staging-db.example.com:5432/agent_db
LOG_LEVEL=INFO
LOG_JSON=true
API_RATE_LIMIT=100
CONFIDENCE_THRESHOLD_LOW=0.65  # Stricter for staging validation
```

### Production

```env
# Production .env
OPENAI_API_KEY=sk-...
OPENAI_MODEL=gpt-4o-mini
QDRANT_URL=https://prod-qdrant.cloud.qdrant.io
QDRANT_API_KEY=...
DATABASE_URL=postgresql://agent_user:password@prod-db.example.com:5432/agent_db
LOG_LEVEL=WARNING
LOG_JSON=true
LOG_FILE=/var/log/agent_api/app.log
API_RATE_LIMIT=100
DB_POOL_SIZE=50        # Higher for production load
DB_MAX_OVERFLOW=100
SESSION_EXPIRY_HOURS=2 # Longer for production
```

## Configuration Validation

The API validates all required configuration on startup:

```python
# backend/agent_api/config.py
class Settings(BaseSettings):
    # Validates:
    # - OPENAI_API_KEY is set
    # - QDRANT_URL is a valid URL
    # - DATABASE_URL is a valid PostgreSQL URL
    # - All thresholds are between 0.0 and 1.0
    # - Numeric values are positive
    ...
```

**Startup Behavior:**
- Missing required vars → HTTP 500 with clear error message
- Invalid values → HTTP 500 with validation error
- Qdrant collection missing → Warning logged, HTTP 503 on requests
- Database unreachable → Warning logged, HTTP 503 on requests

## Accessing Configuration in Code

```python
from agent_api.config import settings

# Access settings
api_key = settings.openai_api_key
model = settings.openai_model
top_k = settings.top_k_default

# Settings are validated and typed
assert isinstance(settings.confidence_threshold_high, float)
assert 0.0 <= settings.confidence_threshold_high <= 1.0
```

## Security Best Practices

### Never Commit Secrets

```bash
# Add to .gitignore
.env
.env.local
.env.*.local
*.key
*.pem
```

### Use Secret Management

**Development:**
- Use `.env` file (ignored by git)

**Staging/Production:**
- Use environment variables from CI/CD
- Use secret managers: AWS Secrets Manager, GCP Secret Manager, HashiCorp Vault
- Rotate API keys regularly

### Kubernetes Secrets

```yaml
# k8s-secrets.yaml
apiVersion: v1
kind: Secret
metadata:
  name: agent-api-secrets
type: Opaque
stringData:
  OPENAI_API_KEY: "sk-..."
  QDRANT_API_KEY: "..."
  DATABASE_URL: "postgresql://..."
```

## Troubleshooting

### Error: "OPENAI_API_KEY not found"

```bash
# Check .env file exists
ls -la backend/.env

# Check variable is set
grep OPENAI_API_KEY backend/.env

# Ensure no spaces around =
OPENAI_API_KEY=sk-...  # Correct
OPENAI_API_KEY = sk-... # WRONG - spaces not allowed
```

### Error: "Qdrant connection failed"

```bash
# Check Qdrant is running
curl http://localhost:6333/collections

# Check collection exists
curl http://localhost:6333/collections/textbook_chunks

# Check QDRANT_URL is correct
echo $QDRANT_URL
```

### Error: "Database connection failed"

```bash
# Test PostgreSQL connection
psql "postgresql://agent_user:agent_pass@localhost:5432/agent_db"

# Check DATABASE_URL format
# Correct: postgresql://user:pass@host:port/db
# Wrong: postgres://... (should be postgresql://)
```

### Performance Issues

```bash
# Increase connection pool size
DB_POOL_SIZE=50
DB_MAX_OVERFLOW=100

# Lower confidence thresholds (more refusals, faster responses)
CONFIDENCE_THRESHOLD_LOW=0.70

# Reduce top-K (fewer chunks retrieved)
TOP_K_DEFAULT=3
```

## Configuration Checklist

Before deploying, verify:

- [ ] All required environment variables set
- [ ] `.env` file in `.gitignore`
- [ ] Qdrant collection exists and is populated
- [ ] PostgreSQL database initialized (`init_db.py`)
- [ ] Database connection pool sized for expected load
- [ ] Confidence thresholds tuned for use case
- [ ] Rate limiting configured appropriately
- [ ] Logging level set (DEBUG for dev, INFO/WARNING for prod)
- [ ] API keys rotated and secured
- [ ] Health check endpoint tested

---

**Last Updated**: 2025-12-17
