# Quickstart Guide: RAG Agent API

**Feature**: 003-rag-agent-api
**Last Updated**: 2025-12-17
**Estimated Setup Time**: 15 minutes

This guide helps developers get the RAG Agent API running locally and make their first successful API request.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Local Setup](#local-setup)
3. [Environment Configuration](#environment-configuration)
4. [Starting the Services](#starting-the-services)
5. [Making Your First Request](#making-your-first-request)
6. [Testing the API](#testing-the-api)
7. [Troubleshooting](#troubleshooting)
8. [Next Steps](#next-steps)

---

## Prerequisites

Before starting, ensure you have the following installed and configured:

### Required Software

- **Python 3.11+** - Check with `python --version`
- **uv** (Python package manager) - Install: `curl -LsSf https://astral.sh/uv/install.sh | sh`
- **Docker & Docker Compose** - For PostgreSQL and Qdrant
- **Git** - For cloning the repository

### Required Accounts & API Keys

- **OpenAI API Key** - Get from [platform.openai.com/api-keys](https://platform.openai.com/api-keys)
  - Required models: `gpt-4.0` (agent), `text-embedding-3-small` (embeddings)
  - Minimum quota: $10 credit recommended for testing

### Completed Dependencies

- **RAG Ingestion Pipeline (001)** - Textbook must be ingested and indexed in Qdrant
  - Verify: Check Qdrant collection `textbook_chunks` exists with embeddings
  - If not completed, run: `/sp.implement` on feature 001-rag-ingestion-pipeline first

---

## Local Setup

### 1. Clone Repository (if not already done)

```bash
git clone https://github.com/yourusername/physical-ai-humanoid-book.git
cd physical-ai-humanoid-book
```

### 2. Checkout Feature Branch

```bash
git checkout 003-rag-agent-api
```

### 3. Navigate to Backend Directory

```bash
cd backend
```

### 4. Install Python Dependencies

Using `uv` (recommended for fast dependency resolution):

```bash
uv sync
```

This installs all required packages from `pyproject.toml`:
- `fastapi` - Web framework
- `openai` - OpenAI Agents SDK and API client
- `qdrant-client` - Vector database client
- `sqlalchemy` - Database ORM for conversation storage
- `psycopg2-binary` - PostgreSQL driver
- `pydantic` - Data validation
- `uvicorn` - ASGI server

**Expected output**: ~30 packages installed in <30 seconds

---

## Environment Configuration

### 1. Create `.env` File

Copy the template and fill in your credentials:

```bash
cp .env.example .env
```

### 2. Configure Environment Variables

Edit `.env` with your values:

```bash
# OpenAI Configuration
OPENAI_API_KEY=sk-proj-...your-key-here...
OPENAI_MODEL=gpt-4.0
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Qdrant Configuration
QDRANT_URL=http://localhost:6333
QDRANT_API_KEY=  # Leave empty for local dev (no auth)
QDRANT_COLLECTION=textbook_chunks

# PostgreSQL Configuration
DATABASE_URL=postgresql://chatbot_user:chatbot_password@localhost:5432/chatbot_db

# API Configuration
API_HOST=0.0.0.0
API_PORT=8000
API_RATE_LIMIT=100  # Requests per minute per API key
LOG_LEVEL=INFO

# Agent Configuration
AGENT_SYSTEM_PROMPT_PATH=agent_api/prompts/system.txt
CONFIDENCE_THRESHOLD_HIGH=0.85
CONFIDENCE_THRESHOLD_MEDIUM=0.75
CONFIDENCE_THRESHOLD_LOW=0.60
TOP_K_DEFAULT=5
SIMILARITY_THRESHOLD_DEFAULT=0.7

# Session Configuration
SESSION_EXPIRY_HOURS=1
MAX_MESSAGES_PER_SESSION=50
```

**Security Notes**:
- **Never commit `.env` to version control** (already in `.gitignore`)
- Rotate `OPENAI_API_KEY` monthly for security
- Use strong passwords for `DATABASE_URL` in production

---

## Starting the Services

### 1. Start Infrastructure Services (PostgreSQL + Qdrant)

From the `backend/` directory:

```bash
docker-compose up -d
```

This starts:
- **PostgreSQL** on port `5432` (conversation storage)
- **Qdrant** on port `6333` (vector search)

**Verify services are running**:

```bash
# Check containers
docker-compose ps

# Expected output:
# NAME                STATUS    PORTS
# postgres            Up        0.0.0.0:5432->5432/tcp
# qdrant              Up        0.0.0.0:6333->6333/tcp

# Test PostgreSQL connection
docker exec -it postgres psql -U chatbot_user -d chatbot_db -c "SELECT 1;"

# Test Qdrant API
curl http://localhost:6333/collections/textbook_chunks
```

### 2. Initialize Database Schema

Create the conversation threads table:

```bash
uv run python -m agent_api.db.init_db
```

**Expected output**:
```
Creating database schema...
✓ Table 'conversation_threads' created successfully
✓ Indexes created: idx_threads_updated, idx_threads_created
Database initialization complete.
```

### 3. Start FastAPI Application

From the `backend/` directory:

```bash
uv run uvicorn agent_api.main:app --reload --host 0.0.0.0 --port 8000
```

**Expected output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

**Verify API is running**:

Open browser to [http://localhost:8000/docs](http://localhost:8000/docs) - you should see the Swagger UI with API documentation.

---

## Making Your First Request

### 1. Health Check (No Authentication Required)

Verify all dependencies are healthy:

```bash
curl http://localhost:8000/health
```

**Expected response**:
```json
{
  "status": "healthy",
  "dependencies": {
    "qdrant": {"status": "up", "latency_ms": 12},
    "openai": {"status": "up", "latency_ms": 156},
    "postgresql": {"status": "up", "latency_ms": 8}
  },
  "timestamp": "2025-12-17T14:30:00.000Z"
}
```

### 2. Ask Your First Question (Synchronous)

```bash
curl -X POST http://localhost:8000/chat/run \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer demo-api-key" \
  -d '{
    "message": "What is a ROS 2 node?",
    "top_k": 5,
    "similarity_threshold": 0.7
  }'
```

**Expected response** (high confidence):
```json
{
  "response": "A ROS 2 node is a process that performs computation in the ROS graph. Nodes communicate with each other through topics, services, and actions. Each node typically handles a specific task, such as reading sensor data, controlling motors, or performing calculations.",
  "confidence": 0.89,
  "sources": [
    {
      "chunk_text": "ROS 2 nodes are the fundamental building blocks of a ROS 2 application...",
      "similarity_score": 0.92,
      "chapter": "Chapter 3: Robot Operating System 2",
      "section": "3.1 ROS 2 Architecture",
      "url": "https://example.com/textbook/chapter3#section3-1",
      "chunk_index": 12
    }
  ],
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-17T14:23:45.123Z",
  "confidence_level": "high",
  "should_answer": true
}
```

### 3. Ask a Follow-Up Question (Conversation Context)

Use the `session_id` from the previous response:

```bash
curl -X POST http://localhost:8000/chat/run \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer demo-api-key" \
  -d '{
    "message": "How do I create one?",
    "session_id": "550e8400-e29b-41d4-a716-446655440000",
    "top_k": 5
  }'
```

The agent understands "one" refers to "ROS 2 node" from conversation context.

### 4. Test Streaming Response (Server-Sent Events)

```bash
curl -X POST http://localhost:8000/chat/stream \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer demo-api-key" \
  -d '{
    "message": "Explain URDF in simple terms",
    "stream": true
  }' \
  --no-buffer
```

**Expected output** (streamed tokens):
```
data: {"type": "token", "content": "URDF"}

data: {"type": "token", "content": " (Unified"}

data: {"type": "token", "content": " Robot"}

data: {"type": "done", "metadata": {"confidence": 0.91, "sources": [...]}}
```

---

## Testing the API

### 1. Run Unit Tests

From the `backend/` directory:

```bash
uv run pytest tests/unit/ -v
```

**Expected output**:
```
tests/unit/test_confidence.py::test_high_confidence PASSED
tests/unit/test_confidence.py::test_medium_confidence PASSED
tests/unit/test_confidence.py::test_insufficient_confidence PASSED
tests/unit/test_validation.py::test_message_length_validation PASSED
...
===================== 15 passed in 2.34s =====================
```

### 2. Run Integration Tests

Requires running services (PostgreSQL, Qdrant, OpenAI):

```bash
uv run pytest tests/integration/ -v --slow
```

**Expected output**:
```
tests/integration/test_chat_endpoint.py::test_basic_question PASSED
tests/integration/test_chat_endpoint.py::test_conversation_context PASSED
tests/integration/test_session_management.py::test_session_persistence PASSED
...
===================== 8 passed in 12.45s =====================
```

### 3. Manual Quality Testing

Test edge cases manually:

**Out-of-domain question** (should refuse):
```bash
curl -X POST http://localhost:8000/chat/run \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer demo-api-key" \
  -d '{
    "message": "What is the capital of France?"
  }'
```

**Expected response**:
```json
{
  "response": "I don't have sufficient information in the textbook to answer that question.",
  "confidence": 0.32,
  "sources": [],
  "session_id": "...",
  "timestamp": "...",
  "confidence_level": "insufficient",
  "should_answer": false
}
```

**Very technical question** (high confidence):
```bash
curl -X POST http://localhost:8000/chat/run \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer demo-api-key" \
  -d '{
    "message": "Explain the difference between ROS 2 topics and services"
  }'
```

---

## Troubleshooting

### Issue: `ImportError: No module named 'openai'`

**Cause**: Dependencies not installed

**Solution**:
```bash
cd backend
uv sync
```

### Issue: `Connection refused` when calling API

**Cause**: FastAPI server not running

**Solution**:
```bash
# Check if server is running
ps aux | grep uvicorn

# Start server if not running
uv run uvicorn agent_api.main:app --reload --host 0.0.0.0 --port 8000
```

### Issue: `OPENAI_API_KEY not found`

**Cause**: Environment variables not loaded

**Solution**:
```bash
# Verify .env file exists
ls -la .env

# Check environment variable
echo $OPENAI_API_KEY

# If empty, export manually for testing
export OPENAI_API_KEY=sk-proj-...your-key...
```

### Issue: `Qdrant collection 'textbook_chunks' not found`

**Cause**: RAG ingestion pipeline (feature 001) not completed

**Solution**:
```bash
# Check if collection exists
curl http://localhost:6333/collections/textbook_chunks

# If 404, run ingestion pipeline first
cd backend
uv run python ingest_textbook.py
```

### Issue: `PostgreSQL connection failed`

**Cause**: Database not running or wrong credentials

**Solution**:
```bash
# Check Docker container
docker-compose ps

# If not running, start it
docker-compose up -d postgres

# Test connection
docker exec -it postgres psql -U chatbot_user -d chatbot_db -c "SELECT 1;"

# If credentials wrong, check .env DATABASE_URL matches docker-compose.yml
```

### Issue: `Rate limit exceeded` during testing

**Cause**: Exceeded 100 requests/minute default limit

**Solution**:
```bash
# Increase rate limit in .env
API_RATE_LIMIT=1000

# Or use different API key for each test client
```

### Issue: Low confidence scores for valid questions

**Cause**:
- Similarity threshold too high
- Top-K too low
- Question phrasing doesn't match textbook language

**Solution**:
```bash
# Adjust retrieval parameters in request
curl -X POST http://localhost:8000/chat/run \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer demo-api-key" \
  -d '{
    "message": "Your question here",
    "top_k": 10,
    "similarity_threshold": 0.6
  }'

# Or adjust defaults in .env
TOP_K_DEFAULT=10
SIMILARITY_THRESHOLD_DEFAULT=0.6
```

### Issue: Agent hallucinates information not in textbook

**Cause**: Hallucination prevention layers not working

**Solution**:
1. Check confidence thresholds in `.env` are properly set
2. Verify system prompt file exists: `agent_api/prompts/system.txt`
3. Enable OpenAI Guardrails (if available in your tier)
4. Report to development team for investigation

---

## Next Steps

### 1. Explore API Documentation

Visit the interactive Swagger UI at [http://localhost:8000/docs](http://localhost:8000/docs) to:
- View all available endpoints
- Test API calls directly in browser
- See request/response schemas
- Copy code examples (curl, Python, JavaScript)

### 2. Review Conversation History

Retrieve full conversation thread:

```bash
SESSION_ID="550e8400-e29b-41d4-a716-446655440000"

curl -X GET "http://localhost:8000/session/${SESSION_ID}/history" \
  -H "Authorization: Bearer demo-api-key"
```

### 3. Delete a Session

Clean up test sessions:

```bash
SESSION_ID="550e8400-e29b-41d4-a716-446655440000"

curl -X DELETE "http://localhost:8000/session/${SESSION_ID}" \
  -H "Authorization: Bearer demo-api-key"
```

### 4. Monitor Logs

Watch real-time logs for debugging:

```bash
# In terminal running uvicorn, logs appear automatically

# For structured logging, check:
tail -f logs/agent_api.log
```

### 5. Performance Testing

Load test the API with Apache Bench:

```bash
# Install Apache Bench (if not already installed)
sudo apt-get install apache2-utils  # Ubuntu/Debian
brew install httpd  # macOS

# Run 100 requests with 10 concurrent connections
ab -n 100 -c 10 -T 'application/json' -H 'Authorization: Bearer demo-api-key' \
  -p request.json http://localhost:8000/chat/run
```

**Create `request.json`**:
```json
{
  "message": "What is ROS 2?",
  "top_k": 5
}
```

**Expected results**:
- **p95 latency**: <3000ms
- **Success rate**: 100%
- **Throughput**: >30 requests/second (single worker)

### 6. Deploy to Production

See [deployment.md](./deployment.md) for:
- Docker image building
- Kubernetes deployment
- Environment-specific configuration
- Monitoring and alerting setup
- API key management

---

## Additional Resources

- **Full API Specification**: See [contracts/api-schema.yaml](./contracts/api-schema.yaml)
- **Data Models**: See [data-model.md](./data-model.md)
- **Architecture Plan**: See [plan.md](./plan.md)
- **Technical Research**: See [research.md](./research.md)
- **OpenAI Agents SDK Docs**: [platform.openai.com/docs/agents](https://platform.openai.com/docs/agents)
- **FastAPI Documentation**: [fastapi.tiangolo.com](https://fastapi.tiangolo.com)
- **Qdrant Documentation**: [qdrant.tech/documentation](https://qdrant.tech/documentation)

---

## Success Checklist

After completing this quickstart, you should be able to:

- [ ] Start all required services (PostgreSQL, Qdrant, FastAPI)
- [ ] Make a successful health check request
- [ ] Ask a question and receive a grounded answer with sources
- [ ] Continue a multi-turn conversation using session_id
- [ ] Observe streaming responses with Server-Sent Events
- [ ] See the agent refuse to answer out-of-domain questions
- [ ] Run unit and integration tests successfully
- [ ] View API documentation in Swagger UI
- [ ] Understand how to adjust confidence thresholds and retrieval parameters

**Estimated time to complete checklist**: 15 minutes

---

**Questions or Issues?**

- Check [Troubleshooting](#troubleshooting) section above
- Review logs: `tail -f logs/agent_api.log`
- Open an issue on GitHub: [github.com/yourusername/physical-ai-humanoid-book/issues](https://github.com/yourusername/physical-ai-humanoid-book/issues)
- Contact development team: dev@example.com
