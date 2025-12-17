# Data Model: RAG Agent API

**Feature**: 003-rag-agent-api
**Created**: 2025-12-17
**Status**: Design Phase
**Purpose**: Define core entities, their fields, relationships, and validation rules for the RAG agent API

---

## Entity Overview

This document defines 5 core entities used throughout the RAG agent API:

1. **ChatRequest**: Incoming user question payload
2. **ChatResponse**: Agent's answer with metadata
3. **ConversationThread**: Multi-turn dialogue session
4. **RetrievalResult**: Chunks retrieved from vector database
5. **ConfidenceMetrics**: Multi-metric quality assessment

---

## Entity Definitions

### 1. ChatRequest

**Purpose**: Represents an incoming user question sent to the agent API.

**Fields**:

| Field | Type | Required | Default | Constraints | Description |
|-------|------|----------|---------|-------------|-------------|
| `message` | `str` | Yes | - | 1-1000 chars, non-empty after strip | The user's question about the textbook |
| `session_id` | `str` | No | Auto-generated UUID | Valid UUID v4 format | Unique identifier for conversation thread |
| `stream` | `bool` | No | `false` | - | Enable Server-Sent Events streaming response |
| `top_k` | `int` | No | `5` | 1-10 inclusive | Number of chunks to retrieve from Qdrant |
| `similarity_threshold` | `float` | No | `0.7` | 0.0-1.0 inclusive | Minimum cosine similarity for chunk inclusion |

**Validation Rules**:
- `message` must not be empty after stripping whitespace
- `message` length must be between 1 and 1000 characters
- `session_id` must be valid UUID v4 format if provided
- `top_k` must be between 1 and 10 (capped to prevent excessive retrieval)
- `similarity_threshold` must be between 0.0 and 1.0

**Example**:
```json
{
  "message": "How do ROS 2 nodes communicate?",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "stream": false,
  "top_k": 5,
  "similarity_threshold": 0.7
}
```

**Relationships**:
- Creates or updates a `ConversationThread` identified by `session_id`
- Triggers generation of a `RetrievalResult` via Qdrant search
- Produces a `ChatResponse` as output

---

### 2. ChatResponse

**Purpose**: Represents the agent's answer with confidence metadata and source citations.

**Fields**:

| Field | Type | Required | Default | Constraints | Description |
|-------|------|----------|---------|-------------|-------------|
| `response` | `str` | Yes | - | Non-empty | The agent's generated answer text |
| `confidence` | `float` | Yes | - | 0.0-1.0 | Aggregate confidence score (average similarity) |
| `sources` | `List[Source]` | Yes | - | Min 0 items | List of cited textbook chunks with metadata |
| `session_id` | `str` | Yes | - | Valid UUID v4 | Session identifier for follow-up questions |
| `timestamp` | `str` | Yes | Auto-generated | ISO 8601 format | Response generation timestamp |
| `confidence_level` | `str` | Yes | - | "high" \| "medium" \| "low" \| "insufficient" | Human-readable confidence category |
| `should_answer` | `bool` | Yes | - | - | Whether agent has sufficient context to answer |

**Nested Type: Source**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `chunk_text` | `str` | Yes | The retrieved text chunk (max 500 chars for display) |
| `similarity_score` | `float` | Yes | Cosine similarity score (0.0-1.0) |
| `chapter` | `str` | Yes | Chapter name from metadata |
| `section` | `str` | Yes | Section name from metadata |
| `url` | `str` | Yes | Textbook page URL |
| `chunk_index` | `int` | Yes | Position of chunk within source document |

**Validation Rules**:
- `response` must not be empty
- `confidence` must be between 0.0 and 1.0
- `sources` list can be empty if no relevant chunks found
- `timestamp` must be valid ISO 8601 format (e.g., "2025-12-17T14:23:45.123Z")
- `confidence_level` must match the computed `ConfidenceMetrics.confidence_level`
- `should_answer` must match `ConfidenceMetrics.should_answer`

**Example**:
```json
{
  "response": "ROS 2 nodes communicate through a publish-subscribe mechanism using topics. Nodes can publish messages to topics and subscribe to receive messages from topics they're interested in.",
  "confidence": 0.87,
  "sources": [
    {
      "chunk_text": "ROS 2 nodes use a distributed publish-subscribe architecture...",
      "similarity_score": 0.92,
      "chapter": "Chapter 3: Robot Operating System 2",
      "section": "3.2 ROS 2 Communication Patterns",
      "url": "https://example.com/textbook/chapter3#section3-2",
      "chunk_index": 15
    }
  ],
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-17T14:23:45.123Z",
  "confidence_level": "high",
  "should_answer": true
}
```

**Relationships**:
- Generated from a `ChatRequest`
- Appended to `ConversationThread.messages`
- Built from a `RetrievalResult` and `ConfidenceMetrics`

---

### 3. ConversationThread

**Purpose**: Represents a multi-turn dialogue session with conversation history.

**Fields**:

| Field | Type | Required | Default | Constraints | Description |
|-------|------|----------|---------|-------------|-------------|
| `thread_id` | `str` | Yes | Auto-generated UUID | Valid UUID v4 | Unique session identifier |
| `messages` | `List[Message]` | Yes | `[]` | Max 50 messages | Ordered list of user/agent messages |
| `created_at` | `datetime` | Yes | Auto-generated | UTC timezone | Session creation timestamp |
| `updated_at` | `datetime` | Yes | Auto-updated | UTC timezone | Last message timestamp |
| `topic_summary` | `str` | No | Auto-generated | Max 200 chars | AI-generated summary of conversation topics |
| `metadata` | `Dict[str, Any]` | No | `{}` | - | Additional context (user_id, tags, etc.) |

**Nested Type: Message**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `role` | `str` | Yes | "user" or "assistant" |
| `content` | `str` | Yes | Message text |
| `timestamp` | `datetime` | Yes | Message timestamp (UTC) |
| `confidence` | `float` | No | Confidence score (assistant messages only) |

**Validation Rules**:
- `thread_id` must be valid UUID v4 format
- `messages` list maintains chronological order
- `messages` limited to last 50 messages (older messages archived)
- `created_at` and `updated_at` must be UTC timezone-aware
- `topic_summary` auto-generated when conversation reaches 5+ exchanges
- `messages` must alternate between "user" and "assistant" roles

**Example**:
```json
{
  "thread_id": "550e8400-e29b-41d4-a716-446655440000",
  "messages": [
    {
      "role": "user",
      "content": "What is URDF?",
      "timestamp": "2025-12-17T14:20:00.000Z"
    },
    {
      "role": "assistant",
      "content": "URDF (Unified Robot Description Format) is an XML format for representing robot models...",
      "timestamp": "2025-12-17T14:20:02.150Z",
      "confidence": 0.91
    },
    {
      "role": "user",
      "content": "Can you give an example?",
      "timestamp": "2025-12-17T14:21:00.000Z"
    }
  ],
  "created_at": "2025-12-17T14:20:00.000Z",
  "updated_at": "2025-12-17T14:21:00.000Z",
  "topic_summary": "Discussion about URDF robot description format and examples",
  "metadata": {
    "user_id": "user123",
    "tags": ["ros2", "urdf", "robotics"]
  }
}
```

**Relationships**:
- Created/updated by `ChatRequest` messages
- Contains `ChatResponse` messages in `messages` list
- Persisted in PostgreSQL via SQLAlchemySession
- Used by agent to maintain conversation context

**Lifecycle**:
- **Creation**: First message in a session generates new thread
- **Update**: Each subsequent message appends to `messages` and updates `updated_at`
- **Archival**: Threads inactive for >1 hour moved to cold storage
- **Deletion**: Threads can be explicitly deleted via DELETE endpoint

---

### 4. RetrievalResult

**Purpose**: Represents chunks retrieved from Qdrant vector database for a given query.

**Fields**:

| Field | Type | Required | Default | Constraints | Description |
|-------|------|----------|---------|-------------|-------------|
| `chunks` | `List[str]` | Yes | - | 0-10 items | Retrieved text chunks |
| `scores` | `List[float]` | Yes | - | Same length as chunks | Cosine similarity scores (0.0-1.0) |
| `metadata` | `List[ChunkMetadata]` | Yes | - | Same length as chunks | Metadata for each chunk |
| `average_score` | `float` | Yes | - | 0.0-1.0 | Mean of all similarity scores |
| `has_sufficient_context` | `bool` | Yes | - | - | Whether retrieval quality meets threshold |
| `query_embedding` | `List[float]` | Yes | - | 1536 dimensions | OpenAI embedding vector for query |

**Nested Type: ChunkMetadata**:

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `chapter` | `str` | Yes | Chapter name |
| `section` | `str` | Yes | Section name |
| `url` | `str` | Yes | Source URL |
| `chunk_index` | `int` | Yes | Chunk position in source |
| `content_hash` | `str` | Yes | SHA256 hash of chunk text |

**Validation Rules**:
- `chunks`, `scores`, and `metadata` lists must have equal length
- `scores` values must be between 0.0 and 1.0
- `average_score` must equal mean of `scores` list
- `has_sufficient_context` = `True` if `average_score >= 0.75` AND `len(chunks) >= 3`
- `query_embedding` must have exactly 1536 dimensions (OpenAI text-embedding-3-small)

**Example**:
```json
{
  "chunks": [
    "ROS 2 nodes communicate through a publish-subscribe mechanism...",
    "Topics in ROS 2 are named channels that carry typed messages..."
  ],
  "scores": [0.92, 0.85],
  "metadata": [
    {
      "chapter": "Chapter 3: Robot Operating System 2",
      "section": "3.2 ROS 2 Communication Patterns",
      "url": "https://example.com/textbook/chapter3#section3-2",
      "chunk_index": 15,
      "content_hash": "a1b2c3d4e5f6..."
    },
    {
      "chapter": "Chapter 3: Robot Operating System 2",
      "section": "3.2 ROS 2 Communication Patterns",
      "url": "https://example.com/textbook/chapter3#section3-2",
      "chunk_index": 16,
      "content_hash": "f6e5d4c3b2a1..."
    }
  ],
  "average_score": 0.885,
  "has_sufficient_context": true,
  "query_embedding": [0.012, -0.034, 0.056, ...]
}
```

**Relationships**:
- Generated by `search_textbook` function tool in OpenAI agent
- Consumed by agent to generate `ChatResponse`
- Used to compute `ConfidenceMetrics`

**Business Logic**:
- **Sufficient Context Criteria**:
  - `average_score >= 0.75` (high semantic similarity)
  - `len(chunks) >= 3` (multiple supporting sources)
  - If either condition fails, `has_sufficient_context = False`
- **Score Threshold**: Qdrant search applies `score_threshold=0.7` by default
- **Deduplication**: If multiple chunks have same `content_hash`, keep highest scoring

---

### 5. ConfidenceMetrics

**Purpose**: Multi-metric assessment of retrieval quality to prevent hallucinations.

**Fields**:

| Field | Type | Required | Default | Constraints | Description |
|-------|------|----------|---------|-------------|-------------|
| `average_similarity` | `float` | Yes | - | 0.0-1.0 | Mean cosine similarity across all chunks |
| `min_similarity` | `float` | Yes | - | 0.0-1.0 | Weakest matching chunk score |
| `max_similarity` | `float` | Yes | - | 0.0-1.0 | Best matching chunk score |
| `num_chunks` | `int` | Yes | - | >= 0 | Number of retrieved chunks |
| `chunk_diversity` | `float` | Yes | - | 0.0-1.0 | Semantic diversity score (1 - mean pairwise similarity) |
| `confidence_level` | `str` | Yes | - | "high" \| "medium" \| "low" \| "insufficient" | Human-readable confidence category |
| `should_answer` | `bool` | Yes | - | - | Final decision: answer or refuse |

**Validation Rules**:
- `min_similarity <= average_similarity <= max_similarity`
- `chunk_diversity` between 0.0 (identical chunks) and 1.0 (completely different)
- `confidence_level` computed from scoring rules (see below)
- `should_answer = True` only if `confidence_level` is "high" or "medium"

**Confidence Level Scoring Rules**:

```python
if average_similarity >= 0.85 and num_chunks >= 5:
    confidence_level = "high"
    should_answer = True
elif average_similarity >= 0.75 and num_chunks >= 3:
    confidence_level = "medium"
    should_answer = True
elif average_similarity >= 0.60 and num_chunks >= 2:
    confidence_level = "low"
    should_answer = True  # Answer with disclaimer
else:
    confidence_level = "insufficient"
    should_answer = False  # Refuse to answer
```

**Chunk Diversity Calculation**:

```python
# Compute pairwise cosine similarities between all chunk embeddings
pairwise_sims = []
for i in range(len(embeddings)):
    for j in range(i+1, len(embeddings)):
        sim = cosine_similarity(embeddings[i], embeddings[j])
        pairwise_sims.append(sim)

# Diversity = 1 - average pairwise similarity
chunk_diversity = 1.0 - mean(pairwise_sims) if pairwise_sims else 0.0
```

**Example**:
```json
{
  "average_similarity": 0.87,
  "min_similarity": 0.78,
  "max_similarity": 0.94,
  "num_chunks": 5,
  "chunk_diversity": 0.23,
  "confidence_level": "high",
  "should_answer": true
}
```

**Relationships**:
- Computed from `RetrievalResult`
- Drives `ChatResponse.should_answer` decision
- Included in `ChatResponse` as `confidence` and `confidence_level` fields

**Business Logic**:
- **High Confidence**: Strong matches (avg >= 0.85) with sufficient quantity (5+ chunks)
- **Medium Confidence**: Good matches (avg >= 0.75) with adequate quantity (3+ chunks)
- **Low Confidence**: Acceptable matches (avg >= 0.60) but limited quantity (2+ chunks) - answer with disclaimer
- **Insufficient**: Below thresholds - refuse to answer to prevent hallucination

---

## Entity Relationships Diagram

```
┌─────────────────┐
│  ChatRequest    │
└────────┬────────┘
         │
         │ triggers
         ▼
┌─────────────────┐      uses      ┌──────────────────┐
│ search_textbook │─────────────▶│  Qdrant Vector   │
│  (function_tool)│               │     Database     │
└────────┬────────┘               └──────────────────┘
         │
         │ produces
         ▼
┌─────────────────┐
│ RetrievalResult │
└────────┬────────┘
         │
         │ computes
         ▼
┌─────────────────┐
│ConfidenceMetrics│
└────────┬────────┘
         │
         │ informs
         ▼
┌─────────────────┐      appends to    ┌─────────────────────┐
│  ChatResponse   │───────────────────▶│ ConversationThread  │
└─────────────────┘                    └─────────────────────┘
                                                │
                                                │ persisted in
                                                ▼
                                       ┌─────────────────────┐
                                       │    PostgreSQL       │
                                       │  (SQLAlchemySession)│
                                       └─────────────────────┘
```

---

## Validation Summary

### Input Validation (ChatRequest)
- Message length: 1-1000 characters
- Session ID: Valid UUID v4 format
- Top-K: 1-10 range
- Similarity threshold: 0.0-1.0 range

### Output Validation (ChatResponse)
- Confidence score: 0.0-1.0 range
- Sources: Non-empty if `should_answer = True`
- Timestamp: Valid ISO 8601 format

### Data Integrity (RetrievalResult)
- Chunks/scores/metadata arrays have equal length
- Scores within 0.0-1.0 range
- Embedding dimensions match model (1536)

### Business Rules (ConfidenceMetrics)
- Confidence level correctly categorized
- Should_answer decision matches confidence level
- Diversity score computed correctly

---

## Storage Considerations

### PostgreSQL Schema (ConversationThread)
```sql
CREATE TABLE conversation_threads (
    thread_id UUID PRIMARY KEY,
    messages JSONB NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    topic_summary TEXT,
    metadata JSONB DEFAULT '{}'::jsonb
);

CREATE INDEX idx_threads_updated ON conversation_threads(updated_at DESC);
CREATE INDEX idx_threads_created ON conversation_threads(created_at DESC);
```

### Qdrant Collection Schema (textbook_chunks)
```python
{
    "vectors": {
        "size": 1536,
        "distance": "Cosine"
    },
    "payload_schema": {
        "chapter": "keyword",
        "section": "keyword",
        "url": "keyword",
        "chunk_index": "integer",
        "content_hash": "keyword",
        "text": "text"
    }
}
```

---

## Notes

1. **Embedding Model**: Uses OpenAI `text-embedding-3-small` (1536 dimensions) for consistency with ingestion pipeline
2. **Session Expiry**: Threads inactive for >1 hour are archived but not deleted (supports session resumption)
3. **Rate Limiting**: API keys limited to 100 requests/minute (enforced at FastAPI middleware layer)
4. **Hallucination Prevention**: 3-layer defense (system instructions + guardrails + confidence filtering) ensures 95% grounding accuracy
5. **Streaming Support**: `ChatResponse` schema supports both sync (`/chat/run`) and streaming (`/chat/stream`) modes
