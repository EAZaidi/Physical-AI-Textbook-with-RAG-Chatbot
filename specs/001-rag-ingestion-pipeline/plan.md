# Implementation Plan: RAG Website Ingestion and Embedding Pipeline

**Branch**: `001-rag-ingestion-pipeline` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-ingestion-pipeline/spec.md`
**User Requirements**: Single-file implementation in `backend/main.py` with functions: `get_all_urls`, `extract_text_from_url`, `chunk_text`, `embed`, `create_collection`, `save_chunk_to_qdrant`, and main execution function. Target URL: https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/

## Summary

Build a complete RAG ingestion pipeline that crawls the deployed Physical AI & Humanoid Robotics Textbook Docusaurus site, extracts clean text content, chunks it semantically, generates embeddings via Cohere API, and stores vectors in Qdrant for semantic retrieval. Implementation will be a single Python script using `uv` for package management, targeting production deployment as part of the chatbot backend infrastructure.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**:
- `cohere` (Cohere Python SDK for embeddings)
- `qdrant-client` (Qdrant Python client for vector storage)
- `beautifulsoup4` (HTML parsing and content extraction)
- `requests` (HTTP client for web crawling)
- `tiktoken` (token counting for chunking)

**Storage**: Qdrant vector database (cloud or local instance), no persistent file storage needed
**Testing**: Manual validation via script execution, integration testing with real API endpoints
**Target Platform**: Linux/macOS/Windows (development), Docker container (production)
**Project Type**: Single-file backend script (backend/main.py)
**Performance Goals**: Process 100-200 pages in under 30 minutes, handle 500+ pages without failure
**Constraints**:
- Cohere API rate limits (respect tier limits with batching)
- Qdrant connection timeout < 5s
- Memory-efficient streaming for large page sets
- Zero data loss on network interruptions

**Scale/Scope**:
- ~200 documentation pages from single Docusaurus site
- ~1000-2000 text chunks
- ~1-2M tokens for embedding generation
- Single Qdrant collection named `rag_embedding`

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Source Accuracy & Verifiability
✅ **PASS**: Implementation uses official SDKs (Cohere, Qdrant) with documented APIs. Docusaurus crawling follows standard web scraping practices.

### Educational Clarity & Student Success
✅ **PASS**: This is backend infrastructure for the RAG chatbot, not student-facing tutorial code. However, code will be well-documented for future maintainers.

### Reproducibility & Environment Consistency
✅ **PASS**: `uv` package management ensures reproducible dependencies. Script includes environment variable validation for API keys. README will document setup steps.

### Spec-Driven Content Development
✅ **PASS**: Following full spec → plan → tasks workflow per Spec-Kit Plus methodology.

### RAG Chatbot Fidelity
✅ **PASS**: This pipeline directly supports zero-hallucination requirement by indexing only verified book content with proper metadata for source citation.

### Modular Architecture & Progressive Complexity
✅ **PASS**: Pipeline implements clear phases (crawl → extract → chunk → embed → store) matching user story priorities. Single-file structure is intentionally simple for this scope.

### Production-Ready Deployment Standards
⚠️ **NEEDS ATTENTION**:
- API key management via environment variables (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)
- Logging to stdout/stderr for Docker container compatibility
- Retry logic with exponential backoff for resilience
- Health check: validate Qdrant connection before processing

**Complexity Justification**: Single-file implementation (backend/main.py) is simpler than multi-module architecture for this scope (~300 LOC). Acceptable trade-off given clear functional boundaries and target deployment as standalone ingestion script.

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-ingestion-pipeline/
├── spec.md              # Feature requirements (completed)
├── plan.md              # This file (in progress)
├── research.md          # Phase 0: Technology decisions and best practices
├── data-model.md        # Phase 1: Entity definitions and schemas
├── quickstart.md        # Phase 1: Setup and usage instructions
├── contracts/           # Phase 1: API schemas (Qdrant collection schema)
│   └── qdrant-schema.json
└── tasks.md             # Phase 2: Implementation tasks (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── main.py              # Complete ingestion pipeline (all functions)
├── pyproject.toml       # uv project configuration
├── .env.example         # Environment variable template
└── README.md            # Setup and usage instructions (from quickstart.md)

.gitignore               # Exclude .env, __pycache__, .venv
```

**Structure Decision**: Single backend directory with standalone script. No frontend code in this feature. This script will be called manually or via scheduled jobs to refresh the vector database. Future work may add FastAPI endpoints for on-demand ingestion triggers.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Single-file implementation | 300 LOC script is maintainable for clear linear pipeline with 6 functions. Simplifies deployment and reduces cognitive overhead. | Multi-module split (e.g., crawling/, embedding/, storage/) would add unnecessary indirection for this scope. Each function is independently testable. |
| No incremental update (P5) in MVP | User prioritized P1-P4 (full ingestion). Incremental detection requires content hashing and state management, doubling complexity. | Full re-ingestion acceptable for initial deployment. P5 will be separate task if needed after validating core pipeline. |

## Phase 0: Research & Technology Decisions

**Research Topics** (to be documented in `research.md`):

1. **Docusaurus Crawling Strategy**
   - Decision: Use sitemap.xml (https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/sitemap.xml) vs recursive link following
   - Rationale: Evaluate completeness, reliability, and respect for robots.txt

2. **Content Extraction Approach**
   - Decision: CSS selector targeting (article tags) vs full-page cleaning with `beautifulsoup4`
   - Rationale: Balance between extraction accuracy (<5% noise per SC-002) and robustness to Docusaurus theme changes

3. **Semantic Chunking Algorithm**
   - Decision: Fixed-size with overlap vs sentence-boundary-aware recursive splitting
   - Rationale: Meet SC-003 (300-500 tokens, 90% semantic boundaries)
   - Libraries: `tiktoken` for token counting, custom splitter or `langchain.text_splitter`

4. **Cohere Embedding Model**
   - Decision: `embed-english-v3.0` vs `embed-multilingual-v3.0`
   - Rationale: Dimensionality (1024), performance, and cost per token

5. **Qdrant Collection Configuration**
   - Decision: Distance metric (Cosine vs Dot Product), vector size (1024 for Cohere v3), indexing parameters
   - Rationale: Optimize for retrieval accuracy (SC-006: top 5 relevant results)

6. **Error Handling & Retry Strategy**
   - Decision: Exponential backoff parameters (initial delay, max retries, backoff factor)
   - Rationale: Meet SC-008 (graceful handling) and SC-004 (<1% API failure rate)

7. **Progress Tracking Mechanism**
   - Decision: Simple stdout logging vs structured JSON progress file
   - Rationale: FR-010 (resumption after interruptions) vs deployment simplicity

**Output**: `research.md` with decisions, rationale, and code references

## Phase 1: Design & Contracts

### Data Model (`data-model.md`)

From spec Key Entities, implement as Python dataclasses or Pydantic models:

1. **DocumentPage**
   - `url: str` (unique identifier)
   - `title: str` (extracted from `<title>` or `<h1>`)
   - `text_content: str` (cleaned main article text)
   - `html_source: str` (optional, for debugging)
   - `timestamp: datetime` (extraction time)
   - `status: str` (success/failed)
   - `error_message: Optional[str]`

2. **TextChunk**
   - `chunk_id: str` (UUID or hash)
   - `content: str` (chunk text)
   - `token_count: int` (via tiktoken)
   - `source_url: str` (reference to DocumentPage)
   - `source_title: str`
   - `chunk_index: int` (position in page, 0-indexed)
   - `overlap_tokens: int` (for debugging chunking)

3. **QdrantPayload** (metadata stored with each vector)
   - `url: str`
   - `title: str`
   - `chunk_index: int`
   - `content: str` (full chunk text for display)
   - `timestamp: str` (ISO format)
   - `content_hash: str` (SHA256 of content for duplicate detection, FR-014)

4. **IngestionJob** (in-memory tracking, not persisted in MVP)
   - `base_url: str`
   - `status: str` (running/completed/failed)
   - `pages_crawled: int`
   - `chunks_created: int`
   - `vectors_stored: int`
   - `start_time: datetime`
   - `end_time: Optional[datetime]`
   - `errors: List[str]`

### API Contracts (`contracts/qdrant-schema.json`)

Qdrant collection configuration:

```json
{
  "collection_name": "rag_embedding",
  "vector_config": {
    "size": 1024,
    "distance": "Cosine"
  },
  "optimizers_config": {
    "indexing_threshold": 10000
  },
  "payload_schema": {
    "url": "keyword",
    "title": "text",
    "chunk_index": "integer",
    "content": "text",
    "timestamp": "datetime",
    "content_hash": "keyword"
  }
}
```

### Function Signatures (`backend/main.py`)

```python
def get_all_urls(base_url: str) -> List[str]:
    """
    Crawl Docusaurus site and return all page URLs.

    Args:
        base_url: Root URL of Docusaurus site (e.g., https://example.com)

    Returns:
        List of absolute URLs for all documentation pages

    Raises:
        requests.RequestException: On network errors
    """
    pass

def extract_text_from_url(url: str) -> DocumentPage:
    """
    Fetch URL and extract clean main content text.

    Args:
        url: Absolute URL to fetch

    Returns:
        DocumentPage with extracted text and metadata

    Raises:
        requests.RequestException: On fetch errors
        ValueError: If content extraction fails
    """
    pass

def chunk_text(page: DocumentPage, max_tokens: int = 500, overlap: int = 50) -> List[TextChunk]:
    """
    Split page text into semantic chunks with overlap.

    Args:
        page: DocumentPage with text content
        max_tokens: Target maximum tokens per chunk
        overlap: Token overlap between chunks

    Returns:
        List of TextChunk objects
    """
    pass

def embed(chunks: List[TextChunk], cohere_api_key: str, batch_size: int = 96) -> List[Tuple[TextChunk, List[float]]]:
    """
    Generate embeddings for chunks using Cohere API.

    Args:
        chunks: List of TextChunk objects
        cohere_api_key: Cohere API key
        batch_size: Number of chunks per API request

    Returns:
        List of (chunk, embedding_vector) tuples

    Raises:
        cohere.CohereError: On API errors (retries internally with backoff)
    """
    pass

def create_collection(qdrant_client: QdrantClient, collection_name: str = "rag_embedding") -> None:
    """
    Create Qdrant collection with appropriate configuration.
    Idempotent: skips if collection exists.

    Args:
        qdrant_client: Initialized Qdrant client
        collection_name: Name of collection to create

    Raises:
        qdrant_client.exceptions.UnexpectedResponse: On Qdrant errors
    """
    pass

def save_chunk_to_qdrant(
    qdrant_client: QdrantClient,
    collection_name: str,
    chunk: TextChunk,
    embedding: List[float]
) -> None:
    """
    Upsert single chunk vector and metadata to Qdrant.

    Args:
        qdrant_client: Initialized Qdrant client
        collection_name: Target collection name
        chunk: TextChunk with metadata
        embedding: Vector embedding (1024 dimensions for Cohere v3)

    Raises:
        qdrant_client.exceptions.UnexpectedResponse: On upsert errors
    """
    pass

def main() -> None:
    """
    Execute complete ingestion pipeline.

    Steps:
        1. Load environment variables (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)
        2. Initialize Qdrant client and create collection
        3. Crawl all URLs from base_url
        4. For each URL: extract text, chunk, embed, store
        5. Log progress and final statistics

    Raises:
        SystemExit: On critical errors (missing env vars, connection failures)
    """
    pass

if __name__ == "__main__":
    main()
```

### Quickstart (`quickstart.md`)

Step-by-step setup and execution instructions:

1. **Prerequisites**
   - Python 3.11+
   - `uv` package manager installed
   - Cohere API key (free tier acceptable for testing)
   - Qdrant instance (local Docker or cloud)

2. **Setup**
   ```bash
   cd backend
   uv init
   uv add cohere qdrant-client beautifulsoup4 requests tiktoken python-dotenv
   cp .env.example .env
   # Edit .env with your API keys
   ```

3. **Configuration**
   ```env
   COHERE_API_KEY=your_cohere_key_here
   QDRANT_URL=http://localhost:6333  # or cloud URL
   QDRANT_API_KEY=your_qdrant_key_here  # if using cloud
   BASE_URL=https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/
   ```

4. **Execution**
   ```bash
   uv run python main.py
   ```

5. **Validation**
   - Check stdout logs for progress
   - Query Qdrant collection to verify vector count matches expected chunks
   - Test semantic search with sample queries

6. **Troubleshooting**
   - Common errors: API key issues, Qdrant connection timeouts, rate limit exceeded
   - Debugging: increase logging verbosity, test individual functions

## Phase 2: Task Breakdown

**Note**: Task breakdown will be generated by `/sp.tasks` command after this plan is complete. Tasks will cover:

- TDD approach: Write validation tests before implementation
- Red phase: Implement each function (get_all_urls, extract_text_from_url, etc.)
- Green phase: Integration testing with real APIs
- Refactor phase: Performance optimization and error handling improvements

## Next Steps

1. Complete Phase 0 research (`research.md`)
2. Complete Phase 1 design artifacts (`data-model.md`, `contracts/`, `quickstart.md`)
3. Update agent context with new technologies
4. Re-evaluate Constitution Check
5. Generate tasks with `/sp.tasks` command
6. Begin implementation following TDD workflow

## Architectural Decision Records (ADR)

Three significant decisions warrant ADR documentation:

1. **ADR-001: Single-file implementation vs modular architecture**
   - Decision: Single `main.py` file (~300 LOC) with 6 functions
   - Rationale: Linear pipeline, clear dependencies, simplified deployment
   - Trade-offs: Less reusable, harder to unit test in isolation

2. **ADR-002: Cohere embedding model selection**
   - Decision: `embed-english-v3.0` (1024 dimensions)
   - Rationale: English-only content, best performance/cost for this use case
   - Trade-offs: Not multilingual (acceptable per spec assumptions)

3. **ADR-003: Full re-ingestion vs incremental updates**
   - Decision: MVP implements full re-ingestion only (P1-P4), defer P5 incremental updates
   - Rationale: Simpler implementation, acceptable for infrequent documentation updates
   - Trade-offs: Higher API costs and processing time on each run (~30 min for 200 pages)

**Recommendation**: Document these decisions with `/sp.adr` after plan approval.
