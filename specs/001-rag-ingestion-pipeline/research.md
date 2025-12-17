# Research: RAG Website Ingestion and Embedding Pipeline

**Feature**: 001-rag-ingestion-pipeline
**Date**: 2025-12-16
**Purpose**: Technology decisions and best practices for implementing the RAG ingestion pipeline

## Research Topics & Decisions

### 1. Docusaurus Crawling Strategy

**Decision**: Use sitemap.xml parsing as primary method with link extraction fallback

**Rationale**:
- Docusaurus auto-generates `sitemap.xml` at `/sitemap.xml` with all published pages
- Provides canonical URLs, last modification dates, and priority metadata
- More reliable than recursive link following (avoids missing pages, infinite loops, duplicate detection)
- Respects site structure and intentional page organization
- Faster: single HTTP request vs hundreds for recursive crawling

**Implementation**:
```python
def get_all_urls(base_url: str) -> List[str]:
    sitemap_url = urljoin(base_url, "/sitemap.xml")
    response = requests.get(sitemap_url, timeout=10)
    soup = BeautifulSoup(response.content, "xml")  # Use XML parser
    urls = [loc.text for loc in soup.find_all("loc")]
    # Filter for HTML pages only (exclude PDFs, images, etc.)
    return [url for url in urls if url.endswith(".html") or not "." in url.split("/")[-1]]
```

**Alternatives Considered**:
- Recursive link following: Rejected due to complexity, risk of missing pages with no inbound links
- Manual URL list: Rejected as not maintainable when docs change

**References**:
- Docusaurus sitemap plugin: https://docusaurus.io/docs/api/plugins/@docusaurus/plugin-sitemap
- Sitemap protocol: https://www.sitemaps.org/protocol.html

---

### 2. Content Extraction Approach

**Decision**: CSS selector targeting for `<article>` tag with BeautifulSoup4, fallback to `<main>` tag

**Rationale**:
- Docusaurus consistently wraps main content in `<article class="theme-doc-markdown">` tags
- Excludes navigation (`<nav>`), headers (`<header>`), footers (`<footer>`), sidebars automatically
- Achieves <5% noise requirement (SC-002) without complex heuristics
- Preserves semantic structure: headings, paragraphs, code blocks, lists
- Robust to theme changes (article tag is semantic HTML5 standard)

**Implementation**:
```python
def extract_text_from_url(url: str) -> DocumentPage:
    response = requests.get(url, timeout=10)
    soup = BeautifulSoup(response.content, "html.parser")

    # Extract title
    title = soup.find("title").text if soup.find("title") else soup.find("h1").text

    # Extract main content
    article = soup.find("article") or soup.find("main")
    if not article:
        raise ValueError(f"No article/main content found at {url}")

    # Get text with preserved spacing
    text_content = article.get_text(separator="\n", strip=True)

    return DocumentPage(
        url=url,
        title=title,
        text_content=text_content,
        timestamp=datetime.now(),
        status="success"
    )
```

**Edge Case Handling**:
- Remove code block noise: Preserve code blocks but mark them distinctly (e.g., wrap in backticks)
- Handle tables: Extract as plain text with row/column structure
- Images: Skip (alt text only if meaningful)

**Alternatives Considered**:
- Full-page extraction with readability algorithms: Over-engineered, less accurate
- Markdown parsing: Requires access to source .md files (not available from deployed site)

**References**:
- BeautifulSoup4 documentation: https://www.crummy.com/software/BeautifulSoup/bs4/doc/
- HTML5 semantic elements: https://developer.mozilla.org/en-US/docs/Web/HTML/Element/article

---

### 3. Semantic Chunking Algorithm

**Decision**: Recursive character splitting with sentence boundary awareness using custom implementation

**Rationale**:
- Target: 300-500 tokens per chunk (SC-003) with 50-token overlap for context preservation
- Sentence boundaries prevent mid-sentence splits (improves semantic coherence)
- Paragraph boundaries preferred over sentence boundaries when available
- Section headers stay with their content (improves retrieval relevance)

**Implementation Strategy**:
```python
import tiktoken

def chunk_text(page: DocumentPage, max_tokens: int = 500, overlap: int = 50) -> List[TextChunk]:
    encoding = tiktoken.encoding_for_model("gpt-4")  # Approximation for Cohere

    # Split on double newlines (paragraphs) first
    paragraphs = page.text_content.split("\n\n")
    chunks = []
    current_chunk = []
    current_tokens = 0

    for para in paragraphs:
        para_tokens = len(encoding.encode(para))

        # If paragraph fits in current chunk, add it
        if current_tokens + para_tokens <= max_tokens:
            current_chunk.append(para)
            current_tokens += para_tokens
        else:
            # Save current chunk
            if current_chunk:
                chunks.append("\n\n".join(current_chunk))

            # Start new chunk with overlap from previous
            if chunks and overlap > 0:
                # Take last N tokens from previous chunk
                overlap_text = get_last_n_tokens(chunks[-1], overlap, encoding)
                current_chunk = [overlap_text, para]
            else:
                current_chunk = [para]

            current_tokens = len(encoding.encode("\n\n".join(current_chunk)))

    # Save final chunk
    if current_chunk:
        chunks.append("\n\n".join(current_chunk))

    # Convert to TextChunk objects
    return [
        TextChunk(
            chunk_id=f"{page.url}#{i}",
            content=chunk,
            token_count=len(encoding.encode(chunk)),
            source_url=page.url,
            source_title=page.title,
            chunk_index=i,
            overlap_tokens=overlap if i > 0 else 0
        )
        for i, chunk in enumerate(chunks)
    ]
```

**Alternatives Considered**:
- LangChain RecursiveCharacterTextSplitter: Adds unnecessary dependency for simple use case
- Fixed-size window: Ignores semantic boundaries (rejected per FR-004)
- Token-based splitting without overlap: Loses context between chunks

**References**:
- tiktoken library: https://github.com/openai/tiktoken
- Chunking strategies: https://www.pinecone.io/learn/chunking-strategies/

---

### 4. Cohere Embedding Model

**Decision**: `embed-english-v3.0` with 1024 dimensions

**Rationale**:
- English-only content matches spec assumption (documentation is in English)
- 1024 dimensions balance retrieval quality vs storage/compute costs
- V3 models use cosine similarity (matches Qdrant configuration)
- Supports batching up to 96 texts per API call (optimizes rate limits)
- Cost: $0.10 per 1M tokens (acceptable for ~1-2M token corpus)

**Configuration**:
```python
import cohere

def embed(chunks: List[TextChunk], cohere_api_key: str, batch_size: int = 96) -> List[Tuple[TextChunk, List[float]]]:
    co = cohere.Client(cohere_api_key)
    results = []

    for i in range(0, len(chunks), batch_size):
        batch = chunks[i:i+batch_size]
        texts = [chunk.content for chunk in batch]

        # Retry with exponential backoff
        max_retries = 3
        for attempt in range(max_retries):
            try:
                response = co.embed(
                    texts=texts,
                    model="embed-english-v3.0",
                    input_type="search_document"  # For indexing documents
                )
                embeddings = response.embeddings
                results.extend(zip(batch, embeddings))
                break
            except cohere.CohereError as e:
                if attempt < max_retries - 1:
                    time.sleep(2 ** attempt)  # 1s, 2s, 4s
                else:
                    raise

    return results
```

**Rate Limits** (Free tier):
- 100 API calls per minute
- 1000 API calls per month
- Batch size 96 → ~10 batches for 1000 chunks → well within limits

**Alternatives Considered**:
- `embed-multilingual-v3.0`: Unnecessary overhead, higher dimensionality
- OpenAI `text-embedding-3-small`: Requires separate API key, less optimized for search

**References**:
- Cohere Embed API: https://docs.cohere.com/reference/embed
- Cohere Python SDK: https://github.com/cohere-ai/cohere-python

---

### 5. Qdrant Collection Configuration

**Decision**: Cosine distance, 1024 dimensions, HNSW indexing with default parameters

**Rationale**:
- Cosine distance matches Cohere v3 embedding model recommendation
- HNSW (Hierarchical Navigable Small World) provides fast approximate nearest neighbor search
- Default `m=16` and `ef_construct=100` balance build time vs search quality for <10K vectors
- No quantization needed for small dataset (<2000 vectors, <2GB memory)

**Collection Schema**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PayloadSchemaType

def create_collection(qdrant_client: QdrantClient, collection_name: str = "rag_embedding") -> None:
    # Check if collection exists
    collections = qdrant_client.get_collections().collections
    if any(c.name == collection_name for c in collections):
        print(f"Collection '{collection_name}' already exists, skipping creation")
        return

    # Create collection
    qdrant_client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(
            size=1024,
            distance=Distance.COSINE
        )
    )

    # Create payload indexes for filtering
    qdrant_client.create_payload_index(
        collection_name=collection_name,
        field_name="url",
        field_schema=PayloadSchemaType.KEYWORD
    )
    qdrant_client.create_payload_index(
        collection_name=collection_name,
        field_name="content_hash",
        field_schema=PayloadSchemaType.KEYWORD
    )
```

**Payload Structure**:
```python
payload = {
    "url": chunk.source_url,
    "title": chunk.source_title,
    "chunk_index": chunk.chunk_index,
    "content": chunk.content,  # Full text for display
    "timestamp": datetime.now().isoformat(),
    "content_hash": hashlib.sha256(chunk.content.encode()).hexdigest()
}
```

**Alternatives Considered**:
- Dot product distance: Less suitable for normalized embeddings
- Euclidean distance: Cohere recommends cosine for v3 models

**References**:
- Qdrant documentation: https://qdrant.tech/documentation/
- Distance metrics: https://qdrant.tech/documentation/concepts/search/#metrics

---

### 6. Error Handling & Retry Strategy

**Decision**: Exponential backoff with jitter for API calls, fail-fast for connection errors

**Retry Parameters**:
- Initial delay: 1 second
- Max retries: 3
- Backoff factor: 2 (1s → 2s → 4s)
- Jitter: ±20% randomization to avoid thundering herd

**Implementation**:
```python
import time
import random

def retry_with_backoff(func, max_retries=3, initial_delay=1, backoff_factor=2):
    """Decorator for exponential backoff retry logic"""
    def wrapper(*args, **kwargs):
        delay = initial_delay
        for attempt in range(max_retries):
            try:
                return func(*args, **kwargs)
            except (requests.RequestException, cohere.CohereError) as e:
                if attempt < max_retries - 1:
                    # Add jitter: ±20%
                    jittered_delay = delay * (1 + random.uniform(-0.2, 0.2))
                    print(f"Retry {attempt + 1}/{max_retries} after {jittered_delay:.1f}s: {e}")
                    time.sleep(jittered_delay)
                    delay *= backoff_factor
                else:
                    print(f"Failed after {max_retries} retries: {e}")
                    raise
    return wrapper
```

**Error Categories**:
1. **Transient errors** (retry): Network timeouts, rate limits (429), server errors (500-503)
2. **Permanent errors** (fail-fast): Authentication (401), not found (404), invalid request (400)
3. **Critical errors** (abort): Missing environment variables, Qdrant connection failure

**Logging Strategy**:
- INFO: Progress updates (pages crawled, chunks created, vectors stored)
- WARNING: Retryable errors, skipped pages
- ERROR: Permanent failures, critical errors
- Use structured logging for production (JSON format)

**References**:
- Exponential backoff best practices: https://aws.amazon.com/blogs/architecture/exponential-backoff-and-jitter/

---

### 7. Progress Tracking Mechanism

**Decision**: Structured stdout logging with JSON progress snapshots

**Rationale**:
- Simple implementation: `print()` statements with timestamps
- Docker-compatible: stdout/stderr captured by container runtime
- Optional JSON progress file for resumption (future enhancement, not MVP)
- Real-time monitoring: tail logs during execution

**Implementation**:
```python
import logging
import json
from dataclasses import dataclass, asdict

@dataclass
class IngestionProgress:
    start_time: str
    pages_crawled: int = 0
    pages_failed: int = 0
    chunks_created: int = 0
    vectors_stored: int = 0
    errors: list = None

    def to_json(self):
        return json.dumps(asdict(self), indent=2)

def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S"
    )

    progress = IngestionProgress(start_time=datetime.now().isoformat())

    # ... pipeline execution ...

    logging.info(f"Progress: {progress.pages_crawled} pages, {progress.chunks_created} chunks")

    # Final summary
    print("\n" + "="*50)
    print("INGESTION COMPLETE")
    print("="*50)
    print(progress.to_json())
```

**Future Enhancement**: Persist progress to JSON file for resumption after interruptions (FR-010)

**Alternatives Considered**:
- Database progress tracking: Over-engineered for MVP
- No logging: Unacceptable for production deployment (violates FR-012)

---

## Technology Stack Summary

| Component | Technology | Version | Purpose |
|-----------|-----------|---------|---------|
| Language | Python | 3.11+ | Script implementation |
| Package Manager | uv | Latest | Dependency management |
| HTTP Client | requests | 2.31+ | Web crawling |
| HTML Parsing | beautifulsoup4 | 4.12+ | Content extraction |
| XML Parsing | lxml | 4.9+ | Sitemap parsing (bs4 backend) |
| Token Counting | tiktoken | 0.5+ | Chunking calculations |
| Embedding API | cohere | 5.0+ | Vector generation |
| Vector Database | qdrant-client | 1.7+ | Storage and retrieval |
| Environment Config | python-dotenv | 1.0+ | API key management |

## Dependencies Installation

```bash
cd backend
uv init
uv add cohere qdrant-client beautifulsoup4 requests tiktoken python-dotenv lxml
```

## Environment Variables

```env
# Required
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=http://localhost:6333  # or https://your-qdrant-cloud.io
BASE_URL=https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/

# Optional (if using Qdrant Cloud)
QDRANT_API_KEY=your_qdrant_api_key_here

# Optional (tuning)
MAX_TOKENS_PER_CHUNK=500
CHUNK_OVERLAP=50
COHERE_BATCH_SIZE=96
```

## Validation Steps

1. **Crawling**: Verify sitemap.xml returns expected number of URLs (~200)
2. **Extraction**: Manually inspect 5-10 extracted pages for content quality (<5% noise)
3. **Chunking**: Check token counts distribution (mean ~400, std <100)
4. **Embedding**: Verify embedding dimensions (1024) and no null vectors
5. **Storage**: Query Qdrant for total vector count, test similarity search with sample queries
6. **End-to-End**: Run full pipeline, measure execution time (<30 min for 200 pages)

## Next Steps

Phase 1 deliverables:
1. `data-model.md` - Python dataclass definitions
2. `contracts/qdrant-schema.json` - Collection configuration
3. `quickstart.md` - Setup and execution guide
