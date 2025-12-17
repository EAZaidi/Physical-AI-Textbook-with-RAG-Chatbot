# Data Model: RAG Website Ingestion and Embedding Pipeline

**Feature**: 001-rag-ingestion-pipeline
**Date**: 2025-12-16
**Purpose**: Define data structures and entity relationships for the ingestion pipeline

## Overview

This document defines Python dataclasses representing entities in the RAG ingestion pipeline. These models are used for in-memory processing and metadata storage in Qdrant. No persistent database or file storage is required in MVP.

## Core Entities

### 1. DocumentPage

Represents a single documentation page fetched from the Docusaurus site.

**Python Definition**:
```python
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

@dataclass
class DocumentPage:
    """
    Represents a single documentation page with extracted content.

    Attributes:
        url: Absolute URL of the page (unique identifier)
        title: Page title extracted from <title> or <h1> tag
        text_content: Cleaned main article text (navigation/UI excluded)
        html_source: Raw HTML source (optional, for debugging)
        timestamp: Extraction timestamp
        status: Processing status ('success' or 'failed')
        error_message: Error details if status is 'failed'
    """
    url: str
    title: str
    text_content: str
    timestamp: datetime
    status: str  # 'success' or 'failed'
    html_source: Optional[str] = None
    error_message: Optional[str] = None

    def __post_init__(self):
        if self.status not in ('success', 'failed'):
            raise ValueError(f"Invalid status: {self.status}")
```

**Validation Rules**:
- `url` must be absolute HTTP/HTTPS URL
- `title` must be non-empty string
- `text_content` should be >100 characters (warn if shorter, may indicate extraction failure)
- `status` must be 'success' or 'failed'

**Lifecycle**:
1. Created by `extract_text_from_url(url)` function
2. Passed to `chunk_text(page)` function
3. Not persisted (transient in-memory only)

---

### 2. TextChunk

Represents a semantically meaningful segment of text from a DocumentPage.

**Python Definition**:
```python
@dataclass
class TextChunk:
    """
    Represents a semantically meaningful text segment for embedding.

    Attributes:
        chunk_id: Unique identifier (format: {url}#{index})
        content: Chunk text content
        token_count: Number of tokens (via tiktoken)
        source_url: URL of source DocumentPage
        source_title: Title of source DocumentPage
        chunk_index: Position in page (0-indexed)
        overlap_tokens: Number of overlapping tokens with previous chunk
    """
    chunk_id: str
    content: str
    token_count: int
    source_url: str
    source_title: str
    chunk_index: int
    overlap_tokens: int = 0

    def __post_init__(self):
        if self.token_count <= 0:
            raise ValueError(f"Invalid token_count: {self.token_count}")
        if self.chunk_index < 0:
            raise ValueError(f"Invalid chunk_index: {self.chunk_index}")
```

**Validation Rules**:
- `chunk_id` format: `{url}#{chunk_index}` (e.g., `https://example.com/page#0`)
- `content` must be non-empty
- `token_count` should be 300-500 (warn if outside range, acceptable edge case)
- `chunk_index` starts at 0

**Lifecycle**:
1. Created by `chunk_text(page)` function (returns `List[TextChunk]`)
2. Passed to `embed(chunks)` function
3. Used to build Qdrant payload in `save_chunk_to_qdrant()`

---

### 3. QdrantPayload

Metadata stored with each vector in Qdrant collection.

**Python Definition**:
```python
import hashlib

@dataclass
class QdrantPayload:
    """
    Metadata payload stored with each vector in Qdrant.

    Attributes:
        url: Source page URL (for filtering and citation)
        title: Source page title (for display)
        chunk_index: Position in page (for ordering)
        content: Full chunk text (for display in search results)
        timestamp: Ingestion timestamp (ISO format)
        content_hash: SHA256 hash of content (for duplicate detection)
    """
    url: str
    title: str
    chunk_index: int
    content: str
    timestamp: str  # ISO format: "2025-12-16T10:30:00"
    content_hash: str

    @classmethod
    def from_chunk(cls, chunk: TextChunk) -> dict:
        """
        Create Qdrant payload dict from TextChunk.

        Returns:
            Dictionary suitable for qdrant_client.upsert() payload parameter
        """
        return {
            "url": chunk.source_url,
            "title": chunk.source_title,
            "chunk_index": chunk.chunk_index,
            "content": chunk.content,
            "timestamp": datetime.now().isoformat(),
            "content_hash": hashlib.sha256(chunk.content.encode('utf-8')).hexdigest()
        }
```

**Indexing Strategy**:
- `url`: Keyword index (exact match filtering)
- `content_hash`: Keyword index (duplicate detection)
- `title`, `content`: Full-text search (optional, not implemented in MVP)
- `chunk_index`, `timestamp`: Range queries (optional)

**Usage in Qdrant**:
```python
# Example: Upsert vector with payload
qdrant_client.upsert(
    collection_name="rag_embedding",
    points=[
        {
            "id": hash(chunk.chunk_id),  # Convert string to int hash
            "vector": embedding_vector,
            "payload": QdrantPayload.from_chunk(chunk)
        }
    ]
)
```

---

### 4. IngestionJob

Tracks overall ingestion progress (in-memory only, not persisted in MVP).

**Python Definition**:
```python
@dataclass
class IngestionJob:
    """
    Tracks progress and statistics for a complete ingestion run.

    Attributes:
        base_url: Root URL of Docusaurus site
        status: Job status ('running', 'completed', 'failed')
        pages_crawled: Count of successfully processed pages
        pages_failed: Count of pages that failed extraction
        chunks_created: Total chunks generated
        vectors_stored: Total vectors inserted into Qdrant
        start_time: Job start timestamp
        end_time: Job completion timestamp (None if still running)
        errors: List of error messages
    """
    base_url: str
    status: str  # 'running', 'completed', 'failed'
    pages_crawled: int = 0
    pages_failed: int = 0
    chunks_created: int = 0
    vectors_stored: int = 0
    start_time: datetime = None
    end_time: Optional[datetime] = None
    errors: list = None

    def __post_init__(self):
        if self.start_time is None:
            self.start_time = datetime.now()
        if self.errors is None:
            self.errors = []

    def add_error(self, error: str):
        """Record an error message"""
        self.errors.append(error)

    def to_dict(self) -> dict:
        """Convert to dictionary for logging/serialization"""
        return {
            "base_url": self.base_url,
            "status": self.status,
            "pages_crawled": self.pages_crawled,
            "pages_failed": self.pages_failed,
            "chunks_created": self.chunks_created,
            "vectors_stored": self.vectors_stored,
            "start_time": self.start_time.isoformat(),
            "end_time": self.end_time.isoformat() if self.end_time else None,
            "duration_seconds": (self.end_time - self.start_time).total_seconds() if self.end_time else None,
            "error_count": len(self.errors),
            "errors": self.errors[:10]  # Limit to first 10 errors in output
        }
```

**Usage**:
```python
def main():
    job = IngestionJob(base_url=BASE_URL, status='running')

    try:
        urls = get_all_urls(job.base_url)

        for url in urls:
            try:
                page = extract_text_from_url(url)
                job.pages_crawled += 1

                chunks = chunk_text(page)
                job.chunks_created += len(chunks)

                embeddings = embed(chunks, COHERE_API_KEY)
                for chunk, embedding in embeddings:
                    save_chunk_to_qdrant(qdrant_client, "rag_embedding", chunk, embedding)
                    job.vectors_stored += 1

            except Exception as e:
                job.pages_failed += 1
                job.add_error(f"{url}: {str(e)}")
                logging.warning(f"Failed to process {url}: {e}")

        job.status = 'completed'
        job.end_time = datetime.now()

    except Exception as e:
        job.status = 'failed'
        job.end_time = datetime.now()
        job.add_error(f"Critical error: {str(e)}")
        logging.error(f"Ingestion failed: {e}")
        raise

    finally:
        # Log final statistics
        print(json.dumps(job.to_dict(), indent=2))
```

---

## Entity Relationships

```text
┌─────────────────┐
│  IngestionJob   │ (1) Tracks overall progress
└────────┬────────┘
         │ processes multiple
         ▼
┌─────────────────┐
│  DocumentPage   │ (N) One per URL
└────────┬────────┘
         │ splits into
         ▼
┌─────────────────┐
│   TextChunk     │ (M) Multiple per page
└────────┬────────┘
         │ generates
         ▼
┌─────────────────┐
│  Vector (1024)  │ (M) One per chunk
│  + Payload      │
└─────────────────┘
         │
         ▼ stored in
┌─────────────────┐
│ Qdrant Collection│ (1) Named "rag_embedding"
└─────────────────┘
```

**Cardinality**:
- 1 IngestionJob : N DocumentPages (typically ~200)
- 1 DocumentPage : M TextChunks (typically 5-10)
- 1 TextChunk : 1 Vector + Payload
- Total vectors: ~1000-2000 for complete textbook

---

## Type Hints & Imports

**Complete imports for main.py**:
```python
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional, Tuple
import hashlib
import json
import logging
import os
import time
import random

# External dependencies
import cohere
import requests
import tiktoken
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
```

---

## Validation & Error Handling

### DocumentPage Validation
```python
def validate_page(page: DocumentPage) -> bool:
    """Validate DocumentPage meets quality criteria"""
    if page.status != 'success':
        return False
    if len(page.text_content) < 100:
        logging.warning(f"Short content (<100 chars) at {page.url}")
        return False
    if not page.title:
        logging.warning(f"Missing title at {page.url}")
        return False
    return True
```

### TextChunk Validation
```python
def validate_chunk(chunk: TextChunk, min_tokens=200, max_tokens=600) -> bool:
    """Validate TextChunk meets size criteria"""
    if chunk.token_count < min_tokens:
        logging.warning(f"Chunk {chunk.chunk_id} too small: {chunk.token_count} tokens")
    if chunk.token_count > max_tokens:
        logging.warning(f"Chunk {chunk.chunk_id} too large: {chunk.token_count} tokens")
    return min_tokens <= chunk.token_count <= max_tokens
```

---

## Testing Strategy

### Unit Tests (Future Enhancement)
```python
def test_document_page_creation():
    page = DocumentPage(
        url="https://example.com/page",
        title="Test Page",
        text_content="Sample content " * 50,
        timestamp=datetime.now(),
        status="success"
    )
    assert validate_page(page) == True

def test_text_chunk_id_format():
    chunk = TextChunk(
        chunk_id="https://example.com/page#0",
        content="Test content",
        token_count=100,
        source_url="https://example.com/page",
        source_title="Test",
        chunk_index=0
    )
    assert "#" in chunk.chunk_id
    assert chunk.source_url in chunk.chunk_id
```

---

## Performance Considerations

**Memory Usage**:
- DocumentPage: ~50KB average (5-10KB text + HTML source)
- TextChunk: ~2KB average (500 tokens ≈ 2000 chars)
- Total in-memory: ~200 pages × 50KB + ~2000 chunks × 2KB ≈ 14MB (acceptable)

**Processing Batches**:
- Process pages sequentially (avoid memory spikes from loading all pages)
- Batch chunks for embedding (96 per API call)
- Upsert vectors in batches of 100 (Qdrant best practice)

---

## Next Steps

1. Implement dataclasses in `backend/main.py`
2. Create Qdrant collection schema in `contracts/qdrant-schema.json`
3. Write quickstart guide in `quickstart.md`
4. Update agent context with new technologies
