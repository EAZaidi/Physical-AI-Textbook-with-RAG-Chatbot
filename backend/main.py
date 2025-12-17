"""
RAG Website Ingestion and Embedding Pipeline

Crawls Docusaurus documentation, extracts text, generates embeddings via Cohere,
and stores vectors in Qdrant for semantic search.
"""

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

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

# Load environment variables
load_dotenv()

# Data Models

@dataclass
class DocumentPage:
    """Represents a single documentation page with extracted content."""
    url: str
    title: str
    text_content: str
    timestamp: datetime
    status: str  # 'success' or 'failed'
    error_message: Optional[str] = None


@dataclass
class TextChunk:
    """Represents a semantically meaningful text segment for embedding."""
    chunk_id: str
    content: str
    token_count: int
    source_url: str
    source_title: str
    chunk_index: int
    overlap_tokens: int = 0


@dataclass
class IngestionJob:
    """Tracks progress and statistics for a complete ingestion run."""
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
            "errors": self.errors[:10]  # Limit to first 10 errors
        }


# Pipeline Functions

def get_all_urls(base_url: str) -> List[str]:
    """
    Crawl Docusaurus site and return all page URLs from sitemap.xml.

    Args:
        base_url: Root URL of Docusaurus site

    Returns:
        List of absolute URLs for all documentation pages

    Raises:
        requests.RequestException: On network errors
    """
    sitemap_url = base_url.rstrip('/') + '/sitemap.xml'
    logging.info(f"Fetching sitemap from {sitemap_url}")

    try:
        response = requests.get(sitemap_url, timeout=10)
        response.raise_for_status()

        # Parse XML sitemap
        soup = BeautifulSoup(response.content, 'html.parser')
        urls = [loc.text for loc in soup.find_all('loc')]

        # Replace placeholder URLs with actual BASE_URL
        corrected_urls = []
        for url in urls:
            # Replace any placeholder domain with the actual base_url
            if 'your-docusaurus-site.example.com' in url:
                url = url.replace('https://your-docusaurus-site.example.com', base_url.rstrip('/'))
            corrected_urls.append(url)

        # Filter for HTML pages only (exclude PDFs, images, etc.)
        html_urls = [
            url for url in corrected_urls
            if url.endswith('.html') or '.' not in url.split('/')[-1]
        ]

        logging.info(f"Discovered {len(html_urls)} URLs from sitemap")
        return html_urls

    except Exception as e:
        logging.error(f"Failed to fetch sitemap: {e}")
        raise


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
    try:
        response = requests.get(url, timeout=10)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Extract title
        title = soup.find('title')
        title = title.text if title else soup.find('h1')
        title = title.text if hasattr(title, 'text') else str(title) if title else url

        # Extract main content (Docusaurus uses <article> or <main>)
        article = soup.find('article') or soup.find('main')
        if not article:
            raise ValueError(f"No article/main content found at {url}")

        # Get text with preserved spacing
        text_content = article.get_text(separator="\n", strip=True)

        if len(text_content) < 100:
            logging.warning(f"Short content (<100 chars) at {url}")

        return DocumentPage(
            url=url,
            title=title.strip(),
            text_content=text_content,
            timestamp=datetime.now(),
            status="success"
        )

    except Exception as e:
        logging.error(f"Failed to extract from {url}: {e}")
        return DocumentPage(
            url=url,
            title="",
            text_content="",
            timestamp=datetime.now(),
            status="failed",
            error_message=str(e)
        )


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
    encoding = tiktoken.encoding_for_model("gpt-4")  # Approximation for Cohere

    # Split on double newlines (paragraphs) first
    paragraphs = page.text_content.split("\n\n")
    chunks = []
    current_chunk = []
    current_tokens = 0

    for para in paragraphs:
        if not para.strip():
            continue

        para_tokens = len(encoding.encode(para))

        # If paragraph fits in current chunk, add it
        if current_tokens + para_tokens <= max_tokens:
            current_chunk.append(para)
            current_tokens += para_tokens
        else:
            # Save current chunk if not empty
            if current_chunk:
                chunk_text = "\n\n".join(current_chunk)
                chunks.append(chunk_text)

            # Start new chunk
            current_chunk = [para]
            current_tokens = para_tokens

    # Save final chunk
    if current_chunk:
        chunk_text = "\n\n".join(current_chunk)
        chunks.append(chunk_text)

    # Convert to TextChunk objects
    text_chunks = []
    for i, chunk in enumerate(chunks):
        text_chunks.append(
            TextChunk(
                chunk_id=f"{page.url}#{i}",
                content=chunk,
                token_count=len(encoding.encode(chunk)),
                source_url=page.url,
                source_title=page.title,
                chunk_index=i,
                overlap_tokens=overlap if i > 0 else 0
            )
        )

    return text_chunks


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

                logging.info(f"Generated embeddings for batch {i//batch_size + 1} ({len(batch)} chunks)")
                break

            except Exception as e:
                if attempt < max_retries - 1:
                    delay = (2 ** attempt) * (1 + random.uniform(-0.2, 0.2))  # Jitter
                    logging.warning(f"Retry {attempt + 1}/{max_retries} after {delay:.1f}s: {e}")
                    time.sleep(delay)
                else:
                    logging.error(f"Failed after {max_retries} retries: {e}")
                    raise

    return results


def create_collection(qdrant_client: QdrantClient, collection_name: str = "rag_embedding") -> None:
    """
    Create Qdrant collection with appropriate configuration.
    Idempotent: skips if collection exists.

    Args:
        qdrant_client: Initialized Qdrant client
        collection_name: Name of collection to create

    Raises:
        Exception: On Qdrant errors
    """
    try:
        # Check if collection exists
        collections = qdrant_client.get_collections().collections
        if any(c.name == collection_name for c in collections):
            logging.info(f"Collection '{collection_name}' already exists, skipping creation")
            return

        # Create collection
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=1024,  # Cohere embed-english-v3.0 dimensions
                distance=Distance.COSINE
            )
        )

        logging.info(f"Created collection '{collection_name}'")

    except Exception as e:
        logging.error(f"Failed to create collection: {e}")
        raise


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
        Exception: On upsert errors
    """
    try:
        # Create payload with metadata
        payload = {
            "url": chunk.source_url,
            "title": chunk.source_title,
            "chunk_index": chunk.chunk_index,
            "content": chunk.content,
            "timestamp": datetime.now().isoformat(),
            "content_hash": hashlib.sha256(chunk.content.encode('utf-8')).hexdigest()
        }

        # Generate unique point ID from chunk_id
        point_id = abs(hash(chunk.chunk_id)) % (2**63)  # Convert to positive int

        # Upsert point
        qdrant_client.upsert(
            collection_name=collection_name,
            points=[
                PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload=payload
                )
            ]
        )

    except Exception as e:
        logging.error(f"Failed to upsert chunk {chunk.chunk_id}: {e}")
        raise


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
    # Load environment variables
    COHERE_API_KEY = os.getenv('COHERE_API_KEY')
    QDRANT_URL = os.getenv('QDRANT_URL')
    QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')
    BASE_URL = os.getenv('BASE_URL')
    COLLECTION_NAME = os.getenv('COLLECTION_NAME', 'rag_embedding')
    MAX_TOKENS_PER_CHUNK = int(os.getenv('MAX_TOKENS_PER_CHUNK', '500'))
    CHUNK_OVERLAP = int(os.getenv('CHUNK_OVERLAP', '50'))

    # Validate required environment variables
    if not all([COHERE_API_KEY, QDRANT_URL, BASE_URL]):
        logging.error("Missing required environment variables: COHERE_API_KEY, QDRANT_URL, BASE_URL")
        raise SystemExit(1)

    logging.info("Starting RAG ingestion pipeline")
    logging.info(f"Base URL: {BASE_URL}")
    logging.info(f"Qdrant URL: {QDRANT_URL}")
    logging.info(f"Collection: {COLLECTION_NAME}")

    # Initialize Qdrant client
    try:
        qdrant_client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY
        )
        logging.info("Qdrant connection verified")
    except Exception as e:
        logging.error(f"Failed to connect to Qdrant: {e}")
        raise SystemExit(1)

    # Create collection
    try:
        create_collection(qdrant_client, COLLECTION_NAME)
    except Exception as e:
        logging.error(f"Failed to create collection: {e}")
        raise SystemExit(1)

    # Initialize job tracker
    job = IngestionJob(base_url=BASE_URL, status='running')

    try:
        # Step 1: Crawl URLs
        urls = get_all_urls(BASE_URL)

        # Step 2: Process each URL
        for idx, url in enumerate(urls, 1):
            try:
                logging.info(f"Processing page {idx}/{len(urls)}: {url}")

                # Extract text
                page = extract_text_from_url(url)
                if page.status != 'success':
                    job.pages_failed += 1
                    job.add_error(f"{url}: {page.error_message}")
                    continue

                job.pages_crawled += 1
                logging.info(f"  Extracted {len(page.text_content)} chars")

                # Chunk text
                chunks = chunk_text(page, MAX_TOKENS_PER_CHUNK, CHUNK_OVERLAP)
                job.chunks_created += len(chunks)
                logging.info(f"  Created {len(chunks)} chunks")

                # Generate embeddings
                embeddings = embed(chunks, COHERE_API_KEY)
                logging.info(f"  Generated {len(embeddings)} embeddings")

                # Store in Qdrant
                for chunk, embedding in embeddings:
                    save_chunk_to_qdrant(qdrant_client, COLLECTION_NAME, chunk, embedding)
                    job.vectors_stored += 1

                logging.info(f"  Stored {len(embeddings)} vectors")

                # Log progress every 10 pages
                if idx % 10 == 0:
                    logging.info(f"Progress: {job.pages_crawled} pages, {job.chunks_created} chunks, {job.vectors_stored} vectors")

            except Exception as e:
                job.pages_failed += 1
                job.add_error(f"{url}: {str(e)}")
                logging.warning(f"Failed to process {url}: {e}")
                continue

        # Mark job as completed
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
        print("\n" + "="*50)
        print("INGESTION COMPLETE" if job.status == 'completed' else "INGESTION FAILED")
        print("="*50)
        print(json.dumps(job.to_dict(), indent=2))


if __name__ == "__main__":
    main()
