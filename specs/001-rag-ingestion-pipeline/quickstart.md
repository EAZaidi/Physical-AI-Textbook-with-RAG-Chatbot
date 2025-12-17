# Quickstart Guide: RAG Website Ingestion Pipeline

**Feature**: 001-rag-ingestion-pipeline
**Last Updated**: 2025-12-16
**Estimated Setup Time**: 15-20 minutes

## Overview

This guide walks you through setting up and running the RAG ingestion pipeline to index the Physical AI & Humanoid Robotics Textbook documentation into Qdrant for semantic search.

**What this pipeline does**:
1. Crawls all pages from the deployed Docusaurus site
2. Extracts clean text content from each page
3. Chunks content into semantic segments (~300-500 tokens)
4. Generates embeddings using Cohere API
5. Stores vectors and metadata in Qdrant for retrieval

**Expected Output**: ~1000-2000 vectors in Qdrant collection `rag_embedding`, ready for semantic search queries.

---

## Prerequisites

### Required Software

1. **Python 3.11+**
   ```bash
   python --version  # Should be 3.11 or higher
   ```

2. **uv package manager**
   ```bash
   # Install uv (if not already installed)
   curl -LsSf https://astral.sh/uv/install.sh | sh

   # Verify installation
   uv --version
   ```

### Required API Keys

1. **Cohere API Key** (for embeddings)
   - Sign up: https://cohere.com/
   - Free tier: 100 API calls/min, 1000 calls/month (sufficient for testing)
   - Navigate to Dashboard → API Keys → Create Trial Key

2. **Qdrant Instance** (for vector storage)
   - **Option A: Local Docker** (recommended for development)
     ```bash
     docker run -p 6333:6333 qdrant/qdrant:latest
     # Qdrant will be available at http://localhost:6333
     ```
   - **Option B: Qdrant Cloud** (recommended for production)
     - Sign up: https://cloud.qdrant.io/
     - Create cluster → Get API URL and API Key

---

## Installation

### Step 1: Create Backend Directory

```bash
# From repository root
mkdir -p backend
cd backend
```

### Step 2: Initialize uv Project

```bash
# Initialize uv project
uv init

# Add dependencies
uv add cohere qdrant-client beautifulsoup4 requests tiktoken python-dotenv lxml
```

**Expected output**: `pyproject.toml` created with dependencies listed.

### Step 3: Create Environment Configuration

```bash
# Create .env file from template
cat > .env << 'EOF'
# Required: Cohere API key for embeddings
COHERE_API_KEY=your_cohere_api_key_here

# Required: Qdrant connection
QDRANT_URL=http://localhost:6333  # or https://your-cluster.qdrant.io
QDRANT_API_KEY=  # Only required for Qdrant Cloud

# Required: Target Docusaurus site
BASE_URL=https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/

# Optional: Tuning parameters (defaults shown)
MAX_TOKENS_PER_CHUNK=500
CHUNK_OVERLAP=50
COHERE_BATCH_SIZE=96
COLLECTION_NAME=rag_embedding
EOF
```

**Action Required**: Edit `.env` and replace `your_cohere_api_key_here` with your actual Cohere API key.

### Step 4: Verify Environment

```bash
# Load environment variables
source .env  # Linux/macOS
# Or for PowerShell: Get-Content .env | ForEach-Object { $key, $value = $_ -split '=', 2; [System.Environment]::SetEnvironmentVariable($key, $value) }

# Test Cohere connection
uv run python -c "import cohere; co = cohere.Client('$COHERE_API_KEY'); print('Cohere: OK')"

# Test Qdrant connection
uv run python -c "from qdrant_client import QdrantClient; client = QdrantClient(url='$QDRANT_URL'); print('Qdrant: OK')"
```

**Expected output**: Both "Cohere: OK" and "Qdrant: OK" printed.

---

## Usage

### Running the Pipeline

```bash
# From backend/ directory
uv run python main.py
```

**Expected behavior**:
- Script starts and validates environment variables
- Connects to Qdrant and creates collection (if not exists)
- Fetches sitemap.xml and discovers URLs
- Processes each URL: extract → chunk → embed → store
- Logs progress every 10 pages
- Prints final statistics on completion

**Estimated runtime**: 20-30 minutes for ~200 pages (depends on API rate limits and network speed).

### Sample Output

```
2025-12-16 10:30:00 [INFO] Starting RAG ingestion pipeline
2025-12-16 10:30:01 [INFO] Qdrant connection verified
2025-12-16 10:30:02 [INFO] Collection 'rag_embedding' created
2025-12-16 10:30:03 [INFO] Fetching sitemap from https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/sitemap.xml
2025-12-16 10:30:04 [INFO] Discovered 198 URLs
2025-12-16 10:30:05 [INFO] Processing page 1/198: https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/intro
2025-12-16 10:30:06 [INFO]   Extracted 2847 chars, created 6 chunks
2025-12-16 10:30:07 [INFO]   Generated 6 embeddings, stored 6 vectors
...
2025-12-16 10:55:30 [INFO] Processing page 198/198: ...
2025-12-16 10:55:35 [INFO] Ingestion complete!

==================================================
INGESTION COMPLETE
==================================================
{
  "base_url": "https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/",
  "status": "completed",
  "pages_crawled": 198,
  "pages_failed": 0,
  "chunks_created": 1247,
  "vectors_stored": 1247,
  "start_time": "2025-12-16T10:30:00",
  "end_time": "2025-12-16T10:55:35",
  "duration_seconds": 1535,
  "error_count": 0,
  "errors": []
}
```

---

## Validation

### Step 1: Check Qdrant Collection

```bash
# Get collection info
uv run python -c "
from qdrant_client import QdrantClient
import os

client = QdrantClient(url=os.getenv('QDRANT_URL'))
info = client.get_collection('rag_embedding')
print(f'Total vectors: {info.vectors_count}')
print(f'Indexed vectors: {info.indexed_vectors_count}')
"
```

**Expected output**: `Total vectors: ~1200-1500` (depends on actual page count).

### Step 2: Test Semantic Search

```bash
# Create test query script
cat > test_search.py << 'EOF'
import os
import cohere
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

# Initialize clients
co = cohere.Client(os.getenv('COHERE_API_KEY'))
qdrant = QdrantClient(url=os.getenv('QDRANT_URL'))

# Test query
query = "How do I create a ROS 2 node for robot perception?"

# Generate query embedding
response = co.embed(
    texts=[query],
    model="embed-english-v3.0",
    input_type="search_query"  # Note: different from "search_document"
)
query_vector = response.embeddings[0]

# Search Qdrant
results = qdrant.search(
    collection_name="rag_embedding",
    query_vector=query_vector,
    limit=5
)

# Display results
print(f"Query: {query}\n")
for i, result in enumerate(results, 1):
    print(f"{i}. Score: {result.score:.3f}")
    print(f"   Title: {result.payload['title']}")
    print(f"   URL: {result.payload['url']}")
    print(f"   Content: {result.payload['content'][:150]}...\n")
EOF

# Run test
uv run python test_search.py
```

**Expected output**: Top 5 most relevant documentation chunks with scores > 0.6 (cosine similarity).

### Step 3: Validate Content Quality

Manually inspect a few chunks to verify:
- No navigation/UI text included (e.g., "Next", "Previous", "Table of Contents")
- Code blocks preserved correctly
- Headings and paragraph structure maintained
- Chunk sizes reasonable (300-500 tokens)

```bash
# Sample random chunk
uv run python -c "
from qdrant_client import QdrantClient
import os
import random

client = QdrantClient(url=os.getenv('QDRANT_URL'))
points = client.scroll('rag_embedding', limit=10)[0]
sample = random.choice(points)

print(f\"URL: {sample.payload['url']}\")
print(f\"Title: {sample.payload['title']}\")
print(f\"Chunk Index: {sample.payload['chunk_index']}\")
print(f\"Content:\n{sample.payload['content']}\")
"
```

---

## Troubleshooting

### Error: Missing Environment Variables

**Symptom**: `KeyError: 'COHERE_API_KEY'` or similar

**Solution**:
```bash
# Verify .env file exists and is correctly formatted
cat .env

# Reload environment
source .env  # Linux/macOS
```

### Error: Qdrant Connection Refused

**Symptom**: `ConnectionRefusedError` or `qdrant_client.exceptions.UnexpectedResponse`

**Solution**:
```bash
# Check Qdrant is running
curl http://localhost:6333/collections  # Should return JSON

# If using Docker, verify container is running
docker ps | grep qdrant

# Restart Qdrant
docker stop qdrant && docker start qdrant
```

### Error: Cohere Rate Limit Exceeded

**Symptom**: `cohere.CohereError: rate limit exceeded`

**Solution**:
- Wait 60 seconds for rate limit reset
- Reduce `COHERE_BATCH_SIZE` in `.env` (e.g., from 96 to 50)
- Upgrade to paid Cohere tier for higher limits

### Warning: Short Content Extracted

**Symptom**: Logs show `Short content (<100 chars) at {url}`

**Possible Causes**:
1. Page is mostly images/videos (acceptable, skip these)
2. Content extraction failed (check CSS selector targets correct element)
3. Page returns 404 or redirects (check URL validity)

**Solution**: Inspect page manually, adjust extraction logic if needed.

### Error: Duplicate Vector IDs

**Symptom**: Qdrant upsert fails with `Duplicate point ID` error

**Cause**: `hash(chunk_id)` collision (rare but possible).

**Solution**: Use UUID instead of hash for point IDs:
```python
import uuid
point_id = int(uuid.uuid4().int % (2**63))  # Convert UUID to int
```

---

## Re-running the Pipeline

To re-ingest content (e.g., after documentation updates):

### Option 1: Full Re-index (Recommended)

```bash
# Delete existing collection
uv run python -c "
from qdrant_client import QdrantClient
import os

client = QdrantClient(url=os.getenv('QDRANT_URL'))
client.delete_collection('rag_embedding')
print('Collection deleted')
"

# Run pipeline again
uv run python main.py
```

### Option 2: Incremental Update (Future Enhancement)

Not implemented in MVP. Will require:
- Content hashing to detect changes
- Filtering Qdrant by URL and deleting old chunks
- Re-processing only changed pages

---

## Performance Tuning

### Speed Up Ingestion

1. **Increase Cohere batch size** (if on paid tier):
   ```env
   COHERE_BATCH_SIZE=200  # Up to 200 on paid tier
   ```

2. **Parallel processing** (future enhancement):
   - Process multiple pages concurrently (requires careful rate limit handling)

3. **Skip unchanged pages** (requires incremental update feature):
   - Check `content_hash` in Qdrant before re-processing

### Reduce API Costs

1. **Filter URLs before processing**:
   - Skip non-documentation pages (e.g., `blog/`, `tags/`)
   ```python
   urls = [u for u in urls if '/docs/' in u or '/intro' in u]
   ```

2. **Test on subset first**:
   ```python
   urls = urls[:10]  # Process only first 10 pages for testing
   ```

---

## Next Steps

After successful ingestion:

1. **Build RAG Query API**: Create FastAPI endpoint for semantic search
2. **Integrate with Chatbot UI**: Connect frontend to query backend
3. **Monitor Usage**: Track search queries, latency, and relevance
4. **Schedule Re-ingestion**: Set up cron job or GitHub Action to refresh weekly

---

## Additional Resources

- **Cohere Documentation**: https://docs.cohere.com/
- **Qdrant Documentation**: https://qdrant.tech/documentation/
- **Feature Spec**: [spec.md](./spec.md)
- **Implementation Plan**: [plan.md](./plan.md)
- **Data Model**: [data-model.md](./data-model.md)

---

## Support

For issues or questions:
1. Check logs for specific error messages
2. Verify all environment variables are set correctly
3. Test Cohere and Qdrant connections independently
4. Consult feature specification for requirements and edge cases
