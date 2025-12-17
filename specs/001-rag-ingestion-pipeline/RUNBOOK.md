# RAG Ingestion Pipeline - Operational Runbook

**Feature**: 001-rag-ingestion-pipeline
**Service**: backend/main.py
**Owner**: Physical AI & Humanoid Robotics Textbook Team
**Last Updated**: 2025-12-16

---

## Table of Contents

1. [Service Overview](#service-overview)
2. [Starting the Pipeline](#starting-the-pipeline)
3. [Stopping the Pipeline](#stopping-the-pipeline)
4. [Monitoring & Health Checks](#monitoring--health-checks)
5. [Troubleshooting](#troubleshooting)
6. [Common Operations](#common-operations)
7. [Escalation](#escalation)

---

## Service Overview

### Purpose
Ingests Physical AI & Humanoid Robotics Textbook documentation from Docusaurus deployment into Qdrant vector database for semantic search capabilities.

### Architecture
- **Type**: Batch processing script
- **Language**: Python 3.11+
- **Dependencies**: Cohere API, Qdrant Cloud
- **Runtime**: uv package manager
- **Execution Model**: Manual or scheduled (cron job)

### Key Components
1. **Crawler**: Fetches all URLs from sitemap.xml
2. **Extractor**: Parses HTML and extracts main content
3. **Chunker**: Splits text into semantic chunks (300-500 tokens)
4. **Embedder**: Generates 1024-dim vectors via Cohere API
5. **Storer**: Upserts vectors to Qdrant collection

### Service Dependencies
- **Cohere API**: embedding generation (embed-english-v3.0)
- **Qdrant Cloud**: vector storage (collection: rag_embedding)
- **Target Website**: https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/

---

## Starting the Pipeline

### Prerequisites Check

1. **Verify environment variables**:
   ```bash
   cd backend
   cat .env | grep -E "(COHERE_API_KEY|QDRANT_URL|QDRANT_API_KEY|BASE_URL)"
   ```
   All 4 variables must be set.

2. **Test Qdrant connectivity**:
   ```bash
   uv run python -c "
   from qdrant_client import QdrantClient
   from dotenv import load_dotenv
   import os
   load_dotenv()
   client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
   print('Qdrant connection: OK')
   "
   ```

3. **Test Cohere API**:
   ```bash
   uv run python -c "
   import cohere
   from dotenv import load_dotenv
   import os
   load_dotenv()
   co = cohere.Client(os.getenv('COHERE_API_KEY'))
   response = co.embed(texts=['test'], model='embed-english-v3.0')
   print('Cohere API: OK')
   "
   ```

### Starting the Ingestion

**Normal Start**:
```bash
cd backend
uv run python main.py
```

**Background Execution** (Linux/macOS):
```bash
cd backend
nohup uv run python main.py > ingestion.log 2>&1 &
echo $! > pipeline.pid
```

**Background Execution** (Windows PowerShell):
```powershell
cd backend
Start-Process -NoNewWindow -FilePath "uv" -ArgumentList "run","python","main.py" -RedirectStandardOutput "ingestion.log" -RedirectStandardError "ingestion_error.log"
```

### Expected Startup Behavior

1. Logs "Starting RAG ingestion pipeline"
2. Connects to Qdrant
3. Creates/verifies collection `rag_embedding`
4. Fetches sitemap from BASE_URL
5. Begins processing pages sequentially

**Expected Duration**: 2-5 minutes for 45 pages, 20-30 minutes for 200 pages

---

## Stopping the Pipeline

### Graceful Stop

The pipeline does not have a graceful shutdown mechanism. Use Ctrl+C for interactive sessions.

**Interactive Session**:
```bash
# Press Ctrl+C
```

**Background Process** (Linux/macOS):
```bash
# If you saved the PID
kill $(cat backend/pipeline.pid)

# Or find and kill
ps aux | grep "main.py"
kill <PID>
```

**Background Process** (Windows):
```powershell
Get-Process | Where-Object {$_.ProcessName -eq "python"} | Stop-Process
```

### Force Stop

```bash
kill -9 <PID>
```

⚠️ **Warning**: Force stop may leave partial data in Qdrant. Re-run the pipeline to ensure consistency.

---

## Monitoring & Health Checks

### Real-Time Monitoring

**During Execution**:
- Monitor stdout for progress logs
- Look for "Processing page X/Y" messages
- Check for ERROR/WARNING log levels

**Key Log Messages**:
```
✅ Good: "Processing page 10/45: <url>"
✅ Good: "Generated embeddings for batch 1 (5 chunks)"
⚠️  Warning: "Retry 1/3 after 2.1s: <error>"
❌ Error: "Failed after 3 retries: <error>"
```

### Post-Execution Validation

1. **Check final statistics**:
   ```
   INGESTION COMPLETE
   pages_crawled: 45
   pages_failed: 0
   chunks_created: 45
   vectors_stored: 45
   ```

2. **Verify Qdrant collection**:
   ```bash
   uv run python -c "
   from qdrant_client import QdrantClient
   from dotenv import load_dotenv
   import os
   load_dotenv()
   client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
   info = client.get_collection('rag_embedding')
   print(f'Vectors in collection: {info.points_count}')
   print(f'Vector size: {info.config.params.vectors.size}')
   print(f'Distance: {info.config.params.vectors.distance}')
   "
   ```
   Expected: `Vectors in collection: 45` (or total after full run)

3. **Test semantic search**:
   ```bash
   uv run python -c "
   from qdrant_client import QdrantClient
   import cohere
   from dotenv import load_dotenv
   import os
   load_dotenv()

   client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
   co = cohere.Client(os.getenv('COHERE_API_KEY'))

   query_vec = co.embed(texts=['ROS 2 node'], model='embed-english-v3.0', input_type='search_query').embeddings[0]
   results = client.query_points(collection_name='rag_embedding', query=query_vec, limit=1).points

   print(f'Top result: {results[0].payload[\"title\"]}')
   print(f'Score: {results[0].score:.4f}')
   "
   ```
   Expected: Top result should be relevant to "ROS 2 node"

### Performance Metrics

| Metric | Expected | Alert Threshold |
|--------|----------|-----------------|
| Pages/minute | 20-25 | <10 |
| API failures | 0-1% | >5% |
| Processing time (45 pages) | 2-5 min | >10 min |
| Processing time (200 pages) | 20-30 min | >60 min |
| Vectors stored | = Pages crawled | < Pages - 5 |

---

## Troubleshooting

### Issue 1: Sitemap Not Found

**Symptoms**:
```
ERROR: Failed to fetch sitemap: 404 Client Error
```

**Diagnosis**:
- Check BASE_URL is correct
- Verify sitemap.xml exists at `{BASE_URL}/sitemap.xml`

**Resolution**:
```bash
curl -I https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/sitemap.xml
# Should return 200 OK
```

### Issue 2: Cohere API Rate Limit

**Symptoms**:
```
WARNING: Retry 1/3 after 2.0s: rate limit exceeded
```

**Diagnosis**:
- Cohere API rate limit hit
- Retry logic will handle automatically

**Resolution**:
- If retries fail, wait 1 minute and restart pipeline
- Consider upgrading Cohere API tier for higher limits

### Issue 3: Qdrant Connection Timeout

**Symptoms**:
```
ERROR: Failed to connect to Qdrant: timeout
```

**Diagnosis**:
- Network connectivity issue
- Qdrant Cloud instance unavailable

**Resolution**:
1. Check network connectivity:
   ```bash
   ping $(echo $QDRANT_URL | sed 's/https:\/\///' | cut -d: -f1)
   ```
2. Verify Qdrant Cloud status
3. Check API key validity

### Issue 4: XML Parser Warning

**Symptoms**:
```
XMLParsedAsHTMLWarning: It looks like you're using an HTML parser to parse an XML document
```

**Diagnosis**:
- BeautifulSoup warning (not an error)
- Parser works correctly despite warning

**Resolution** (optional):
```bash
cd backend
uv add lxml
```
Then update main.py line 125 to use XML parser.

### Issue 5: All Pages Failing

**Symptoms**:
```
pages_failed: 45
pages_crawled: 0
```

**Diagnosis**:
- Website unreachable
- Incorrect BASE_URL
- Authentication required

**Resolution**:
1. Test website manually:
   ```bash
   curl -I https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/
   ```
2. Check .env BASE_URL has trailing slash
3. Verify website is publicly accessible

### Issue 6: Memory Errors

**Symptoms**:
```
MemoryError: Unable to allocate array
```

**Diagnosis**:
- Insufficient memory for large page sets
- Too many chunks in memory

**Resolution**:
- Increase system memory
- Process in smaller batches (modify MAX_TOKENS_PER_CHUNK)

---

## Common Operations

### Operation 1: Re-ingest Entire Site

**When**: Content updates, full refresh needed

**Steps**:
1. **Option A**: Keep existing vectors (upsert will update)
   ```bash
   cd backend
   uv run python main.py
   ```

2. **Option B**: Delete collection first (fresh start)
   ```bash
   uv run python -c "
   from qdrant_client import QdrantClient
   from dotenv import load_dotenv
   import os
   load_dotenv()
   client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
   client.delete_collection('rag_embedding')
   print('Collection deleted')
   "

   # Then run pipeline
   uv run python main.py
   ```

### Operation 2: Update Single Page

**When**: One page changed, don't want full re-ingestion

**Steps**:
Currently not supported (incremental updates deferred to post-MVP).
**Workaround**: Run full pipeline (upsert will replace existing vectors for that page).

### Operation 3: Change Target URL

**Steps**:
1. Edit `.env`:
   ```bash
   nano .env
   # Update BASE_URL
   ```
2. Delete old collection (optional):
   ```bash
   # See Operation 1, Option B
   ```
3. Run pipeline:
   ```bash
   uv run python main.py
   ```

### Operation 4: Monitor Qdrant Collection Size

**Steps**:
```bash
uv run python -c "
from qdrant_client import QdrantClient
from dotenv import load_dotenv
import os
load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
info = client.get_collection('rag_embedding')
print(f'Total vectors: {info.points_count}')
print(f'Collection size: {info.points_count * 1024 * 4 / 1024 / 1024:.2f} MB (approx)')
"
```

### Operation 5: Scheduled Ingestion (Cron)

**Daily at 2 AM** (Linux/macOS):
```bash
crontab -e
# Add:
0 2 * * * cd /path/to/backend && /usr/local/bin/uv run python main.py >> /var/log/rag-ingestion.log 2>&1
```

**Weekly on Sunday at 2 AM**:
```bash
0 2 * * 0 cd /path/to/backend && /usr/local/bin/uv run python main.py >> /var/log/rag-ingestion.log 2>&1
```

---

## Escalation

### Level 1: Application Issues
**Contact**: Development Team
**Examples**: Pipeline crashes, API errors, data quality issues
**Response Time**: 24 hours

### Level 2: Infrastructure Issues
**Contact**: DevOps/Platform Team
**Examples**: Qdrant unavailable, network issues, deployment problems
**Response Time**: 4 hours

### Level 3: Critical Outage
**Contact**: On-call Engineer
**Examples**: Complete service failure, data loss, security breach
**Response Time**: Immediate

### Escalation Checklist

Before escalating, collect:
- [ ] Full error logs from stdout/stderr
- [ ] Environment configuration (sanitize secrets)
- [ ] Qdrant collection statistics
- [ ] Recent changes to code or configuration
- [ ] Steps to reproduce the issue

---

## Appendix

### Useful Commands

**Check Python/uv version**:
```bash
python --version
uv --version
```

**List installed packages**:
```bash
cd backend
uv pip list
```

**View recent logs** (if using nohup):
```bash
tail -f backend/ingestion.log
```

**Search logs for errors**:
```bash
grep -i error backend/ingestion.log
```

### Related Documentation

- [Feature Specification](./spec.md)
- [Implementation Plan](./plan.md)
- [Validation Report](./VALIDATION_REPORT.md)
- [Backend README](../../backend/README.md)
- [Quickstart Guide](./quickstart.md)

### Contact Information

- **GitHub Issues**: [physical-ai-humanoid-book/issues](https://github.com/yourusername/physical-ai-humanoid-book/issues)
- **Documentation**: [Docusaurus Site](https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/)
