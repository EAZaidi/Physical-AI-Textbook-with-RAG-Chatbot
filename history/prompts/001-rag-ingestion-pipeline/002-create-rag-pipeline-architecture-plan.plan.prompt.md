---
id: 002
title: Create RAG pipeline architecture plan
stage: plan
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5
feature: 001-rag-ingestion-pipeline
branch: 001-rag-ingestion-pipeline
user: Essa Zaidi
command: /sp.plan
labels: ["rag", "architecture", "design", "cohere", "qdrant", "python", "backend"]
links:
  spec: specs/001-rag-ingestion-pipeline/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-ingestion-pipeline/plan.md
  - specs/001-rag-ingestion-pipeline/research.md
  - specs/001-rag-ingestion-pipeline/data-model.md
  - specs/001-rag-ingestion-pipeline/contracts/qdrant-schema.json
  - specs/001-rag-ingestion-pipeline/quickstart.md
  - CLAUDE.md (agent context updated)
tests:
  - Constitution Check validation (all gates passed)
  - Technology stack research and decisions
---

## Prompt

/sp.plan

- Create `backend/` directory and initialize project using `uv package`
- Setup **Cohere** and **Qdrant** clients
- Fetch, clean, and chunk text from deployed URLs
- Generate embeddings and upsert into Qdrant with metadata
- Only in one file name main.py (get_all_urls, extract_text_from_url chunk_text, embed, create_collection named rag_embedding, save_chunk_to_qdrant and execute in last main function) here is the deployed link: https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/

## Response snapshot

Created comprehensive architectural plan for RAG Website Ingestion and Embedding Pipeline with complete Phase 0 (research) and Phase 1 (design) deliverables.

**Implementation Plan** (plan.md):
- **Technical Context**: Python 3.11+, single-file backend/main.py implementation
- **Dependencies**: cohere, qdrant-client, beautifulsoup4, requests, tiktoken, python-dotenv
- **Architecture**: Linear pipeline (crawl → extract → chunk → embed → store)
- **Constitution Check**: All 7 principles validated (Source Accuracy, Reproducibility, RAG Fidelity, Production Standards)
- **Complexity Justification**: Single-file (~300 LOC) acceptable for clear linear workflow

**Phase 0: Research** (research.md) - 7 technology decisions documented:
1. **Crawling**: Sitemap.xml parsing (reliable, fast, respects robots.txt)
2. **Extraction**: CSS selector targeting `<article>` tags (<5% noise requirement)
3. **Chunking**: Recursive paragraph splitting with sentence boundaries (300-500 tokens, 50 overlap)
4. **Embeddings**: Cohere embed-english-v3.0 (1024 dims, $0.10/1M tokens)
5. **Storage**: Qdrant with Cosine distance, HNSW indexing
6. **Retry Logic**: Exponential backoff (1s → 2s → 4s) with ±20% jitter
7. **Progress**: Structured stdout logging with JSON summaries

**Phase 1: Data Model** (data-model.md) - 4 entities defined:
1. **DocumentPage**: URL, title, text_content, timestamp, status
2. **TextChunk**: chunk_id, content, token_count, source metadata
3. **QdrantPayload**: url, title, chunk_index, content, timestamp, content_hash (SHA256)
4. **IngestionJob**: Progress tracking (pages_crawled, chunks_created, vectors_stored, errors)

**Phase 1: Contracts** (contracts/qdrant-schema.json):
- Collection: `rag_embedding`, 1024 dimensions, Cosine distance
- Indexes: url (keyword), content_hash (keyword) for duplicate detection
- Estimated: ~1000-2000 vectors, ~12-24 MB storage, <50ms search latency

**Phase 1: Quickstart** (quickstart.md):
- Prerequisites: Python 3.11+, uv, Cohere API key, Qdrant instance
- Setup: 5 steps (create directory, install deps, configure .env, verify connections)
- Usage: `uv run python main.py` (~20-30 min for 200 pages)
- Validation: Query Qdrant, test semantic search, inspect content quality

**Function Signatures**:
- `get_all_urls(base_url)` → List[str]
- `extract_text_from_url(url)` → DocumentPage
- `chunk_text(page, max_tokens, overlap)` → List[TextChunk]
- `embed(chunks, api_key, batch_size)` → List[Tuple[TextChunk, List[float]]]
- `create_collection(client, name)` → None
- `save_chunk_to_qdrant(client, name, chunk, embedding)` → None
- `main()` → None (orchestrates full pipeline)

**ADR Recommendations** (3 significant decisions):
1. Single-file implementation vs modular architecture
2. Cohere embed-english-v3.0 model selection
3. Full re-ingestion vs incremental updates (MVP: full only, P5 deferred)

## Outcome

- ✅ Impact: Complete architectural plan ready for task generation (/sp.tasks) and implementation
- 🧪 Tests: Constitution Check passed, technology decisions researched and validated
- 📁 Files: Created plan.md, research.md, data-model.md, contracts/qdrant-schema.json, quickstart.md (5 files, ~1200 lines total)
- 🔁 Next prompts: `/sp.adr` to document 3 architectural decisions, then `/sp.tasks` to generate implementation tasks
- 🧠 Reflection: Single-file design balances simplicity with maintainability for 300 LOC linear pipeline. Research phase resolved all technical unknowns. Ready for implementation with clear contracts and validation strategy.

## Evaluation notes (flywheel)

- Failure modes observed: None - all research questions resolved, no NEEDS CLARIFICATION markers remain
- Graders run and results (PASS/FAIL): Constitution Check (PASS - 7/7 principles), Technical Completeness (PASS - all sections filled), Phase Deliverables (PASS - research.md, data-model.md, contracts/, quickstart.md complete)
- Prompt variant (if applicable): Standard /sp.plan workflow with user-specified single-file constraint
- Next experiment (smallest change to try): Generate ADRs for 3 architectural decisions, then proceed to /sp.tasks for TDD-based implementation breakdown
