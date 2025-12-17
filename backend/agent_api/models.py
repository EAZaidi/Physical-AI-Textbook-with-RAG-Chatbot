"""Pydantic models for RAG Agent API.

Defines request/response schemas for API endpoints and internal data structures.
"""

from datetime import datetime
from typing import List, Optional, Dict, Any
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, field_validator


# ============================================================================
# Request Models
# ============================================================================

class ChatRequest(BaseModel):
    """Request model for chat endpoints."""
    
    message: str = Field(..., min_length=1, max_length=1000, description="User's question about the textbook")
    session_id: Optional[str] = Field(default=None, description="Session ID for conversation continuity")
    stream: bool = Field(default=False, description="Enable Server-Sent Events streaming")
    top_k: int = Field(default=5, ge=1, le=10, description="Number of chunks to retrieve")
    similarity_threshold: float = Field(default=0.7, ge=0.0, le=1.0, description="Minimum similarity score")
    
    @field_validator("message")
    @classmethod
    def validate_message(cls, v: str) -> str:
        """Validate message is not empty after stripping."""
        if not v.strip():
            raise ValueError("Message cannot be empty")
        return v.strip()
    
    @field_validator("session_id")
    @classmethod
    def validate_session_id(cls, v: Optional[str]) -> Optional[str]:
        """Generate UUID if session_id not provided."""
        if v is None:
            return str(uuid4())
        return v


# ============================================================================
# Response Models
# ============================================================================

class Source(BaseModel):
    """Source citation for retrieved chunk."""
    
    chunk_text: str = Field(..., description="Retrieved text chunk (truncated for display)")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Cosine similarity score")
    chapter: str = Field(..., description="Chapter name from metadata")
    section: str = Field(..., description="Section name from metadata")
    url: str = Field(..., description="Textbook page URL")
    chunk_index: int = Field(..., ge=0, description="Position within source document")


class ChatResponse(BaseModel):
    """Response model for chat endpoints."""
    
    response: str = Field(..., description="Agent's generated answer")
    confidence: float = Field(..., ge=0.0, le=1.0, description="Aggregate confidence score")
    sources: List[Source] = Field(default_factory=list, description="Source citations")
    session_id: str = Field(..., description="Session ID for follow-ups")
    timestamp: str = Field(default_factory=lambda: datetime.utcnow().isoformat(), description="Response timestamp")
    confidence_level: str = Field(..., description="Human-readable confidence category")
    should_answer: bool = Field(..., description="Whether agent has sufficient context")


# ============================================================================
# Internal Models (for agent processing)
# ============================================================================

class ChunkMetadata(BaseModel):
    """Metadata for a retrieved chunk."""
    
    chapter: str
    section: str
    url: str
    chunk_index: int
    content_hash: str


class RetrievalResult(BaseModel):
    """Result from Qdrant vector search."""
    
    chunks: List[str] = Field(default_factory=list, description="Retrieved text chunks")
    scores: List[float] = Field(default_factory=list, description="Similarity scores")
    metadata: List[ChunkMetadata] = Field(default_factory=list, description="Chunk metadata")
    average_score: float = Field(default=0.0, description="Mean similarity score")
    has_sufficient_context: bool = Field(default=False, description="Meets quality threshold")
    query_embedding: Optional[List[float]] = Field(default=None, description="Query embedding vector")
    
    @field_validator("chunks", "scores", "metadata")
    @classmethod
    def validate_equal_length(cls, v, info):
        """Ensure chunks, scores, and metadata have equal length."""
        # This validation happens after all fields are set
        return v


class ConfidenceMetrics(BaseModel):
    """Multi-metric confidence assessment."""
    
    average_similarity: float = Field(..., ge=0.0, le=1.0, description="Mean cosine similarity")
    min_similarity: float = Field(..., ge=0.0, le=1.0, description="Weakest match score")
    max_similarity: float = Field(..., ge=0.0, le=1.0, description="Best match score")
    num_chunks: int = Field(..., ge=0, description="Number of retrieved chunks")
    chunk_diversity: float = Field(..., ge=0.0, le=1.0, description="Semantic diversity")
    confidence_level: str = Field(..., description="high | medium | low | insufficient")
    should_answer: bool = Field(..., description="Final decision")


# ============================================================================
# Session Models
# ============================================================================

class Message(BaseModel):
    """Single message in conversation."""
    
    role: str = Field(..., description="user or assistant")
    content: str = Field(..., description="Message text")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Message timestamp")
    confidence: Optional[float] = Field(default=None, description="Confidence for assistant messages")
    
    @field_validator("role")
    @classmethod
    def validate_role(cls, v: str) -> str:
        """Validate role is user or assistant."""
        if v not in ["user", "assistant"]:
            raise ValueError("Role must be 'user' or 'assistant'")
        return v


class ConversationThread(BaseModel):
    """Multi-turn conversation session."""
    
    thread_id: str = Field(..., description="Unique session identifier")
    messages: List[Message] = Field(default_factory=list, description="Ordered conversation history")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Session creation time")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="Last message time")
    topic_summary: Optional[str] = Field(default=None, max_length=200, description="AI-generated summary")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Additional context")


class SessionHistoryResponse(BaseModel):
    """Response for session history endpoint."""
    
    thread_id: str
    messages: List[Message]
    created_at: datetime
    updated_at: datetime
    topic_summary: Optional[str] = None


# ============================================================================
# Error Models
# ============================================================================

class ErrorResponse(BaseModel):
    """Structured error response."""
    
    error_code: str = Field(..., description="Machine-readable error code")
    message: str = Field(..., description="Human-readable error message")
    detail: Optional[str] = Field(default=None, description="Additional error details")
    suggested_action: Optional[str] = Field(default=None, description="Recommended user action")
