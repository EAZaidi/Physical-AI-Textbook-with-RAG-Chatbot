"""Session management for RAG Agent API.

Handles SQLAlchemy session creation, conversation persistence, and session cleanup.
"""

from datetime import datetime, timedelta
from typing import Optional, List, Dict, Any
import json

from sqlalchemy import create_engine, Column, String, DateTime, JSON, Text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, Session
from sqlalchemy.pool import QueuePool

from agent_api.config import settings
from agent_api.models import Message, ConversationThread

# SQLAlchemy Base
Base = declarative_base()


# ============================================================================
# Database Models
# ============================================================================

class ConversationThreadDB(Base):
    """Database model for conversation threads."""
    
    __tablename__ = "conversation_threads"
    
    thread_id = Column(String, primary_key=True)
    messages = Column(JSON, nullable=False, default=[])
    created_at = Column(DateTime, nullable=False, default=datetime.utcnow)
    updated_at = Column(DateTime, nullable=False, default=datetime.utcnow, onupdate=datetime.utcnow)
    topic_summary = Column(Text, nullable=True)
    session_metadata = Column(JSON, nullable=False, default={})


# ============================================================================
# Database Engine Configuration
# ============================================================================

engine = create_engine(
    settings.database_url,
    poolclass=QueuePool,
    pool_size=20,           # Handle 20 concurrent sessions
    max_overflow=40,        # Burst to 60 total connections
    pool_pre_ping=True,     # Verify connections before use
    pool_recycle=3600,      # Recycle connections after 1 hour
    echo=False,             # Set to True for SQL logging
)

# Session factory
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


# ============================================================================
# Session Management Functions
# ============================================================================

def get_db_session() -> Session:
    """Get database session for conversation persistence."""
    return SessionLocal()


def get_session(thread_id: str) -> ConversationThread:
    """Get or create conversation thread.
    
    Args:
        thread_id: Unique session identifier
        
    Returns:
        ConversationThread with loaded history or new thread
    """
    db = get_db_session()
    try:
        # Try to load existing thread
        thread_db = db.query(ConversationThreadDB).filter(
            ConversationThreadDB.thread_id == thread_id
        ).first()
        
        if thread_db:
            # Convert database model to Pydantic model
            messages = [Message(**msg) for msg in thread_db.messages]
            return ConversationThread(
                thread_id=thread_db.thread_id,
                messages=messages,
                created_at=thread_db.created_at,
                updated_at=thread_db.updated_at,
                topic_summary=thread_db.topic_summary,
                metadata=thread_db.session_metadata or {}
            )
        else:
            # Create new thread
            new_thread = ConversationThread(
                thread_id=thread_id,
                messages=[],
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
                metadata={}
            )
            
            # Persist to database
            thread_db = ConversationThreadDB(
                thread_id=new_thread.thread_id,
                messages=[],
                created_at=new_thread.created_at,
                updated_at=new_thread.updated_at,
                session_metadata={}
            )
            db.add(thread_db)
            db.commit()
            
            return new_thread
    finally:
        db.close()


def save_message(thread_id: str, message: Message) -> None:
    """Save message to conversation thread.
    
    Args:
        thread_id: Session identifier
        message: Message to append
    """
    db = get_db_session()
    try:
        thread_db = db.query(ConversationThreadDB).filter(
            ConversationThreadDB.thread_id == thread_id
        ).first()
        
        if not thread_db:
            # Create thread if doesn't exist
            thread_db = ConversationThreadDB(
                thread_id=thread_id,
                messages=[],
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
                session_metadata={}
            )
            db.add(thread_db)
        
        # Append message (convert Pydantic to dict)
        messages = thread_db.messages or []
        messages.append(message.model_dump(mode='json'))
        
        # Limit to last MAX_MESSAGES_PER_SESSION
        if len(messages) > settings.max_messages_per_session:
            messages = messages[-settings.max_messages_per_session:]
        
        thread_db.messages = messages
        thread_db.updated_at = datetime.utcnow()
        
        db.commit()
    finally:
        db.close()


def delete_session(thread_id: str) -> bool:
    """Delete conversation thread.
    
    Args:
        thread_id: Session identifier
        
    Returns:
        True if deleted, False if not found
    """
    db = get_db_session()
    try:
        thread_db = db.query(ConversationThreadDB).filter(
            ConversationThreadDB.thread_id == thread_id
        ).first()
        
        if thread_db:
            db.delete(thread_db)
            db.commit()
            return True
        return False
    finally:
        db.close()


def update_topic_summary(thread_id: str, summary: str) -> None:
    """Update conversation topic summary.
    
    Args:
        thread_id: Session identifier
        summary: AI-generated topic summary (max 200 chars)
    """
    db = get_db_session()
    try:
        thread_db = db.query(ConversationThreadDB).filter(
            ConversationThreadDB.thread_id == thread_id
        ).first()
        
        if thread_db:
            thread_db.topic_summary = summary[:200]  # Enforce max length
            thread_db.updated_at = datetime.utcnow()
            db.commit()
    finally:
        db.close()


def cleanup_expired_sessions() -> int:
    """Delete sessions inactive for more than SESSION_EXPIRY_HOURS.
    
    Returns:
        Number of sessions deleted
    """
    db = get_db_session()
    try:
        expiry_time = datetime.utcnow() - timedelta(hours=settings.session_expiry_hours)
        
        expired = db.query(ConversationThreadDB).filter(
            ConversationThreadDB.updated_at < expiry_time
        ).all()
        
        count = len(expired)
        for thread in expired:
            db.delete(thread)
        
        db.commit()
        return count
    finally:
        db.close()


def get_conversation_history(thread_id: str, last_n_turns: int = 5) -> List[Message]:
    """Get last N turns of conversation (2*N messages).
    
    Args:
        thread_id: Session identifier
        last_n_turns: Number of turns (user + assistant pairs) to retrieve
        
    Returns:
        List of last 2*N messages
    """
    thread = get_session(thread_id)
    # Get last 2*N messages (N user + N assistant)
    return thread.messages[-(last_n_turns * 2):]


async def generate_conversation_summary(thread_id: str) -> None:
    """Generate AI summary for conversation (T031).
    
    Automatically called when conversation reaches 10+ exchanges.
    
    Args:
        thread_id: Session identifier
    """
    import asyncio
    from openai import OpenAI
    
    thread = get_session(thread_id)
    
    # Only generate if 10+ exchanges (20+ messages)
    if len(thread.messages) < 20:
        return
    
    # Build conversation text
    conversation_text = "\n".join([
        f"{msg.role}: {msg.content[:200]}"
        for msg in thread.messages[-20:]  # Last 10 turns
    ])
    
    # Generate summary using OpenAI
    from agent_api.config import settings
    client = OpenAI(api_key=settings.openai_api_key)
    
    try:
        response = await asyncio.to_thread(
            client.chat.completions.create,
            model="gpt-4o-mini",
            messages=[
                {
                    "role": "system",
                    "content": "Summarize this conversation in max 200 characters. Focus on the main topics discussed."
                },
                {
                    "role": "user",
                    "content": conversation_text
                }
            ],
            max_tokens=60
        )
        
        summary = response.choices[0].message.content
        if summary:
            update_topic_summary(thread_id, summary[:200])
    except Exception as e:
        # Don't fail the request if summarization fails
        import logging
        logger = logging.getLogger(__name__)
        logger.warning(f"Failed to generate summary for {thread_id}: {e}")
