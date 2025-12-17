"""Session management endpoints for RAG Agent API."""

import logging

from fastapi import APIRouter, HTTPException, status

from agent_api.models import SessionHistoryResponse
from agent_api.sessions import get_session, delete_session
from agent_api.agent import clear_agent_cache

logger = logging.getLogger(__name__)

router = APIRouter()


@router.get("/{session_id}/history", response_model=SessionHistoryResponse)
async def get_session_history(session_id: str):
    """Get conversation history for a session (T027).

    Args:
        session_id: Session identifier

    Returns:
        SessionHistoryResponse with messages and metadata
    """
    try:
        thread = get_session(session_id)

        return SessionHistoryResponse(
            thread_id=thread.thread_id,
            messages=thread.messages,
            created_at=thread.created_at,
            updated_at=thread.updated_at,
            topic_summary=thread.topic_summary
        )
    except Exception as e:
        logger.error(f"Error retrieving session history: {e}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail={
                "error_code": "SESSION_NOT_FOUND",
                "message": f"Session {session_id} not found",
                "suggested_action": "Check the session_id or start a new conversation"
            }
        )


@router.delete("/{session_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_session_endpoint(session_id: str):
    """Delete a conversation session (T029).

    Args:
        session_id: Session identifier

    Returns:
        204 No Content on success
    """
    # Clear from agent cache
    clear_agent_cache(session_id)

    # Delete from database
    deleted = delete_session(session_id)

    if not deleted:
        logger.warning(f"Attempted to delete non-existent session: {session_id}")
        # Still return 204 for idempotency

    logger.info(f"Session deleted: {session_id}")
    return None
