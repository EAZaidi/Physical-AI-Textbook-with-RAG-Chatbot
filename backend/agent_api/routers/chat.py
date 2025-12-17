"""Chat endpoints for RAG Agent API."""

import time
import logging
import json
from typing import Dict, List, AsyncGenerator

from fastapi import APIRouter, HTTPException, status
from fastapi.responses import StreamingResponse
from qdrant_client.http.exceptions import UnexpectedResponse as QdrantError
from openai import RateLimitError, APIError

from agent_api.models import ChatRequest, ChatResponse, Source
from agent_api.agent import create_rag_agent, run_agent, run_agent_stream
from agent_api.sessions import get_session, save_message, get_conversation_history
from agent_api.models import Message, ConfidenceMetrics
from agent_api.confidence import get_confidence_explanation

logger = logging.getLogger(__name__)

router = APIRouter()


def extract_sources(tool_results: Dict) -> List[Source]:
    """Extract Source objects from tool results (T018).

    Args:
        tool_results: Results from search_textbook tool

    Returns:
        List of Source objects with citations
    """
    if not tool_results or "chunks" not in tool_results:
        return []

    sources = []
    chunks = tool_results.get("chunks", [])
    scores = tool_results.get("scores", [])
    metadata_list = tool_results.get("metadata", [])

    for i, chunk in enumerate(chunks):
        if i < len(scores) and i < len(metadata_list):
            metadata = metadata_list[i]

            # Truncate chunk for display (max 500 chars)
            chunk_display = chunk[:500] + "..." if len(chunk) > 500 else chunk

            source = Source(
                chunk_text=chunk_display,
                similarity_score=scores[i],
                chapter=metadata.get("chapter", "Unknown"),
                section=metadata.get("section", "Unknown"),
                url=metadata.get("url", ""),
                chunk_index=metadata.get("chunk_index", 0)
            )
            sources.append(source)

    return sources


@router.post("/run", response_model=ChatResponse)
async def chat_run(request: ChatRequest):
    """Synchronous chat endpoint (T017).

    Processes user question, retrieves context, generates grounded response.
    Includes error handling (T019) and logging (T020).
    """
    start_time = time.time()

    # Logging (T020)
    logger.info(f"Chat request: session_id={request.session_id}, message_length={len(request.message)}")

    try:
        # Create or get agent
        agent_data = create_rag_agent(request.session_id)

        # Get conversation history
        conversation_history = []
        try:
            thread = get_session(request.session_id)
            # Convert to OpenAI message format
            for msg in thread.messages[-10:]:  # Last 5 turns = 10 messages
                conversation_history.append({
                    "role": msg.role,
                    "content": msg.content
                })
        except Exception as e:
            logger.warning(f"Could not load conversation history: {e}")

        # Run agent (T017)
        try:
            result = await run_agent(agent_data, request.message, conversation_history)
        except RateLimitError as e:
            # Error handling (T019) - OpenAI rate limit
            logger.error(f"OpenAI rate limit: {e}")
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail={
                    "error_code": "OPENAI_RATE_LIMIT",
                    "message": "OpenAI API rate limit exceeded",
                    "suggested_action": "Please retry in a few seconds"
                }
            )
        except APIError as e:
            # Error handling (T019) - OpenAI API error
            logger.error(f"OpenAI API error: {e}")
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail={
                    "error_code": "OPENAI_API_ERROR",
                    "message": "OpenAI API temporarily unavailable",
                    "suggested_action": "Please retry in a moment"
                }
            )

        # Extract sources (T018)
        sources = extract_sources(result.get("tool_results"))

        # Extract multi-metric confidence assessment (T039)
        tool_results = result.get("tool_results", {})
        confidence_data = tool_results.get("confidence_metrics", {}) if tool_results else {}

        # Create ConfidenceMetrics object from tool results
        if confidence_data:
            confidence_metrics = ConfidenceMetrics(
                average_similarity=confidence_data.get("average_similarity", 0.0),
                min_similarity=confidence_data.get("min_similarity", 0.0),
                max_similarity=confidence_data.get("max_similarity", 0.0),
                num_chunks=confidence_data.get("num_chunks", 0),
                chunk_diversity=confidence_data.get("chunk_diversity", 0.0),
                confidence_level=confidence_data.get("confidence_level", "insufficient"),
                should_answer=confidence_data.get("should_answer", False)
            )
        else:
            # Fallback if confidence_metrics not available
            confidence_metrics = ConfidenceMetrics(
                average_similarity=0.0,
                min_similarity=0.0,
                max_similarity=0.0,
                num_chunks=0,
                chunk_diversity=0.0,
                confidence_level="insufficient",
                should_answer=False
            )

        # Generate confidence explanation (T037)
        confidence_explanation = get_confidence_explanation(confidence_metrics)

        # Build response
        response_text = result.get("response", "")

        chat_response = ChatResponse(
            response=response_text,
            confidence=confidence_metrics.average_similarity,
            sources=sources,
            session_id=request.session_id,
            confidence_level=confidence_metrics.confidence_level,
            should_answer=confidence_metrics.should_answer
        )

        # Save messages to session
        try:
            user_msg = Message(role="user", content=request.message)
            assistant_msg = Message(role="assistant", content=response_text, confidence=confidence_metrics.average_similarity)

            save_message(request.session_id, user_msg)
            save_message(request.session_id, assistant_msg)

            # Generate summary if conversation is long enough (T031)
            from agent_api.sessions import generate_conversation_summary
            await generate_conversation_summary(request.session_id)
        except Exception as e:
            logger.warning(f"Could not save messages: {e}")

        # Logging with multi-metric confidence (T040)
        execution_time = time.time() - start_time
        logger.info(
            f"Chat response: session_id={request.session_id}, "
            f"confidence_level={confidence_metrics.confidence_level}, "
            f"avg_sim={confidence_metrics.average_similarity:.3f}, "
            f"min_sim={confidence_metrics.min_similarity:.3f}, "
            f"max_sim={confidence_metrics.max_similarity:.3f}, "
            f"diversity={confidence_metrics.chunk_diversity:.3f}, "
            f"num_chunks={confidence_metrics.num_chunks}, "
            f"should_answer={confidence_metrics.should_answer}, "
            f"sources={len(sources)}, "
            f"execution_time={execution_time:.2f}s"
        )

        return chat_response

    except QdrantError as e:
        # Error handling (T019) - Qdrant connection error
        logger.error(f"Qdrant error: {e}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail={
                "error_code": "QDRANT_UNAVAILABLE",
                "message": "Vector search service unavailable",
                "suggested_action": "Please retry in a moment"
            }
        )
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Error handling (T019) - Unexpected errors
        logger.error(f"Unexpected error in chat endpoint: {e}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error_code": "INTERNAL_ERROR",
                "message": "An unexpected error occurred",
                "suggested_action": "Please try again or contact support if the issue persists"
            }
        )


async def event_generator(request: ChatRequest) -> AsyncGenerator[str, None]:
    """Generate Server-Sent Events for streaming responses (T054).

    Yields SSE-formatted events:
    - event: tool_call - When agent calls search_textbook
    - event: retrieval - Retrieval results with confidence
    - event: content - Response text deltas
    - event: done - Completion with usage stats
    - event: error - Error occurred during streaming

    Args:
        request: Chat request with stream=true

    Yields:
        SSE-formatted event strings
    """
    try:
        # Get agent and conversation history
        agent_data = create_rag_agent(request.session_id)

        conversation_history = []
        try:
            thread = get_session(request.session_id)
            for msg in thread.messages[-10:]:
                conversation_history.append({
                    "role": msg.role,
                    "content": msg.content
                })
        except Exception as e:
            logger.warning(f"Could not load conversation history: {e}")

        # Stream agent response
        full_response = ""
        tool_results = None

        async for event in run_agent_stream(agent_data, request.message, conversation_history):
            event_type = event.get("event")
            event_data = event.get("data", {})

            # Format as SSE
            sse_event = f"event: {event_type}\ndata: {json.dumps(event_data)}\n\n"
            yield sse_event

            # Accumulate response text
            if event_type == "content":
                full_response += event_data.get("delta", "")

            # Store tool results for session saving
            if event_type == "done":
                tool_results = event_data.get("tool_results")

        # Save messages to session after streaming completes
        if full_response:
            try:
                user_msg = Message(role="user", content=request.message)

                # Extract confidence from tool results
                confidence = 0.0
                if tool_results:
                    confidence_metrics = tool_results.get("confidence_metrics", {})
                    confidence = confidence_metrics.get("average_similarity", 0.0)

                assistant_msg = Message(role="assistant", content=full_response, confidence=confidence)

                save_message(request.session_id, user_msg)
                save_message(request.session_id, assistant_msg)

                # Generate summary if needed
                from agent_api.sessions import generate_conversation_summary
                await generate_conversation_summary(request.session_id)
            except Exception as e:
                logger.warning(f"Could not save messages after streaming: {e}")

    except QdrantError as e:
        # Error event for Qdrant failures
        logger.error(f"Qdrant error during streaming: {e}")
        error_data = {
            "error_code": "QDRANT_UNAVAILABLE",
            "message": "Vector search service unavailable",
            "suggested_action": "Please retry in a moment"
        }
        yield f"event: error\ndata: {json.dumps(error_data)}\n\n"

    except RateLimitError as e:
        # Error event for OpenAI rate limits
        logger.error(f"OpenAI rate limit during streaming: {e}")
        error_data = {
            "error_code": "OPENAI_RATE_LIMIT",
            "message": "OpenAI API rate limit exceeded",
            "suggested_action": "Please retry in a few seconds"
        }
        yield f"event: error\ndata: {json.dumps(error_data)}\n\n"

    except Exception as e:
        # Generic error event
        logger.error(f"Error during streaming: {e}", exc_info=True)
        error_data = {
            "error_code": "STREAMING_ERROR",
            "message": str(e),
            "suggested_action": "Please try again"
        }
        yield f"event: error\ndata: {json.dumps(error_data)}\n\n"


@router.post("/stream")
async def chat_stream(request: ChatRequest):
    """Streaming chat endpoint with Server-Sent Events (T053).

    Enables real-time token-by-token streaming of agent responses.

    **Event Types:**
    - `tool_call`: Agent called search_textbook function
    - `retrieval`: Retrieved chunks with confidence metrics
    - `content`: Response text delta (stream this to user)
    - `done`: Completion with usage statistics
    - `error`: Error occurred (includes error_code, message, suggested_action)

    **Example Usage:**
    ```bash
    curl -X POST http://localhost:8000/chat/stream \\
      -H "Content-Type: application/json" \\
      -H "X-API-Key: your-api-key" \\
      --no-buffer \\
      -d '{
        "message": "What is a ROS 2 node?",
        "session_id": "550e8400-e29b-41d4-a716-446655440000",
        "stream": true
      }'
    ```

    **Expected Output:**
    ```
    event: tool_call
    data: {"function": "search_textbook", "arguments": {"query": "ROS 2 node", "top_k": 5}}

    event: retrieval
    data: {"num_chunks": 5, "average_score": 0.87, "confidence_level": "high"}

    event: content
    data: {"delta": "A"}

    event: content
    data: {"delta": " ROS"}

    event: content
    data: {"delta": " 2"}

    event: content
    data: {"delta": " node"}

    ...

    event: done
    data: {"tool_results": {...}, "usage": {"prompt_tokens": 150, "completion_tokens": 200}}
    ```

    Args:
        request: ChatRequest with stream=true

    Returns:
        StreamingResponse with text/event-stream media type
    """
    # Validate that streaming is requested
    if not request.stream:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail={
                "error_code": "INVALID_REQUEST",
                "message": "Streaming not enabled in request",
                "suggested_action": "Set stream=true or use /chat/run endpoint"
            }
        )

    # Log streaming request
    logger.info(f"Streaming chat request: session_id={request.session_id}, message_length={len(request.message)}")

    # Return streaming response with SSE headers (T055)
    return StreamingResponse(
        event_generator(request),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no"  # Disable nginx buffering
        }
    )
