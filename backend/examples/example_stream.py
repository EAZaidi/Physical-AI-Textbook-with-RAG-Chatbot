"""Example: Streaming chat with Server-Sent Events (SSE).

Demonstrates:
- Making a streaming request to /chat/stream
- Parsing SSE events in real-time
- Handling different event types (tool_call, retrieval, content, done, error)
- Displaying streaming responses progressively
"""

import requests
import json
import sys


# Configuration
API_URL = "http://localhost:8000"
API_KEY = "your-api-key-here"  # Replace with actual API key if authentication is enabled


def parse_sse_line(line: str) -> tuple:
    """Parse a single SSE line.

    Args:
        line: SSE formatted line

    Returns:
        Tuple of (field, value) or (None, None) if not a data line
    """
    if line.startswith("event:"):
        return ("event", line[6:].strip())
    elif line.startswith("data:"):
        return ("data", line[5:].strip())
    else:
        return (None, None)


def chat_stream(message: str, session_id: str = None):
    """Send a streaming chat request and yield events.

    Args:
        message: Question to ask the agent
        session_id: Optional session ID for conversation continuity

    Yields:
        Parsed event dictionaries
    """
    endpoint = f"{API_URL}/chat/stream"

    payload = {
        "message": message,
        "session_id": session_id or "example-stream-session",
        "stream": True,
        "top_k": 5,
        "similarity_threshold": 0.7
    }

    headers = {
        "Content-Type": "application/json",
        "Accept": "text/event-stream",
    }

    # Add API key if using authentication
    if API_KEY and API_KEY != "your-api-key-here":
        headers["X-API-Key"] = API_KEY

    # Stream the response
    with requests.post(endpoint, json=payload, headers=headers, stream=True) as response:
        response.raise_for_status()

        current_event = None
        current_data = None

        # Parse SSE stream
        for line in response.iter_lines(decode_unicode=True):
            if not line:
                # Empty line indicates end of event
                if current_event and current_data:
                    try:
                        parsed_data = json.loads(current_data)
                        yield {
                            "event": current_event,
                            "data": parsed_data
                        }
                    except json.JSONDecodeError:
                        yield {
                            "event": current_event,
                            "data": current_data
                        }

                    current_event = None
                    current_data = None
                continue

            field, value = parse_sse_line(line)

            if field == "event":
                current_event = value
            elif field == "data":
                current_data = value


def main():
    """Run example streaming chat interaction."""

    print("=" * 70)
    print("RAG Agent API - Streaming Chat Example")
    print("=" * 70)
    print()

    question = "What is a ROS 2 node and how does it communicate?"
    print(f"Question: {question}")
    print()
    print("Streaming response:")
    print("-" * 70)

    try:
        full_response = ""
        tool_results = None
        retrieval_info = None

        for event in chat_stream(question):
            event_type = event.get("event")
            event_data = event.get("data", {})

            if event_type == "tool_call":
                # Agent called search_textbook
                function = event_data.get("function")
                args = event_data.get("arguments", {})
                print(f"\n🔧 Tool Call: {function}")
                print(f"   Query: {args.get('query')}")
                print()

            elif event_type == "retrieval":
                # Retrieved chunks from Qdrant
                retrieval_info = event_data
                print(f"📚 Retrieved {event_data.get('num_chunks')} chunks")
                print(f"   Average Score: {event_data.get('average_score', 0):.3f}")
                print(f"   Confidence: {event_data.get('confidence_level')}")
                print()
                print("💬 Agent Response: ", end="", flush=True)

            elif event_type == "content":
                # Streaming response token
                delta = event_data.get("delta", "")
                full_response += delta
                print(delta, end="", flush=True)

            elif event_type == "done":
                # Stream completed successfully
                print()
                print()
                tool_results = event_data.get("tool_results", {})
                usage = event_data.get("usage", {})

                print("-" * 70)
                print("✅ Stream Completed")
                print()
                print(f"📊 Usage Statistics:")
                print(f"   Prompt Tokens: {usage.get('prompt_tokens', 0)}")
                print(f"   Completion Tokens: {usage.get('completion_tokens', 0)}")
                print(f"   Total Tokens: {usage.get('total_tokens', 0)}")
                print()

                if tool_results and "confidence_metrics" in tool_results:
                    metrics = tool_results["confidence_metrics"]
                    print(f"🎯 Confidence Metrics:")
                    print(f"   Average Similarity: {metrics.get('average_similarity', 0):.3f}")
                    print(f"   Min Similarity: {metrics.get('min_similarity', 0):.3f}")
                    print(f"   Max Similarity: {metrics.get('max_similarity', 0):.3f}")
                    print(f"   Chunk Diversity: {metrics.get('chunk_diversity', 0):.3f}")
                    print(f"   Confidence Level: {metrics.get('confidence_level')}")
                    print(f"   Should Answer: {metrics.get('should_answer')}")

            elif event_type == "error":
                # Error occurred during streaming
                print()
                print()
                print(f"❌ Error: {event_data.get('message')}")
                print(f"   Error Code: {event_data.get('error_code')}")
                print(f"   Suggested Action: {event_data.get('suggested_action')}")

        print()
        print("=" * 70)
        print("Example completed!")
        print("=" * 70)

    except requests.exceptions.RequestException as e:
        print(f"\n❌ Request Error: {e}")
        if hasattr(e.response, 'text'):
            print(f"Response: {e.response.text[:500]}")

    except KeyboardInterrupt:
        print("\n\n⚠️  Stream interrupted by user")


if __name__ == "__main__":
    main()
