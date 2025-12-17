"""Example: Basic synchronous chat with the RAG Agent API.

Demonstrates:
- Making a POST request to /chat/run
- Handling the JSON response
- Extracting sources and citations
- Working with confidence scores
"""

import requests
import json
from typing import Dict, Any


# Configuration
API_URL = "http://localhost:8000"
API_KEY = "your-api-key-here"  # Replace with actual API key if authentication is enabled


def chat_sync(message: str, session_id: str = None) -> Dict[str, Any]:
    """Send a synchronous chat request.

    Args:
        message: Question to ask the agent
        session_id: Optional session ID for conversation continuity

    Returns:
        API response dictionary
    """
    endpoint = f"{API_URL}/chat/run"

    payload = {
        "message": message,
        "session_id": session_id or "example-session",
        "top_k": 5,
        "similarity_threshold": 0.7
    }

    headers = {
        "Content-Type": "application/json",
    }

    # Add API key if using authentication
    if API_KEY and API_KEY != "your-api-key-here":
        headers["X-API-Key"] = API_KEY

    response = requests.post(endpoint, json=payload, headers=headers)
    response.raise_for_status()

    return response.json()


def main():
    """Run example synchronous chat interactions."""

    print("=" * 70)
    print("RAG Agent API - Synchronous Chat Example")
    print("=" * 70)
    print()

    # Example 1: Basic question
    print("Example 1: Basic Question")
    print("-" * 70)

    question1 = "What is a ROS 2 node?"
    print(f"Question: {question1}")
    print()

    try:
        result = chat_sync(question1)

        print(f"Response: {result['response'][:200]}...")
        print()
        print(f"Confidence: {result['confidence']:.2f}")
        print(f"Confidence Level: {result['confidence_level']}")
        print(f"Should Answer: {result['should_answer']}")
        print(f"Session ID: {result['session_id']}")
        print()

        # Display sources
        print(f"Sources ({len(result['sources'])} found):")
        for i, source in enumerate(result['sources'][:3], 1):
            print(f"  {i}. Score: {source['similarity_score']:.3f}")
            print(f"     Chapter: {source['chapter']}")
            print(f"     Section: {source['section']}")
            print(f"     Snippet: {source['chunk_text'][:100]}...")
            print()

    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")
        if hasattr(e.response, 'json'):
            error_detail = e.response.json()
            print(f"Error Details: {json.dumps(error_detail, indent=2)}")

    print()

    # Example 2: Out-of-domain question (should refuse)
    print("Example 2: Out-of-Domain Question (Should Refuse)")
    print("-" * 70)

    question2 = "What is the capital of France?"
    print(f"Question: {question2}")
    print()

    try:
        result = chat_sync(question2)

        print(f"Response: {result['response']}")
        print()
        print(f"Confidence: {result['confidence']:.2f}")
        print(f"Confidence Level: {result['confidence_level']}")
        print(f"Should Answer: {result['should_answer']}")
        print()

        if result['confidence_level'] in ['low', 'insufficient']:
            print("✓ Agent correctly refused to answer out-of-domain question")

    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")

    print()

    # Example 3: Technical question
    print("Example 3: Technical Question")
    print("-" * 70)

    question3 = "How does URDF describe robot kinematics?"
    print(f"Question: {question3}")
    print()

    try:
        result = chat_sync(question3)

        print(f"Response: {result['response'][:300]}...")
        print()
        print(f"Confidence: {result['confidence']:.2f}")
        print(f"Confidence Level: {result['confidence_level']}")
        print(f"Number of Sources: {len(result['sources'])}")
        print()

    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")

    print("=" * 70)
    print("Example completed!")
    print("=" * 70)


if __name__ == "__main__":
    main()
