"""Example: Multi-turn conversation with session management.

Demonstrates:
- Maintaining conversation context across multiple requests
- Using the same session_id for follow-up questions
- Retrieving conversation history
- Deleting sessions
"""

import requests
import json
from uuid import uuid4


# Configuration
API_URL = "http://localhost:8000"
API_KEY = "your-api-key-here"  # Replace with actual API key if authentication is enabled


def chat(message: str, session_id: str):
    """Send a chat message within a session.

    Args:
        message: Question to ask
        session_id: Session ID for conversation continuity

    Returns:
        API response dictionary
    """
    endpoint = f"{API_URL}/chat/run"

    payload = {
        "message": message,
        "session_id": session_id
    }

    headers = {"Content-Type": "application/json"}
    if API_KEY and API_KEY != "your-api-key-here":
        headers["X-API-Key"] = API_KEY

    response = requests.post(endpoint, json=payload, headers=headers)
    response.raise_for_status()

    return response.json()


def get_history(session_id: str):
    """Retrieve conversation history for a session.

    Args:
        session_id: Session ID to retrieve

    Returns:
        Session history dictionary
    """
    endpoint = f"{API_URL}/session/{session_id}/history"

    headers = {}
    if API_KEY and API_KEY != "your-api-key-here":
        headers["X-API-Key"] = API_KEY

    response = requests.get(endpoint, headers=headers)
    response.raise_for_status()

    return response.json()


def delete_session(session_id: str):
    """Delete a conversation session.

    Args:
        session_id: Session ID to delete
    """
    endpoint = f"{API_URL}/session/{session_id}"

    headers = {}
    if API_KEY and API_KEY != "your-api-key-here":
        headers["X-API-Key"] = API_KEY

    response = requests.delete(endpoint, headers=headers)
    response.raise_for_status()


def main():
    """Run example multi-turn conversation."""

    print("=" * 70)
    print("RAG Agent API - Multi-Turn Conversation Example")
    print("=" * 70)
    print()

    # Create a new session
    session_id = str(uuid4())
    print(f"Session ID: {session_id}")
    print()

    # Turn 1: Ask about ROS 2 nodes
    print("Turn 1: Initial Question")
    print("-" * 70)

    question1 = "What is a ROS 2 node?"
    print(f"User: {question1}")
    print()

    try:
        response1 = chat(question1, session_id)
        print(f"Agent: {response1['response'][:200]}...")
        print()
        print(f"Confidence: {response1['confidence']:.2f} ({response1['confidence_level']})")
        print(f"Sources: {len(response1['sources'])}")
        print()

    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")
        return

    # Turn 2: Follow-up question (uses context from Turn 1)
    print("Turn 2: Follow-Up Question")
    print("-" * 70)

    question2 = "Can you give me an example?"  # Refers to "ROS 2 node" from context
    print(f"User: {question2}")
    print()

    try:
        response2 = chat(question2, session_id)
        print(f"Agent: {response2['response'][:200]}...")
        print()
        print(f"Confidence: {response2['confidence']:.2f} ({response2['confidence_level']})")
        print()

        if "node" in response2['response'].lower() or "ros" in response2['response'].lower():
            print("✓ Agent maintained conversation context!")
        else:
            print("⚠️  Context may not have been maintained")

    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")
        return

    # Turn 3: Another follow-up
    print("Turn 3: Another Follow-Up")
    print("-" * 70)

    question3 = "How do they communicate with each other?"
    print(f"User: {question3}")
    print()

    try:
        response3 = chat(question3, session_id)
        print(f"Agent: {response3['response'][:200]}...")
        print()
        print(f"Confidence: {response3['confidence']:.2f} ({response3['confidence_level']})")
        print()

    except requests.exceptions.RequestException as e:
        print(f"Error: {e}")
        return

    # Retrieve full conversation history
    print("Retrieving Conversation History")
    print("-" * 70)

    try:
        history = get_history(session_id)

        print(f"Thread ID: {history['thread_id']}")
        print(f"Created: {history['created_at']}")
        print(f"Updated: {history['updated_at']}")
        print(f"Topic Summary: {history.get('topic_summary', 'Not yet generated')}")
        print()
        print(f"Total Messages: {len(history['messages'])}")
        print()

        # Display message history
        for i, msg in enumerate(history['messages'], 1):
            role_emoji = "👤" if msg['role'] == 'user' else "🤖"
            preview = msg['content'][:80] + "..." if len(msg['content']) > 80 else msg['content']
            print(f"{i}. {role_emoji} {msg['role'].title()}: {preview}")

        print()

    except requests.exceptions.RequestException as e:
        print(f"Error retrieving history: {e}")

    # Delete the session
    print("Cleaning Up Session")
    print("-" * 70)

    try:
        delete_session(session_id)
        print(f"✓ Session {session_id} deleted successfully")
        print()

        # Verify deletion
        try:
            get_history(session_id)
            print("⚠️  Session still exists (unexpected)")
        except requests.exceptions.HTTPError as e:
            if e.response.status_code == 404:
                print("✓ Session confirmed deleted (404 error as expected)")

    except requests.exceptions.RequestException as e:
        print(f"Error deleting session: {e}")

    print()
    print("=" * 70)
    print("Example completed!")
    print("=" * 70)


if __name__ == "__main__":
    main()
