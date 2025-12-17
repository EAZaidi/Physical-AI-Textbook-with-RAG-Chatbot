"""Example: Error handling for common failure scenarios.

Demonstrates:
- Handling 503 Service Unavailable (Qdrant offline)
- Handling 429 Rate Limit errors
- Handling 400 Bad Request (validation errors)
- Parsing structured error responses
- Implementing retry logic
"""

import requests
import json
import time
from typing import Optional


# Configuration
API_URL = "http://localhost:8000"
API_KEY = "your-api-key-here"  # Replace with actual API key if authentication is enabled


def chat_with_error_handling(
    message: str,
    session_id: str = "error-example",
    max_retries: int = 3,
    retry_delay: float = 2.0
) -> Optional[dict]:
    """Send a chat request with comprehensive error handling.

    Args:
        message: Question to ask
        session_id: Session ID
        max_retries: Maximum retry attempts for transient errors
        retry_delay: Initial delay between retries (exponential backoff)

    Returns:
        API response dictionary or None if all retries failed
    """
    endpoint = f"{API_URL}/chat/run"

    payload = {
        "message": message,
        "session_id": session_id
    }

    headers = {"Content-Type": "application/json"}
    if API_KEY and API_KEY != "your-api-key-here":
        headers["X-API-Key"] = API_KEY

    for attempt in range(max_retries):
        try:
            response = requests.post(
                endpoint,
                json=payload,
                headers=headers,
                timeout=30
            )

            # Check for success
            if response.status_code == 200:
                return response.json()

            # Handle specific error codes
            error_data = response.json() if response.headers.get('content-type') == 'application/json' else {}

            if response.status_code == 400:
                # Bad Request - validation error (don't retry)
                print(f"❌ Validation Error (400):")
                print(f"   Error Code: {error_data.get('error_code', 'UNKNOWN')}")
                print(f"   Message: {error_data.get('message', response.text)}")
                print(f"   Suggested Action: {error_data.get('suggested_action', 'Fix the request')}")
                return None

            elif response.status_code == 429:
                # Rate Limit - retry with backoff
                retry_after = response.headers.get('Retry-After', retry_delay * (2 ** attempt))
                print(f"⚠️  Rate Limit Exceeded (429):")
                print(f"   Message: {error_data.get('message', 'Too many requests')}")
                print(f"   Retry After: {retry_after} seconds")

                if attempt < max_retries - 1:
                    print(f"   Retrying in {retry_after} seconds... (attempt {attempt + 1}/{max_retries})")
                    time.sleep(float(retry_after))
                    continue
                else:
                    print(f"   Max retries reached. Giving up.")
                    return None

            elif response.status_code == 503:
                # Service Unavailable - retry with backoff
                wait_time = retry_delay * (2 ** attempt)
                print(f"⚠️  Service Unavailable (503):")
                print(f"   Error Code: {error_data.get('error_code', 'SERVICE_UNAVAILABLE')}")
                print(f"   Message: {error_data.get('message', 'Service temporarily unavailable')}")
                print(f"   Suggested Action: {error_data.get('suggested_action', 'Retry later')}")

                if attempt < max_retries - 1:
                    print(f"   Retrying in {wait_time:.1f} seconds... (attempt {attempt + 1}/{max_retries})")
                    time.sleep(wait_time)
                    continue
                else:
                    print(f"   Max retries reached. Service may be down.")
                    return None

            else:
                # Other errors
                print(f"❌ HTTP Error {response.status_code}:")
                print(f"   {error_data.get('message', response.text[:200])}")
                return None

        except requests.exceptions.Timeout:
            print(f"⚠️  Request Timeout:")
            wait_time = retry_delay * (2 ** attempt)

            if attempt < max_retries - 1:
                print(f"   Retrying in {wait_time:.1f} seconds... (attempt {attempt + 1}/{max_retries})")
                time.sleep(wait_time)
                continue
            else:
                print(f"   Max retries reached.")
                return None

        except requests.exceptions.ConnectionError as e:
            print(f"❌ Connection Error:")
            print(f"   Cannot connect to {API_URL}")
            print(f"   Details: {str(e)[:100]}")
            return None

        except requests.exceptions.RequestException as e:
            print(f"❌ Request Error:")
            print(f"   {str(e)[:200]}")
            return None

    return None


def main():
    """Run error handling examples."""

    print("=" * 70)
    print("RAG Agent API - Error Handling Examples")
    print("=" * 70)
    print()

    # Example 1: Valid request (should succeed)
    print("Example 1: Valid Request")
    print("-" * 70)

    result = chat_with_error_handling("What is a ROS 2 node?")

    if result:
        print(f"✅ Success!")
        print(f"   Response: {result['response'][:100]}...")
        print(f"   Confidence: {result['confidence']:.2f}")
    else:
        print(f"❌ Request failed")

    print()

    # Example 2: Invalid request - message too long
    print("Example 2: Validation Error (Message Too Long)")
    print("-" * 70)

    long_message = "x" * 1001  # Exceeds MAX_QUERY_LENGTH=1000
    result = chat_with_error_handling(long_message)

    if not result:
        print(f"✓ Validation error handled correctly")

    print()

    # Example 3: Invalid request - empty message
    print("Example 3: Validation Error (Empty Message)")
    print("-" * 70)

    result = chat_with_error_handling("")

    if not result:
        print(f"✓ Validation error handled correctly")

    print()

    # Example 4: Simulate rate limiting (if you make many requests)
    print("Example 4: Rate Limiting Simulation")
    print("-" * 70)
    print("Note: Send 100+ requests rapidly to trigger rate limiting")
    print("This example demonstrates the retry logic for 429 errors")
    print()

    # Example 5: Service unavailable handling
    print("Example 5: Service Unavailable Handling")
    print("-" * 70)
    print("Note: Stop Qdrant (docker-compose down) to test 503 errors")
    print("The client will retry with exponential backoff")
    print()

    # Health check example
    print("Example 6: Health Check")
    print("-" * 70)

    try:
        response = requests.get(f"{API_URL}/health", timeout=5)

        if response.status_code == 200:
            health = response.json()
            print("✅ API is healthy")
            print(f"   Status: {health.get('status')}")

            if 'dependencies' in health:
                print(f"   Dependencies:")
                for service, info in health['dependencies'].items():
                    status_icon = "✅" if info.get('status') == 'up' else "❌"
                    print(f"      {status_icon} {service}: {info.get('status')} ({info.get('latency_ms')}ms)")
        else:
            print(f"⚠️  API health check returned {response.status_code}")

    except requests.exceptions.RequestException as e:
        print(f"❌ Cannot reach API health endpoint")
        print(f"   {str(e)[:100]}")

    print()
    print("=" * 70)
    print("Error Handling Examples Completed")
    print("=" * 70)
    print()
    print("Key Takeaways:")
    print("- 400 errors: Fix validation issues (don't retry)")
    print("- 429 errors: Respect rate limits, use exponential backoff")
    print("- 503 errors: Service unavailable, retry with backoff")
    print("- Timeouts: Retry with exponential backoff")
    print("- Always check health endpoint before bulk operations")


if __name__ == "__main__":
    main()
