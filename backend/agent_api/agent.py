"""Agent initialization and management.

Creates and caches agent instances with strict RAG instructions.
"""

from typing import Dict, Optional
from datetime import datetime, timedelta
import json

from openai import OpenAI

from agent_api.config import settings, load_system_prompt
from agent_api.tools import search_textbook

# Initialize OpenAI client
client = OpenAI(api_key=settings.openai_api_key)

# Agent cache: {session_id: (agent_data, last_access_time)}
agent_cache: Dict[str, tuple] = {}


def create_rag_agent(session_id: str) -> Dict:
    """Create or retrieve cached agent configuration.
    
    Args:
        session_id: Unique session identifier
        
    Returns:
        Dictionary with agent configuration
    """
    # Check cache
    if session_id in agent_cache:
        agent_data, last_access = agent_cache[session_id]
        
        # Check if expired
        if datetime.utcnow() - last_access < timedelta(hours=settings.session_expiry_hours):
            # Update access time
            agent_cache[session_id] = (agent_data, datetime.utcnow())
            return agent_data
        else:
            # Expired, remove from cache
            del agent_cache[session_id]
    
    # Create new agent configuration
    system_prompt = load_system_prompt()
    
    agent_data = {
        "session_id": session_id,
        "model": settings.openai_model,
        "system_prompt": system_prompt,
        "tools": [
            {
                "type": "function",
                "function": {
                    "name": "search_textbook",
                    "description": "Search the Physical AI & Humanoid Robotics textbook for technical information. Use this for questions about ROS 2, robotics, AI, sensors, control systems, or any technical concepts. Do NOT use for greetings or general conversation.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {
                                "type": "string",
                                "description": "The user's technical question to search for in the textbook"
                            },
                            "top_k": {
                                "type": "integer",
                                "description": "Number of chunks to retrieve (1-10)",
                                "default": 5
                            },
                            "similarity_threshold": {
                                "type": "number",
                                "description": "Minimum similarity score (0.0-1.0)",
                                "default": 0.7
                            }
                        },
                        "required": ["query"]
                    }
                }
            }
        ]
    }
    
    # Cache agent data
    agent_cache[session_id] = (agent_data, datetime.utcnow())
    
    return agent_data


async def run_agent(agent_data: Dict, user_message: str, conversation_history: list = None) -> Dict:
    """Run agent with message and conversation history.
    
    Args:
        agent_data: Agent configuration from create_rag_agent
        user_message: User's question
        conversation_history: Previous messages in conversation
        
    Returns:
        Dictionary with response and metadata
    """
    if conversation_history is None:
        conversation_history = []
    
    # Build messages
    messages = [
        {"role": "system", "content": agent_data["system_prompt"]}
    ]
    
    # Add conversation history (last 5 turns = 10 messages)
    messages.extend(conversation_history[-10:])
    
    # Add current user message
    messages.append({"role": "user", "content": user_message})
    
    # Call OpenAI API with function calling
    response = client.chat.completions.create(
        model=agent_data["model"],
        messages=messages,
        tools=agent_data["tools"],
        tool_choice="auto"
    )
    
    # Check if tool was called
    assistant_message = response.choices[0].message
    
    # Handle tool calls
    if assistant_message.tool_calls:
        # Extract tool call
        tool_call = assistant_message.tool_calls[0]
        function_name = tool_call.function.name
        function_args = json.loads(tool_call.function.arguments)
        
        # Execute search_textbook
        if function_name == "search_textbook":
            tool_result = await search_textbook(**function_args)
            
            # Add tool response to messages
            messages.append({
                "role": "assistant",
                "content": None,
                "tool_calls": [
                    {
                        "id": tool_call.id,
                        "type": "function",
                        "function": {
                            "name": function_name,
                            "arguments": tool_call.function.arguments
                        }
                    }
                ]
            })
            
            messages.append({
                "role": "tool",
                "tool_call_id": tool_call.id,
                "name": function_name,
                "content": json.dumps(tool_result)
            })
            
            # Get final response
            final_response = client.chat.completions.create(
                model=agent_data["model"],
                messages=messages
            )
            
            return {
                "response": final_response.choices[0].message.content,
                "tool_results": tool_result,
                "usage": final_response.usage.model_dump() if final_response.usage else {}
            }
    
    # No tool call (e.g., for greetings or general conversation)
    return {
        "response": assistant_message.content or "Hello! How can I help you today?",
        "tool_results": None,
        "usage": response.usage.model_dump() if response.usage else {}
    }


async def run_agent_stream(agent_data: Dict, user_message: str, conversation_history: list = None):
    """Run agent with streaming response via Server-Sent Events.

    Args:
        agent_data: Agent configuration from create_rag_agent
        user_message: User's question
        conversation_history: Previous messages in conversation

    Yields:
        Dictionary events for SSE streaming
    """
    if conversation_history is None:
        conversation_history = []

    # Build messages
    messages = [
        {"role": "system", "content": agent_data["system_prompt"]}
    ]

    # Add conversation history (last 5 turns = 10 messages)
    messages.extend(conversation_history[-10:])

    # Add current user message
    messages.append({"role": "user", "content": user_message})

    try:
        # First call - check for tool calls (non-streaming for tool detection)
        response = client.chat.completions.create(
            model=agent_data["model"],
            messages=messages,
            tools=agent_data["tools"],
            tool_choice="auto"
        )

        # Check if tool was called
        assistant_message = response.choices[0].message

        # Handle tool calls
        if assistant_message.tool_calls:
            # Extract tool call
            tool_call = assistant_message.tool_calls[0]
            function_name = tool_call.function.name
            function_args = json.loads(tool_call.function.arguments)

            # Yield tool call event
            yield {
                "event": "tool_call",
                "data": {
                    "function": function_name,
                    "arguments": function_args
                }
            }

            # Execute search_textbook
            if function_name == "search_textbook":
                tool_result = await search_textbook(**function_args)

                # Yield retrieval result event
                yield {
                    "event": "retrieval",
                    "data": {
                        "num_chunks": len(tool_result.get("chunks", [])),
                        "average_score": tool_result.get("average_score", 0.0),
                        "confidence_level": tool_result.get("confidence_metrics", {}).get("confidence_level", "unknown")
                    }
                }

                # Add tool response to messages
                messages.append({
                    "role": "assistant",
                    "content": None,
                    "tool_calls": [
                        {
                            "id": tool_call.id,
                            "type": "function",
                            "function": {
                                "name": function_name,
                                "arguments": tool_call.function.arguments
                            }
                        }
                    ]
                })

                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "name": function_name,
                    "content": json.dumps(tool_result)
                })

                # Get streaming response
                stream = client.chat.completions.create(
                    model=agent_data["model"],
                    messages=messages,
                    stream=True
                )

                # Stream response tokens
                for chunk in stream:
                    if chunk.choices[0].delta.content:
                        yield {
                            "event": "content",
                            "data": {
                                "delta": chunk.choices[0].delta.content
                            }
                        }

                # Yield completion event with metadata
                yield {
                    "event": "done",
                    "data": {
                        "tool_results": tool_result,
                        "usage": response.usage.model_dump() if response.usage else {}
                    }
                }
        else:
            # No tool call (e.g., for greetings) - stream the direct response
            if assistant_message.content:
                # Stream the response character by character for consistency
                for char in assistant_message.content:
                    yield {
                        "event": "content",
                        "data": {
                            "delta": char
                        }
                    }

                # Yield completion event
                yield {
                    "event": "done",
                    "data": {
                        "tool_results": None,
                        "usage": response.usage.model_dump() if response.usage else {}
                    }
                }
            else:
                yield {
                    "event": "error",
                    "data": {
                        "error_code": "EMPTY_RESPONSE",
                        "message": "No response generated. Please try again."
                    }
                }

    except Exception as e:
        # Yield error event
        yield {
            "event": "error",
            "data": {
                "error_code": "STREAMING_ERROR",
                "message": str(e)
            }
        }


def clear_agent_cache(session_id: Optional[str] = None):
    """Clear agent cache for a session or all sessions.

    Args:
        session_id: Session to clear, or None to clear all
    """
    if session_id:
        agent_cache.pop(session_id, None)
    else:
        agent_cache.clear()
