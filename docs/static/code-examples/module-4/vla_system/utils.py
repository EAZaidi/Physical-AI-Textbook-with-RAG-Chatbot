"""Shared utilities for VLA system.

Provides common functions used across multiple nodes:
- Audio processing helpers
- JSON parsing and validation
- Coordinate transformations
- Logging utilities
"""

import json
import numpy as np
from typing import Dict, List, Any, Optional
import logging


def setup_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    """Create a configured logger for VLA nodes.

    Args:
        name: Logger name (typically module name)
        level: Logging level (default: INFO)

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)

    handler = logging.StreamHandler()
    handler.setLevel(level)

    formatter = logging.Formatter(
        '[%(asctime)s] [%(name)s] [%(levelname)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    handler.setFormatter(formatter)

    if not logger.handlers:
        logger.addHandler(handler)

    return logger


def parse_llm_json(text: str) -> Optional[Dict[str, Any]]:
    """Parse JSON from LLM output, handling markdown code blocks.

    LLMs often wrap JSON in markdown code blocks like:
    ```json
    {"action": "navigate_to", ...}
    ```

    This function extracts and parses the JSON content.

    Args:
        text: LLM output text

    Returns:
        Parsed JSON dict, or None if parsing fails
    """
    # Remove markdown code blocks
    if "```json" in text:
        text = text.split("```json")[1].split("```")[0]
    elif "```" in text:
        text = text.split("```")[1].split("```")[0]

    # Try to parse JSON
    try:
        return json.loads(text.strip())
    except json.JSONDecodeError as e:
        logging.error(f"Failed to parse LLM JSON: {e}")
        return None


def validate_action_schema(action: Dict[str, Any]) -> bool:
    """Validate that an action follows the expected schema.

    Expected schema:
    {
        "action": str,  # Action name
        "params": dict,  # Action parameters
        "reasoning": str  # Why this action was chosen
    }

    Args:
        action: Action dictionary from LLM

    Returns:
        True if valid, False otherwise
    """
    required_keys = {"action", "params"}

    if not all(key in action for key in required_keys):
        return False

    if not isinstance(action["action"], str):
        return False

    if not isinstance(action["params"], dict):
        return False

    return True


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple:
    """Convert quaternion to Euler angles (roll, pitch, yaw).

    Args:
        x, y, z, w: Quaternion components

    Returns:
        (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def normalize_audio(audio: np.ndarray) -> np.ndarray:
    """Normalize audio signal to [-1, 1] range.

    Args:
        audio: Audio samples as numpy array

    Returns:
        Normalized audio
    """
    if audio.dtype == np.int16:
        audio = audio.astype(np.float32) / 32768.0
    elif audio.dtype == np.int32:
        audio = audio.astype(np.float32) / 2147483648.0

    # Clip to valid range
    audio = np.clip(audio, -1.0, 1.0)

    return audio


def apply_vad(audio: np.ndarray,
              sample_rate: int = 16000,
              frame_duration_ms: int = 30) -> List[tuple]:
    """Apply Voice Activity Detection to audio signal.

    Args:
        audio: Audio samples
        sample_rate: Sample rate in Hz
        frame_duration_ms: Frame duration in milliseconds

    Returns:
        List of (start_time, end_time) tuples for speech segments
    """
    # This is a placeholder implementation
    # In production, use webrtcvad or similar library

    # Simple energy-based VAD
    frame_size = int(sample_rate * frame_duration_ms / 1000)
    energy_threshold = 0.01

    speech_segments = []
    current_segment_start = None

    for i in range(0, len(audio) - frame_size, frame_size):
        frame = audio[i:i + frame_size]
        energy = np.sum(frame ** 2) / len(frame)

        if energy > energy_threshold:
            if current_segment_start is None:
                current_segment_start = i / sample_rate
        else:
            if current_segment_start is not None:
                speech_segments.append((
                    current_segment_start,
                    i / sample_rate
                ))
                current_segment_start = None

    # Handle segment that extends to end
    if current_segment_start is not None:
        speech_segments.append((
            current_segment_start,
            len(audio) / sample_rate
        ))

    return speech_segments
