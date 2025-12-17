"""Configuration management for VLA system.

Loads configuration from environment variables and provides
default values for all system parameters.
"""

import os
from dataclasses import dataclass
from typing import Optional
from dotenv import load_dotenv


# Load environment variables from .env file
load_dotenv()


@dataclass
class WhisperConfig:
    """Configuration for Whisper speech-to-text."""

    model_size: str = "base"  # tiny, base, small, medium, large
    device: str = "cuda"  # cuda or cpu
    compute_type: str = "float16"  # float16, int8, int8_float16
    language: Optional[str] = None  # None for auto-detect
    sample_rate: int = 16000
    vad_enabled: bool = True
    buffer_duration: float = 5.0  # seconds
    min_speech_duration: float = 0.3  # seconds

    @classmethod
    def from_env(cls):
        """Load configuration from environment variables."""
        return cls(
            model_size=os.getenv("WHISPER_MODEL", "base"),
            device=os.getenv("WHISPER_DEVICE", "cuda"),
            compute_type=os.getenv("WHISPER_COMPUTE_TYPE", "float16"),
            language=os.getenv("WHISPER_LANGUAGE", None),
        )


@dataclass
class LLMConfig:
    """Configuration for LLM cognitive planner."""

    provider: str = "openai"  # openai, anthropic, local
    model: str = "gpt-4o"
    api_key: Optional[str] = None
    temperature: float = 0.7
    max_tokens: int = 1000
    timeout: float = 30.0  # seconds
    max_retries: int = 3

    @classmethod
    def from_env(cls):
        """Load configuration from environment variables."""
        provider = os.getenv("LLM_PROVIDER", "openai")

        if provider == "openai":
            model = os.getenv("OPENAI_MODEL", "gpt-4o")
            api_key = os.getenv("OPENAI_API_KEY")
        elif provider == "anthropic":
            model = os.getenv("ANTHROPIC_MODEL", "claude-3-5-sonnet-20241022")
            api_key = os.getenv("ANTHROPIC_API_KEY")
        else:  # local
            model = os.getenv("LOCAL_MODEL", "meta-llama/Meta-Llama-3-8B-Instruct")
            api_key = None

        return cls(
            provider=provider,
            model=model,
            api_key=api_key,
            temperature=float(os.getenv("LLM_TEMPERATURE", "0.7")),
            max_tokens=int(os.getenv("LLM_MAX_TOKENS", "1000")),
        )


@dataclass
class GroundingConfig:
    """Configuration for Grounding DINO visual grounding."""

    model_config: str = "GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
    model_checkpoint: str = "models/groundingdino_swint_ogc.pth"
    device: str = "cuda"
    box_threshold: float = 0.35
    text_threshold: float = 0.25

    @classmethod
    def from_env(cls):
        """Load configuration from environment variables."""
        return cls(
            model_config=os.getenv("GROUNDING_CONFIG", cls.model_config),
            model_checkpoint=os.getenv("GROUNDING_CHECKPOINT", cls.model_checkpoint),
            device=os.getenv("GROUNDING_DEVICE", "cuda"),
            box_threshold=float(os.getenv("GROUNDING_BOX_THRESHOLD", "0.35")),
            text_threshold=float(os.getenv("GROUNDING_TEXT_THRESHOLD", "0.25")),
        )


@dataclass
class ROSConfig:
    """Configuration for ROS 2 topics and services."""

    # Topics
    voice_command_topic: str = "/voice/command"
    camera_image_topic: str = "/camera/image_raw"
    camera_depth_topic: str = "/camera/depth"
    detected_objects_topic: str = "/detected_objects"

    # Services
    plan_task_service: str = "/plan_task"
    execute_action_service: str = "/execute_action"

    # Action servers
    navigate_action: str = "/navigate_to_pose"
    grasp_action: str = "/grasp_object"

    # Frames
    world_frame: str = "map"
    robot_frame: str = "base_link"
    camera_frame: str = "camera_link"

    @classmethod
    def from_env(cls):
        """Load configuration from environment variables."""
        return cls(
            voice_command_topic=os.getenv("VOICE_TOPIC", "/voice/command"),
            world_frame=os.getenv("WORLD_FRAME", "map"),
            robot_frame=os.getenv("ROBOT_FRAME", "base_link"),
            camera_frame=os.getenv("CAMERA_FRAME", "camera_link"),
        )


@dataclass
class VLAConfig:
    """Complete VLA system configuration."""

    whisper: WhisperConfig
    llm: LLMConfig
    grounding: GroundingConfig
    ros: ROSConfig

    @classmethod
    def from_env(cls):
        """Load all configuration from environment variables."""
        return cls(
            whisper=WhisperConfig.from_env(),
            llm=LLMConfig.from_env(),
            grounding=GroundingConfig.from_env(),
            ros=ROSConfig.from_env(),
        )


# Global configuration instance
config = VLAConfig.from_env()
