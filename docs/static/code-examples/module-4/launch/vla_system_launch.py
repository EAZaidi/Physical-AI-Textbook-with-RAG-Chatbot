"""
VLA System Launch File

Launches all components:
- Whisper Transcription Node
- LLM Planner Node
- Visual Grounding Node
- Action Executor
- Capstone Integration

Usage:
  ros2 launch vla_system vla_system_launch.py
  ros2 launch vla_system vla_system_launch.py llm_provider:=anthropic model_name:=claude-3-5-sonnet-20241022
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    llm_provider_arg = DeclareLaunchArgument(
        'llm_provider',
        default_value='openai',
        description='LLM provider: openai, anthropic, or local'
    )

    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='gpt-4o',
        description='Model name (gpt-4o, claude-3-5-sonnet-20241022, llama-3-70b)'
    )

    whisper_model_arg = DeclareLaunchArgument(
        'whisper_model',
        default_value='base',
        description='Whisper model size: tiny, base, small, medium'
    )

    whisper_device_arg = DeclareLaunchArgument(
        'whisper_device',
        default_value='cpu',
        description='Device for Whisper: cpu or cuda'
    )

    grounding_device_arg = DeclareLaunchArgument(
        'grounding_device',
        default_value='cuda',
        description='Device for Grounding DINO: cpu or cuda'
    )

    # Whisper Transcription Node
    whisper_node = Node(
        package='vla_system',
        executable='whisper_transcription_node',
        name='whisper_node',
        parameters=[{
            'model_size': LaunchConfiguration('whisper_model'),
            'device': LaunchConfiguration('whisper_device'),
            'compute_type': 'int8',
            'sample_rate': 16000,
            'vad_threshold': 0.5,
            'buffer_duration': 5.0,
            'wake_word': 'robot'
        }],
        output='screen'
    )

    # LLM Planner Node
    llm_planner_node = Node(
        package='vla_system',
        executable='llm_planner_node',
        name='llm_planner',
        parameters=[{
            'llm_provider': LaunchConfiguration('llm_provider'),
            'model_name': LaunchConfiguration('model_name'),
            'temperature': 0.1,
            'max_tokens': 1000,
            'timeout': 10.0,
            'prompt_template_path': 'prompts/system_prompt.txt'
        }],
        output='screen'
    )

    # Visual Grounding Node
    visual_grounding_node = Node(
        package='vla_system',
        executable='visual_grounding_node',
        name='visual_grounding',
        parameters=[{
            'box_threshold': 0.35,
            'text_threshold': 0.25,
            'device': LaunchConfiguration('grounding_device'),
            'model_config': 'GroundingDINO_SwinT_OGC.py',
            'model_checkpoint': 'groundingdino_swint_ogc.pth'
        }],
        output='screen'
    )

    # Action Executor Node
    action_executor_node = Node(
        package='vla_system',
        executable='action_executor',
        name='action_executor',
        parameters=[{
            'navigation_timeout': 30.0,
            'manipulation_timeout': 15.0,
            'detection_timeout': 5.0
        }],
        output='screen'
    )

    # Capstone Integration Node
    capstone_integration_node = Node(
        package='vla_system',
        executable='capstone_integration',
        name='vla_system',
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        llm_provider_arg,
        model_name_arg,
        whisper_model_arg,
        whisper_device_arg,
        grounding_device_arg,

        # Nodes
        whisper_node,
        llm_planner_node,
        visual_grounding_node,
        action_executor_node,
        capstone_integration_node
    ])
