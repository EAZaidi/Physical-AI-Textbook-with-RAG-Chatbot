#!/usr/bin/env python3
"""
Whisper Transcription Node for ROS 2

Captures audio from microphone, uses Voice Activity Detection (VAD),
and transcribes speech to text using OpenAI Whisper (faster-whisper).
Publishes transcribed commands to /voice/command topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel
import queue
import threading
from collections import deque


class WhisperTranscriptionNode(Node):
    def __init__(self):
        super().__init__('whisper_transcription_node')

        # Declare parameters
        self.declare_parameter('model_size', 'base')  # tiny, base, small, medium
        self.declare_parameter('device', 'cpu')  # cpu or cuda
        self.declare_parameter('compute_type', 'int8')  # int8, float16, float32
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('vad_threshold', 0.5)  # Voice activity detection threshold
        self.declare_parameter('buffer_duration', 5.0)  # seconds
        self.declare_parameter('wake_word', 'robot')  # Optional wake word

        # Get parameters
        model_size = self.get_parameter('model_size').value
        device = self.get_parameter('device').value
        compute_type = self.get_parameter('compute_type').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.vad_threshold = self.get_parameter('vad_threshold').value
        self.buffer_duration = self.get_parameter('buffer_duration').value
        self.wake_word = self.get_parameter('wake_word').value.lower()

        # Load Whisper model
        self.get_logger().info(f'Loading Whisper model: {model_size} on {device}')
        self.model = WhisperModel(
            model_size,
            device=device,
            compute_type=compute_type
        )
        self.get_logger().info('Whisper model loaded successfully')

        # Audio buffer (circular buffer for last N seconds)
        buffer_samples = int(self.sample_rate * self.buffer_duration)
        self.audio_buffer = deque(maxlen=buffer_samples)

        # Audio queue for processing
        self.audio_queue = queue.Queue()

        # Publisher
        self.command_publisher = self.create_publisher(
            String,
            '/voice/command',
            10
        )

        # State tracking
        self.is_recording = False
        self.silence_duration = 0.0
        self.speech_detected = False

        # Start audio capture thread
        self.audio_thread = threading.Thread(target=self.audio_capture_loop, daemon=True)
        self.audio_thread.start()

        # Start processing thread
        self.processing_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.processing_thread.start()

        self.get_logger().info('Whisper transcription node initialized. Listening...')

    def audio_capture_loop(self):
        """Continuously capture audio from microphone"""
        def callback(indata, frames, time_info, status):
            if status:
                self.get_logger().warn(f'Audio callback status: {status}')

            # Convert to mono if needed
            audio = indata[:, 0] if self.channels == 1 else indata

            # Add to circular buffer
            self.audio_buffer.extend(audio)

            # Simple VAD: check RMS energy
            rms = np.sqrt(np.mean(audio**2))

            if rms > self.vad_threshold:
                if not self.speech_detected:
                    self.get_logger().info('Speech detected')
                    self.speech_detected = True
                self.silence_duration = 0.0
            else:
                if self.speech_detected:
                    self.silence_duration += frames / self.sample_rate

                    # End of speech: 1 second of silence
                    if self.silence_duration > 1.0:
                        self.get_logger().info('End of speech detected')
                        # Queue audio for transcription
                        audio_data = np.array(self.audio_buffer)
                        self.audio_queue.put(audio_data)
                        self.speech_detected = False
                        self.silence_duration = 0.0

        # Start audio stream
        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=self.channels,
            callback=callback,
            blocksize=1024
        ):
            # Keep stream open
            while rclpy.ok():
                sd.sleep(100)

    def processing_loop(self):
        """Process audio from queue and transcribe"""
        while rclpy.ok():
            try:
                # Get audio from queue (blocking with timeout)
                audio_data = self.audio_queue.get(timeout=1.0)

                # Transcribe
                self.get_logger().info('Transcribing audio...')
                transcription = self.transcribe_audio(audio_data)

                if transcription:
                    self.get_logger().info(f'Transcription: "{transcription}"')

                    # Check for wake word (optional)
                    if self.wake_word and self.wake_word not in transcription.lower():
                        self.get_logger().info(f'Wake word "{self.wake_word}" not detected, ignoring')
                        continue

                    # Remove wake word from command
                    if self.wake_word:
                        transcription = transcription.lower().replace(self.wake_word, '').strip()

                    # Publish command
                    msg = String()
                    msg.data = transcription
                    self.command_publisher.publish(msg)
                    self.get_logger().info(f'Published command: "{transcription}"')

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Processing error: {str(e)}')

    def transcribe_audio(self, audio_data):
        """Transcribe audio using Whisper"""
        try:
            # Normalize audio to [-1, 1]
            audio_data = audio_data.astype(np.float32)
            if np.abs(audio_data).max() > 0:
                audio_data = audio_data / np.abs(audio_data).max()

            # Transcribe
            segments, info = self.model.transcribe(
                audio_data,
                language='en',
                vad_filter=True,
                vad_parameters=dict(
                    threshold=0.5,
                    min_speech_duration_ms=250
                )
            )

            # Concatenate all segments
            transcription = ' '.join([segment.text for segment in segments]).strip()

            return transcription

        except Exception as e:
            self.get_logger().error(f'Transcription error: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)

    try:
        node = WhisperTranscriptionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
