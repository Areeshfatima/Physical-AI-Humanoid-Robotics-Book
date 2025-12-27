"""ROS 2 node for speech recognition using Whisper."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
from ..models.voice_command import VoiceCommand
from .whisper_processor import WhisperProcessor
import numpy as np
import struct
from ..utils.audio_helpers import record_audio, audio_to_wav_bytes


class SpeechRecognitionNode(Node):
    """ROS 2 node that handles speech recognition and publishes voice commands."""
    
    def __init__(self):
        super().__init__('speech_recognition_node')
        
        # Get parameters
        self.declare_parameter('whisper_model_size', 'base')
        self.declare_parameter('whisper_language', 'en')
        self.declare_parameter('audio_input_device', 0)
        self.declare_parameter('transcription_confidence_threshold', 0.85)
        
        model_size = self.get_parameter('whisper_model_size').value
        language = self.get_parameter('whisper_language').value
        self.confidence_threshold = self.get_parameter('transcription_confidence_threshold').value
        
        # Initialize Whisper processor
        self.whisper_processor = WhisperProcessor(
            model_size=model_size,
            language=language
        )
        
        # Create publisher for voice commands
        qos_profile = QoSProfile(depth=10)
        self.voice_command_publisher = self.create_publisher(
            String,  # Using String for now - in a complete implementation we'd have a custom message
            'voice_commands',
            qos_profile
        )
        
        # Create subscriber for audio data
        self.audio_subscriber = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            qos_profile
        )

        # Create subscriber for continuous audio streaming (if needed)
        self.audio_stream_subscriber = self.create_subscription(
            AudioData,
            'audio_stream',
            self.audio_stream_callback,
            qos_profile
        )
        
        # Create a timer for continuous audio recording (if needed)
        self.continuous_recording = self.create_timer(5.0, self.continuous_recording_callback)
        
        self.get_logger().info(
            f'Speech Recognition Node initialized with Whisper {model_size} model, '
            f'language {language}'
        )
    
    def audio_callback(self, msg: AudioData):
        """Callback for audio data messages."""
        try:
            # Convert audio data from message
            # AudioData.data is a list of uint8 values
            audio_bytes = bytes(msg.data)

            # Convert to numpy array based on encoding
            # For handling different input sources, we check the encoding and format
            if msg.encoding in ['PCM_16', 'int16']:
                audio_int16 = np.frombuffer(audio_bytes, dtype=np.int16)
                audio_float32 = audio_int16.astype(np.float32) / 32768.0  # Normalize to [-1, 1]
            elif msg.encoding in ['PCM_32', 'int32']:
                audio_int32 = np.frombuffer(audio_bytes, dtype=np.int32)
                audio_float32 = audio_int32.astype(np.float32) / 2147483648.0  # Normalize to [-1, 1]
            elif msg.encoding in ['PCM_U8', 'uint8']:
                audio_uint8 = np.frombuffer(audio_bytes, dtype=np.uint8)
                audio_float32 = (audio_uint8.astype(np.float32) - 128.0) / 128.0  # Normalize to [-1, 1]
            elif msg.encoding == 'PCM_F32':
                audio_float32 = np.frombuffer(audio_bytes, dtype=np.float32)
            else:
                # Default to int16 if encoding is unknown
                audio_int16 = np.frombuffer(audio_bytes, dtype=np.int16)
                audio_float32 = audio_int16.astype(np.float32) / 32768.0  # Normalize to [-1, 1]

            # Create voice command from audio
            voice_command = self.whisper_processor.create_voice_command(
                audio_data=audio_float32,
                sample_rate=msg.sample_rate
            )
            
            if voice_command and voice_command.confidence >= self.confidence_threshold:
                # Publish the voice command
                command_msg = String()
                command_msg.data = f"{voice_command.id}:{voice_command.text}"
                self.voice_command_publisher.publish(command_msg)
                
                self.get_logger().info(
                    f'Published voice command: "{voice_command.text}" '
                    f'with confidence: {voice_command.confidence:.2f}'
                )
            elif voice_command:
                self.get_logger().warn(
                    f'Voice command below confidence threshold: '
                    f'"{voice_command.text}" confidence: {voice_command.confidence:.2f} '
                    f'(threshold: {self.confidence_threshold})'
                )
            else:
                self.get_logger().info('No voice command detected in audio')
                
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {str(e)}')

    def audio_stream_callback(self, msg: AudioData):
        """Callback for continuous audio stream processing (shorter segments)."""
        try:
            # Convert audio data from message
            audio_bytes = bytes(msg.data)

            # Convert to numpy array based on encoding (same as audio_callback)
            if msg.encoding in ['PCM_16', 'int16']:
                audio_int16 = np.frombuffer(audio_bytes, dtype=np.int16)
                audio_float32 = audio_int16.astype(np.float32) / 32768.0  # Normalize to [-1, 1]
            elif msg.encoding in ['PCM_32', 'int32']:
                audio_int32 = np.frombuffer(audio_bytes, dtype=np.int32)
                audio_float32 = audio_int32.astype(np.float32) / 2147483648.0  # Normalize to [-1, 1]
            elif msg.encoding in ['PCM_U8', 'uint8']:
                audio_uint8 = np.frombuffer(audio_bytes, dtype=np.uint8)
                audio_float32 = (audio_uint8.astype(np.float32) - 128.0) / 128.0  # Normalize to [-1, 1]
            elif msg.encoding == 'PCM_F32':
                audio_float32 = np.frombuffer(audio_bytes, dtype=np.float32)
            else:
                # Default to int16 if encoding is unknown
                audio_int16 = np.frombuffer(audio_bytes, dtype=np.int16)
                audio_float32 = audio_int16.astype(np.float32) / 32768.0  # Normalize to [-1, 1]

            # Create voice command from audio
            voice_command = self.whisper_processor.create_voice_command(
                audio_data=audio_float32,
                sample_rate=msg.sample_rate
            )

            if voice_command and voice_command.confidence >= self.confidence_threshold:
                # Publish the voice command
                command_msg = String()
                command_msg.data = f"{voice_command.id}:{voice_command.text}"
                self.voice_command_publisher.publish(command_msg)

                self.get_logger().info(
                    f'Published voice command: "{voice_command.text}" '
                    f'with confidence: {voice_command.confidence:.2f}'
                )
            elif voice_command:
                self.get_logger().warn(
                    f'Voice command below confidence threshold: '
                    f'"{voice_command.text}" confidence: {voice_command.confidence:.2f} '
                    f'(threshold: {self.confidence_threshold})'
                )
            # Note: Not publishing anything if no command is detected, to avoid spam

        except Exception as e:
            self.get_logger().error(f'Error processing audio stream: {str(e)}')

    def continuous_recording_callback(self):
        """Callback for continuous audio recording."""
        try:
            # Record audio for 3 seconds
            audio_data, sample_rate = record_audio(duration=3.0)

            # Create voice command from recorded audio
            voice_command = self.whisper_processor.create_voice_command(
                audio_data=audio_data,
                sample_rate=sample_rate
            )

            if voice_command and voice_command.confidence >= self.confidence_threshold:
                # Publish the voice command
                command_msg = String()
                command_msg.data = f"{voice_command.id}:{voice_command.text}"
                self.voice_command_publisher.publish(command_msg)

                self.get_logger().info(
                    f'Published voice command: "{voice_command.text}" '
                    f'with confidence: {voice_command.confidence:.2f}'
                )
            elif voice_command:
                self.get_logger().warn(
                    f'Voice command below confidence threshold: '
                    f'"{voice_command.text}" confidence: {voice_command.confidence:.2f} '
                    f'(threshold: {self.confidence_threshold})'
                )

        except Exception as e:
            self.get_logger().error(f'Error during continuous recording: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = SpeechRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Speech Recognition Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()