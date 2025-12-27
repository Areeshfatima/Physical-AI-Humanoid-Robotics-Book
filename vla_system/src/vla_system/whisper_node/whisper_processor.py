"""Whisper-based speech recognition processor for the VLA system."""

import os
import torch
import whisper
import numpy as np
from typing import Optional, Tuple
from datetime import datetime
from ..models.voice_command import VoiceCommand
from ..utils.audio_helpers import preprocess_audio, calculate_audio_confidence


class WhisperProcessor:
    """Processor for converting speech to text using OpenAI Whisper."""
    
    def __init__(self, model_size: str = "base", language: str = "en"):
        """
        Initialize the Whisper processor.
        
        Args:
            model_size: Size of the Whisper model ('tiny', 'base', 'small', 'medium', 'large')
            language: Language code for speech recognition (e.g., 'en', 'es', 'fr')
        """
        self.model_size = model_size
        self.language = language
        
        # Load the Whisper model
        print(f"Loading Whisper model: {model_size}")
        self.model = whisper.load_model(model_size)
        print("Whisper model loaded successfully")
        
        # Check if CUDA is available and use GPU if possible
        if torch.cuda.is_available():
            print("Using GPU for Whisper processing")
            self.model = self.model.cuda()
        else:
            print("Using CPU for Whisper processing")
    
    def transcribe_audio(self, audio_data: np.ndarray, sample_rate: int = 16000) -> Tuple[str, float]:
        """
        Transcribe audio data to text using Whisper.
        
        Args:
            audio_data: Audio data as numpy array
            sample_rate: Sample rate of the audio data
            
        Returns:
            Tuple of (transcribed_text, confidence_score)
        """
        try:
            # Preprocess audio data
            processed_audio = preprocess_audio(audio_data, sample_rate)
            
            # Calculate audio quality confidence
            quality_confidence = calculate_audio_confidence(processed_audio, sample_rate)
            
            # Convert numpy array to the format expected by Whisper
            # Make sure audio is in the right format (float32, [-1, 1] range)
            if processed_audio.dtype != np.float32:
                processed_audio = processed_audio.astype(np.float32)
            
            # Transcribe the audio
            result = self.model.transcribe(
                processed_audio, 
                language=self.language,
                without_timestamps=True
            )
            
            # Extract text and calculate confidence
            text = result.get("text", "").strip()
            
            # Calculate confidence based on audio quality and text characteristics
            # In a more sophisticated implementation, we might use Whisper's internal probabilities
            # or implement a separate confidence model

            # Additional factors for confidence calculation
            text_length_factor = min(len(text.strip()) / 5.0, 1.0)  # Prefer non-empty text
            all_caps_factor = 0.0 if text.isupper() and len(text) > 10 else 1.0  # Penalize all caps (might indicate noise)

            # Combine factors (adjust weights as needed)
            confidence = 0.6 * quality_confidence + 0.3 * text_length_factor + 0.1 * all_caps_factor
            confidence = min(confidence, 1.0)  # Ensure it's not above 1.0
            
            return text, confidence
            
        except Exception as e:
            print(f"Error during transcription: {str(e)}")
            return "", 0.0
    
    def create_voice_command(self, audio_data: np.ndarray, sample_rate: int = 16000) -> Optional[VoiceCommand]:
        """
        Process audio data and create a VoiceCommand object.
        
        Args:
            audio_data: Audio data as numpy array
            sample_rate: Sample rate of the audio data
            
        Returns:
            VoiceCommand object or None if processing failed
        """
        try:
            # Transcribe the audio
            text, confidence = self.transcribe_audio(audio_data, sample_rate)
            
            # Only create command if we have text and reasonable confidence
            if text and confidence > 0.1:  # Using 0.1 as minimum threshold
                import uuid
                command_id = str(uuid.uuid4())
                
                voice_command = VoiceCommand(
                    id=command_id,
                    text=text,
                    timestamp=datetime.now(),
                    confidence=confidence,
                    source="microphone",  # This would be configurable in a real system
                    processed=False
                )
                
                return voice_command
            else:
                print(f"Audio processing failed: text='{text}', confidence={confidence:.2f}")
                return None
                
        except Exception as e:
            print(f"Error creating voice command: {str(e)}")
            return None