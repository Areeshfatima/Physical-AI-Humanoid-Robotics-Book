"""Shared audio processing utilities for the VLA system."""

import numpy as np
import scipy.signal as signal
import sounddevice as sd
from typing import Optional, Tuple
import io
import wave


def preprocess_audio(audio_data: np.ndarray, sample_rate: int = 16000) -> np.ndarray:
    """
    Preprocess audio data for speech recognition.
    
    Args:
        audio_data: Raw audio data as numpy array
        sample_rate: Sample rate of the audio data
        
    Returns:
        Preprocessed audio data
    """
    # Normalize audio to [-1, 1] range
    if audio_data.dtype == np.int16:
        audio_data = audio_data.astype(np.float32) / 32768.0
    elif audio_data.dtype == np.int32:
        audio_data = audio_data.astype(np.float32) / 2147483648.0
    
    # Resample if needed
    if sample_rate != 16000:
        # Calculate new length
        new_length = int(len(audio_data) * 16000 / sample_rate)
        audio_data = np.interp(
            np.linspace(0, len(audio_data), new_length),
            np.arange(len(audio_data)),
            audio_data
        )
    
    # Apply pre-emphasis filter
    audio_data = signal.lfilter([1.0, -0.97], 1, audio_data)
    
    return audio_data


def detect_silence(audio_data: np.ndarray, threshold: float = 0.01, window_size: int = 1024) -> bool:
    """
    Detect if the audio contains silence based on energy threshold.
    
    Args:
        audio_data: Audio data to analyze
        threshold: Energy threshold for silence detection
        window_size: Size of the window to analyze
        
    Returns:
        True if silence is detected, False otherwise
    """
    if len(audio_data) < window_size:
        window = audio_data
    else:
        # Use the first window for analysis
        window = audio_data[:window_size]
    
    # Calculate root mean square (RMS) energy
    rms = np.sqrt(np.mean(window ** 2))
    
    return rms < threshold


def record_audio(duration: float = 5.0, sample_rate: int = 16000) -> Tuple[np.ndarray, int]:
    """
    Record audio from the default input device.
    
    Args:
        duration: Duration to record in seconds
        sample_rate: Sample rate for recording
        
    Returns:
        Tuple of (audio_data, sample_rate)
    """
    # Record audio
    audio_data = sd.rec(
        int(duration * sample_rate),
        samplerate=sample_rate,
        channels=1,
        dtype='float32'
    )
    sd.wait()  # Wait for recording to complete
    
    return audio_data.flatten(), sample_rate


def audio_to_wav_bytes(audio_data: np.ndarray, sample_rate: int = 16000) -> bytes:
    """
    Convert audio data to WAV format bytes.
    
    Args:
        audio_data: Audio data as numpy array
        sample_rate: Sample rate of the audio
        
    Returns:
        WAV formatted bytes
    """
    # Normalize to int16 range
    if audio_data.dtype == np.float32:
        audio_data = (audio_data * 32767).astype(np.int16)
    
    # Create in-memory WAV file
    wav_buffer = io.BytesIO()
    
    with wave.open(wav_buffer, 'wb') as wav_file:
        wav_file.setnchannels(1)  # Mono
        wav_file.setsampwidth(2)  # 16-bit
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(audio_data.tobytes())
    
    return wav_buffer.getvalue()


def calculate_audio_confidence(audio_data: np.ndarray, sample_rate: int = 16000) -> float:
    """
    Calculate a confidence score for audio quality.
    
    Args:
        audio_data: Audio data to analyze
        sample_rate: Sample rate of the audio
        
    Returns:
        Confidence score between 0.0 and 1.0
    """
    # Calculate RMS energy
    rms = np.sqrt(np.mean(audio_data ** 2))
    
    # Calculate zero-crossing rate (helps detect speech vs silence)
    zero_crossings = np.sum(np.abs(np.diff(np.sign(audio_data))))
    zcr = zero_crossings / len(audio_data)
    
    # Calculate spectral centroid (indicates frequency content)
    fft = np.fft.fft(audio_data)
    magnitude = np.abs(fft[:len(fft)//2])  # Only positive frequencies
    freqs = np.arange(len(magnitude)) * sample_rate / len(audio_data)
    
    # Weighted frequency centroid
    if np.sum(magnitude) > 0:
        spectral_centroid = np.sum(freqs * magnitude) / np.sum(magnitude)
    else:
        spectral_centroid = 0
    
    # Normalize components to [0, 1] range
    # Typical ranges: RMS (0-0.1), ZCR (0-0.1), Spectral Centroid (0-4000Hz)
    normalized_rms = min(rms / 0.1, 1.0)  # Assume max RMS of 0.1 for speech
    normalized_zcr = min(zcr / 0.05, 1.0)  # Assume max ZCR of 0.05 for speech
    normalized_spectral = min(spectral_centroid / 4000.0, 1.0)  # Assume max freq of 4000Hz
    
    # Weighted combination (adjust weights as needed)
    confidence = 0.4 * normalized_rms + 0.3 * normalized_zcr + 0.3 * normalized_spectral
    
    return min(confidence, 1.0)  # Ensure it's not above 1.0