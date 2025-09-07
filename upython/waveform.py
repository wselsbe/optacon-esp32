import math

class WaveformGenerator:
    """Pre-calculated waveform generator with circular buffer"""
    
    def __init__(self, sample_rate=8000, target_buffer_size=100):
        self.sample_rate = sample_rate
        self.target_buffer_size = target_buffer_size
        self.waveform_data = None
        self.period_samples = 0
        self.index = 0
    
    def generate_sine(self, frequency, gain=1.0):
        """Generate pre-calculated sine wave data
        
        Args:
            frequency: Frequency in Hz
            gain: Amplitude scaling factor (0.1 to 1.0, default 1.0)
        
        Returns:
            memoryview of the waveform data
        """
        # Clamp gain to valid range
        gain = max(0.1, min(1.0, gain))
        
        # Calculate number of samples in one period
        self.period_samples = int(self.sample_rate / frequency)
        
        # Total buffer size: one period + target buffer + margin
        total_size = self.period_samples + self.target_buffer_size + 10
        
        # Calculate amplitude 
        # Use 128 to ensure we can reach -128 at negative peaks
        amplitude = 128 * gain
        
        # Pre-allocate bytearray
        data = bytearray(total_size)
        
        # Generate one complete period
        for i in range(self.period_samples):
            # Calculate phase (0 to 2π)
            phase = 2 * math.pi * i / self.period_samples
            
            # Generate sine value (-1 to 1)
            sine_value = math.sin(phase)
            
            # Convert to signed 8-bit two's complement with gain scaling
            # 0 is midpoint, amplitude is scaled by gain
            # Use round to get better accuracy
            sample = round(amplitude * sine_value)
            
            # Clamp to valid signed range (-128 to +127)
            # Allow -128 for negative peaks
            if sample < 0:
                sample = max(-128, sample)
            else:
                sample = min(127, sample)
            
            # Convert to unsigned byte representation
            data[i] = sample & 0xFF
        
        # Copy the beginning of the waveform to fill the extra space
        # This ensures seamless looping
        for i in range(self.period_samples, total_size):
            data[i] = data[i % self.period_samples]
        
        # Store as memoryview for performance
        self.waveform_data = memoryview(data)
        
        # Reset index
        self.index = 0
        
        return self.waveform_data
    
    def get_next_chunk(self):
        """Get next chunk of waveform data
        
        Returns:
            memoryview slice of the next target_buffer_size bytes
        """
        if self.waveform_data is None:
            raise ValueError("No waveform generated yet")
        
        # Return slice without updating index
        return self.waveform_data[self.index:self.index + self.target_buffer_size]
    
    def update_index(self, bytes_written):
        """Update index based on actual bytes written
        
        Args:
            bytes_written: Number of bytes actually written
        """
        self.index = (self.index + bytes_written) % self.period_samples
    
    def reset(self):
        """Reset playback to beginning of waveform"""
        self.index = 0
    
    def get_info(self):
        """Get information about current waveform"""
        if self.waveform_data is None:
            return "No waveform generated"
        
        return {
            'buffer_size': len(self.waveform_data),
            'period_samples': self.period_samples,
            'current_index': self.index,
            'target_buffer_size': self.target_buffer_size
        }