import unittest
from waveform import WaveformGenerator

class TestWaveformGenerator(unittest.TestCase):
    
    def setUp(self):
        """Set up waveform generator for tests"""
        self.gen = WaveformGenerator(sample_rate=8000, target_buffer_size=100)
    
    def _to_signed(self, value):
        """Convert unsigned byte to signed two's complement"""
        return value - 256 if value > 127 else value
    
    def _find_min_max(self, waveform, period_samples):
        """Helper to find min/max values considering signed interpretation"""
        min_val = None
        max_val = None
        
        for i in range(period_samples):
            val = waveform[i]
            signed_val = self._to_signed(val)
            
            if min_val is None or signed_val < self._to_signed(min_val):
                min_val = val
            if max_val is None or signed_val > self._to_signed(max_val):
                max_val = val
        
        return min_val, max_val
    
    def test_gain_1_maximum(self):
        """Test maximum value with gain=1.0"""
        waveform = self.gen.generate_sine(frequency=100, gain=1.0)
        _, max_val = self._find_min_max(waveform, self.gen.period_samples)
        
        self.assertEqual(max_val, 0x7F, f"Maximum should be 0x7F, got 0x{max_val:02X}")
    
    def test_gain_1_minimum(self):
        """Test minimum value with gain=1.0"""
        waveform = self.gen.generate_sine(frequency=100, gain=1.0)
        min_val, _ = self._find_min_max(waveform, self.gen.period_samples)
        
        self.assertEqual(min_val, 0x80, f"Minimum should be 0x80, got 0x{min_val:02X}")
    
    def test_gain_0_5_maximum(self):
        """Test maximum value with gain=0.5"""
        waveform = self.gen.generate_sine(frequency=100, gain=0.5)
        _, max_val = self._find_min_max(waveform, self.gen.period_samples)
        max_signed = self._to_signed(max_val)
        
        # With gain=0.5, max should be around +63 (0x3F)
        self.assertAlmostEqual(max_signed, 63, delta=1, 
                              msg=f"Max with gain=0.5 should be ~+63, got {max_signed}")
    
    def test_gain_0_5_minimum(self):
        """Test minimum value with gain=0.5"""
        waveform = self.gen.generate_sine(frequency=100, gain=0.5)
        min_val, _ = self._find_min_max(waveform, self.gen.period_samples)
        min_signed = self._to_signed(min_val)
        
        # With gain=0.5, min should be around -63
        self.assertAlmostEqual(min_signed, -63, delta=1,
                              msg=f"Min with gain=0.5 should be ~-63, got {min_signed}")
    
    def test_value_at_0_degrees(self):
        """Test value at 0 degrees (sin(0) = 0)"""
        waveform = self.gen.generate_sine(frequency=100, gain=1.0)
        
        # At 0 degrees, value should be 0x00
        self.assertEqual(waveform[0], 0x00, 
                        f"Value at 0° should be 0x00, got 0x{waveform[0]:02X}")
    
    def test_value_at_90_degrees(self):
        """Test value at 90 degrees (sin(π/2) = 1)"""
        waveform = self.gen.generate_sine(frequency=100, gain=1.0)
        quarter = self.gen.period_samples // 4
        
        # At 90 degrees, value should be maximum (0x7F)
        self.assertEqual(waveform[quarter], 0x7F,
                        f"Value at 90° should be 0x7F, got 0x{waveform[quarter]:02X}")
    
    def test_value_at_180_degrees(self):
        """Test value at 180 degrees (sin(π) = 0)"""
        waveform = self.gen.generate_sine(frequency=100, gain=1.0)
        half = self.gen.period_samples // 2
        
        # At 180 degrees, value should be 0x00
        self.assertEqual(waveform[half], 0x00,
                        f"Value at 180° should be 0x00, got 0x{waveform[half]:02X}")
    
    def test_value_at_270_degrees(self):
        """Test value at 270 degrees (sin(3π/2) = -1)"""
        waveform = self.gen.generate_sine(frequency=100, gain=1.0)
        three_quarter = (3 * self.gen.period_samples) // 4
        
        # At 270 degrees, value should be minimum (0x80)
        self.assertEqual(waveform[three_quarter], 0x80,
                        f"Value at 270° should be 0x80, got 0x{waveform[three_quarter]:02X}")
    
    def test_zero_crossings(self):
        """Test that we have exactly two zero crossings per period"""
        waveform = self.gen.generate_sine(frequency=100, gain=1.0)
        zero_count = 0
        
        for i in range(self.gen.period_samples):
            if waveform[i] == 0x00:
                zero_count += 1
        
        self.assertEqual(zero_count, 2, 
                        f"Should have 2 zero crossings, got {zero_count}")

if __name__ == '__main__':
    unittest.main()