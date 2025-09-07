import machine
import time
import uasyncio as asyncio
from drv2665 import DRV2665
from shift_register import ShiftRegister
from waveform import WaveformGenerator

# Pin configuration
I2C_SCL_PIN = 21
I2C_SDA_PIN = 47
SPI_MOSI_PIN = 6
SPI_MISO_PIN = 7
SPI_SCK_PIN = 9
SPI_CS_PIN = 10

# DRV2665 parameters
SAMPLE_RATE = 8000  # 8kHz sample rate
FIFO_SIZE = 100

class HapticController:
    """Main controller for haptic feedback system"""
    
    def __init__(self):
        """Initialize hardware interfaces and devices"""
        print("Initializing Haptic Controller...")
        
        # Initialize I2C
        self.i2c = machine.I2C(0, 
                              scl=machine.Pin(I2C_SCL_PIN),
                              sda=machine.Pin(I2C_SDA_PIN),
                              freq=400000)
        print(f"I2C initialized on SCL={I2C_SCL_PIN}, SDA={I2C_SDA_PIN}")
        
        # Scan I2C bus
        devices = self.i2c.scan()
        print(f"I2C devices found: {[hex(d) for d in devices]}")
        
        # Initialize SPI
        self.spi = machine.SPI(1,
                              baudrate=1000000,
                              polarity=0,
                              phase=0,
                              sck=machine.Pin(SPI_SCK_PIN),
                              mosi=machine.Pin(SPI_MOSI_PIN),
                              miso=machine.Pin(SPI_MISO_PIN))
        
        self.cs = machine.Pin(SPI_CS_PIN, machine.Pin.OUT)
        print(f"SPI initialized on SCK={SPI_SCK_PIN}, MOSI={SPI_MOSI_PIN}, CS={SPI_CS_PIN}")
        
        # Initialize devices
        self.drv = DRV2665(self.i2c)
        print("DRV2665 initialized")
        
        self.shiftreg = ShiftRegister(self.spi, self.cs)
        print("Shift register initialized")
        
        # Initialize waveform generator
        self.wavegen = WaveformGenerator(sample_rate=SAMPLE_RATE, 
                                         target_buffer_size=FIFO_SIZE)
        
        # Task control flags
        self.writer_running = False
        self.shiftreg_running = False
        
    def setup_haptic(self, gain=DRV2665.CTRL2_GAIN_50VPP, timeout=DRV2665.CTRL2_TIMEOUT_10MS):
        """Configure DRV2665 for digital input mode
        
        Args:
            gain: Output gain setting (CTRL2_GAIN_25VPP/50VPP/75VPP/100VPP)
            timeout: Idle timeout setting (CTRL2_TIMEOUT_5MS/10MS/15MS/20MS)
        """
        print(f"Setting up DRV2665 in digital mode...")
        
        # Enable digital mode
        self.drv.enable_digital(gain=gain, timeout=timeout)
        
        # Check status
        status = self.drv.get_fifo_status()
        status_str = {
            self.drv.FIFO_OK: "OK",
            self.drv.FIFO_EMPTY: "EMPTY",
            self.drv.FIFO_FULL: "FULL"
        }.get(status, "UNKNOWN")
        
        print(f"DRV2665 setup complete. FIFO status: {status_str}")
        
    
    def test_hardware(self):
        """Test all hardware components"""
        print("\n=== Hardware Test ===")
        
        # Read DRV2665 registers
        print("\nDRV2665 register values:")
        ctrl1 = self.drv.read_register(self.drv.REG_CTRL_1)
        ctrl2 = self.drv.read_register(self.drv.REG_CTRL_2)
        status = self.drv.read_register(self.drv.REG_STATUS)
        print(f"  CTRL1: 0x{ctrl1:02X}")
        print(f"  CTRL2: 0x{ctrl2:02X}")
        print(f"  STATUS: 0x{status:02X}")
        
        print("\nHardware test complete!")
    
    async def writer_task(self, frequency):
        """Async task to keep DRV2665 FIFO filled with waveform data"""
        print(f"Starting writer task at {frequency}Hz")
        self.writer_running = True
        
        # Generate waveform
        gain = 0.5  # Use 50% gain for safety
        self.wavegen.generate_sine(frequency, gain)
        
        # Calculate delay based on buffer playback time
        # Time to play buffer = buffer_size / sample_rate
        # Sleep for half that time to ensure we refill before empty
        buffer_time_ms = (FIFO_SIZE * 1000) // SAMPLE_RATE
        sleep_time_ms = buffer_time_ms // 2
        print(f"Buffer playback time: {buffer_time_ms}ms, sleep time: {sleep_time_ms}ms")
        
        while self.writer_running:
            # Get next chunk and write
            chunk = self.wavegen.get_next_chunk()
            bytes_written = self.drv.write_fifo(chunk)
            self.wavegen.update_index(bytes_written)
            
            # Sleep for calculated time
            await asyncio.sleep_ms(sleep_time_ms)
    
    async def shiftreg_task(self):
        """Async task to cycle through shift register outputs"""
        print("Starting shift register task")
        self.shiftreg_running = True
        
        while self.shiftreg_running:
            # Enable each pin one by one
            for pin in range(20):
                if not self.shiftreg_running:
                    break
                
                # Clear all pins (but don't write yet)
                self.shiftreg.clear(write=False)
                
                # Enable current pin and write
                self.shiftreg.set_pin(pin, 1, write=True)
                
                # Wait 500ms
                await asyncio.sleep_ms(500)
    
    async def run_async(self, frequency=100, duration_sec=10):
        """Run the haptic controller with async tasks"""
        print(f"\nRunning haptic controller at {frequency}Hz for {duration_sec} seconds")
        
        # Setup haptic driver
        self.setup_haptic()
        
        # Create tasks
        writer = asyncio.create_task(self.writer_task(frequency))
        shiftreg = asyncio.create_task(self.shiftreg_task())
        
        # Run for specified duration
        await asyncio.sleep(duration_sec)
        
        # Stop tasks
        print("Stopping tasks...")
        self.writer_running = False
        self.shiftreg_running = False
        
        # Wait for tasks to complete
        await asyncio.gather(writer, shiftreg)
        
        # Clean up
        self.shiftreg.clear(write=True)
        self.drv.standby()
        print("Done!")
    
    async def test_frequencies(self, frequencies=[50, 100, 200], duration_sec=3):
        """Test different frequencies sequentially"""
        for freq in frequencies:
            print(f"\n=== Testing {freq}Hz ===")
            await self.run_async(freq, duration_sec)

# Main entry point
def main():
    """Synchronous main for basic testing"""
    controller = HapticController()
    controller.setup_haptic()
    controller.test_hardware()
    
    print("\nController ready.")

async def async_main():
    """Async main for running the full system"""
    controller = HapticController()
    
    # Run test frequencies
    await controller.test_frequencies([50, 100, 200], duration_sec=2)
    
    print("\nAsync test complete!")

if __name__ == "__main__":
    # Run async main
    asyncio.run(async_main())
    
    # Or run sync main for basic testing
    # main()