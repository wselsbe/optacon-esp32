# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Commands

Build the firmware:
```bash
idf.py build
```

Flash to ESP32-S3:
```bash
idf.py -p PORT flash
```

Monitor serial output:
```bash
idf.py -p PORT monitor
```

Combined build, flash and monitor:
```bash
idf.py -p PORT flash monitor
```

Clean build:
```bash
idf.py fullclean
```

Configure project settings:
```bash
idf.py menuconfig
```

## Architecture Overview

This is an ESP32-S3 firmware project for an Optacon motherboard that controls haptic feedback through piezo actuators. The system architecture consists of:

### Core Components

1. **DRV2665 Haptic Driver** (`drv2665.c/h`)
   - Controls TI DRV2665 piezo haptic driver via I2C
   - Manages FIFO buffer for waveform playback
   - Supports both analog and digital input modes
   - I2C address: 0x59, 400kHz clock

2. **Main Application** (`motherboard_main.c`)
   - Coordinates all subsystems
   - Implements FreeRTOS tasks for concurrent operation
   - Generates sine wave patterns for haptic feedback
   - Controls 20-channel shift register via SPI

3. **Peripheral Configuration**
   - I2C: SDA=GPIO47, SCL=GPIO21 (for DRV2665)
   - SPI: MOSI=GPIO6, MISO=GPIO7, SCK=GPIO9, CS=GPIO10 (for shift registers)
   - I2S PDM: CLK=GPIO3, DATA=GPIO5 (for audio output)

### Key Design Patterns

- **FIFO Management**: The DRV2665 has a 100-byte FIFO that must be kept filled. The `writer_task` monitors FIFO status and refills when space is available.
- **Waveform Generation**: Sine waves are generated at runtime based on frequency parameters and written to the haptic driver's FIFO.
- **Task Architecture**: Uses FreeRTOS tasks for concurrent operation of haptic control, shift register updates, and potential audio output.

### Important Implementation Details

- The project is configured for ESP32-S3 with 2MB flash
- ESP-IDF v5.1.1 is the target SDK version
- The shift register chain controls 20 output pins for device multiplexing
- Haptic feedback runs at configurable frequencies (default testing at 50Hz, 100Hz, 200Hz)