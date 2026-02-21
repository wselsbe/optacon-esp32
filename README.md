# Optacon Firmware

MicroPython firmware for ESP32-S3 driving piezo actuators via DRV2665 + HV509 shift registers.

## Hardware Requirements

- **ESP32-S3-WROOM-1** — main microcontroller module
- **DRV2665** — TI piezo haptic driver, controlled over I2C at address 0x59
- **2x HV509** — high-voltage shift registers for actuator multiplexing, controlled over SPI
- **20 piezo actuators** — connected to HV509 outputs

## Pin Assignments

| Function        | GPIO |
|-----------------|------|
| I2C SDA         | 47   |
| I2C SCL         | 21   |
| SPI MOSI        | 6    |
| SPI MISO        | 7    |
| SPI SCK         | 9    |
| SPI CS          | 10   |
| Polarity toggle | 34   |

## Quick Start

Build the Docker image (first time only):

```bash
docker compose build
```

Build the firmware:

```bash
docker compose run --rm dev bash /workspace/scripts/build.sh
```

Flash to the device:

```bash
docker compose run --rm dev bash /workspace/scripts/flash.sh /dev/ttyUSB0
```

## API Reference

The `pz_actuator` MicroPython module exposes the following functions:

---

### `pz_actuator.init()`

Initializes the DRV2665 haptic driver over I2C, initializes the HV509 shift registers over SPI, and configures default settings (250 Hz, 100 Vpp gain). Must be called before any other function.

Raises `OSError` if hardware initialization fails.

---

### `pz_actuator.start()`

Starts the background FreeRTOS task that continuously streams the waveform to the DRV2665 FIFO. Must call `init()` first.

Raises `RuntimeError` if not initialized. Raises `OSError` if the task cannot be created.

---

### `pz_actuator.stop()`

Stops the background waveform task. Safe to call even if the task is not running.

---

### `pz_actuator.set_frequency(hz)`

Sets the waveform frequency. Range: 50–4000 Hz. Default: 250 Hz.

If the task is running, the change is applied asynchronously on the next waveform cycle. If the task is stopped, the change is applied immediately.

Raises `ValueError` if `hz` is outside the valid range.

---

### `pz_actuator.get_frequency()`

Returns the current waveform frequency as an `int` (Hz).

---

### `pz_actuator.set_pin(pin, value, flush=True)`

Sets a single actuator channel. `pin` is 0–19. `value` is a bool (True = on, False = off).

If `flush` is True (default), the change is committed to the shift registers immediately. Pass `flush=False` to batch multiple `set_pin` calls and commit them together with `flush()`.

Raises `ValueError` if `pin` is out of range.

---

### `pz_actuator.get_pin(pin)`

Returns the current logical state of a single actuator channel as a `bool`. `pin` is 0–19.

Raises `ValueError` if `pin` is out of range.

---

### `pz_actuator.set_pins(list, flush=True)`

Sets all 20 actuator channels at once. `list` must be a list or tuple of exactly 20 `bool` values, where index 0 corresponds to channel 0.

If `flush` is True (default), all changes are committed to the shift registers immediately.

Raises `ValueError` if the list length is not 20.

---

### `pz_actuator.get_all()`

Returns a tuple of 20 `bool` values representing the current logical state of all actuator channels.

---

### `pz_actuator.set_all(value, flush=True)`

Sets all 20 actuator channels to the same state. `value` is a bool.

If `flush` is True (default), the change is committed immediately.

---

### `pz_actuator.flush()`

Commits any pending shift register state changes to the hardware. Use this after one or more `set_pin` or `set_pins` calls made with `flush=False`.

---

### `pz_actuator.toggle_polarity()`

Requests a polarity toggle on the HV509 shift registers (via GPIO 34). This reverses the high-voltage drive polarity across all actuators and is applied asynchronously by the background task.

---

### `pz_actuator.get_polarity()`

Returns the current polarity state as a `bool` (False = normal, True = inverted).

---

### `pz_actuator.set_gain(gain)`

Sets the DRV2665 output gain. Valid values are 25, 50, 75, or 100 (Vpp). Default: 100.

Raises `ValueError` if `gain` is not one of the four valid values. Raises `OSError` if the I2C write fails.

---

### `pz_actuator.is_running()`

Returns `True` if the background waveform task is currently active, `False` otherwise.

---

## Architecture

- `modules/pz_actuator/` — C user module (DRV2665 I2C, shift register SPI, FreeRTOS background task)
- `python/` — MicroPython scripts (frozen into firmware)
- `scripts/` — Build and flash helpers

The background task keeps the DRV2665's 100-byte FIFO filled by regenerating the sine waveform on demand. Frequency changes and polarity toggles are signaled to the task via atomic flags so the MicroPython thread never blocks on hardware I/O.

## Building

Build the Docker image (required once before first build):

```bash
docker compose build
```

Build the firmware inside the container:

```bash
docker compose run --rm dev bash /workspace/scripts/build.sh
```

Flash the firmware to the device:

```bash
docker compose run --rm dev bash /workspace/scripts/flash.sh /dev/ttyUSB0
```

To clean the build output before rebuilding:

```bash
docker compose run --rm dev bash -c "cd /workspace && make clean"
```

## Troubleshooting

**USB device not found during flash**

The flash script cannot open the serial port. Try the following:

- Verify the device is connected and recognized: `ls /dev/tty*`
- The port may be `/dev/ttyACM0` instead of `/dev/ttyUSB0` depending on your OS and USB adapter
- On Linux, add your user to the `dialout` group: `sudo usermod -aG dialout $USER` (requires logout/login)
- Pass the correct port explicitly: `bash /workspace/scripts/flash.sh /dev/ttyACM0`

**Build fails inside Docker**

- Ensure the Docker image is built: `docker compose build`
- If a previous build left a corrupt cache, clean it: `docker compose run --rm dev bash -c "cd /workspace && make clean"`
- Check that the `modules/` directory is correctly mounted inside the container
