# Waveform Task Redesign

## Context

The pz_actuator background task feeds waveform data to the DRV2665 FIFO and flushes the HV509 shift registers at sine wave troughs. The current implementation has several issues:

- Assumes FIFO is full after write without checking FIFO_FULL status register
- Sleep time is hardcoded (6ms) rather than adapting to actual fill level
- Waveform generation is in C (inflexible for experimenting with different shapes)
- Trough calculation uses assumed FIFO depth instead of actual bytes written

## Design

### Python-Provided Waveform Buffer

Move waveform generation to Python. The C module becomes a pure playback engine.

**New API:**
- `set_waveform(buf: bytearray)` — one period of signed int8 samples. Trough at index 0 by convention. Always repeats.
- Remove `set_frequency()` / `get_frequency()` — frequency is implicit: `freq = 8000 / len(buf)`

**Python example — 250Hz sine:**
```python
import math, array
SAMPLE_RATE = 8000
freq = 250
period = SAMPLE_RATE // freq  # 32 samples

# Phase starts at -pi/2 so index 0 is the trough (minimum)
buf = bytearray(
    int(math.sin(-math.pi/2 + 2*math.pi*i/period) * 127) & 0xFF
    for i in range(period)
)

import pz_actuator
pz_actuator.init()
pz_actuator.set_waveform(buf)
pz_actuator.start()
```

### C-Side State Changes

Replace `waveform_t` in `pz_task_state_t` with:

```c
int8_t *waveform_buf;     // pointer into Python bytearray data
size_t  waveform_len;     // period length in samples
size_t  write_index;      // circular position in waveform
```

Remove `waveform.c` / `waveform.h`. The fill_buffer logic is inlined in the task: copy from `waveform_buf[write_index..]` wrapping circularly.

### Task Loop

Single FreeRTOS task, each iteration:

```
1. FILL      Generate samples from waveform_buf, write to DRV2665 FIFO
2. VERIFY    Read FIFO_FULL status register to confirm
3. TROUGH    If SR pending: predict trough, wait or skip
4. SLEEP     Adaptive sleep, capped at half-buffer time
```

#### Step 1 — FIFO Fill

- Record `write_index_before = write_index` (needed for trough calculation)
- Copy up to 100 samples from `waveform_buf[write_index..]`, wrapping at `waveform_len`
- Write to DRV2665 FIFO via `drv2665_write_fifo()` — returns exact bytes written
- Advance `write_index` by `bytes_written` (not by 100 — only advance what was actually accepted)
- Record `fill_timestamp = esp_timer_get_time()`

#### Step 2 — FIFO Status Verification

- Read `DRV2665_REG_STATUS` via `drv2665_read_status()`
- Log `FIFO_FULL` bit (bit 0) for debugging
- Not fatal if not full — just means FIFO wasn't empty before our write

#### Step 3 — Trough-Synchronized SR Flush

If `sr.pending_commit` or `sr.pending_polarity`:

1. Compute playback head position:
   `playback_idx = (write_index_before - bytes_written + waveform_len) % waveform_len`
   (This is where the DRV2665 is currently playing from — it's `bytes_written` samples behind where we started writing.)

   Wait, more precisely: after filling, the FIFO contains `bytes_written` new samples. The first sample in the FIFO (at the playback head, assuming FIFO was empty before) is `waveform_buf[write_index_before]`. But the FIFO likely wasn't empty. We need to account for pre-existing FIFO content.

   Simplification: assume FIFO was nearly empty before fill (we wake up when half-drained). The playback head is approximately at `write_index_before`. The FIFO depth is approximately `bytes_written`.

2. Samples to trough (index 0):
   `playback_idx = write_index_before % waveform_len`
   `samples_to_trough = (waveform_len - playback_idx) % waveform_len`

3. Convert to microseconds: `wait_us = samples_to_trough * 125`

4. Half-buffer cap: `max_wait_us = (bytes_written * 125) / 2`

5. If `wait_us > max_wait_us` — skip this trough, catch next iteration

6. If `wait_us <= max_wait_us` — hybrid wait:
   - `vTaskDelay((wait_us / 1000) - 1)` for the coarse portion (yields CPU)
   - Busy-wait the final ~1ms
   - Execute `shift_register_commit()` — SPI transaction (~32us at 1MHz), CS-high latches HV509

#### Step 4 — Adaptive Sleep

- `sleep_us = (bytes_written * 125) / 2`
- Convert to FreeRTOS ticks, minimum 1 tick
- This wakes us when FIFO is roughly half-drained

### HV509 Latch Mechanism

The HV509 shift register latches data on CS rising edge. The ESP-IDF SPI driver handles CS automatically:
- CS goes low at start of `spi_device_polling_transmit()`
- 32 bits clocked in
- CS goes high at end — this is the latch pulse

No changes needed to SPI configuration. The SPI transaction at the trough moment inherently latches at the right time. The ~32us transaction duration adds negligible jitter (0.25 samples at 8kHz).

### Error Handling

- `start()` raises `RuntimeError` if `set_waveform()` not called
- If `bytes_written == 0` (FIFO was already full), skip sleep and retry next iteration
- FIFO underrun: if FIFO_EMPTY bit is set when we expected data, log warning

### Files Modified

- `pz_actuator.c` — add `set_waveform()`, remove `set_frequency()`/`get_frequency()`, update task state
- `task.c` / `task.h` — rewrite task loop, remove waveform_t from state, add waveform buffer fields
- Remove `waveform.c` / `waveform.h`
- `micropython.cmake` — remove waveform.c
