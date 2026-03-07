# WAV Audio Playback — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Play .wav files through the DRV2665 via the analog PWM path, with phase-accumulator resampling to handle any sample rate.

**Architecture:** Python reads WAV file, parses header, converts to 8-bit unsigned mono, passes buffer + sample rate to C. The existing 32 kHz GPTimer ISR in `pwm.c` gets a new "sample playback" mode that walks through the buffer using a fixed-point phase accumulator, with optional looping.

**Tech Stack:** C (GPTimer ISR, LEDC PWM), MicroPython (WAV parsing, file I/O), existing RC filter hardware

---

### Task 1: Add sample playback mode to `pwm.c`

**Files:**
- Modify: `modules/pz_drive/pwm.c`
- Modify: `modules/pz_drive/pz_drive.h`

**Step 1: Add static state variables for sample playback**

In `pwm.c`, after the existing DDS state variables, add:

```c
// ─── Sample playback state ──────────────────────────────────────────────────
static const uint8_t *s_sample_buf = NULL;
static volatile uint32_t s_sample_len = 0;     // buffer length in samples
static volatile uint32_t s_sample_pos = 0;      // fixed-point position (16.16)
static volatile uint32_t s_sample_step = 0;     // fixed-point step per ISR tick
static volatile bool s_sample_loop = false;
static volatile bool s_sample_mode = false;     // true = sample playback, false = DDS
```

**Step 2: Add sample playback branch to the ISR**

In `timer_isr_callback`, at the top of the function body (before the existing DDS waveform generation), add a branch for sample mode:

```c
if (s_sample_mode) {
    uint32_t pos = s_sample_pos;
    uint32_t idx = pos >> 16;
    if (idx >= s_sample_len) {
        if (s_sample_loop) {
            pos = 0;
            idx = 0;
        } else {
            // Playback finished — stop from ISR context
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            s_running = false;
            s_sample_mode = false;
            return true; // ISR will be stopped by the check below
        }
    }

    // Linear interpolation between adjacent samples
    uint8_t s0 = s_sample_buf[idx];
    uint8_t s1 = (idx + 1 < s_sample_len) ? s_sample_buf[idx + 1] : s0;
    uint32_t frac = (pos >> 8) & 0xFF; // 8-bit fractional part
    uint32_t duty = s0 + (((int32_t)(s1 - s0) * (int32_t)frac) >> 8);

    // Scale to resolution
    if (s_resolution == 10) {
        duty = (duty << 2) | (duty >> 6);
    }

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    s_sample_pos = pos + s_sample_step;
    return true;
}
```

Note: when `s_running` becomes false in the ISR, the existing `pwm_stop_internal()` or a check in the main loop needs to actually stop the GPTimer. Since the ISR can't safely call `gptimer_stop()` from ISR context, set a flag and have the stop happen on next Python interaction. Alternatively, just let the ISR keep firing with duty=0 until `stop()` is called. The simplest approach: set duty to the midpoint (128 for 8-bit, 512 for 10-bit) which is silence for AC-coupled input, and let `stop()` clean up the timer.

Revised approach for end-of-playback: set duty to 128 (silence) and set `s_sample_mode = false`. The ISR continues to fire but does nothing expensive (it falls through to the DDS path which won't update duty since we can set `s_phase_step = 0`). The Python side can poll `pwm_is_running()` or just call `stop()`.

Actually simpler: when playback ends, set `s_sample_mode = false` and `s_running = false`, set duty to 128. The timer keeps running but the user will call `stop()` which cleans up. Or we can add a `pzd_pwm_is_playing()` check.

Let's keep it simple: when buffer ends and not looping, set duty to midpoint (silence), set `s_sample_mode = false`. `s_running` stays true (timer still fires). Python polls `pzd_pwm_is_sample_done()` or just calls `stop()`.

**Step 3: Add `pzd_pwm_play_samples()` function**

```c
void pzd_pwm_play_samples(const uint8_t *buf, size_t len, uint32_t sample_rate, bool loop) {
    // Stop any existing playback
    if (s_running) {
        pwm_stop_internal();
    }

    ensure_hw_init();

    s_sample_buf = buf;
    s_sample_len = (uint32_t)len;
    s_sample_pos = 0;
    s_sample_step = (sample_rate << 16) / PWM_SAMPLE_RATE_HZ; // 16.16 fixed-point
    s_sample_loop = loop;
    s_sample_mode = true;
    s_running = true;

    gptimer_start(s_timer);

    mp_printf(&mp_plat_print, "pz_drive: playing %u samples at %u Hz (step=%u, loop=%d)\n",
              (unsigned)len, (unsigned)sample_rate, (unsigned)s_sample_step, loop);
}
```

**Step 4: Add `pzd_pwm_is_sample_done()` function**

```c
bool pzd_pwm_is_sample_done(void) {
    return !s_sample_mode && s_sample_buf != NULL;
}
```

Returns true when the buffer was set but playback has finished (non-loop mode ended).

**Step 5: Update `pwm_stop_internal()` to clear sample state**

Add to the existing `pwm_stop_internal()`:

```c
s_sample_mode = false;
s_sample_buf = NULL;
s_sample_len = 0;
```

**Step 6: Declare in header**

In `pz_drive.h`, add:

```c
void pzd_pwm_play_samples(const uint8_t *buf, size_t len, uint32_t sample_rate, bool loop);
bool pzd_pwm_is_sample_done(void);
```

**Step 7: Commit**

```bash
git add modules/pz_drive/pwm.c modules/pz_drive/pz_drive.h
git commit -m "feat: add sample playback mode to PWM ISR with phase-accumulator resampling"
```

---

### Task 2: Add Python binding in `pz_drive.c`

**Files:**
- Modify: `modules/pz_drive/pz_drive.c`

**Step 1: Add `pwm_play_samples(buf, sample_rate, loop=False)` binding**

```c
// ── pwm_play_samples(buf, sample_rate, loop=False) ──────────────────────
static mp_obj_t pz_drive_pwm_play_samples(size_t n_args, const mp_obj_t *pos_args,
                                           mp_map_t *kw_args) {
    enum { ARG_buf, ARG_sample_rate, ARG_loop };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_buf, MP_ARG_REQUIRED | MP_ARG_OBJ},
        {MP_QSTR_sample_rate, MP_ARG_REQUIRED | MP_ARG_INT},
        {MP_QSTR_loop, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buf].u_obj, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len == 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("buffer must not be empty"));
    }
    uint32_t sample_rate = (uint32_t)args[ARG_sample_rate].u_int;
    if (sample_rate == 0 || sample_rate > 48000) {
        mp_raise_ValueError(MP_ERROR_TEXT("sample_rate must be 1-48000"));
    }

    pzd_pwm_play_samples((const uint8_t *)bufinfo.buf, bufinfo.len, sample_rate,
                          args[ARG_loop].u_bool);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_drive_pwm_play_samples_obj, 2, pz_drive_pwm_play_samples);
```

**Step 2: Add `pwm_is_sample_done()` binding**

```c
static mp_obj_t pz_drive_pwm_is_sample_done(void) {
    return mp_obj_new_bool(pzd_pwm_is_sample_done());
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_pwm_is_sample_done_obj, pz_drive_pwm_is_sample_done);
```

**Step 3: Register in module globals table**

Add to `pz_drive_module_globals_table[]`:

```c
{MP_ROM_QSTR(MP_QSTR_pwm_play_samples), MP_ROM_PTR(&pz_drive_pwm_play_samples_obj)},
{MP_ROM_QSTR(MP_QSTR_pwm_is_sample_done), MP_ROM_PTR(&pz_drive_pwm_is_sample_done_obj)},
```

**Step 4: Commit**

```bash
git add modules/pz_drive/pz_drive.c
git commit -m "feat: add pwm_play_samples and pwm_is_sample_done Python bindings"
```

---

### Task 3: Add `play_wav()` to `PzActuator`

**Files:**
- Modify: `python/pz_actuator_py.py`

**Step 1: Add `play_wav()` method**

```python
def play_wav(self, path, loop=False):
    """Play a WAV file through the analog PWM path.

    Args:
        path: filesystem path to WAV file (8/16-bit PCM, mono/stereo, any rate)
        loop: if True, loop until stop() is called
    """
    with open(path, "rb") as f:
        header = f.read(44)
        if len(header) < 44 or header[:4] != b"RIFF" or header[8:12] != b"WAVE":
            raise ValueError("not a valid WAV file")

        num_channels = int.from_bytes(header[22:24], "little")
        sample_rate = int.from_bytes(header[24:28], "little")
        bits_per_sample = int.from_bytes(header[34:36], "little")

        raw = f.read()

    # Convert to 8-bit unsigned mono
    if bits_per_sample == 16:
        if num_channels == 2:
            # Stereo 16-bit: take left channel (every other 2-byte pair)
            raw = bytearray(
                ((raw[i + 1] ^ 0x80) if i + 1 < len(raw) else 128)
                for i in range(0, len(raw), 4)
            )
        else:
            # Mono 16-bit: high byte + offset to unsigned
            raw = bytearray(
                ((raw[i + 1] ^ 0x80) if i + 1 < len(raw) else 128)
                for i in range(0, len(raw), 2)
            )
    elif bits_per_sample == 8:
        if num_channels == 2:
            # Stereo 8-bit: take every other byte
            raw = bytearray(raw[i] for i in range(0, len(raw), 2))
        else:
            raw = bytearray(raw)
    else:
        raise ValueError("unsupported bits_per_sample: " + str(bits_per_sample))

    self._mode = MODE_ANALOG
    self.drv.init_analog(self.GAINS[self._gain])
    pz_drive.pwm_play_samples(raw, sample_rate, loop=loop)
```

Note: for 16-bit signed PCM, the high byte represents the sample roughly. `^ 0x80` converts from signed to unsigned (flips the sign bit). This is a quick conversion — takes the MSB of each 16-bit sample and shifts to unsigned range.

**Step 2: Commit**

```bash
git add python/pz_actuator_py.py
git commit -m "feat: add play_wav() method for WAV file playback"
```

---

### Task 4: Build, flash, and test

**Step 1: Build firmware**

```bash
docker compose up -d dev
MSYS_NO_PATHCONV=1 docker compose exec dev bash -c "source /opt/esp/idf/export.sh > /dev/null 2>&1 && bash /workspace/scripts/build.sh"
```

**Step 2: Flash**

Enter bootloader via `board_utils.enter_bootloader()`, then:

```bash
./scripts/flash.sh COM4
```

**Step 3: Create a test WAV file**

On the host, create a short test tone WAV:

```bash
python3 -c "
import struct, math
sr = 32000; dur = 1; freq = 440
n = sr * dur
data = bytes(int(128 + 127 * math.sin(2 * math.pi * freq * i / sr)) for i in range(n))
with open('test_tone.wav', 'wb') as f:
    f.write(b'RIFF')
    f.write(struct.pack('<I', 36 + n))
    f.write(b'WAVEfmt ')
    f.write(struct.pack('<IHHIIHH', 16, 1, 1, sr, sr, 1, 8))
    f.write(b'data')
    f.write(struct.pack('<I', n))
    f.write(data)
print(f'Created test_tone.wav: {n} samples, {sr} Hz, {dur}s')
"
```

**Step 4: Upload test file**

```bash
mpremote connect COM5 mkdir :audio 2>/dev/null
mpremote connect COM5 cp test_tone.wav :audio/test_tone.wav
```

**Step 5: Test playback**

```python
from pz_actuator_py import PzActuator
pa = PzActuator()
pa.play_wav("/audio/test_tone.wav")
# Should hear 440 Hz tone for 1 second, then silence
```

**Step 6: Test with different sample rates**

Create 8 kHz and 44.1 kHz test files, upload and play, verify pitch is correct.

**Step 7: Commit if fixes needed**

```bash
git add -A
git commit -m "fix: adjustments from WAV playback testing"
```
