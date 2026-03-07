# Analog Frequency Sweep Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add ISR-level frequency sweep to the analog DDS waveform generator, supporting linear and logarithmic modes.

**Architecture:** Extend the existing 32 kHz GPTimer ISR in `pwm.c` to modulate `s_phase_step` each tick. Python computes per-tick delta (linear) or fixed-point ratio (log) and passes them to C via a new `pwm_set_sweep()` binding. A new `sweep_analog()` method on `PzActuator` provides the high-level API.

**Tech Stack:** C (ESP-IDF ISR), MicroPython C bindings, Python

---

### Task 1: Add sweep state variables to `pwm.c`

**Files:**
- Modify: `modules/pz_drive/pwm.c:80-85` (after fullwave state vars)

**Step 1: Add sweep state variables after the fullwave state block (after line 85)**

```c
// ─── Sweep state ────────────────────────────────────────────────────────────
static volatile bool     s_sweep_active = false;   // ISR modulates phase_step
static volatile uint32_t s_sweep_target = 0;       // end phase_step (clamp here)
static volatile bool     s_sweep_linear = true;    // true=linear, false=log
static volatile bool     s_sweep_up = true;        // true=freq increasing
static volatile int32_t  s_sweep_delta = 0;        // per-tick add (linear)
static volatile uint32_t s_sweep_ratio = 0;        // 1.31 fixed-point multiplier (log)
```

**Step 2: Clear sweep state in `pwm_stop_internal` (after line 315, with the other state clears)**

Add after `s_sample_len = 0;` (line 315):

```c
    // Clear sweep state
    s_sweep_active = false;
```

**Step 3: Commit**

```bash
git add modules/pz_drive/pwm.c
git commit -m "feat(sweep): add sweep state variables to pwm.c"
```

---

### Task 2: Add sweep logic to the ISR

**Files:**
- Modify: `modules/pz_drive/pwm.c:132-135` (DDS waveform section of ISR)

**Step 1: Add sweep modulation after `s_phase_acc += s_phase_step` (line 135), before the waveform index lookup (line 138)**

Insert between the `s_phase_acc += s_phase_step;` line and the `uint8_t index = ...` line:

```c
    // ── Sweep: modulate phase_step each tick ─────────────────────────
    if (s_sweep_active) {
        if (s_sweep_linear) {
            s_phase_step = (uint32_t)((int32_t)s_phase_step + s_sweep_delta);
        } else {
            s_phase_step = (uint32_t)(((uint64_t)s_phase_step * s_sweep_ratio) >> 31);
        }
        // Clamp at target and stop sweeping
        if (s_sweep_up ? (s_phase_step >= s_sweep_target)
                       : (s_phase_step <= s_sweep_target)) {
            s_phase_step = s_sweep_target;
            s_sweep_active = false;
        }
    }
```

**Note:** The `uint64_t` multiply + shift for log sweep takes ~5 cycles on Xtensa LX7 (hardware multiply). The branch on `s_sweep_active` adds 1 cycle when sweep is inactive. Negligible at 32 kHz (31.25 us budget per tick).

**Step 2: Commit**

```bash
git add modules/pz_drive/pwm.c
git commit -m "feat(sweep): add sweep modulation to DDS ISR"
```

---

### Task 3: Add `pzd_pwm_set_sweep` C function

**Files:**
- Modify: `modules/pz_drive/pwm.c:408` (after `pzd_pwm_set_frequency`)
- Modify: `modules/pz_drive/pz_drive.h:39` (after `pzd_pwm_is_sample_done`)

**Step 1: Add the C function to `pwm.c` after `pzd_pwm_set_frequency` (after line 408)**

```c
void pzd_pwm_set_sweep(int target_step, int increment, bool logarithmic) {
    s_sweep_target = (uint32_t)target_step;
    s_sweep_linear = !logarithmic;
    s_sweep_up = ((uint32_t)target_step > s_phase_step);

    if (logarithmic) {
        // increment is offset from 1.0 in 1.31 fixed-point
        // ratio = 0x80000000 + increment
        s_sweep_ratio = (uint32_t)((int32_t)0x80000000 + increment);
    } else {
        s_sweep_delta = (int32_t)increment;
    }

    // Activate last so ISR sees consistent state
    s_sweep_active = true;

    mp_printf(&mp_plat_print, "pz_drive: sweep %s to step=%u (inc=%d)\n",
              logarithmic ? "log" : "linear", (unsigned)target_step, increment);
}
```

**Step 2: Add declaration to `pz_drive.h` after `pzd_pwm_is_sample_done` (line 39)**

```c
void pzd_pwm_set_sweep(int target_step, int increment, bool logarithmic);
bool pzd_pwm_is_sweep_done(void);
```

**Step 3: Add `pzd_pwm_is_sweep_done` to `pwm.c` (after `pzd_pwm_is_sample_done`, line 462)**

```c
bool pzd_pwm_is_sweep_done(void) {
    return !s_sweep_active;
}
```

**Step 4: Commit**

```bash
git add modules/pz_drive/pwm.c modules/pz_drive/pz_drive.h
git commit -m "feat(sweep): add pzd_pwm_set_sweep C function"
```

---

### Task 4: Add Python bindings to `pz_drive.c`

**Files:**
- Modify: `modules/pz_drive/pz_drive.c:211-239` (after `pwm_is_sample_done`, before module table)

**Step 1: Add the `pwm_set_sweep` binding after `pwm_is_sample_done` (after line 215)**

```c
// ── pwm_set_sweep(target_step, increment, logarithmic=False) ─────────
static mp_obj_t pz_drive_pwm_set_sweep(size_t n_args, const mp_obj_t *pos_args,
                                       mp_map_t *kw_args) {
    enum { ARG_target_step, ARG_increment, ARG_logarithmic };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_target_step, MP_ARG_REQUIRED | MP_ARG_INT},
        {MP_QSTR_increment, MP_ARG_REQUIRED | MP_ARG_INT},
        {MP_QSTR_logarithmic, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    pzd_pwm_set_sweep(args[ARG_target_step].u_int, args[ARG_increment].u_int,
                       args[ARG_logarithmic].u_bool);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_drive_pwm_set_sweep_obj, 2, pz_drive_pwm_set_sweep);

// ── pwm_is_sweep_done() ─────────────────────────────────────────────
static mp_obj_t pz_drive_pwm_is_sweep_done(void) {
    return mp_obj_new_bool(pzd_pwm_is_sweep_done());
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_pwm_is_sweep_done_obj, pz_drive_pwm_is_sweep_done);
```

**Step 2: Add entries to the module globals table (inside `pz_drive_module_globals_table`, before the closing `}`)**

Add after the `pwm_is_sample_done` entry (line 238):

```c
    {MP_ROM_QSTR(MP_QSTR_pwm_set_sweep), MP_ROM_PTR(&pz_drive_pwm_set_sweep_obj)},
    {MP_ROM_QSTR(MP_QSTR_pwm_is_sweep_done), MP_ROM_PTR(&pz_drive_pwm_is_sweep_done_obj)},
```

**Step 3: Commit**

```bash
git add modules/pz_drive/pz_drive.c
git commit -m "feat(sweep): add pwm_set_sweep Python binding"
```

---

### Task 5: Add `sweep_analog` method to `pz_drive_py.py`

**Files:**
- Modify: `python/pz_drive_py.py:207-226` (after `play_wav`, before `get_status`)

**Step 1: Add the `sweep_analog` method after `play_wav` (after line 207)**

```python
    _PWM_SAMPLE_RATE = 32000

    def sweep_analog(
        self,
        start_hz,
        end_hz,
        duration_ms,
        logarithmic=False,
        waveform="sine",
        resolution=8,
        amplitude=100,
        gain=100,
    ):
        """Sweep frequency from start_hz to end_hz over duration_ms.

        After the sweep completes, output holds at end_hz until stop().

        Args:
            start_hz: starting frequency (1-500)
            end_hz: ending frequency (1-500)
            duration_ms: sweep duration in milliseconds (1-60000)
            logarithmic: if True, logarithmic sweep; if False, linear
            waveform: 'sine', 'triangle', or 'square'
            resolution: 8 or 10 bits
            amplitude: 0-100 (percentage)
            gain: 25, 50, 75, or 100 Vpp
        """
        if start_hz < 1 or start_hz > 500:
            raise ValueError("start_hz must be 1-500")
        if end_hz < 1 or end_hz > 500:
            raise ValueError("end_hz must be 1-500")
        if duration_ms < 1 or duration_ms > 60000:
            raise ValueError("duration_ms must be 1-60000")
        if start_hz == end_hz:
            raise ValueError("start_hz and end_hz must differ")

        # Configure waveform at starting frequency
        self.set_frequency_analog(
            start_hz,
            resolution=resolution,
            amplitude=amplitude,
            waveform=waveform,
        )

        # Compute sweep parameters
        total_ticks = duration_ms * (self._PWM_SAMPLE_RATE // 1000)
        step_start = (start_hz << 32) // self._PWM_SAMPLE_RATE
        step_end = (end_hz << 32) // self._PWM_SAMPLE_RATE

        if logarithmic:
            # ratio = 1 + ln(end/start) / total_ticks, in 1.31 fixed-point
            # increment = offset from 1.0 = ln(end/start) / total_ticks * 2^31
            log_ratio = math.log(end_hz / start_hz)
            increment = int(log_ratio / total_ticks * (1 << 31))
        else:
            # delta = (step_end - step_start) / total_ticks
            increment = (step_end - step_start) // total_ticks

        pz_drive.pwm_set_sweep(
            step_end, increment, logarithmic=logarithmic
        )

        # Start output
        self.start(gain=gain)
```

**Step 2: Commit**

```bash
git add python/pz_drive_py.py
git commit -m "feat(sweep): add sweep_analog method to PzActuator"
```

---

### Task 6: Build firmware

**Step 1: Build**

Run: `MSYS_NO_PATHCONV=1 cmd.exe /C "C:\Projects\Optacon\optacon-firmware\run-build.bat"`

Expected: Build succeeds with no errors.

**Step 2: Fix any compilation errors and re-build if needed**

**Step 3: Commit any fixes**

```bash
git commit -m "fix(sweep): fix build errors"
```

---

### Task 7: Flash and test on hardware

**Step 1: Enter bootloader**

Use `mcp__micropython__enter_bootloader` tool.

**Step 2: Flash firmware**

Run: `./scripts/flash.sh COM4` (check port with `python -m serial.tools.list_ports -v`)

**Step 3: Test linear sweep**

```python
from pz_drive_py import PzActuator
pa = PzActuator()
pa.sweep_analog(50, 500, 3000, waveform='square')
# Should hear/see frequency rising from 50 Hz to 500 Hz over 3 seconds
# Then holds at 500 Hz
import time; time.sleep(5)
pa.stop()
```

**Step 4: Test logarithmic sweep**

```python
pa.sweep_analog(50, 500, 3000, logarithmic=True, waveform='sine')
import time; time.sleep(5)
pa.stop()
```

**Step 5: Test down-sweep**

```python
pa.sweep_analog(500, 50, 3000, waveform='sine')
import time; time.sleep(5)
pa.stop()
```

**Step 6: Verify sweep completion flag**

```python
pa.sweep_analog(100, 200, 1000)
import time; time.sleep(0.5)
print(pz_drive.pwm_is_sweep_done())  # Should be False
time.sleep(1)
print(pz_drive.pwm_is_sweep_done())  # Should be True
pa.stop()
```

---

### Task 8: Update CLAUDE.md API docs

**Files:**
- Modify: `CLAUDE.md` (Python API section)

**Step 1: Add sweep_analog to the PzActuator API docs**

Add after the `pa.get_polarity()` line in the Python API section:

```python
pa.sweep_analog(start_hz, end_hz, duration_ms,
                logarithmic=False, waveform='sine',
                resolution=8, amplitude=100, gain=100)  # frequency sweep
```

**Step 2: Add `pwm_set_sweep` to the C Module APIs section**

Add after `pz_drive.pwm_is_running()`:

```python
pz_drive.pwm_set_sweep(target_step, increment, logarithmic=False)
pz_drive.pwm_is_sweep_done()
```

**Step 3: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: add sweep API to CLAUDE.md"
```
