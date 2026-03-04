# Polyphonic DDS — THX Deep Note Implementation Plan

> **Status: ABANDONED** — Implemented and tested but sound quality through the piezo signal chain was too poor (barely audible/recognizable). All poly code was removed. The core-1 ISR pinning fix discovered during this work was retained.

**Goal:** Add 12-voice polyphonic DDS synthesis to the pz_drive C module and use it to play the THX Deep Note.

**Architecture:** New `poly_voice_t` struct array in pwm.c with per-voice phase accumulator and sweep. ISR runs at 16kHz in poly mode (vs 32kHz mono). Separate Python API (`pwm_poly_*`). THX sequence orchestrated from Python in music.py.

**Tech Stack:** C (ESP-IDF GPTimer ISR, LEDC PWM), MicroPython C bindings, Python

---

### Task 1: Add poly voice struct and state to pwm.c

**Files:**
- Modify: `modules/pz_drive/pwm.c:20-55`

**Step 1: Add poly configuration and state**

After `#define PWM_DEFAULT_RESOLUTION 8` (line 22), add:

```c
#define POLY_SAMPLE_RATE_HZ    16000
#define POLY_NUM_VOICES        12
```

After `static volatile bool s_sweep_active = false;` (around line 75, after all existing state), add:

```c
// ---- Polyphonic DDS state --------------------------------------------------

typedef struct {
    uint32_t phase_acc;
    uint32_t phase_step;
    uint8_t  amplitude;      // 0-128
    bool     sweep_active;
    uint32_t sweep_target;   // target phase_step
    int32_t  sweep_delta;    // per-tick linear change
    bool     sweep_up;
} poly_voice_t;

static volatile poly_voice_t s_poly_voices[POLY_NUM_VOICES];
static volatile bool s_poly_mode = false;
```

**Step 2: Commit**

```bash
git add modules/pz_drive/pwm.c
git commit -m "feat(pz_drive): add polyphonic voice struct and state"
```

---

### Task 2: Add poly ISR path to timer_isr_callback

**Files:**
- Modify: `modules/pz_drive/pwm.c:103-228`

**Step 1: Add poly mode branch in ISR**

At the top of `timer_isr_callback`, after the sample playback mode block (line 138, after `return false;` of sample mode), add a new block before the existing DDS code:

```c
    // ── Polyphonic DDS mode ──────────────────────────────────────────────
    if (s_poly_mode) {
        int32_t sum = 0;
        int active = 0;
        for (int i = 0; i < POLY_NUM_VOICES; i++) {
            volatile poly_voice_t *v = &s_poly_voices[i];
            if (v->amplitude == 0) continue;
            active++;
            v->phase_acc += v->phase_step;

            // Per-voice sweep
            if (v->sweep_active) {
                int32_t next = (int32_t)v->phase_step + v->sweep_delta;
                if (next < 0) next = 0;
                v->phase_step = (uint32_t)next;
                if (v->sweep_up ? (v->phase_step >= v->sweep_target)
                                : (v->phase_step <= v->sweep_target)) {
                    v->phase_step = v->sweep_target;
                    v->sweep_active = false;
                }
            }

            uint8_t index = (uint8_t)(v->phase_acc >> 24);
            int32_t raw = (int32_t)sine_lut[index] - 128;
            sum += (raw * (int32_t)v->amplitude) >> 7;
        }

        // Average and clamp
        if (active > 0) sum /= active;
        int32_t duty_s = 128 + sum;
        if (duty_s < 0) duty_s = 0;
        if (duty_s > 255) duty_s = 255;
        uint32_t duty = (uint32_t)duty_s;

        if (s_resolution == 10) {
            duty = (duty << 2) | (duty >> 6);
        }
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
        return false;
    }
```

**Step 2: Commit**

```bash
git add modules/pz_drive/pwm.c
git commit -m "feat(pz_drive): add polyphonic DDS ISR path"
```

---

### Task 3: Add poly start/stop/set_voice/sweep_voice C functions

**Files:**
- Modify: `modules/pz_drive/pwm.c` (after `pzd_pwm_is_sweep_done`, before end of file)
- Modify: `modules/pz_drive/pz_drive.h:32-42`

**Step 1: Add declarations to pz_drive.h**

After the existing PWM declarations (after `bool pzd_pwm_is_sweep_done(void);`), add:

```c
// ── pwm.c — polyphonic DDS ──────────────────────────────────────────────
void pzd_pwm_poly_start(void);
void pzd_pwm_poly_stop(void);
bool pzd_pwm_poly_is_running(void);
void pzd_pwm_poly_set_voice(int index, int hz, int amplitude);
void pzd_pwm_poly_sweep_voice(int index, int target_hz, int duration_ms);
```

**Step 2: Add implementations to pwm.c**

Add after the existing `pzd_pwm_is_sweep_done()` function, before the end of the file:

```c
// ---- Polyphonic DDS API ----------------------------------------------------

void pzd_pwm_poly_start(void) {
    // Stop any running mono/fifo mode
    pwm_stop_internal();

    esp_err_t err = ensure_hw_init();
    if (err != ESP_OK) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("hw init failed"));
    }

    // Clear all voices
    for (int i = 0; i < POLY_NUM_VOICES; i++) {
        s_poly_voices[i].phase_acc = 0;
        s_poly_voices[i].phase_step = 0;
        s_poly_voices[i].amplitude = 0;
        s_poly_voices[i].sweep_active = false;
    }

    // Reconfigure timer to 16kHz
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = 1000000 / POLY_SAMPLE_RATE_HZ,  // 62 ticks
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(s_timer, &alarm_cfg);

    s_poly_mode = true;
    s_running = true;
    gptimer_start(s_timer);
}

void pzd_pwm_poly_stop(void) {
    if (s_running && s_timer) {
        gptimer_stop(s_timer);
    }
    s_poly_mode = false;
    s_running = false;

    // Restore 32kHz alarm for mono mode
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = 1000000 / PWM_SAMPLE_RATE_HZ,  // 31 ticks
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(s_timer, &alarm_cfg);

    // Silence output
    if (s_hw_initialized) {
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
    }
}

bool pzd_pwm_poly_is_running(void) {
    return s_poly_mode && s_running;
}

void pzd_pwm_poly_set_voice(int index, int hz, int amplitude) {
    if (index < 0 || index >= POLY_NUM_VOICES) {
        mp_raise_ValueError(MP_ERROR_TEXT("voice index out of range"));
    }
    if (hz < 0 || hz > 1000) {
        mp_raise_ValueError(MP_ERROR_TEXT("hz must be 0-1000"));
    }
    if (amplitude < 0) amplitude = 0;
    if (amplitude > 128) amplitude = 128;

    volatile poly_voice_t *v = &s_poly_voices[index];
    v->phase_step = (hz == 0) ? 0 : (uint32_t)(((uint64_t)hz << 32) / POLY_SAMPLE_RATE_HZ);
    v->amplitude = (uint8_t)amplitude;
    v->sweep_active = false;
}

void pzd_pwm_poly_sweep_voice(int index, int target_hz, int duration_ms) {
    if (index < 0 || index >= POLY_NUM_VOICES) {
        mp_raise_ValueError(MP_ERROR_TEXT("voice index out of range"));
    }
    if (target_hz < 1 || target_hz > 1000) {
        mp_raise_ValueError(MP_ERROR_TEXT("target_hz must be 1-1000"));
    }
    if (duration_ms < 1 || duration_ms > 60000) {
        mp_raise_ValueError(MP_ERROR_TEXT("duration_ms must be 1-60000"));
    }

    volatile poly_voice_t *v = &s_poly_voices[index];
    uint32_t target_step = (uint32_t)(((uint64_t)target_hz << 32) / POLY_SAMPLE_RATE_HZ);
    uint32_t total_ticks = duration_ms * (POLY_SAMPLE_RATE_HZ / 1000);

    v->sweep_target = target_step;
    v->sweep_up = (target_step > v->phase_step);
    v->sweep_delta = (int32_t)(target_step - v->phase_step) / (int32_t)total_ticks;

    // Ensure non-zero delta if there's a difference
    if (v->sweep_delta == 0 && target_step != v->phase_step) {
        v->sweep_delta = v->sweep_up ? 1 : -1;
    }

    v->sweep_active = true;
}
```

**Step 3: Commit**

```bash
git add modules/pz_drive/pwm.c modules/pz_drive/pz_drive.h
git commit -m "feat(pz_drive): add poly start/stop/set_voice/sweep_voice"
```

---

### Task 4: Add Python bindings for poly API

**Files:**
- Modify: `modules/pz_drive/pz_drive.c:257-284`

**Step 1: Add binding functions**

Before the module table (before line 259), add:

```c
// ── pwm_poly_start() ───────────────────────────────────────────────────
static mp_obj_t pz_drive_pwm_poly_start(void) {
    pzd_pwm_poly_start();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_pwm_poly_start_obj, pz_drive_pwm_poly_start);

// ── pwm_poly_stop() ────────────────────────────────────────────────────
static mp_obj_t pz_drive_pwm_poly_stop(void) {
    pzd_pwm_poly_stop();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_pwm_poly_stop_obj, pz_drive_pwm_poly_stop);

// ── pwm_poly_is_running() ──────────────────────────────────────────────
static mp_obj_t pz_drive_pwm_poly_is_running(void) {
    return mp_obj_new_bool(pzd_pwm_poly_is_running());
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_drive_pwm_poly_is_running_obj, pz_drive_pwm_poly_is_running);

// ── pwm_poly_set_voice(index, hz, amplitude) ───────────────────────────
static mp_obj_t pz_drive_pwm_poly_set_voice(size_t n_args, const mp_obj_t *pos_args,
                                             mp_map_t *kw_args) {
    enum { ARG_index, ARG_hz, ARG_amplitude };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_index, MP_ARG_REQUIRED | MP_ARG_INT, {0}},
        {MP_QSTR_hz, MP_ARG_REQUIRED | MP_ARG_INT, {0}},
        {MP_QSTR_amplitude, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 100}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    pzd_pwm_poly_set_voice(args[ARG_index].u_int, args[ARG_hz].u_int,
                            args[ARG_amplitude].u_int);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_drive_pwm_poly_set_voice_obj, 2,
                                   pz_drive_pwm_poly_set_voice);

// ── pwm_poly_sweep_voice(index, target_hz, duration_ms) ────────────────
static mp_obj_t pz_drive_pwm_poly_sweep_voice(size_t n_args, const mp_obj_t *pos_args,
                                               mp_map_t *kw_args) {
    enum { ARG_index, ARG_target_hz, ARG_duration_ms };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_index, MP_ARG_REQUIRED | MP_ARG_INT, {0}},
        {MP_QSTR_target_hz, MP_ARG_REQUIRED | MP_ARG_INT, {0}},
        {MP_QSTR_duration_ms, MP_ARG_REQUIRED | MP_ARG_INT, {0}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    pzd_pwm_poly_sweep_voice(args[ARG_index].u_int, args[ARG_target_hz].u_int,
                              args[ARG_duration_ms].u_int);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_drive_pwm_poly_sweep_voice_obj, 3,
                                   pz_drive_pwm_poly_sweep_voice);
```

**Step 2: Add entries to module table**

In `pz_drive_module_globals_table`, after the `pwm_is_sweep_done` entry, add:

```c
    {MP_ROM_QSTR(MP_QSTR_pwm_poly_start), MP_ROM_PTR(&pz_drive_pwm_poly_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_pwm_poly_stop), MP_ROM_PTR(&pz_drive_pwm_poly_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_pwm_poly_is_running), MP_ROM_PTR(&pz_drive_pwm_poly_is_running_obj)},
    {MP_ROM_QSTR(MP_QSTR_pwm_poly_set_voice), MP_ROM_PTR(&pz_drive_pwm_poly_set_voice_obj)},
    {MP_ROM_QSTR(MP_QSTR_pwm_poly_sweep_voice), MP_ROM_PTR(&pz_drive_pwm_poly_sweep_voice_obj)},
```

**Step 3: Commit**

```bash
git add modules/pz_drive/pz_drive.c
git commit -m "feat(pz_drive): add Python bindings for poly API"
```

---

### Task 5: Add THX Deep Note sequence to music.py

**Files:**
- Modify: `python/music.py` (after SONGS dict, before `play_song()`)

**Step 1: Add thx() function**

```python
def thx(gain=100, duration_ms=10000):
    """THX Deep Note: 12 voices converge from a random cluster to a D chord."""
    import random
    import pz_drive
    from drv2665 import DRV2665
    from pz_drive_py import PzActuator

    # D chord across octaves + A5 for the fifth
    targets = [73, 73, 146, 146, 293, 293, 293, 587, 587, 587, 440, 440]

    drv = DRV2665()
    drv.init_analog(PzActuator.GAINS[gain])
    pz_drive.pwm_poly_start()

    try:
        # Phase 1: random cluster (200-400 Hz), hold 2s
        for i in range(12):
            pz_drive.pwm_poly_set_voice(i, random.randint(200, 400), amplitude=80)
        time.sleep_ms(2000)

        # Phase 2: sweep all voices to targets
        for i in range(12):
            pz_drive.pwm_poly_sweep_voice(i, targets[i], duration_ms)
        time.sleep_ms(duration_ms)

        # Phase 3: sustain the chord
        time.sleep_ms(3000)
    except KeyboardInterrupt:
        pass
    finally:
        pz_drive.pwm_poly_stop()
        drv.standby()
```

**Step 2: Commit**

```bash
git add python/music.py
git commit -m "feat(music): add THX Deep Note sequence"
```

---

### Task 6: Ensure mono stop clears poly mode

**Files:**
- Modify: `modules/pz_drive/pwm.c`

**Step 1: Add poly cleanup to pwm_stop_internal**

In `pwm_stop_internal()` (line 330), after `s_running = false;` (line 334), add:

```c
    s_poly_mode = false;
```

Also, in `pzd_pwm_poly_start()`, make sure it calls `pwm_stop_internal()` first (already done in Task 3). And in `pwm_start_internal()`, add `s_poly_mode = false;` at the start to ensure mono start disables poly.

**Step 2: Commit**

```bash
git add modules/pz_drive/pwm.c
git commit -m "fix(pz_drive): ensure mutual exclusion between mono and poly modes"
```

---

### Task 7: Build, flash, and test

**Step 1: Build firmware**

```bash
./scripts/build.sh
```

Expected: Build succeeds with new poly functions compiled.

**Step 2: Flash**

Use `/flash` skill — enter bootloader, flash, upload filesystem Python, verify boot.

**Step 3: Verify poly API exists**

```python
import pz_drive
print('pwm_poly_start' in dir(pz_drive))  # True
```

**Step 4: Test THX**

```python
from music import thx
thx(gain=75)
```

Expected: 2s of random cluster → 10s sweep to D chord → 3s sustain → stop.

**Step 5: Commit any fixes**

```bash
git add -A
git commit -m "fix: post-test adjustments for polyphonic DDS"
```
