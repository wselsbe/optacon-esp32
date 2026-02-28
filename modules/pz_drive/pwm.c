// modules/pz_drive/pwm.c — Analog DDS ISR with shift register latch
//
// Generates a sine/triangle/square wave on GPIO 5 using a DDS phase
// accumulator clocked by a GPTimer ISR at 32 kHz.  The ISR indexes a
// 256-entry sine LUT and updates the LEDC duty cycle each tick.
//
// At zero-crossing (fullwave) or cycle wrap (non-fullwave) the ISR calls
// hv509_sr_latch_if_pending() so staged shift register data is committed
// at a safe moment.  Polarity toggling delegates to hv509_pol_set().

#include "pz_drive.h"

#include "py/runtime.h"
#include "py/obj.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"

// ---- Configuration ---------------------------------------------------------

#define PWM_GPIO               5
#define PWM_SAMPLE_RATE_HZ     32000
#define PWM_DEFAULT_RESOLUTION 8

#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE

// ---- Sine LUT (256 entries, unsigned 0-255) --------------------------------

#define SINE_LUT_SIZE 256

static const uint8_t sine_lut[SINE_LUT_SIZE] = {
    // sin(0)=128, sin(pi/2)=255, sin(pi)=128, sin(3pi/2)=0
    // Generated with: round(127.5 + 127.5 * sin(2*pi * i / 256)) for i in 0..255
    128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170, 173,
    176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211, 213, 215,
    218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 239, 240, 241, 243, 244,
    245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255,
    255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251, 250, 250, 249, 248, 246,
    245, 244, 243, 241, 240, 239, 237, 235, 234, 232, 230, 228, 226, 224, 222, 220,
    218, 215, 213, 211, 208, 206, 203, 201, 198, 196, 193, 190, 188, 185, 182, 179,
    176, 173, 170, 167, 165, 162, 158, 155, 152, 149, 146, 143, 140, 137, 134, 131,
    128, 124, 121, 118, 115, 112, 109, 106, 103, 100,  97,  93,  90,  88,  85,  82,
     79,  76,  73,  70,  67,  65,  62,  59,  57,  54,  52,  49,  47,  44,  42,  40,
     37,  35,  33,  31,  29,  27,  25,  23,  21,  20,  18,  16,  15,  14,  12,  11,
     10,   9,   7,   6,   5,   5,   4,   3,   2,   2,   1,   1,   1,   0,   0,   0,
      0,   0,   0,   0,   1,   1,   1,   2,   2,   3,   4,   5,   5,   6,   7,   9,
     10,  11,  12,  14,  15,  16,  18,  20,  21,  23,  25,  27,  29,  31,  33,  35,
     37,  40,  42,  44,  47,  49,  52,  54,  57,  59,  62,  65,  67,  70,  73,  76,
     79,  82,  85,  88,  90,  93,  97, 100, 103, 106, 109, 112, 115, 118, 121, 124,
};

// ---- Module state ----------------------------------------------------------

static gptimer_handle_t s_timer = NULL;
static bool s_hw_initialized = false;  // LEDC + GPTimer configured
static bool s_running = false;         // ISR actively updating duty
static uint8_t s_resolution = PWM_DEFAULT_RESOLUTION;

// DDS state -- written by Python, read by ISR
static volatile uint32_t s_phase_acc = 0;
static volatile uint32_t s_phase_step = 0;
static volatile uint8_t s_amplitude = 128;  // 0-128

// Frequency configured via set_frequency (0 = DC)
static uint16_t s_freq_hz = 0;
static bool s_freq_configured = false;

// Waveform type
#define WAVEFORM_SINE     0
#define WAVEFORM_TRIANGLE 1
#define WAVEFORM_SQUARE   2
static volatile uint8_t s_waveform = WAVEFORM_SINE;

// Fullwave mode state
static volatile bool s_fullwave = false;
static volatile uint8_t s_prev_half = 0;  // 0=first half (0..pi), 1=second half (pi..2pi)
static volatile uint32_t s_dead_phase = 0;  // phase margin around zero-crossings (force duty=128)
static volatile uint32_t s_pol_advance = 0;  // phase advance for polarity toggle (compensate output lag)

// ---- Helpers ---------------------------------------------------------------

static uint32_t ledc_max_duty(void) {
    return (1U << s_resolution) - 1;
}

// ---- GPTimer ISR: DDS phase accumulator -> waveform LUT -> LEDC duty -------

static bool IRAM_ATTR timer_isr_callback(gptimer_handle_t timer,
                                         const gptimer_alarm_event_data_t *edata,
                                         void *user_data) {
    // Save previous phase for cycle-wrap detection
    uint32_t prev_phase = s_phase_acc;
    s_phase_acc += s_phase_step;

    // Top 8 bits of phase accumulator index the waveform
    uint8_t index = (uint8_t)(s_phase_acc >> 24);

    // Generate waveform value centered at 0 (-128 to +127)
    int32_t raw;
    switch (s_waveform) {
    case WAVEFORM_TRIANGLE:
        raw = (index < 128) ? (index * 2) : (510 - index * 2);
        raw -= 128;
        break;
    case WAVEFORM_SQUARE:
        raw = (index < 128) ? 127 : -128;
        break;
    default:  // WAVEFORM_SINE
        raw = (int32_t)sine_lut[index] - 128;
        break;
    }

    // Scale by amplitude and re-center to unsigned 0-255
    int32_t scaled = 128 + ((raw * (int32_t)s_amplitude) >> 7);
    uint32_t duty = (uint32_t)scaled;

    if (s_fullwave) {
        // Full-wave rectification: mirror negative half around 128
        uint8_t half = (uint8_t)(s_phase_acc >> 31);
        if (half) {
            duty = 256 - duty;  // mirror: values below 128 become above 128
        }

        // Dead time: force zero output near zero-crossings so DRV2665
        // output settles before polarity toggles
        if (s_dead_phase) {
            // Distance from current phase to nearest half-boundary (0 or 0x80000000)
            uint32_t half_phase = s_phase_acc & 0x7FFFFFFFU;
            uint32_t dist = half_phase;
            if (half_phase > 0x40000000U) {
                dist = 0x80000000U - half_phase;
            }
            if (dist < s_dead_phase) {
                duty = 128;  // zero output (midpoint)
            }
        }

        // Toggle polarity at zero-crossing, advanced by s_pol_advance to
        // compensate for DRV2665 output lag
        uint8_t pol_half = (uint8_t)((s_phase_acc + s_pol_advance) >> 31);
        if (pol_half != s_prev_half) {
            s_prev_half = pol_half;
            hv509_pol_set(pol_half ? true : false);
            hv509_sr_latch_if_pending();    // latch at zero-crossing
        }
    } else {
        // No fullwave -- latch at cycle start (phase wrap)
        if (s_phase_acc < prev_phase) {     // phase wrapped around
            hv509_sr_latch_if_pending();    // latch at cycle start
        }
    }

    // Scale 8-bit value to current resolution
    if (s_resolution == 10) {
        duty = (duty << 2) | (duty >> 6);  // 0-255 -> 0-1023
    }

    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);

    return false;  // no need to yield
}

// ---- Hardware init (LEDC + GPTimer) ----------------------------------------

static esp_err_t configure_ledc(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = s_resolution,
        .freq_hz = 80000000U >> s_resolution,  // 312.5kHz@8bit, 78.1kHz@10bit
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) return err;

    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_SPEED_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .gpio_num = PWM_GPIO,
        .duty = 0,
        .hpoint = 0,
    };
    return ledc_channel_config(&ch_cfg);
}

static esp_err_t configure_gptimer(void) {
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 MHz (1 us per tick)
    };
    esp_err_t err = gptimer_new_timer(&cfg, &s_timer);
    if (err != ESP_OK) return err;

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = 1000000 / PWM_SAMPLE_RATE_HZ,  // 31 ticks = 31.25 us
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    err = gptimer_set_alarm_action(s_timer, &alarm_cfg);
    if (err != ESP_OK) return err;

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_isr_callback,
    };
    return gptimer_register_event_callbacks(s_timer, &cbs, NULL);
}

static esp_err_t ensure_hw_init(void) {
    if (s_hw_initialized) return ESP_OK;

    esp_err_t err = configure_ledc();
    if (err != ESP_OK) return err;

    err = configure_gptimer();
    if (err != ESP_OK) return err;

    err = gptimer_enable(s_timer);
    if (err != ESP_OK) return err;

    s_hw_initialized = true;
    return ESP_OK;
}

// ---- Internal start/stop ---------------------------------------------------

static esp_err_t pwm_start_internal(void) {
    if (!s_hw_initialized) return ESP_ERR_INVALID_STATE;

    // Stop current output first
    if (s_running) {
        gptimer_stop(s_timer);
        s_running = false;
    }

    if (s_freq_hz == 0) {
        // DC mode: 100% duty, no timer needed
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, ledc_max_duty());
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
        s_running = true;
        return ESP_OK;
    }

    // Calculate DDS phase step: step = (freq / sample_rate) * 2^32
    s_phase_acc = 0;
    s_phase_step = (uint32_t)(((uint64_t)s_freq_hz << 32) / PWM_SAMPLE_RATE_HZ);

    if (s_fullwave) {
        s_prev_half = 0;
        hv509_pol_set(false);
    }

    // Start GPTimer ISR
    esp_err_t err = gptimer_start(s_timer);
    if (err != ESP_OK) return err;

    s_running = true;
    return ESP_OK;
}

static esp_err_t pwm_stop_internal(void) {
    if (s_running && s_freq_hz != 0 && s_timer) {
        gptimer_stop(s_timer);
    }
    s_running = false;

    if (s_fullwave) {
        hv509_pol_set(false);
    }

    // Set duty to 0
    if (s_hw_initialized) {
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
    }

    return ESP_OK;
}

// ---- Public API (called by pz_drive.c bindings) ----------------------------

bool pzd_pwm_is_running(void) {
    return s_running;
}

void pzd_pwm_set_frequency(int hz, int resolution, int amplitude, bool fullwave,
                            int dead_time, int phase_advance, int waveform) {
    // Validate frequency: 0 (DC) or 50-400 Hz
    if (hz != 0 && (hz < 50 || hz > 400)) {
        mp_raise_ValueError(MP_ERROR_TEXT("hz must be 0 (DC) or 50-400"));
    }

    // Validate resolution
    if (resolution != 8 && resolution != 10) {
        mp_raise_ValueError(MP_ERROR_TEXT("resolution must be 8 or 10"));
    }

    // Validate and clamp amplitude
    if (amplitude < 0) amplitude = 0;
    if (amplitude > 128) amplitude = 128;

    // If resolution changed, reconfigure LEDC
    bool was_running = s_running;
    if (was_running) {
        pwm_stop_internal();
    }

    if ((uint8_t)resolution != s_resolution) {
        s_resolution = (uint8_t)resolution;
        if (s_hw_initialized) {
            esp_err_t err = configure_ledc();
            if (err != ESP_OK) {
                mp_raise_OSError(err);
            }
        }
    }

    if (waveform < 0 || waveform > 2) waveform = WAVEFORM_SINE;

    s_freq_hz = (uint16_t)hz;
    s_amplitude = (uint8_t)amplitude;
    s_fullwave = fullwave;
    s_waveform = (uint8_t)waveform;
    s_freq_configured = true;

    // Compute phase-domain parameters from tick counts
    uint32_t phase_step = (uint32_t)(((uint64_t)hz << 32) / PWM_SAMPLE_RATE_HZ);

    if (dead_time < 0) dead_time = 0;
    if (dead_time > 200) dead_time = 200;
    s_dead_phase = (uint32_t)dead_time * phase_step;

    if (phase_advance < 0) phase_advance = 0;
    if (phase_advance > 200) phase_advance = 200;
    s_pol_advance = (uint32_t)phase_advance * phase_step;

    // Restart if was running
    if (was_running) {
        esp_err_t err = ensure_hw_init();
        if (err != ESP_OK) mp_raise_OSError(err);
        err = pwm_start_internal();
        if (err != ESP_OK) mp_raise_OSError(err);
    }

    if (hz == 0) {
        mp_printf(&mp_plat_print, "pz_drive: DC mode, %d-bit, amplitude=%d\n", resolution, amplitude);
    } else {
        const char *wf_names[] = {"sine", "triangle", "square"};
        const char *wf_name = wf_names[waveform];
        if (fullwave) {
            mp_printf(&mp_plat_print, "pz_drive: %d Hz, %s, %d-bit, amplitude=%d [fullwave, dt=%d, pa=%d]\n",
                      hz, wf_name, resolution, amplitude, dead_time, phase_advance);
        } else {
            mp_printf(&mp_plat_print, "pz_drive: %d Hz, %s, %d-bit, amplitude=%d\n",
                      hz, wf_name, resolution, amplitude);
        }
    }
}

void pzd_pwm_start(void) {
    if (!s_freq_configured) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("call pwm_set_frequency() first"));
    }

    esp_err_t err = ensure_hw_init();
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "pz_drive: hardware init failed: %d\n", err);
        mp_raise_OSError(err);
    }

    err = pwm_start_internal();
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "pz_drive: start failed: %d\n", err);
        mp_raise_OSError(err);
    }

    mp_printf(&mp_plat_print, "pz_drive: pwm started\n");
}

void pzd_pwm_stop(void) {
    pwm_stop_internal();
    mp_printf(&mp_plat_print, "pz_drive: pwm stopped\n");
}
