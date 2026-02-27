// pz_pwm — Standalone MicroPython C module for analog DDS sine wave via LEDC PWM
//
// Generates a sine wave on GPIO 5 using a DDS phase accumulator clocked by a
// GPTimer ISR at 32 kHz. The ISR indexes a 256-entry sine LUT and updates the
// LEDC duty cycle each tick. Supports 8-bit and 10-bit PWM resolution, variable
// amplitude, and DC mode (0 Hz = 100% duty).
//
// Python API:
//   pz_pwm.set_frequency(hz, resolution=8, amplitude=128)
//   pz_pwm.start()
//   pz_pwm.stop()
//   pz_pwm.is_running()

#include "py/runtime.h"
#include "py/obj.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "esp_log.h"

// ─── Configuration ───────────────────────────────────────────────────────────

#define PWM_GPIO               5
#define PWM_SAMPLE_RATE_HZ     32000
#define PWM_DEFAULT_RESOLUTION 8

#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE

// ─── Sine LUT (256 entries, unsigned 0-255) ──────────────────────────────────

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

// ─── Module state ────────────────────────────────────────────────────────────

static gptimer_handle_t s_timer = NULL;
static bool s_hw_initialized = false;  // LEDC + GPTimer configured
static bool s_running = false;         // ISR actively updating duty
static uint8_t s_resolution = PWM_DEFAULT_RESOLUTION;

// DDS state — written by Python, read by ISR
static volatile uint32_t s_phase_acc = 0;
static volatile uint32_t s_phase_step = 0;
static volatile uint8_t s_amplitude = 128;  // 0-128

// Frequency configured via set_frequency (0 = DC)
static uint16_t s_freq_hz = 0;
static bool s_freq_configured = false;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static uint32_t ledc_max_duty(void) {
    return (1U << s_resolution) - 1;
}

// ─── GPTimer ISR: DDS phase accumulator → sine LUT → LEDC duty ──────────────

static bool IRAM_ATTR timer_isr_callback(gptimer_handle_t timer,
                                         const gptimer_alarm_event_data_t *edata,
                                         void *user_data) {
    s_phase_acc += s_phase_step;

    // Top 8 bits of phase accumulator index the 256-entry LUT
    uint8_t index = (uint8_t)(s_phase_acc >> 24);

    // Scale sine by amplitude: center at 128, scale deviation, re-center
    int32_t raw = (int32_t)sine_lut[index] - 128;  // -128 to +127
    int32_t scaled = 128 + ((raw * (int32_t)s_amplitude) >> 7);
    uint32_t duty = (uint32_t)scaled;

    // Scale 8-bit value to current resolution
    if (s_resolution == 10) {
        duty = (duty << 2) | (duty >> 6);  // 0-255 → 0-1023
    }

    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);

    return false;  // no need to yield
}

// ─── Hardware init (LEDC + GPTimer) ──────────────────────────────────────────

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

// ─── Internal start/stop ─────────────────────────────────────────────────────

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

    // Set duty to 0
    if (s_hw_initialized) {
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
    }

    return ESP_OK;
}

// ─── MicroPython bindings ────────────────────────────────────────────────────

// pz_pwm.set_frequency(hz, resolution=8, amplitude=128)
static mp_obj_t pz_pwm_set_frequency(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_hz, ARG_resolution, ARG_amplitude };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_hz,         MP_ARG_REQUIRED | MP_ARG_INT, {0}},
        {MP_QSTR_resolution, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 8}},
        {MP_QSTR_amplitude,  MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 128}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int hz = args[ARG_hz].u_int;
    int resolution = args[ARG_resolution].u_int;
    int amplitude = args[ARG_amplitude].u_int;

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

    s_freq_hz = (uint16_t)hz;
    s_amplitude = (uint8_t)amplitude;
    s_freq_configured = true;

    // Restart if was running
    if (was_running) {
        esp_err_t err = ensure_hw_init();
        if (err != ESP_OK) mp_raise_OSError(err);
        err = pwm_start_internal();
        if (err != ESP_OK) mp_raise_OSError(err);
    }

    if (hz == 0) {
        mp_printf(&mp_plat_print, "pz_pwm: DC mode, %d-bit, amplitude=%d\n", resolution, amplitude);
    } else {
        mp_printf(&mp_plat_print, "pz_pwm: %d Hz, %d-bit, amplitude=%d\n", hz, resolution, amplitude);
    }

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pz_pwm_set_frequency_obj, 1, pz_pwm_set_frequency);

// pz_pwm.start()
static mp_obj_t pz_pwm_start(void) {
    if (!s_freq_configured) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("call set_frequency() first"));
    }

    esp_err_t err = ensure_hw_init();
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "pz_pwm: hardware init failed: %d\n", err);
        mp_raise_OSError(err);
    }

    err = pwm_start_internal();
    if (err != ESP_OK) {
        mp_printf(&mp_plat_print, "pz_pwm: start failed: %d\n", err);
        mp_raise_OSError(err);
    }

    mp_printf(&mp_plat_print, "pz_pwm: started\n");
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_pwm_start_obj, pz_pwm_start);

// pz_pwm.stop()
static mp_obj_t pz_pwm_stop(void) {
    pwm_stop_internal();
    mp_printf(&mp_plat_print, "pz_pwm: stopped\n");
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_pwm_stop_obj, pz_pwm_stop);

// pz_pwm.is_running()
static mp_obj_t pz_pwm_is_running(void) {
    return mp_obj_new_bool(s_running);
}
static MP_DEFINE_CONST_FUN_OBJ_0(pz_pwm_is_running_obj, pz_pwm_is_running);

// ─── Module registration ─────────────────────────────────────────────────────

static const mp_rom_map_elem_t pz_pwm_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__),       MP_ROM_QSTR(MP_QSTR_pz_pwm)},
    {MP_ROM_QSTR(MP_QSTR_set_frequency),  MP_ROM_PTR(&pz_pwm_set_frequency_obj)},
    {MP_ROM_QSTR(MP_QSTR_start),          MP_ROM_PTR(&pz_pwm_start_obj)},
    {MP_ROM_QSTR(MP_QSTR_stop),           MP_ROM_PTR(&pz_pwm_stop_obj)},
    {MP_ROM_QSTR(MP_QSTR_is_running),     MP_ROM_PTR(&pz_pwm_is_running_obj)},
};
static MP_DEFINE_CONST_DICT(pz_pwm_globals, pz_pwm_globals_table);

const mp_obj_module_t pz_pwm_module = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&pz_pwm_globals,
};

MP_REGISTER_MODULE(MP_QSTR_pz_pwm, pz_pwm_module);
