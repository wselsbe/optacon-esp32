#include "pwm.h"
#include "sine.h"
#include "driver/ledc.h"
#include "driver/gptimer.h"
#include "esp_log.h"

static const char *TAG = "pwm";

// State
static gptimer_handle_t s_timer = NULL;
static bool s_initialized = false;
static bool s_running = false;
static uint8_t s_resolution = PWM_DEFAULT_RESOLUTION;

// DDS state (accessed from ISR)
static volatile uint32_t s_phase_acc = 0;
static volatile uint32_t s_phase_step = 0;
static volatile uint8_t s_amplitude = 128; // 0-128, default full range

#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_SPEED_MODE LEDC_LOW_SPEED_MODE

static uint32_t ledc_max_duty(void) {
    return (1U << s_resolution) - 1;
}

// GPTimer ISR: update LEDC duty from sine LUT
static bool IRAM_ATTR timer_isr_callback(gptimer_handle_t timer,
                                         const gptimer_alarm_event_data_t *edata, void *user_data) {
    s_phase_acc += s_phase_step;

    // Map phase accumulator top 8 bits to LUT index
    uint8_t index = (uint8_t)(s_phase_acc >> 24);

    // Scale sine by amplitude: center at 128, scale deviation, re-center
    int32_t raw = (int32_t)sine_lut_8bit[index] - 128; // -128 to +127
    int32_t scaled = 128 + ((raw * (int32_t)s_amplitude) >> 7);
    uint32_t duty = (uint32_t)scaled;

    // Scale 8-bit value to current resolution
    if (s_resolution == 10) {
        duty = (duty << 2) | (duty >> 6); // scale 0-255 to 0-1023
    }

    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);

    return false; // no need to yield
}

static esp_err_t configure_ledc(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = s_resolution,
        .freq_hz = 80000000U >> s_resolution, // 312.5kHz@8bit, 78.1kHz@10bit
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

static esp_err_t configure_timer(void) {
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz (1 us per tick)
    };
    esp_err_t err = gptimer_new_timer(&cfg, &s_timer);
    if (err != ESP_OK) return err;

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = 1000000 / PWM_SAMPLE_RATE_HZ, // period in us
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

esp_err_t pwm_init(void) {
    if (s_initialized) return ESP_OK;

    esp_err_t err = configure_ledc();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC config failed: %d", err);
        return err;
    }

    err = configure_timer();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPTimer config failed: %d", err);
        return err;
    }

    err = gptimer_enable(s_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPTimer enable failed: %d", err);
        return err;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t pwm_deinit(void) {
    if (!s_initialized) return ESP_OK;

    pwm_stop();

    if (s_timer) {
        gptimer_disable(s_timer);
        gptimer_del_timer(s_timer);
        s_timer = NULL;
    }

    ledc_stop(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
    s_initialized = false;
    return ESP_OK;
}

esp_err_t pwm_start_sine(uint16_t freq_hz) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    // Stop current output first
    if (s_running) {
        gptimer_stop(s_timer);
        s_running = false;
    }

    if (freq_hz == 0) {
        // DC mode: 100% duty, no timer
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, ledc_max_duty());
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
        return ESP_OK;
    }

    // Clamp frequency
    if (freq_hz < 50) freq_hz = 50;
    if (freq_hz > 400) freq_hz = 400;

    // Calculate DDS phase step: step = (freq / sample_rate) * 2^32
    s_phase_acc = 0;
    s_phase_step = (uint32_t)(((uint64_t)freq_hz << 32) / PWM_SAMPLE_RATE_HZ);

    // Start timer
    esp_err_t err = gptimer_start(s_timer);
    if (err != ESP_OK) return err;

    s_running = true;
    return ESP_OK;
}

esp_err_t pwm_stop(void) {
    if (s_running) {
        gptimer_stop(s_timer);
        s_running = false;
    }

    // Set duty to 0 (no output)
    if (s_initialized) {
        ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL);
    }

    return ESP_OK;
}

esp_err_t pwm_set_resolution(uint8_t bits) {
    if (bits != 8 && bits != 10) return ESP_ERR_INVALID_ARG;
    if (bits == s_resolution) return ESP_OK;

    bool was_running = s_running;
    uint16_t saved_freq = 0;

    if (was_running) {
        // Save current frequency from phase_step
        saved_freq = (uint16_t)(((uint64_t)s_phase_step * PWM_SAMPLE_RATE_HZ) >> 32);
        pwm_stop();
    }

    s_resolution = bits;
    esp_err_t err = configure_ledc();
    if (err != ESP_OK) return err;

    if (was_running && saved_freq > 0) {
        return pwm_start_sine(saved_freq);
    }

    return ESP_OK;
}

void pwm_set_amplitude(uint8_t amplitude) {
    s_amplitude = (amplitude > 128) ? 128 : amplitude;
}

bool pwm_is_running(void) {
    return s_running;
}
