#ifndef PZ_PWM_H
#define PZ_PWM_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// Compile-time configuration
#define PWM_GPIO               5
#define PWM_DEFAULT_RESOLUTION 8     // 8 or 10
#define PWM_SAMPLE_RATE_HZ     32000 // DDS update rate

// Initialize LEDC and GPTimer (call once from pz_actuator init)
esp_err_t pwm_init(void);

// Deinitialize PWM and timer
esp_err_t pwm_deinit(void);

// Start sine wave output at given frequency (50-400 Hz).
// freq_hz == 0 means DC fully on (100% duty).
esp_err_t pwm_start_sine(uint16_t freq_hz);

// Stop PWM output (sets duty to 0)
esp_err_t pwm_stop(void);

// Change resolution at runtime (8 or 10 bits).
// Stops and restarts output if currently running.
esp_err_t pwm_set_resolution(uint8_t bits);

// Set sine amplitude (0 = silent, 128 = full 0-3.3V swing).
// ~70 gives 1.8V pk-pk, the DRV2665 full-scale analog input.
// Safe to call while running (atomic write to ISR variable).
void pwm_set_amplitude(uint8_t amplitude);

// Returns true if PWM sine is currently running
bool pwm_is_running(void);

#endif // PZ_PWM_H
