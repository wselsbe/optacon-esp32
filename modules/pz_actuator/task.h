#ifndef PZ_TASK_H
#define PZ_TASK_H

#include "drv2665.h"
#include "shift_register.h"
#include "waveform.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

typedef struct {
    drv2665_t drv;
    shift_register_t sr;
    waveform_t waveform;
    TaskHandle_t task_handle;
    bool running;
    uint16_t target_frequency;  // for frequency change while running
    bool frequency_changed;
} pz_task_state_t;

// Start the background task.
esp_err_t pz_task_start(pz_task_state_t *state);

// Stop the background task. Puts DRV2665 in standby.
esp_err_t pz_task_stop(pz_task_state_t *state);

#endif // PZ_TASK_H
