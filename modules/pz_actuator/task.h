#ifndef PZ_TASK_H
#define PZ_TASK_H

#include "drv2665.h"
#include "shift_register.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    drv2665_t drv;
    shift_register_t sr;
    TaskHandle_t task_handle;
    bool running;
    bool sync_trough;           // if true, wait for waveform trough before SR commit

    // Python-provided waveform buffer (one period, trough at index 0)
    int8_t *waveform_buf;       // pointer into Python bytearray data
    size_t  waveform_len;       // period length in samples
    size_t  write_index;        // circular position in waveform
} pz_task_state_t;

// Start the background task.
esp_err_t pz_task_start(pz_task_state_t *state);

// Stop the background task. Puts DRV2665 in standby.
esp_err_t pz_task_stop(pz_task_state_t *state);

#endif // PZ_TASK_H
