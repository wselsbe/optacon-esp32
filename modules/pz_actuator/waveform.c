#include "waveform.h"
#include <math.h>
#include <string.h>

void waveform_init(waveform_t *wf, uint16_t frequency_hz) {
    memset(wf, 0, sizeof(waveform_t));
    waveform_set_frequency(wf, frequency_hz);
}

void waveform_set_frequency(waveform_t *wf, uint16_t frequency_hz) {
    if (frequency_hz == 0) frequency_hz = 1;

    wf->frequency = frequency_hz;
    wf->period_len = WAVEFORM_SAMPLE_RATE / frequency_hz;
    if (wf->period_len > WAVEFORM_MAX_PERIOD) {
        wf->period_len = WAVEFORM_MAX_PERIOD;
    }
    if (wf->period_len < 2) {
        wf->period_len = 2;
    }

    // Generate sine LUT: phase starts at -PI/2 so sample 0 is the trough (minimum)
    int8_t min_val = 0;
    size_t min_idx = 0;
    for (size_t i = 0; i < wf->period_len; i++) {
        double phase = -M_PI / 2.0 + (2.0 * M_PI * i) / wf->period_len;
        double value = sin(phase);
        wf->lut[i] = (int8_t)(value * 127.0);
        if (wf->lut[i] < min_val || i == 0) {
            min_val = wf->lut[i];
            min_idx = i;
        }
    }
    wf->trough_index = min_idx;
    wf->write_index = 0;
}

size_t waveform_fill_buffer(waveform_t *wf, int8_t *out_buf, size_t count) {
    for (size_t i = 0; i < count; i++) {
        out_buf[i] = wf->lut[wf->write_index];
        wf->write_index = (wf->write_index + 1) % wf->period_len;
    }
    return count;
}

size_t waveform_samples_until_trough(const waveform_t *wf, size_t fifo_depth) {
    // The playback head is at: write_index - fifo_depth (mod period_len)
    // We need to find how many samples from the playback head to the next trough.

    // playback_index: where in the LUT the DRV2665 is currently playing
    size_t playback_index;
    size_t depth_mod = fifo_depth % wf->period_len;
    if (wf->write_index >= depth_mod) {
        playback_index = wf->write_index - depth_mod;
    } else {
        playback_index = wf->period_len - (depth_mod - wf->write_index);
    }

    // Distance from playback_index to the next trough
    if (wf->trough_index >= playback_index) {
        return wf->trough_index - playback_index;
    } else {
        return wf->period_len - playback_index + wf->trough_index;
    }
}
