#ifndef WAVEFORM_H
#define WAVEFORM_H

#include <stdint.h>
#include <stddef.h>

#define WAVEFORM_SAMPLE_RATE 8000

// Maximum period: lowest supported frequency = 50Hz -> 160 samples
#define WAVEFORM_MAX_PERIOD 160

typedef struct {
    int8_t lut[WAVEFORM_MAX_PERIOD];  // one full period of sine wave
    size_t period_len;                 // samples per period at current frequency
    size_t write_index;                // circular index into the LUT
    uint16_t frequency;                // current frequency in Hz
    size_t trough_index;               // sample index of the minimum value within one period
} waveform_t;

// Initialize waveform at given frequency. Populates the LUT.
void waveform_init(waveform_t *wf, uint16_t frequency_hz);

// Change frequency. Regenerates the LUT. Resets write_index.
void waveform_set_frequency(waveform_t *wf, uint16_t frequency_hz);

// Copy the next `count` bytes from the LUT into out_buf, advancing write_index circularly.
// Returns number of bytes written (always == count).
size_t waveform_fill_buffer(waveform_t *wf, int8_t *out_buf, size_t count);

// Given the current write_index and a known fifo_depth (bytes currently in FIFO),
// calculate how many samples from NOW until the next trough is played back.
// Returns the number of samples (multiply by 125 for microseconds at 8kHz).
size_t waveform_samples_until_trough(const waveform_t *wf, size_t fifo_depth);

#endif // WAVEFORM_H
