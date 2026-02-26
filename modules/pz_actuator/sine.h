#ifndef PZ_SINE_H
#define PZ_SINE_H

#include <stdint.h>
#include <stddef.h>

// 256-entry sine table, values 0-255 (one full cycle, peak at index 64)
#define SINE_LUT_SIZE 256

extern const uint8_t sine_lut_8bit[SINE_LUT_SIZE];

// Generate a signed sine waveform for the DRV2665 digital FIFO.
// Writes one full period into `buf`. Returns number of samples written.
// buf must be at least (sample_rate / freq_hz) bytes.
// Output range: -127 to +127 (signed 8-bit, trough at index 0).
size_t sine_generate_digital(int8_t *buf, size_t buf_max, uint16_t freq_hz, uint16_t sample_rate);

#endif // PZ_SINE_H
