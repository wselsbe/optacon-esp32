// Soft (bit-bang) I2C master — adapted from MicroPython extmod/machine_i2c.c
// Returns bytes written on NACK, unlike the ESP-IDF hardware I2C driver.

#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    int sda_pin;
    int scl_pin;
    uint32_t us_delay;   // half-period delay in microseconds
    uint32_t us_timeout; // clock-stretching timeout
} soft_i2c_t;

// Initialize: configure GPIOs as open-drain, send STOP to release bus
void soft_i2c_init(soft_i2c_t *i2c, int sda_pin, int scl_pin, uint32_t freq_hz);

// Deinitialize: release GPIOs
void soft_i2c_deinit(soft_i2c_t *i2c);

// Write `len` bytes to device at `addr` (7-bit).
// Returns number of data bytes ACKed (0..len), or <0 on error.
//   -1 = address NACK (device not present)
//   -2 = timeout (clock stretching)
int soft_i2c_write(soft_i2c_t *i2c, uint8_t addr, const uint8_t *data, size_t len);

// Read `len` bytes from device at `addr` (7-bit).
// Returns 0 on success, <0 on error.
int soft_i2c_read(soft_i2c_t *i2c, uint8_t addr, uint8_t *data, size_t len);

// Write then read (repeated start). Returns 0 on success, <0 on error.
int soft_i2c_write_read(soft_i2c_t *i2c, uint8_t addr,
                        const uint8_t *wr_data, size_t wr_len,
                        uint8_t *rd_data, size_t rd_len);

#endif // SOFT_I2C_H
