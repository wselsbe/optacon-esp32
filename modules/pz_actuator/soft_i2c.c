// Soft (bit-bang) I2C master — adapted from MicroPython extmod/machine_i2c.c
// Uses ESP-IDF GPIO APIs instead of MicroPython's mp_hal_pin_* primitives.

#include "soft_i2c.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_rom_gpio.h"

// ─── Low-level GPIO primitives ───────────────────────────────────────────────

static inline void sda_low(soft_i2c_t *i2c) {
    gpio_set_level(i2c->sda_pin, 0);
}

static inline void sda_release(soft_i2c_t *i2c) {
    gpio_set_level(i2c->sda_pin, 1);
}

static inline int sda_read(soft_i2c_t *i2c) {
    return gpio_get_level(i2c->sda_pin);
}

static inline void scl_low(soft_i2c_t *i2c) {
    gpio_set_level(i2c->scl_pin, 0);
}

static inline void delay(soft_i2c_t *i2c) {
    esp_rom_delay_us(i2c->us_delay);
}

// Release SCL and wait for clock stretching. Returns 0 on success, -2 on timeout.
static int scl_release(soft_i2c_t *i2c) {
    gpio_set_level(i2c->scl_pin, 1);
    delay(i2c);
    // Wait for slave to release SCL (clock stretching)
    uint32_t count = i2c->us_timeout;
    for (; !gpio_get_level(i2c->scl_pin) && count; --count) {
        esp_rom_delay_us(1);
    }
    if (count == 0) {
        return -2;  // timeout
    }
    return 0;
}

// ─── I2C protocol primitives ─────────────────────────────────────────────────

static int i2c_start(soft_i2c_t *i2c) {
    sda_release(i2c);
    delay(i2c);
    int ret = scl_release(i2c);
    if (ret != 0) return ret;
    sda_low(i2c);
    delay(i2c);
    return 0;
}

static int i2c_stop(soft_i2c_t *i2c) {
    delay(i2c);
    sda_low(i2c);
    delay(i2c);
    int ret = scl_release(i2c);
    sda_release(i2c);
    delay(i2c);
    return ret;
}

// Write one byte, MSB first. Returns 0=ACK, 1=NACK, <0=error.
static int i2c_write_byte(soft_i2c_t *i2c, uint8_t byte) {
    delay(i2c);
    scl_low(i2c);

    for (int i = 7; i >= 0; i--) {
        if ((byte >> i) & 1) {
            sda_release(i2c);
        } else {
            sda_low(i2c);
        }
        delay(i2c);
        int ret = scl_release(i2c);
        if (ret != 0) { sda_release(i2c); return ret; }
        scl_low(i2c);
    }

    // Read ACK: release SDA, clock SCL, read SDA
    sda_release(i2c);
    delay(i2c);
    int ret = scl_release(i2c);
    if (ret != 0) return ret;
    int ack = sda_read(i2c);
    delay(i2c);
    scl_low(i2c);

    return ack;
}

// Read one byte, MSB first. Send ACK if nack==0, NACK if nack==1.
static int i2c_read_byte(soft_i2c_t *i2c, uint8_t *byte, int nack) {
    delay(i2c);
    scl_low(i2c);
    delay(i2c);

    uint8_t val = 0;
    for (int i = 7; i >= 0; i--) {
        int ret = scl_release(i2c);
        if (ret != 0) return ret;
        val = (val << 1) | sda_read(i2c);
        scl_low(i2c);
        delay(i2c);
    }

    // Send ACK or NACK
    if (nack) {
        sda_release(i2c);
    } else {
        sda_low(i2c);
    }
    delay(i2c);
    int ret = scl_release(i2c);
    if (ret != 0) return ret;
    delay(i2c);
    scl_low(i2c);
    sda_release(i2c);

    *byte = val;
    return 0;
}

// ─── Public API ──────────────────────────────────────────────────────────────

void soft_i2c_init(soft_i2c_t *i2c, int sda_pin, int scl_pin, uint32_t freq_hz) {
    i2c->sda_pin = sda_pin;
    i2c->scl_pin = scl_pin;
    i2c->us_delay = 500000 / freq_hz;
    if (i2c->us_delay == 0) i2c->us_delay = 1;
    i2c->us_timeout = 50000;  // 50ms clock-stretching timeout

    // Configure as open-drain — match MicroPython's mp_hal_pin_open_drain()
    esp_rom_gpio_pad_select_gpio(sda_pin);
    esp_rom_gpio_pad_select_gpio(scl_pin);
    gpio_set_direction(sda_pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(scl_pin, GPIO_MODE_INPUT_OUTPUT_OD);

    // Release bus
    sda_release(i2c);
    scl_release(i2c);
    i2c_stop(i2c);
}

void soft_i2c_deinit(soft_i2c_t *i2c) {
    gpio_reset_pin(i2c->sda_pin);
    gpio_reset_pin(i2c->scl_pin);
}

int soft_i2c_write(soft_i2c_t *i2c, uint8_t addr, const uint8_t *data, size_t len) {
    int ret = i2c_start(i2c);
    if (ret != 0) { i2c_stop(i2c); return -2; }

    ret = i2c_write_byte(i2c, addr << 1);  // write bit = 0
    if (ret != 0) { i2c_stop(i2c); return -1; }  // address NACK

    int written = 0;
    for (size_t i = 0; i < len; i++) {
        ret = i2c_write_byte(i2c, data[i]);
        if (ret != 0) break;  // NACK or error
        written++;
    }

    i2c_stop(i2c);
    return written;
}

int soft_i2c_read(soft_i2c_t *i2c, uint8_t addr, uint8_t *data, size_t len) {
    int ret = i2c_start(i2c);
    if (ret != 0) { i2c_stop(i2c); return -2; }

    ret = i2c_write_byte(i2c, (addr << 1) | 1);  // read bit = 1
    if (ret != 0) { i2c_stop(i2c); return -1; }

    for (size_t i = 0; i < len; i++) {
        int nack = (i == len - 1) ? 1 : 0;  // NACK on last byte
        ret = i2c_read_byte(i2c, &data[i], nack);
        if (ret != 0) { i2c_stop(i2c); return -2; }
    }

    i2c_stop(i2c);
    return 0;
}

int soft_i2c_write_read(soft_i2c_t *i2c, uint8_t addr,
                        const uint8_t *wr_data, size_t wr_len,
                        uint8_t *rd_data, size_t rd_len) {
    // Write phase (no STOP — use repeated start)
    int ret = i2c_start(i2c);
    if (ret != 0) { i2c_stop(i2c); return -2; }

    ret = i2c_write_byte(i2c, addr << 1);
    if (ret != 0) { i2c_stop(i2c); return -1; }

    for (size_t i = 0; i < wr_len; i++) {
        ret = i2c_write_byte(i2c, wr_data[i]);
        if (ret != 0) { i2c_stop(i2c); return -2; }
    }

    // Repeated start + read phase
    ret = i2c_start(i2c);
    if (ret != 0) { i2c_stop(i2c); return -2; }

    ret = i2c_write_byte(i2c, (addr << 1) | 1);
    if (ret != 0) { i2c_stop(i2c); return -1; }

    for (size_t i = 0; i < rd_len; i++) {
        int nack = (i == rd_len - 1) ? 1 : 0;
        ret = i2c_read_byte(i2c, &rd_data[i], nack);
        if (ret != 0) { i2c_stop(i2c); return -2; }
    }

    i2c_stop(i2c);
    return 0;
}
