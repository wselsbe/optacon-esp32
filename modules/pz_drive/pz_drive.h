// modules/pz_drive/pz_drive.h
#ifndef PZ_DRIVE_H
#define PZ_DRIVE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// ── hv509.c — SPI shift register + polarity GPIOs ──────────────────────
void hv509_init(void);
void hv509_sr_stage(uint32_t word32);
void hv509_sr_write(uint32_t word32);
void hv509_sr_latch_if_pending(void);
void hv509_pol_init(void);
void hv509_pol_set(bool val);
bool hv509_pol_get(void);
void hv509_pol_toggle(void);

// ── drv2665.c — I2C bus + register access ───────────────────────────────
void drv2665_bus_init(void);
int drv2665_read_reg(uint8_t reg);
void drv2665_write_reg(uint8_t reg, uint8_t val);
void drv2665_write_fifo_bulk(const uint8_t *data, size_t len);
void drv2665_write_fifo_byte(uint8_t val);
uint8_t drv2665_read_status(void);

// ── pwm.c — analog DDS ISR ─────────────────────────────────────────────
bool pwm_is_running(void);

// ── fifo.c — digital FIFO background task ───────────────────────────────
bool fifo_is_running(void);

#endif // PZ_DRIVE_H
