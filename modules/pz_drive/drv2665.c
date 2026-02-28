#include "pz_drive.h"
void drv2665_bus_init(void) {}
int drv2665_read_reg(uint8_t reg) { (void)reg; return 0; }
void drv2665_write_reg(uint8_t reg, uint8_t val) { (void)reg; (void)val; }
void drv2665_write_fifo_bulk(const uint8_t *data, size_t len) { (void)data; (void)len; }
__attribute__((weak)) void drv2665_write_fifo_byte(uint8_t val) { (void)val; }
__attribute__((weak)) uint8_t drv2665_read_status(void) { return 0; }
