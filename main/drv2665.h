
#include <stdio.h>

#define I2C_MASTER_NUM 0
#define I2C_MASTER_CLK 400000
#define I2C_SCL_PIN   21
#define I2C_SDA_PIN   47

#define DRV2665_ADDR    0x59
#define DRV_REGISTER_0  0x00
#define DRV_REGISTER_1  0x01
#define DRV_REGISTER_2  0x02
#define DRV_REGISTER_DATA  0x0B
#define DRV_RESET       1 << 7

#define _CHIPID_MASK 0b01111000

#define _INPUT_MASK 0b00000100
#define INPUT_DIGITAL 0 << 2
#define INPUT_ANALOG  1 << 2

#define _GAIN_MASK 0b00000011
#define GAIN_25V 0
#define GAIN_50V 1
#define GAIN_75V 2
#define GAIN_100V 3

#define _STANDBY_MASK 0b01000000
#define _STANDBY_FALSE 0 << 6
#define _STANDBY_TRUE 1 << 6

#define _TIMEOUT_MASK  0b00001100
#define TIMEOUT_5MS 0 << 2
#define TIMEOUT_10MS 1 << 2
#define TIMEOUT_15MS 2 << 2
#define TIMEOUT_20MS 3 << 2

#define _ENABLE_MASK 0b00000010
#define ENABLE_AUTO 0 << 1
#define ENABLE_OVERRIDE 1 << 1

#define FIFO_BUFFER_SIZE 100
#define SAMPLE_RATE 8000 // 8 kHz

void drv_init();

void drv_write_fifo(uint8_t* data, size_t data_len);

void drv_enable_digital();