// modules/pz_drive/hv509.c
#include "pz_drive.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include <string.h>

#define SPI_MOSI_GPIO 6
#define SPI_MISO_GPIO 7
#define SPI_SCK_GPIO  9
#define SPI_CS_GPIO   10
#define POL_A_GPIO    12
#define POL_B_GPIO    13

#define SPI_CLK_HZ 1000000

static spi_device_handle_t s_spi_dev;
static bool s_spi_inited = false;
static bool s_pol_inited = false;
static bool s_pol_value = false;

// Stage/latch state
static volatile bool s_latch_pending = false;

// ── Polarity GPIOs ──────────────────────────────────────────────────────

void hv509_pol_init(void) {
    if (s_pol_inited) return;
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << POL_A_GPIO) | (1ULL << POL_B_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(POL_A_GPIO, 0);
    gpio_set_level(POL_B_GPIO, 0);
    s_pol_value = false;
    s_pol_inited = true;
}

void hv509_pol_set(bool val) {
    s_pol_value = val;
    gpio_set_level(POL_A_GPIO, val ? 1 : 0);
    gpio_set_level(POL_B_GPIO, val ? 1 : 0);
}

bool hv509_pol_get(void) {
    return s_pol_value;
}

void hv509_pol_toggle(void) {
    hv509_pol_set(!s_pol_value);
}

// ── SPI shift register ─────────────────────────────────────────────────

// Latch: rising edge on LE transfers shift register to outputs.
// After latch, LE stays high (idle state).
// ISR-safe: only uses gpio_set_level().
static inline void latch_le(void) {
    gpio_set_level(SPI_CS_GPIO, 1); // rising edge latches data; stays high
}

// Clock data into shift register: pull LE low, shift data, leave LE low.
// Caller must call latch_le() afterwards to commit.
static void spi_shift_data(uint32_t word32) {
    word32 &= 0x03FFFFC0U;
    gpio_set_level(SPI_CS_GPIO, 0); // LE low: enable shifting
    uint8_t buf[4];
    buf[0] = (word32 >> 24) & 0xFF;
    buf[1] = (word32 >> 16) & 0xFF;
    buf[2] = (word32 >> 8) & 0xFF;
    buf[3] = word32 & 0xFF;
    spi_transaction_t txn = {
        .length = 32,
        .tx_buffer = buf,
    };
    spi_device_transmit(s_spi_dev, &txn);
    // LE stays low — data is shifted in but not latched yet
}

void hv509_init(void) {
    if (s_spi_inited) return;
    // Polarity GPIOs MUST be configured before SPI (IOMUX conflict)
    hv509_pol_init();

    // Configure LE (latch enable) as manual GPIO output BEFORE SPI init
    gpio_config_t le_conf = {
        .pin_bit_mask = (1ULL << SPI_CS_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&le_conf);
    gpio_set_level(SPI_CS_GPIO, 1); // idle state: LE high

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI_GPIO,
        .miso_io_num = SPI_MISO_GPIO,
        .sclk_io_num = SPI_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_DISABLED);

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = SPI_CLK_HZ,
        .mode = 0,
        .spics_io_num = -1, // No automatic CS — we control LE manually
        .queue_size = 1,
    };
    spi_bus_add_device(SPI2_HOST, &dev_cfg, &s_spi_dev);

    // Clear all outputs: shift in zeros, then latch
    spi_shift_data(0);
    latch_le();
    s_spi_inited = true;
}

void hv509_sr_write(uint32_t word32) {
    if (!s_spi_inited) hv509_init();
    spi_shift_data(word32);
    latch_le();
}

void hv509_sr_stage(uint32_t word32) {
    if (!s_spi_inited) hv509_init();
    // Clock data into shift register now (safe, non-ISR context)
    spi_shift_data(word32);
    // If neither ISR nor task is running, latch immediately
    if (!pzd_pwm_is_running() && !pzd_fifo_is_running()) {
        latch_le();
        s_latch_pending = false;
    } else {
        // ISR will pulse LE at the right moment
        s_latch_pending = true;
    }
}

void hv509_sr_latch_if_pending(void) {
    // ISR-safe: only uses gpio_set_level()
    if (s_latch_pending) {
        latch_le();
        s_latch_pending = false;
    }
}
