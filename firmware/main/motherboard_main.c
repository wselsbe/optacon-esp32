/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include <math.h>
#include "driver/i2s_pdm.h"
#include "drv2665.h"
#include "driver/gpio.h"

static const char *TAG = "motherboard";


#define SPI_BUS  SPI2_HOST
#define SPI_SCK  GPIO_NUM_9
#define SPI_MISO GPIO_NUM_7
#define SPI_MOSI GPIO_NUM_6
#define SPI_CS   GPIO_NUM_10


size_t sine_wave_size;
uint8_t sine_wave[8000]; // Pre-calculated sine wave for one period

#define EXAMPLE_BUFF_SIZE   2048
#define EXAMPLE_PDM_TX_CLK_IO           GPIO_NUM_3      // I2S PDM TX clock io number
#define EXAMPLE_PDM_TX_DOUT_IO          GPIO_NUM_5      // I2S PDM TX data out io number

#define EXAMPLE_PDM_TX_FREQ_HZ          1000           // I2S PDM TX frequency
#define EXAMPLE_WAVE_AMPLITUDE          (1000.0)        // 1~32767
#define CONST_PI                        (3.1416f)
#define EXAMPLE_SINE_WAVE_LEN(tone)     (uint32_t)((EXAMPLE_PDM_TX_FREQ_HZ / (float)tone) + 0.5) // The sample point number per sine wave to generate the tone
#define EXAMPLE_TONE_LAST_TIME_MS       500
#define EXAMPLE_BYTE_NUM_EVERY_TONE     (EXAMPLE_TONE_LAST_TIME_MS * EXAMPLE_PDM_TX_FREQ_HZ / 1000)

/* The frequency of tones: do, re, mi, fa, so, la, si, in Hz. */
static const uint32_t tone[3][7] = {{262, 294, 330, 349, 392, 440, 494},            // bass
                                    {523, 587, 659, 698, 784, 880, 988},            // alto
                                    {1046, 1175, 1318, 1397, 1568, 1760, 1976}};    // treble
/* Numbered musical notation of 'twinkle twinkle little star' */
static const uint8_t song[28] = {1, 1, 5, 5, 6, 6, 5,
                                 4, 4, 3, 3, 2, 2, 1,
                                 5, 5, 4, 4, 3, 3, 2,
                                 5, 5, 4, 4, 3, 3, 2};
/* Rhythm of 'twinkle twinkle little star', it's repeated in four sections */
static const uint8_t rhythm[7] = {1, 1, 1, 1, 1, 1, 2};

static const char *tone_name[3] = {"bass", "alto", "treble"};

static i2s_chan_handle_t i2s_example_init_pdm_tx(void)
{
    i2s_chan_handle_t tx_chan;        // I2S tx channel handler
    /* Setp 1: Determine the I2S channel configuration and allocate TX channel only
     * The default configuration can be generated by the helper macro,
     * it only requires the I2S controller id and I2S role,
     * but note that PDM channel can only be registered on I2S_NUM_0 */
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    tx_chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));

    /* Step 2: Setting the configurations of PDM TX mode and initialize the TX channel
     * The slot configuration and clock configuration can be generated by the macros
     * These two helper macros is defined in 'i2s_pdm.h' which can only be used in PDM TX mode.
     * They can help to specify the slot and clock configurations for initialization or re-configuring */
    i2s_pdm_tx_config_t pdm_tx_cfg = {
        .clk_cfg = I2S_PDM_TX_CLK_DEFAULT_CONFIG(EXAMPLE_PDM_TX_FREQ_HZ),
        /* The data bit-width of PDM mode is fixed to 16 */
        .slot_cfg = I2S_PDM_TX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = EXAMPLE_PDM_TX_CLK_IO,
            .dout = EXAMPLE_PDM_TX_DOUT_IO,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_pdm_tx_mode(tx_chan, &pdm_tx_cfg));

    /* Step 3: Enable the tx channel before writing data */
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));

    return tx_chan;
}

void i2s_example_pdm_tx_task(void *args)
{
    int16_t *w_buf = (int16_t *)calloc(1, EXAMPLE_BUFF_SIZE);
    assert(w_buf);
    i2s_chan_handle_t tx_chan = i2s_example_init_pdm_tx();

    size_t w_bytes = 0;

    uint8_t cnt = 0;            // The current index of the song
    uint8_t tone_select = 0;    // To selecting the tone level

    printf("Playing %s `twinkle twinkle little star`\n", tone_name[tone_select]);
    while (1) {
        int tone_point = EXAMPLE_SINE_WAVE_LEN(tone[tone_select][song[cnt]-1]);
        /* Generate the tone buffer */
        for (int i = 0; i < tone_point; i++) {
            w_buf[i] =  (int16_t)((sin(2 * (float)i * CONST_PI / tone_point)) * EXAMPLE_WAVE_AMPLITUDE);
        }
        for (int tot_bytes = 0; tot_bytes < EXAMPLE_BYTE_NUM_EVERY_TONE * rhythm[cnt % 7]; tot_bytes += w_bytes) {
            /* Play the tone */
            if (i2s_channel_write(tx_chan, w_buf, tone_point * sizeof(int16_t), &w_bytes, 1000) != ESP_OK) {
                printf("Write Task: i2s write failed\n");
            }
        }
        cnt++;
        /* If finished playing, switch the tone level */
        if (cnt == sizeof(song)) {
            cnt = 0;
            tone_select++;
            tone_select %= 3;
            printf("Playing %s `twinkle twinkle little star`\n", tone_name[tone_select]);
        }
        /* Gap between the tones */
        vTaskDelay(15);
    }
    free(w_buf);
    vTaskDelete(NULL);
}


void generate_sine_wave(float frequency) {
    float phase = -M_PI / 2; // Shift phase by 90 degrees to start at minimum value
    float sine_wave_step = 2 * M_PI / sine_wave_size;

    for (size_t i = 0; i < sine_wave_size; i++) {
        float value = sin(phase);
        int twos_comp_value = (int)(value * 127);
        sine_wave[i] = (uint8_t)(twos_comp_value < 0 ? twos_comp_value + 0xFF : twos_comp_value);

        ESP_LOGI(TAG, "sine_wave[%i]=%x",i,sine_wave[i]);

        phase += sine_wave_step;
        if (phase >= 2 * M_PI) {
            phase -= 2 * M_PI;
        }
    }
}

void spi_task(void *arg) {
    spi_bus_config_t bus_config = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCK,

        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = 10 * 1000,
        .mode = 0,                              // SPI mode 0
        .spics_io_num = SPI_CS,
        .queue_size = 1,
    };

    spi_device_handle_t spi;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_BUS, &bus_config, 0));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_BUS, &dev_config, &spi));

    // gpio_set_direction(SPI_CS, GPIO_MODE_OUTPUT);
    // gpio_set_level(SPI_CS, 0);

    while (1) {
        for (int i = 0; i < 20; i++) {
                
            int byte_index;
            uint8_t byte_mask;
            uint8_t tx_data[4] = {0x00, 0x00, 0x00, 0x00};
            if (i < 2) {
                byte_index = 0;
                byte_mask = 1 << (1 - i);
            } else if (i < 10) {
                byte_index = 1;
                byte_mask = 1 << (9 - i);
            } else if (i < 18){
                byte_index = 2;
                byte_mask = 1 << (17 - i);
            } else {
                byte_index = 3;
                byte_mask = 1 << (25 - i);
            }
            tx_data[byte_index] = byte_mask;            
           
            spi_transaction_t trans = {
                .length = 4 * 8, // Length in bits
                .tx_buffer = tx_data,
            };

            ESP_LOGI(TAG, "shift registers pin %d data %x %x %x %x ", i, tx_data[0], tx_data[1], tx_data[2], tx_data[3]);
            spi_device_transmit(spi, &trans);
            
            // gpio_set_level(SPI_CS, 1);
            // vTaskDelay(pdMS_TO_TICKS(1));
            // gpio_set_level(SPI_CS, 0);

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}


void writer_task(void *pvParameters) {
    size_t sine_wave_index = 0;

    while (1) {
        // bool fifo_full = true;
        // do {   
        //    fifo_full = drv_read_register(DRV_REGISTER_0) & DRV_FIFO_FULL_MASK;
        //    if (fifo_full) {
        //        vTaskDelay(pdMS_TO_TICKS(1));
        //    }
        //} while (fifo_full);
        size_t write_size = 50;
        uint8_t buffer[write_size];
        for (size_t i = 0; i < write_size; i++) {
            buffer[i] = sine_wave[sine_wave_index];
            sine_wave_index = (sine_wave_index + 1) % sine_wave_size;
        }
        drv_write_fifo(buffer, write_size);
        // Calculate write delay ticks based on write_size and sample rate
        TickType_t write_delay_ticks = (write_size / SAMPLE_RATE)* portTICK_PERIOD_MS * 1000;
        vTaskDelay(write_delay_ticks);   

        //drv_write_register(DRV_REGISTER_DATA, sine_wave[sine_wave_index]);
        //sine_wave_index = (sine_wave_index + 1) % sine_wave_size;

    }
    vTaskDelete(NULL);
}

void app_main(void) {

    float frequency = 250.0; // Desired frequency in Hz
    sine_wave_size = (size_t)((SAMPLE_RATE / frequency) + 0.5); // Rounded up

    generate_sine_wave(frequency);

    gpio_reset_pin(GPIO_NUM_4);
    gpio_set_pull_mode(GPIO_NUM_4, GPIO_FLOATING);
    gpio_reset_pin(GPIO_NUM_5);
    gpio_set_pull_mode(GPIO_NUM_5, GPIO_FLOATING);

    ESP_LOGI(TAG, "Connecting to peripherals");
    drv_init();
    
    ESP_LOGI(TAG, "DRV2665 initialized successfully");
    // drv_enable_digital();
    drv_enable_analog();
    xTaskCreate(writer_task, "writer_task", configMINIMAL_STACK_SIZE * 4, NULL, 1, NULL);
    
    xTaskCreate(spi_task, "spi_task", 4096, NULL, 10, NULL);
    
    // printf("I2S PDM TX example start\n---------------------------\n");
    // xTaskCreate(i2s_example_pdm_tx_task, "i2s_example_pdm_tx_task", 4096, NULL, 5, NULL);

    while(1) {
        vTaskDelay(1000*portTICK_PERIOD_MS);
    }

}
