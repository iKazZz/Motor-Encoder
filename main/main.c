#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <unistd.h>
#include "esp_log.h"
#include <sys/param.h>
#include "sdkconfig.h"

#define HOST            SPI2_HOST
#define PIN_NUM_MISO    19
#define PIN_NUM_MOSI    23
#define PIN_NUM_CLK     18
#define PIN_NUM_SS      5


esp_err_t lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    // if (len == 0){
    //     return;
    // }
    memset(&t, 0, sizeof(t));
    t.length = len * 8;
    t.tx_buffer = data;
    t.user = (void*)1;
    ret = spi_device_polling_transmit(spi, &t);
    //assert(ret == ESP_OK);
    return ret;
}

void app_main(void)
{
    esp_err_t ret;
    spi_device_handle_t spi;
spi_bus_config_t buscfg = {
    .miso_io_num = PIN_NUM_MISO,
    .mosi_io_num = PIN_NUM_MOSI,
    .sclk_io_num = PIN_NUM_CLK,
    .quadhd_io_num = -1,
    .quadwp_io_num = -1,
    .max_transfer_sz = 4 * 320 * 2 + 8
};
spi_device_interface_config_t devcfg = {
    #ifdef CONFIG_OVERCLOCK
    .clock_speed_hz = 26 * 1000 * 1000,
    #else
    .clock_speed_hz = 10 * 1000 * 1000,
    #endif
    .mode = 0,
    .spics_io_num = PIN_NUM_SS,
    .queue_size = 7
};
    ret = spi_bus_initialize(HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    while(1)
    {
        uint8_t test_buf[64] = "";
        for (int i = 0; i < 64; i++)
        {
            ret = lcd_data(spi, &test_buf[i], 8);
            ESP_ERROR_CHECK(ret);
            ESP_LOGI("transmit", "%s", test_buf);
        }
        vTaskDelay(500);
    }

}

