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
#include <math.h>

#define HOST            SPI3_HOST
#define PIN_NUM_MISO    19
#define PIN_NUM_MOSI    23
#define PIN_NUM_CLK     18
#define PIN_NUM_SS      5


esp_err_t data(spi_device_handle_t spi, uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = 0;
    t.cmd = 0x01;
    t.addr = 0;
    t.length = len * 8;
    t.tx_buffer = 0;
    t.rxlength = t.length;
    t.rx_buffer = data;
    t.user = 0;
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
    .max_transfer_sz = 1
    };

    spi_device_interface_config_t devcfg = {
    #ifdef CONFIG_OVERCLOCK
    .clock_speed_hz = 26 * 1000 * 1000,
    #else
    .clock_speed_hz = 1 * 1000 * 1000,
    #endif
    .mode = 0,
    .spics_io_num = PIN_NUM_SS,
    .queue_size = 1,
    .command_bits = 8,
    .address_bits = 0,
    .dummy_bits = 0,

};
    ret = spi_bus_initialize(HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    while(1)
    {
        uint8_t test_buf[8] = "00000000";
        //ESP_LOGI("MISO", "%02X %02X %02X %02X %02X %02X %02X %02X", test_buf[0],test_buf[1],test_buf[2],test_buf[3],test_buf[4],test_buf[5],test_buf[6],test_buf[7]);

        ret = data(spi, test_buf, sizeof(test_buf));
        
        if (ret == ESP_OK) {
            ESP_LOGI("MISO", "%02X %02X %02X %02X %02X %02X %02X %02X", test_buf[0],test_buf[1],test_buf[2],test_buf[3],test_buf[4],test_buf[5],test_buf[6],test_buf[7]);
            int32_t pos = ((uint32_t)test_buf[1] << 16) + ((uint32_t)test_buf[2] << 8) + (uint32_t)test_buf[3];
            uint32_t timecode = ((uint32_t)test_buf[4] << 24) + ((uint32_t)test_buf[5] << 16) + ((uint32_t)test_buf[6] << 8) + (uint32_t)test_buf[7];
            ESP_LOGI("flag, pos, angle, time", "%i %i %.2f %.2f", test_buf[0] / 128, (pos - 1048576) / 4, (((float)pos - 1048576) / 4) * 360 / 2048, (double)timecode * 20 / 1000000000 );
        }
        
        vTaskDelay(pdMS_TO_TICKS(200));  
    }

    /*while(1)
    {
        uint8_t test_buf[8];
        ret = lcd_data(spi, test_buf, sizeof(test_buf));
        ESP_ERROR_CHECK(ret);
        ESP_LOGI("transmit", "%s", test_buf);
        vTaskDelay(500);
    }
    */

}

