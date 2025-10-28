#include "driver/twai.h"
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "servosila_sc.h"
#include "esp_log.h"
#include <math.h>
#include "servosila_codes.h"

static const char *TAG = "servo";

#define TIMEOUT_MS  (100)

#define MAX_FLOAT16 ( 128.0f)
#define MIN_FLOAT16 (-MAX_FLOAT16)
float SC25_decode_float16 (int16_t bits)
{
  //de-scaling
  const float f32 = ((float)(bits)) * (MAX_FLOAT16 / 32767.0f); //LINEAR SCALING FORMULA
  return f32;
}

int16_t SC25_encode_float16 (float f32)
{
  //clipping the value
  if(f32 > MAX_FLOAT16) f32 = MAX_FLOAT16;
  if(f32 < MIN_FLOAT16) f32 = MIN_FLOAT16;
  //scaling
  const int16_t bits = lroundf(f32 * (32767.0f / MAX_FLOAT16)); //LINEAR SCALING FORMULA
  return bits;
}

esp_err_t sc_cmd_STOP(const uint32_t id, TickType_t timeout)
{
    const twai_message_t message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 1,                // Is single shot (won't retry on error or NACK)
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8

    // Message ID and payload
    .identifier = id + SC25_COB_STOP,
    .data_length_code = 1,
    .data = {SC25_CMD_STOP, 0, 0, 0, 
             0, 0, 0, 0}
    };

    return twai_transmit(&message, timeout);
}

esp_err_t sc_cmd_RESET(const uint32_t id, TickType_t timeout)
{
    const twai_message_t message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 1,                // Is single shot (won't retry on error or NACK)
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8

    // Message ID and payload
    .identifier = id + SC25_COB_RESET,
    .data_length_code = 1,
    .data = {SC25_CMD_RESET, 0, 0, 0, 
             0, 0, 0, 0}
    };

    return twai_transmit(&message, timeout);
}

esp_err_t sc_cmd_RESET_WORKZONE(const uint32_t id, TickType_t timeout)
{
    const twai_message_t message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 1,                // Is single shot (won't retry on error or NACK)
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8

    // Message ID and payload
    .identifier = id + SC25_COB_RESET_WORKZONE,
    .data_length_code = 1,
    .data = {SC25_CMD_RESET_WORKZONE, 0, 0, 0, 
             0, 0, 0, 0}
    };

    return twai_transmit(&message, timeout);
}

esp_err_t sc_cmd_ESC_HZ(const uint32_t id, const float freq, TickType_t timeout)
{
    const twai_message_t message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 1,                // Is single shot (won't retry on error or NACK)
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8

    // Message ID and payload
    .identifier = id + SC25_COB_ESC_HZ,
    .data_length_code = 8,
    .data = {SC25_CMD_ESC_HZ, 0, 0, 0, 
             *((uint8_t*)&freq), *((uint8_t*)&freq + 1), *((uint8_t*)&freq + 2), *((uint8_t*)&freq + 3)}
    };

    return twai_transmit(&message, timeout);
};

esp_err_t sc_cmd_MODULO_COUNT(const uint32_t id, const uint32_t count, tMotionType motion_type, TickType_t timeout)
{
    const twai_message_t message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 1,                // Is single shot (won't retry on error or NACK)
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8

    // Message ID and payload
    .identifier = id + SC25_COB_MODULO_COUNT,
    .data_length_code = 8,
    .data = {SC25_CMD_MODULO_COUNT, 0, *((uint8_t*)&motion_type), *((uint8_t*)&motion_type + 1), 
             *((uint8_t*)&count), *((uint8_t*)&count + 1), *((uint8_t*)&count + 2), *((uint8_t*)&count + 3)}
    };

    return twai_transmit(&message, timeout);
};

esp_err_t sc_cmd_DFCPOS(const uint32_t id, float voltage, float pos, TickType_t timeout)
{
    uint16_t encoded_voltage = SC25_encode_float16(voltage);

    const twai_message_t message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 1,                // Is single shot (won't retry on error or NACK)
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8

    // Message ID and payload
    .identifier = id + SC25_COB_DFC_POS,
    .data_length_code = 8,
    .data = {SC25_CMD_DFC_POS, 0, encoded_voltage & 0xff, (encoded_voltage >> 8) & 0xff, 
             *((uint8_t*)&pos), *((uint8_t*)&pos + 1), *((uint8_t*)&pos + 2), *((uint8_t*)&pos + 3)}
    };

    return twai_transmit(&message, timeout);
}

esp_err_t sc_cmd_read_QUADRATURE(const uint32_t id, TickType_t timeout)
{
    const twai_message_t message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 1,                // Is single shot (won't retry on error or NACK)
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8

    // Message ID and payload
    .identifier = id + SC25_COB_READ,
    .data_length_code = 8,
    .data = {SC25_CMD_READ, SC25_TELEMETRY_INDEX_QUADRATURE_COUNT & 0xFF, (SC25_TELEMETRY_INDEX_QUADRATURE_COUNT >> 8) & 0xFF, SC25_TELEMETRY_SUB_INDEX_QUADRATURE_COUNT, 
             0, 0, 0, 0}
    };

    return twai_transmit(&message, timeout);
}

bool sc_decode_QUADRATURE(uint32_t id, const twai_message_t msg, uint32_t *pCount)
{
  if (msg.identifier != (id + SC25_COB_READ_RESPONSE)) return false;
  if (msg.data[0] != (SC25_CMD_RESPONSE_4B)) return false;

  if ((msg.data[1] != (SC25_TELEMETRY_INDEX_QUADRATURE_COUNT & 0xFF)) ||
      (msg.data[2] != ((SC25_TELEMETRY_INDEX_QUADRATURE_COUNT >> 8) & 0xFF)) || 
      (msg.data[3] != SC25_TELEMETRY_SUB_INDEX_QUADRATURE_COUNT)) return false;

  *((uint8_t*)pCount) = msg.data[4];
  *((uint8_t*)pCount + 1) = msg.data[5];
  *((uint8_t*)pCount + 2) = msg.data[6];
  *((uint8_t*)pCount + 3) = msg.data[7];

  return true;
}

esp_err_t sc_cmd_write_ENCODER_BIAS(const uint32_t id, uint32_t count, TickType_t timeout)
{
    const twai_message_t message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 1,                // Is single shot (won't retry on error or NACK)
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8

    // Message ID and payload
    .identifier = id + SC25_COB_WRITE,
    .data_length_code = 8,
    .data = {SC25_CMD_WRITE, SC25_CONFIG_INDEX_ENCODER_BIAS & 0xFF, (SC25_CONFIG_INDEX_ENCODER_BIAS >> 8) & 0xFF, SC25_CONFIG_SUB_INDEX_ENCODER_BIAS, 
             *((uint8_t*)&count), *((uint8_t*)&count + 1), *((uint8_t*)&count + 2), *((uint8_t*)&count + 3)}
    };

    return twai_transmit(&message, timeout);
}

esp_err_t sc_init_encoder_bias(const uint32_t id, float voltage, TickType_t settle_time, TickType_t timeout)
{
    esp_err_t err;
    
    err = sc_cmd_STOP(id, timeout);
    if (err != ESP_OK) return err; 

    vTaskDelay(settle_time);
    err = sc_cmd_RESET_WORKZONE(id, timeout);
    if (err != ESP_OK) return err; 

    err = sc_cmd_write_ENCODER_BIAS(id, 0, timeout);
    if (err != ESP_OK) return err; 

    err = sc_cmd_DFCPOS(id, voltage, 0, timeout);
    if (err != ESP_OK) return err; 

    vTaskDelay(settle_time);
    err = sc_cmd_read_QUADRATURE(id, timeout);
    if (err != ESP_OK) return err; 

    uint32_t count = 0;
    bool flag_received_quadrature = false;
    for (int i = 0; (i < 100) && !flag_received_quadrature; i++)
    {
        twai_message_t msg;
        if (ESP_OK == twai_receive(&msg, timeout))
        {
            if (sc_decode_QUADRATURE(id, msg, &count))
            {
                flag_received_quadrature = true;
            } 
        }
        else break;
    }
    if (flag_received_quadrature) 
    {
        ESP_LOGI(TAG, "Recevied encoder value %ld", count);
        err = sc_cmd_write_ENCODER_BIAS(id, count, timeout);
        if (err != ESP_OK) return err; 
    }

    return ESP_OK;
}

esp_err_t sc_cmd_read_MODULOCOUNT(const uint32_t id, TickType_t timeout)
{
    const twai_message_t message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 1,                // Is single shot (won't retry on error or NACK)
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8

    // Message ID and payload
    .identifier = id + SC25_COB_READ,
    .data_length_code = 8,
    .data = {SC25_CMD_READ, SC25_TELEMETRY_INDEX_MODULO_COUNT & 0xFF, (SC25_TELEMETRY_INDEX_MODULO_COUNT >> 8) & 0xFF, SC25_TELEMETRY_SUB_INDEX_MODULO_COUNT, 
             0, 0, 0, 0}
    };

    return twai_transmit(&message, timeout);
}

bool sc_decode_MODULO_COUNT(uint32_t id, const twai_message_t msg, uint32_t *pCount)
{
  if (msg.identifier != (id + SC25_COB_READ_RESPONSE)) return false;
  if (msg.data[0] != (SC25_CMD_RESPONSE_4B)) return false;

  if ((msg.data[1] != (SC25_TELEMETRY_INDEX_MODULO_COUNT & 0xFF)) ||
      (msg.data[2] != ((SC25_TELEMETRY_INDEX_MODULO_COUNT >> 8) & 0xFF)) || 
      (msg.data[3] != SC25_TELEMETRY_SUB_INDEX_MODULO_COUNT)) return false;

  *((uint8_t*)pCount) = msg.data[4];
  *((uint8_t*)pCount + 1) = msg.data[5];
  *((uint8_t*)pCount + 2) = msg.data[6];
  *((uint8_t*)pCount + 3) = msg.data[7];

  return true;
}