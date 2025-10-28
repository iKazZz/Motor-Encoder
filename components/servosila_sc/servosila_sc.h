#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"

struct twai_message_t;

#ifdef __cplusplus
extern "C" {
#endif

typedef enum 
{
    MOTION_NORMAL = 0,
    MOTION_NEGATIVE = 1,
    MOTION_POSITIVE = 2,
    MOTION_SHORTEST = 3
} tMotionType;

esp_err_t sc_cmd_RESET(const uint32_t id, TickType_t timeout);
esp_err_t sc_cmd_RESET_WORKZONE(const uint32_t id, TickType_t timeout);
esp_err_t sc_cmd_STOP(const uint32_t id, TickType_t timeout);
esp_err_t sc_cmd_ESC_HZ(uint32_t id, const float freq, TickType_t timeout);
esp_err_t sc_cmd_DFCPOS(const uint32_t id, float voltage, float pos, TickType_t timeout);
esp_err_t sc_cmd_read_QUADRATURE(const uint32_t id, TickType_t timeout);
bool sc_decode_QUADRATURE(uint32_t id, const twai_message_t msg, uint32_t *pCount);
esp_err_t sc_cmd_write_ENCODER_BIAS(const uint32_t id, uint32_t count, TickType_t timeout);
esp_err_t sc_init_encoder_bias(const uint32_t id, float voltage, TickType_t settle_time, TickType_t timeout);
esp_err_t sc_cmd_read_MODULOCOUNT(const uint32_t id, TickType_t timeout);
bool sc_decode_MODULO_COUNT(uint32_t id, const twai_message_t msg, uint32_t *pCount);
esp_err_t sc_cmd_MODULO_COUNT(const uint32_t id, const uint32_t count, tMotionType motion_type, TickType_t timeout);

#ifdef __cplusplus
}
#endif
