#include "freertos/FreeRTOS.h"
#include "driver/twai.h"

#define CAN_ID_COMMANDER                        (111)
#define CAN_ID_DRIVER                           (112)

#define MIM_CMD_GROUP_CONFIG                    (001)
#define MIM_CMD_GROUP_TELEMETRY                 (002)
#define MIM_CMD_GROUP_MOTION_CONTROL            (003)
#define MIM_CMD_GROUP_NVS                       (004)

// Group NVS

#define MIM_CMD_SAVE_TO_FLASH                   (001)
#define MIM_CMD_READ_FROM_FLASH                 (002)

// Group Config

#define MIM_CMD_SEND_CONFIG                     (000)
#define MIM_CMD_READ_DFC_SPEED                  (001)
#define MIM_CMD_READ_DFC_CURRENT                (002)
#define MIM_CMD_READ_FOC_CURRENT                (003)
#define MIM_CMD_READ_ESC_GOAL                   (004)
#define MIM_CMD_READ_SERVO_GOAL                 (005)
#define MIM_CMD_READ_END_OF_CONFIG              (255)

// Group Telemetry

#define MIM_CMD_SEND_TELEMETRY                  (000)
#define MIM_CMD_READ_TIME                       (001)
#define MIM_CMD_READ_POS                        (002)
#define MIM_CMD_READ_SPEED                      (003)
#define MIM_CMD_READ_END_OF_TELEMETRY           (255)

// Group Motion Control

#define MIM_CMD_STOP                            (000)
#define MIM_CMD_CALIBRATION                     (001)
#define MIM_CMD_DFC                             (002)
#define MIM_CMD_FOC                             (003)
#define MIM_CMD_ESC                             (004)
#define MIM_CMD_SERVO                           (005)



esp_err_t mim_cmd_SEND_INT16(uint32_t id, uint8_t cmd_group, uint8_t cmd_id, int16_t value, TickType_t timeout)
{
    twai_message_t msg = {
        .extd = 0,              
        .rtr = 0,               
        .ss = 1,                
        .self = 0,              
        .dlc_non_comp = 0,

        .identifier = id,
        .data_length_code = 6,

        .data = {cmd_group, cmd_id, 0, 0, *(uint8_t*)&value, *((uint8_t*)&value + 1)}
        
    };
    return twai_transmit(&msg, timeout);
}

esp_err_t mim_cmd_SEND_UINT32(uint32_t id, uint8_t cmd_group, uint8_t cmd_id, uint32_t value, TickType_t timeout)
{
    twai_message_t msg = {
        .extd = 0,              
        .rtr = 0,               
        .ss = 1,                
        .self = 0,              
        .dlc_non_comp = 0,

        .identifier = id,
        .data_length_code = 6,

        .data = {cmd_group, cmd_id, *(uint8_t*)&value, *((uint8_t*)&value + 1), *((uint8_t*)&value + 2), *((uint8_t*)&value + 3)}
        
    };
    return twai_transmit(&msg, timeout);
}

esp_err_t mim_cmd_SEND_FLOAT(uint32_t id, uint8_t cmd_group, uint8_t cmd_id, float value, TickType_t timeout)
{
    twai_message_t msg = {
        .extd = 0,              
        .rtr = 0,               
        .ss = 1,                
        .self = 0,              
        .dlc_non_comp = 0,

        .identifier = id,
        .data_length_code = 6,

        .data = {cmd_group, cmd_id, *(uint8_t*)&value, *((uint8_t*)&value + 1), *((uint8_t*)&value + 2), *((uint8_t*)&value + 3)}
        
    };
    return twai_transmit(&msg, timeout);
}


int16_t mim_DECODE_INT16(uint8_t *msg_data)
{
    int16_t value = 0;

    *(uint8_t*)(&value) = *(msg_data + 4);
    *((uint8_t*)(&value) + 1) = *(msg_data + 5);
    
    return (int16_t)value;
}

uint32_t mim_DECODE_UINT32(uint8_t *msg_data)
{
    uint32_t value = 0;

    *(uint8_t*)(&value) = *(msg_data + 2);
    *((uint8_t*)(&value) + 1) = *(msg_data + 3);
    *((uint8_t*)(&value) + 2) = *(msg_data + 4);
    *((uint8_t*)(&value) + 3) = *(msg_data + 5);
    
    return (uint32_t)value;
}

float mim_DECODE_FLOAT(uint8_t *msg_data)
{
    float value = 0;

    *(uint8_t*)(&value) = *(msg_data + 2);
    *((uint8_t*)(&value) + 1) = *(msg_data + 3);
    *((uint8_t*)(&value) + 2) = *(msg_data + 4);
    *((uint8_t*)(&value) + 3) = *(msg_data + 5);
    
    return (float)value;
}