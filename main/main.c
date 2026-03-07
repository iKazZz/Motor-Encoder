#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "esp_log.h"
#include "ethernet.h"
#include "udp_server.h"
#include "main_config.h"
#include "sdkconfig.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/twai.h"
#include "servosila_sc.h"
#include <cJSON.h>
#include <math.h>
#include "driver/twai.h"
#include "servosila_sc.c"

#ifndef max
    #define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
    #define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#define SC_ID       (12)

#define TIMEOUT_MS  (100)

#define MAX_FLOAT16 ( 128.0f)
#define MIN_FLOAT16 (-MAX_FLOAT16)

uint32_t g_servo_bias;
uint32_t SERVO_TIMEOUT = pdMS_TO_TICKS(1000);
uint32_t g_servoCPR = 60000;
uint32_t DEFAULT_SERVO_POLEPAIRS = 20;
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_35, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();


bool calibrate()
{
    esp_err_t err;
    err = sc_cmd_DFCPOS(SC_ID, 2, 0, pdMS_TO_TICKS(1000));
    if(err != ESP_OK)
    {
        ESP_LOGI("Calibration", "Failed to transmit cmd (DFCPOS)");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    err = sc_cmd_read_QUADRATURE(SC_ID, pdMS_TO_TICKS(1000));
    if(err != ESP_OK)
    {
        ESP_LOGI("Calibration", "Failed to transmit cmd (Quadrature)");
        return false;
    }

    uint32_t count;
    bool flag_received_quadrature = false;
    for (int i = 0; (i < 100) && !flag_received_quadrature; i++)
    {
        twai_message_t msg;
        if (ESP_OK == twai_receive(&msg, pdMS_TO_TICKS(1000)))
        {
            if (sc_decode_QUADRATURE(SC_ID, msg, &count))
            {
                flag_received_quadrature = true;
                g_servo_bias = count;
            } 
        }
        else break;
    }

    if(!flag_received_quadrature)
    {
        ESP_LOGI("Calibration", "Recieved smth wrong");
        return false;
    }

    err = sc_cmd_write_ENCODER_BIAS(SC_ID, count, pdMS_TO_TICKS(1000));
    if(err != ESP_OK)
    {
        ESP_LOGI("Calibration", "Failed to transmit cmd (Write Bias)");
        return false;
    }

    err = sc_cmd_STOP(SC_ID, pdMS_TO_TICKS(1000));
    if(err != ESP_OK)
    {
        ESP_LOGI("Calibration", "Failed to transmit cmd (Stop)");
    }
    return true;
}


void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(5000));
    esp_err_t err;

    err = twai_driver_install(&g_config, &t_config, &f_config);
    if(err != ESP_OK)
    {
        ESP_LOGI("app_main", "Failed to install driver");
    }
    else
    {
        ESP_LOGI("app_main", "Driver installed succesfully");
    }
    err = twai_start();
    if(err != ESP_OK)
    {
        ESP_LOGI("app_main", "Failed to start");
    }
    else
    {
        ESP_LOGI("app_main", "Started succesfully");
    }

    bool calibrated_flag = calibrate();
    if(calibrated_flag)
    {
        ESP_LOGI("app_main", "Calibrated succesfully");
    }

    // err = sc_cmd_write_ENCODER_BIAS(SC_ID, 0, pdMS_TO_TICKS(1000));
    // if(err != ESP_OK)
    // {
    //     ESP_LOGI("Calibration", "Failed to transmit cmd (Write Bias)");
    // }

    int counter = 0;
    bool dir = 1;
    uint32_t left = 0;
    uint32_t right = 0;

    twai_message_t msg;
    uint32_t count;
    while(1)
    {

        // err = sc_cmd_read_QUADRATURE(SC_ID, 1000);
        // if(err != ESP_OK)
        // {
        //     ESP_LOGI("while", "Failed to transmit cmd (Quadrature)");
        // }

        // if(twai_receive(&msg, 0) != ESP_OK)
        // {
        //     ESP_LOGI("while", "Failed to recieve");
        // }
        // else
        // {
        //     if(sc_decode_QUADRATURE(SC_ID, msg, &count))
        //     {
        //         ESP_LOGI("while", "Position: %i", count);
        //     }
        // }

        err = sc_cmd_read_LIMIT_SWITCH_NEG(SC_ID, 1000);
        if(err != ESP_OK)
        {
            ESP_LOGI("while", "Failed to transmit cmd (Limit Switch Neg)");
        }

        if(twai_receive(&msg, 0) != ESP_OK)
        {
            ESP_LOGI("while", "Failed to recieve");
        }
        else
        {
            if(!sc_decode_LIMIT_SWITCH_NEG(SC_ID, msg, &left))
            {
                ESP_LOGI("while", "Failed to decode Switch Neg");
            };
        }

        err = sc_cmd_read_LIMIT_SWITCH_POS(SC_ID, 1000);
        if(err != ESP_OK)
        {
            ESP_LOGI("while", "Failed to transmit cmd (Limit Switch Pos)");
        }

        if(twai_receive(&msg, 0) != ESP_OK)
        {
            ESP_LOGI("while", "Failed to recieve");
        }
        else
        {
            if(!sc_decode_LIMIT_SWITCH_POS(SC_ID, msg, &right))
            {
                ESP_LOGI("while", "Failed to decode Switch Pos");
            };
        }
        ESP_LOGI("while", "Left = %d, Right = %d", left, right);

        if(calibrated_flag)
        {
            // err = sc_cmd_ESC_HZ(SC_ID, 1, pdMS_TO_TICKS(1000));
            // if(err != ESP_OK)
            // {
            //     ESP_LOGI("while", "Failed to transmit cmd (HZ)");
            // }

            // if(counter < 100 && dir == 1)
            // {
            //     err = sc_cmd_SERVO(SC_ID, 40000, 1000);
            //     if(err != ESP_OK)
            //     {
            //         ESP_LOGI("servo", "Failed to transmit cmd (Servo)");
            //     }
            //     counter++;
            // }
            // else if(counter > 0 && dir == 0)
            // {
            //     err = sc_cmd_SERVO(SC_ID, 0, 1000);
            //     if(err != ESP_OK)
            //     {
            //         ESP_LOGI("servo", "Failed to transmit cmd (Servo)");
            //     }
            //     counter--;
            // }
            // else
            // {
            //     dir = !dir;
            // }
        }
        
        
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}