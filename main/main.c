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

struct sockaddr_storage g_last_cmd_source_addr;
char g_ip_addr[64] = DEFAULT_STATIC_IP_ADDR;
char g_ssid[64] = DEFAULT_WIFI_STA_SSID;
char g_pass[64] = DEFAULT_WIFI_STA_PASS;

uint32_t g_servo_bias;
uint32_t SERVO_TIMEOUT = pdMS_TO_TICKS(1000);
uint32_t g_servoCPR = 60000;
uint32_t DEFAULT_SERVO_POLEPAIRS = 20;
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_35, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();


void append_telemetry_data(cJSON *json)
{
    cJSON_AddStringToObject(json, STR_TELEMETRY, "true");
    //cJSON_AddNumberToObject(json, "graph_count", graph_count);
    cJSON_AddNumberToObject(json, "encoder_pos", encoder_pos);
}

char* build_telemetry_string()
{
    cJSON *json = cJSON_CreateObject();

    append_telemetry_data(json);

    char* str = cJSON_Print(json);
    cJSON_Delete(json);
    return str;
}

char* build_config_string(bool for_nvs)
{
    cJSON *json = cJSON_CreateObject();

    cJSON_AddStringToObject(json, STR_IP_ADDR, g_ip_addr);
    cJSON_AddStringToObject(json, STR_SSID, g_ssid);
    cJSON_AddStringToObject(json, STR_PASS, g_pass);
    cJSON_AddNumberToObject(json, STR_PAUSE, g_pause_ms);
    cJSON_AddNumberToObject(json, STR_CALIBRATION_TIMEOUT, g_calibration_timeout_ms);
    cJSON_AddNumberToObject(json, STR_FLAG_SEND_TELEMETRY, g_flag_send_telemetry);
    cJSON_AddNumberToObject(json, "goal_pos", goal_pos);
    cJSON_AddNumberToObject(json, "kp", kp);
    cJSON_AddNumberToObject(json, "ki", ki);
    cJSON_AddNumberToObject(json, "kd", kd);
    cJSON_AddNumberToObject(json, "u_integral_max", u_integral_max);
    cJSON_AddNumberToObject(json, "min_freq", min_freq);

    if (!for_nvs)
    {
    }

    char* str = cJSON_Print(json);
    cJSON_Delete(json);
    return str;
}

void parse_config_string(const char *str)
{
    cJSON *parsed_cmd = cJSON_Parse(str);
    if (!parsed_cmd)
    {
        ESP_LOGE(TAG, "Failed to parse config string");
    }
    else
    {
        bool flag_got_command = false;
        if (cJSON_HasObjectItem(parsed_cmd, "type"))
        {
            cJSON *type = cJSON_GetObjectItem(parsed_cmd, "type");
            char *type_str = cJSON_GetStringValue(type);
            if (!type_str) ESP_LOGE(TAG, "Invalid type field");
            else 
            {
                ESP_LOGI(TAG, "Type field: %s", type_str);
                if (!strcmp(type_str, "cmd")) flag_got_command = true;
            }
        }
        
        for (int i=0; i < cJSON_GetArraySize(parsed_cmd); i++)
        {   
            cJSON *subitem = cJSON_GetArrayItem(parsed_cmd, i);

            ESP_LOGI(TAG, "Item %s", subitem->string);
            
            char *param_name = subitem->string;

            if (!strcmp(param_name, STR_IP_ADDR)) strncpy(g_ip_addr, subitem->valuestring, sizeof(g_ip_addr)-1);
            if (!strcmp(param_name, STR_SSID)) strncpy(g_ssid, subitem->valuestring, sizeof(g_ssid)-1);
            if (!strcmp(param_name, STR_PASS)) strncpy(g_pass, subitem->valuestring, sizeof(g_pass)-1);
            if (!strcmp(param_name, STR_PAUSE)) g_pause_ms = subitem->valueint;
            if (!strcmp(param_name, STR_CALIBRATION_TIMEOUT)) g_calibration_timeout_ms = subitem->valueint;
            if (!strcmp(param_name, STR_FLAG_SEND_TELEMETRY)) g_flag_send_telemetry = subitem->valueint;
            if (!strcmp(param_name, "goal_pos"))
            {
                if (graph_count != subitem->valueint)
                {
                    int i = 0;
                    while (i < GRAPH_ARRAY_SIZE)
                    {
                        char str[10];
                        itoa(time_arr[i], str, 10);
                        time_arr[i] = 0;
                        encoder_pos_arr[i] = 0;
                        i++;
                    }
                    graph_count = 0;
                }
                goal_pos = subitem->valueint;
            } 
            if (!strcmp(param_name, "kp")) kp = subitem->valuedouble;
            if (!strcmp(param_name, "ki")) ki = subitem->valuedouble;
            if (!strcmp(param_name, "kd")) kd = subitem->valuedouble;
            if (!strcmp(param_name, "u_integral_max")) u_integral_max = subitem->valuedouble;
            if (!strcmp(param_name, "goal_pos")) goal_pos = subitem->valueint;

            if (!strcmp(param_name, STR_CMD_READ_FLASH) && subitem->valueint) nvs_read_config();
            if (!strcmp(param_name, STR_CMD_WRITE_FLASH) && subitem->valueint) nvs_write_config();
        }
    }

    cJSON_Delete(parsed_cmd);
}

void command_processing_task(void *pvParameters)
{
    t_command cmd;
    while (1)
    {
        ESP_LOGI(TAG, "Will wait command");
        if (!xQueueReceive(g_command_queue, &cmd, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Failed to xQueueReceive");
        }
        else
        {
            ESP_LOGI(TAG, "Got command %s", cmd.cmd);            
            g_last_cmd_source_addr = cmd.source_addr;
            parse_config_string(cmd.cmd);
            
            char *config_to_send = build_config_string(false);            
            if (config_to_send)
            {
                ESP_LOGI(TAG, "Will send config %s", config_to_send);
                int err = sendto(cmd.sock, config_to_send, strlen(config_to_send), 0, (struct sockaddr *)&cmd.source_addr, sizeof(struct sockaddr));
                if (err < 0) 
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
            }
            free(config_to_send);

            char *telemetry_to_send = build_telemetry_string();            
            if (telemetry_to_send)
            {
                //ESP_LOGI(TAG, "Will send telemetry %s", telemetry_to_send);
                int err = sendto(cmd.sock, telemetry_to_send, strlen(telemetry_to_send), 0, (struct sockaddr *)&cmd.source_addr, sizeof(struct sockaddr));
                if (err < 0) 
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
            }
            free(telemetry_to_send);
        }
    }
    vTaskDelete(NULL);
}

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
    t_eth_config config = {.ip = g_ip_addr, .pass = g_pass, .ssid = g_ssid, .use_eth = USE_COMMM_ETHERNET, .use_wifi_ap = USE_COMMM_WIFI_AP, .use_wifi_sta = USE_COMMM_WIFI_STA};
    eth_start(config);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    g_command_queue = xQueueCreate(QUEUE_SIZE, COMMAND_MAX_SIZE);
    assert(g_command_queue != NULL);

    if (USE_COMMM_ETHERNET || USE_COMMM_WIFI_AP || USE_COMMM_WIFI_STA)
    {
        xTaskCreate(command_processing_task, "command_processor", 4096, NULL, 1, NULL);
        xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 1, NULL);
    }

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
    int telemetry_counter = 0;
    bool g_flag_send_telemetry = false;

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

        
            if (telemetry_counter++ >= 25)
            {
                if (g_flag_send_telemetry)
                {
                    char *telemetry_to_send = build_telemetry_string();
                    if (telemetry_to_send)
                    {
                        // Отправка телеметрии на последний известный адрес
                        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
                        if (sock >= 0)
                        {
                            int err = sendto(sock, telemetry_to_send, strlen(telemetry_to_send), 0,
                                             (struct sockaddr *)&g_last_cmd_source_addr, sizeof(struct sockaddr));
                            if (err < 0)
                            {
                                // ESP_LOGW(TAG, "Failed to send telemetry: errno %d", errno);
                            }
                            else
                            {
                                // ESP_LOGI(TAG, "Telemetry sent: enc_pos=%d", enc_pos);
                            }
                            close(sock);
                        }
                        free(telemetry_to_send);

                    }
                                // ESP_LOGI("111", "4");

                }
                telemetry_counter = 0;
            }
        
        
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}