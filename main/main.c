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

QueueHandle_t g_command_queue;
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

uint32_t left = 0;
uint32_t right = 0;
int32_t wz_count = 0;
uint32_t count = 0;
uint32_t time_count = 0;
uint16_t feed_rate = 50000;

int encoder_pos;
bool g_flag_send_telemetry;

void nvs_read_config();
void nvs_write_config();

void append_telemetry_data(cJSON *json)
{
    cJSON_AddStringToObject(json, "telemetry", "true");
    cJSON_AddNumberToObject(json, "sample_count", count);
    cJSON_AddNumberToObject(json, "time_count", time_count);
    cJSON_AddNumberToObject(json, "wz_count", wz_count);
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

            if (!strcmp(param_name, STR_FLAG_SEND_TELEMETRY)) g_flag_send_telemetry = subitem->valueint;
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

void nvs_read_config()
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_STORAGE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error nvs_open %s", esp_err_to_name(err));
    }
    else
    {
        size_t len = 0;
        err = nvs_get_str(nvs_handle, NVS_STR_CONFIG, NULL, &len);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error nvs_get_str %s", esp_err_to_name(err));
        }

        if (len) 
        {
            char *str = malloc(len);
            if (!str) 
            {
                ESP_LOGE(TAG, "Failed to alloc str, len=%d", len);
                nvs_close(nvs_handle);
                return;
            }

            err = nvs_get_str(nvs_handle, "config", str, &len);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error nvs_get_str %s", esp_err_to_name(err));
            }

            ESP_LOGI(TAG, "Read config %s", str);
            parse_config_string(str);

            free(str);
        }

        nvs_close(nvs_handle);
    }    
}

void nvs_write_config()
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_STORAGE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error nvs_open %s", esp_err_to_name(err));
    }
    else
    {
        char *str = build_config_string(true);
        if (str && strlen(str))
        {
            ESP_LOGI(TAG, "Write config %s", str);
            err = nvs_set_str(nvs_handle, NVS_STR_CONFIG, str);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error nvs_set_str %s", esp_err_to_name(err));
            }

            err = nvs_commit(nvs_handle);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error nvs_commit %s", esp_err_to_name(err));
            }

            free(str);
        }

        nvs_close(nvs_handle);
    }
}

bool calibrate()
{
    static const char *TAG = "calibrate";
    uint32_t quadrature_count;
    bool flag_received_quadrature = false;
    bool flag_received_wz = false;
    twai_message_t msg;

    esp_err_t err;
    ESP_LOGI(TAG, "Calibration started");
    for (int i = 0; i < 10; i++)
    {
        err = sc_cmd_DFCPOS(SC_ID, 2, 0, pdMS_TO_TICKS(1000));
        if(err != ESP_OK)
        {
            ESP_LOGI(TAG, "Failed to transmit cmd (DFCPOS)");
            return false;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
    for (int i = 0; (i < 100) && !flag_received_quadrature; i++)
    {
        err = sc_cmd_read_QUADRATURE(SC_ID, pdMS_TO_TICKS(1000));
        if(err != ESP_OK)
        {
            ESP_LOGI(TAG, "Failed to transmit cmd (Quadrature)");
            return false;
        }

        if (ESP_OK == twai_receive(&msg, pdMS_TO_TICKS(1000)))
        {
            if (sc_decode_QUADRATURE(SC_ID, msg, &quadrature_count))
            {
                flag_received_quadrature = true;
                g_servo_bias = quadrature_count;
            } 
        }
    }

    if(!flag_received_quadrature)
    {
        ESP_LOGI(TAG, "Recieved smth wrong");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    err = sc_cmd_write_ENCODER_BIAS(SC_ID, g_servo_bias, pdMS_TO_TICKS(1000));
    if(err != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to transmit cmd (Write Bias)");
        return false;
    }
    else
    {
        ESP_LOGI(TAG, "Bias = %d", g_servo_bias);
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    err = sc_cmd_ESC_HZ(SC_ID, -2, pdMS_TO_TICKS(1000));
    if(err != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to transmit cmd (ESC_HZ)");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    for (int i = 0; (i < 500) && (!left); i++)
    {
        err = sc_cmd_read_LIMIT_SWITCH_NEG(SC_ID, pdMS_TO_TICKS(1000));
        if(err != ESP_OK)
        {
            ESP_LOGI(TAG, "Failed to transmit cmd (Limit Switch Neg)");
        }
        ESP_LOGI(TAG, "i = %d", i);
        if(twai_receive(&msg, 1000) == ESP_OK)
        {
            // ESP_LOGI(TAG, "Left: %02x %02x %02x %02x %02x %02x %02x %02x", msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],msg.data[5], msg.data[6],msg.data[7]);
            if(!sc_decode_LIMIT_SWITCH_NEG(SC_ID, msg, &left))
            {
                ESP_LOGI(TAG, "Failed to decode (Limit Switch Neg)");
            }
            else
            {
                ESP_LOGI(TAG, "left = %d", left);
            }
        }
        if (i % 100 == 0)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    if(left != 1)
    {
        ESP_LOGI(TAG, "Failed to get (Limit Switch Neg)");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));

    err = sc_cmd_write_WZ_OFFSET(SC_ID, 0, pdMS_TO_TICKS(1000));
    if(err != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to transmit cmd (WZ Offset)");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    for (int i = 0; (i < 100) && !flag_received_wz; i++)
    {
        err = sc_cmd_read_WORKZONE_COUNT(SC_ID, pdMS_TO_TICKS(1000));
        if(err != ESP_OK)
        {
            ESP_LOGI(TAG, "Failed to transmit cmd (Workzone)");
        }

        if (ESP_OK == twai_receive(&msg, pdMS_TO_TICKS(1000)))
        {
            if (sc_decode_WORKZONE_COUNT(SC_ID, msg, &wz_count))
            {
                flag_received_wz = true;
                err = sc_cmd_write_WZ_OFFSET(SC_ID, wz_count, pdMS_TO_TICKS(1000));
                if(err != ESP_OK)
                {
                    ESP_LOGI(TAG, "Failed to transmit cmd (WZ Offset)");
                    return false;
                }
            } 
        }
    }

    return true;
}


void app_main(void)
{
    static const char *TAG = "app_main";
    esp_err_t err = nvs_flash_init();

    nvs_read_config();
    t_eth_config config = {.ip = g_ip_addr, .pass = g_pass, .ssid = g_ssid, .use_eth = USE_COMMM_ETHERNET, .use_wifi_ap = USE_COMMM_WIFI_AP, .use_wifi_sta = USE_COMMM_WIFI_STA};
    eth_start(config);

    g_command_queue = xQueueCreate(QUEUE_SIZE, COMMAND_MAX_SIZE);
    assert(g_command_queue != NULL);

    if (USE_COMMM_ETHERNET || USE_COMMM_WIFI_AP || USE_COMMM_WIFI_STA)
    {
        xTaskCreate(command_processing_task, "command_processor", 4096, NULL, 1, NULL);
        xTaskCreate(udp_server_task, "udp_server", 4096, (void *)AF_INET, 1, NULL);
    }

    err = twai_driver_install(&g_config, &t_config, &f_config);
    if(err != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to install driver");
    }
    else
    {
        ESP_LOGI(TAG, "Driver installed succesfully");
    }
    err = twai_start();
    if(err != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to start");
    }
    else
    {
        ESP_LOGI(TAG, "Started succesfully");
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
    bool calibrated_flag = calibrate();
    if(calibrated_flag)
    {
        ESP_LOGI("app_main", "Calibrated succesfully");
    }

    int counter = 0;
    bool dir = 1;
    int telemetry_counter = 0;
    bool g_flag_send_telemetry = true;

    twai_message_t msg;
    while(1)
    {
        static const char *TAG = "while";
 
        
        // err = sc_cmd_read_SAMPLE_NUM(SC_ID, 0);
        // if(err != ESP_OK)
        // {
        //     ESP_LOGI(TAG, "Failed to transmit cmd (Sample)");
        // }

        // if(twai_receive(&msg, 0) != ESP_OK)
        // {
        //     ESP_LOGI(TAG, "Failed to recieve");
        // }
        // else
        // {
        //     if(!sc_decode_SAMPLE_NUM(SC_ID, msg, &count))
        //     {
        //         sc_decode_WORKZONE_COUNT(SC_ID, msg, &wz_count);
        //     }
        // }

        // err = sc_cmd_read_WORKZONE_COUNT(SC_ID, 0);
        // if(err != ESP_OK)
        // {
        //     ESP_LOGI(TAG, "Failed to transmit cmd (Sample)");
        // }

        // if(twai_receive(&msg, 0) != ESP_OK)
        // {
        //     ESP_LOGI(TAG, "Failed to recieve");
        // }
        // else
        // {
        //     if(!sc_decode_WORKZONE_COUNT(SC_ID, msg, &wz_count))
        //     {
        //         sc_decode_SAMPLE_NUM(SC_ID, msg, &count);
        //     }
        // }

        // // ESP_LOGI("while", "Encoder: %02x %02x %02x %02x %02x %02x %02x %02x", msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],msg.data[5], msg.data[6],msg.data[7]);
        
        if(calibrated_flag)
        {
            if((counter < 33) && (dir == 1))
            {
                if (counter % 3 == 0)
                {
                    err = sc_cmd_SERVO_LINEAR(SC_ID, 0, feed_rate, 30000, 0);
                    if(err != ESP_OK)
                    {
                        ESP_LOGI("servo", "Failed to transmit cmd (Servo)");
                    }
                }
                counter++;
                
            }
            else if((counter > 0) && (dir == 0))
            {
                if (counter % 3 == 0)
                {
                    err = sc_cmd_SERVO_LINEAR(SC_ID, 0, feed_rate, 0, 0);
                    if(err != ESP_OK)
                    {
                        ESP_LOGI("servo", "Failed to transmit cmd (Servo)");
                    }
                }
                counter--;
            }
            else
            {
                dir = !dir;
            }

            // if (telemetry_counter++ >= 5)
            // {
            //     err = sc_cmd_read_WORKZONE_COUNT(SC_ID, 0);
            //     if(err != ESP_OK)
            //     {
            //         ESP_LOGI(TAG, "Failed to transmit cmd (Workzone)");
            //     }

            //     if(twai_receive(&msg, 0) != ESP_OK)
            //     {
            //         ESP_LOGI(TAG, "Failed to recieve");
            //     }
            //     else
            //     {
            //         if(sc_decode_WORKZONE_COUNT(SC_ID, msg, &count))
            //         {
            //             ESP_LOGI(TAG, "Workzone: %i", count);
            //             ESP_LOGI(TAG, "Workzone: %02x %02x %02x %02x %02x %02x %02x %02x\n", msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],msg.data[5], msg.data[6],msg.data[7]);
            //         }
            //     }
            //     telemetry_counter = 0;
            // }
        }
        
        if (telemetry_counter++ >= 1)
        {
            time_count++;
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
                            // ESP_LOGI(TAG, "Telemetry sent: enc_pos=%d", count);
                        }
                        close(sock);
                    }
                    else
                    {
                        // ESP_LOGI("11", "11");
                    }
                    free(telemetry_to_send);

                }
                else
                {
                    // ESP_LOGI("22", "4");
                }

            }
            telemetry_counter = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}