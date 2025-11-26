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
#include "driver/spi_master.h"
#include <unistd.h>
#include <sys/param.h>
#include <math.h>
#include "main_config.h"

#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

void nvs_read_config();
void nvs_write_config();
char *build_config_string(bool for_nvs);

static const char *TAG = "step_controller";

char g_ip_addr[64] = DEFAULT_STATIC_IP_ADDR;
char g_ssid[64] = DEFAULT_WIFI_STA_SSID;
char g_pass[64] = DEFAULT_WIFI_STA_PASS;

QueueHandle_t g_command_queue;
QueueHandle_t g_spi_data_queue;
SemaphoreHandle_t g_encoder_mutex;
SemaphoreHandle_t g_pid_mutex;
SemaphoreHandle_t g_pwm_mutex;
SemaphoreHandle_t g_avg_mutex;

QueueHandle_t g_command_queue;

bool g_flag_send_telemetry = true;

struct sockaddr_storage g_last_cmd_source_addr; // TODO: mutex protect

gptimer_handle_t g_gptimer;

void append_telemetry_data(cJSON *json)
{
    cJSON_AddBoolToObject(json, "telemetry", true);
}

char *build_telemetry_string()
{
    cJSON *json = cJSON_CreateObject();

    append_telemetry_data(json);

    char *str = cJSON_Print(json);
    cJSON_Delete(json);
    return str;
}

char *build_config_string(bool for_nvs)
{
    cJSON *json = cJSON_CreateObject();

    cJSON_AddStringToObject(json, STR_IP_ADDR, g_ip_addr);

    if (!for_nvs)
    {
    }

    char *str = cJSON_Print(json);
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
            if (!type_str)
                ESP_LOGE(TAG, "Invalid type field");
            else
            {
                ESP_LOGI(TAG, "Type field: %s", type_str);
                if (!strcmp(type_str, "cmd"))
                    flag_got_command = true;
            }
        }

        for (int i = 0; i < cJSON_GetArraySize(parsed_cmd); i++)
        {
            cJSON *subitem = cJSON_GetArrayItem(parsed_cmd, i);

            ESP_LOGI(TAG, "Item %s", subitem->string);

            char *param_name = subitem->string;

            if (!strcmp(param_name, STR_IP_ADDR))
                strncpy(g_ip_addr, subitem->valuestring, sizeof(g_ip_addr) - 1);
            if (!strcmp(param_name, STR_SSID))
                strncpy(g_ssid, subitem->valuestring, sizeof(g_ssid) - 1);
            if (!strcmp(param_name, STR_PASS))
                strncpy(g_pass, subitem->valuestring, sizeof(g_pass) - 1);
            if (!strcmp(param_name, STR_FLAG_SEND_TELEMETRY))
                g_flag_send_telemetry = subitem->valueint;
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
                ESP_LOGI(TAG, "Will send telemetry %s", telemetry_to_send);
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

void myTask(void *pvParameters)
{
    while(1)
    {
        ESP_LOGI("myTask", "aboba");
        ESP_LOGI("myTask", "fifa");
    } 
    
    vTaskDelete(NULL);
}

void delayTask(void *pvParameters)
{
    while(1)
    {
        ESP_LOGI("main", "delay");
        vTaskDelay(100);
    } 
    vTaskDelete(NULL);
}

void init_pins()
{
    gpio_reset_pin(PIN_LED);
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 0);

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
    ESP_LOGI(TAG, "Will nvs_open");
    esp_err_t err = nvs_open(NVS_STORAGE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error nvs_open %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Will build_config_string");
        char *str = build_config_string(true);
        if (str && strlen(str))
        {
            ESP_LOGI(TAG, "Write config %s", str);
            err = nvs_set_str(nvs_handle, NVS_STR_CONFIG, str);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error nvs_set_str %s", esp_err_to_name(err));
            }

            ESP_LOGI(TAG, "Will nvs_commit");
            err = nvs_commit(nvs_handle);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error nvs_commit %s", esp_err_to_name(err));
            }

            ESP_LOGI(TAG, "Will free");
            free(str);
        }

        ESP_LOGI(TAG, "Will nvs_close");
        nvs_close(nvs_handle);
    }
}

void encoder_reading_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {

        if (xSemaphoreTake(g_encoder_mutex, portMAX_DELAY)) {    
            xSemaphoreGive(g_encoder_mutex);
            
            //xQueueSend(g_spi_data_queue, &spi_data, 0);
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    init_pins();

    // esp_err_t err = nvs_flash_init();
    // if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    // {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     err = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(err);

    // nvs_read_config();
    // t_eth_config config = {.ip = g_ip_addr, .pass = g_pass, .ssid = g_ssid, .use_eth = USE_COMMM_ETHERNET, .use_wifi_ap = USE_COMMM_WIFI_AP, .use_wifi_sta = USE_COMMM_WIFI_STA};
    // eth_start(config);

    // g_command_queue = xQueueCreate(QUEUE_SIZE, COMMAND_MAX_SIZE);
    // assert(g_command_queue != NULL);

    if (USE_COMMM_ETHERNET || USE_COMMM_WIFI_AP || USE_COMMM_WIFI_STA)
    {
        //xTaskCreate(command_processing_task, "command_processor", 4096, NULL, 1, NULL);
        //xTaskCreate(udp_server_task, "udp_server", 4096, (void *)AF_INET, 1, NULL);
        
        
    }
    xTaskCreate(myTask, "myTask", 4096, (void *)AF_INET, 0, NULL);
    xTaskCreate(delayTask, "delayTask", 4096, (void *)AF_INET, 1, NULL);

    while (1)
    {
        
    }
}
