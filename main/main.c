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

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

void nvs_read_config();
void nvs_write_config();
char* build_config_string(bool for_nvs);

static const char *TAG = "step_controller";

char g_ip_addr[64] = DEFAULT_STATIC_IP_ADDR;
char g_ssid[64] = DEFAULT_WIFI_STA_SSID;
char g_pass[64] = DEFAULT_WIFI_STA_PASS;

#define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_8_BIT

int bit = 8;

signed int s = 0;
float Kp = 0.01;
float Ki = 0;
float Kd = 0;
float  min_freq = 50;
signed int goal = 1200;

QueueHandle_t g_command_queue;

bool g_flag_send_telemetry = false;

volatile unsigned long g_step_fwd_period_us = DEFAULT_STEP_PERIOD_US;
volatile unsigned long g_step_bwd_period_us = DEFAULT_STEP_PERIOD_US;
volatile unsigned long g_steps = DEFAULT_STEPS;
volatile unsigned long g_pause_ms = DEFAULT_PAUSE;
volatile unsigned long g_calibration_timeout_ms = DEFAULT_CALIBRATION_TIMEOUT;

struct sockaddr_storage g_last_cmd_source_addr; // TODO: mutex protect

gptimer_handle_t g_gptimer;

void append_telemetry_data(cJSON *json)
{
    cJSON_AddBoolToObject(json, STR_TELEMETRY, true);
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
    cJSON_AddNumberToObject(json, STR_STEP_PERIOD_BWD, g_step_bwd_period_us);
    cJSON_AddNumberToObject(json, STR_STEP_PERIOD_FWD, g_step_fwd_period_us);
    cJSON_AddNumberToObject(json, STR_STEPS, g_steps);
    cJSON_AddNumberToObject(json, STR_PAUSE, g_pause_ms);
    cJSON_AddNumberToObject(json, STR_CALIBRATION_TIMEOUT, g_calibration_timeout_ms);
    cJSON_AddNumberToObject(json, STR_FLAG_SEND_TELEMETRY, g_flag_send_telemetry);
    cJSON_AddNumberToObject(json, "goal", goal);
    cJSON_AddNumberToObject(json, "Kp", Kp);
    cJSON_AddNumberToObject(json, "Ki", Ki);
    cJSON_AddNumberToObject(json, "Kd", Kd);
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
            if (!strcmp(param_name, STR_STEPS)) g_steps = subitem->valueint;
            if (!strcmp(param_name, STR_STEP_PERIOD_BWD)) g_step_bwd_period_us = subitem->valueint;
            if (!strcmp(param_name, STR_STEP_PERIOD_FWD)) g_step_fwd_period_us = subitem->valueint;
            if (!strcmp(param_name, STR_PAUSE)) g_pause_ms = subitem->valueint;
            if (!strcmp(param_name, STR_CALIBRATION_TIMEOUT)) g_calibration_timeout_ms = subitem->valueint;
            if (!strcmp(param_name, STR_FLAG_SEND_TELEMETRY)) g_flag_send_telemetry = subitem->valueint;
            if (!strcmp(param_name, "goal")) goal = subitem->valueint;
            if (!strcmp(param_name, "Kp")) Kp = subitem->valuedouble;
            if (!strcmp(param_name, "Ki")) Ki = subitem->valuedouble;
            if (!strcmp(param_name, "Kd")) Kd = subitem->valuedouble;
            if (!strcmp(param_name, "min_freq")) min_freq = subitem->valueint;

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

static void IRAM_ATTR do_step(bool dir, uint32_t n_steps, long step_period, bool flag_wait)
{
    ledc_timer_config_t ledc_timer1 = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = DEFAULT_DUTY_RESOLUTION ,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = step_period,  // Set output frequency according to g_step_period_us
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer1));

    gpio_set_level(PIN_DIR, dir);
    
    //ledc_set_fade(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 1, LEDC_DUTY_DIR_DECREASE, 1, n_steps, 1);
    //ledc_fade_start(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, flag_wait ? LEDC_FADE_WAIT_DONE : LEDC_FADE_NO_WAIT);
}

static void IRAM_ATTR funA(void *param)
{
    int a = gpio_get_level(PIN_A);
    int b = gpio_get_level(PIN_B);

    if (a == 1 && b == 1) s-=1;
    if (a == 1 && b == 0) s+=1;
    if (a == 0 && b == 1) s+=1;
    if (a == 0 && b == 0) s-=1;
}

static void IRAM_ATTR funB(void *param)
{
    int a = gpio_get_level(PIN_A);
    int b = gpio_get_level(PIN_B);

    if (b == 1 && a == 1) s+=1;
    if (b == 1 && a == 0) s-=1;
    if (b == 0 && a == 1) s-=1;
    if (b == 0 && a == 0) s+=1;
}

void IRAM_ATTR sense_stop_isr(void *arg)
{
    BaseType_t flag_yield = 0;

    xTaskNotifyFromISR((TaskHandle_t)arg, 0, 0, &flag_yield);

    portYIELD_FROM_ISR(flag_yield);
}

void init_pins()
{
    gpio_reset_pin(PIN_STEP);
    gpio_set_direction(PIN_STEP, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_STEP, 0);

    gpio_reset_pin(PIN_DIR);
    gpio_set_direction(PIN_DIR, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_DIR, 0);

    gpio_reset_pin(PIN_STOP);
    gpio_set_direction(PIN_STOP, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_STOP, GPIO_PULLUP_ONLY);

    gpio_reset_pin(PIN_LED);
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 1);

    gpio_reset_pin(PIN_A);
    gpio_set_direction(PIN_A, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_A, GPIO_PULLUP_ONLY);

    gpio_reset_pin(PIN_B);
    gpio_set_direction(PIN_B, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_B, GPIO_PULLUP_ONLY);


    //install gpio isr service
    gpio_install_isr_service(0);

    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(PIN_STOP, sense_stop_isr, xTaskGetCurrentTaskHandle());
    gpio_set_intr_type(PIN_STOP, GPIO_INTR_ANYEDGE);

    gpio_isr_handler_add(PIN_A, funA, 0);
    gpio_set_intr_type(PIN_A, GPIO_INTR_ANYEDGE);

    gpio_isr_handler_add(PIN_B, funB, 0);
    gpio_set_intr_type(PIN_B, GPIO_INTR_ANYEDGE);
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

void calibrate_stepper()
{
    if (!gpio_get_level(PIN_STOP))
    {
        ESP_LOGI(TAG, "Already stopped");    
        return;
    } 
    
    ESP_LOGI(TAG, "Will calibrate stepper");

    gpio_set_level(PIN_LED, 1);
    gpio_set_level(PIN_DIR, 0);
    gpio_intr_enable(PIN_STOP);

    ledc_set_duty_and_update(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 1, 0);

    xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(g_calibration_timeout_ms));

    gpio_intr_disable(PIN_STOP);
    ESP_LOGI(TAG, "Finished locking pos");
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);

    gpio_set_level(PIN_LED, 0);
    
    vTaskDelay(pdMS_TO_TICKS(1000));    
}

int step_max = 100;
long step_acc = 0;
int step;

float r = 1;
int dir = 1;
float u = 0;
float r_prev = 0;
float r_prev_prev = 0;
float u_prev = 0;
//float dt = g_step_fwd_period_us; //us
bool flag = 0;

static float u_integral = 0;     
static float u_prev_error = 0;   
static bool g_pid_enabled = true;   
static  bool timer_paused = false; 


void app_main(void)
{
    init_pins();
    
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    nvs_read_config();

    t_eth_config config = {.ip = g_ip_addr, .pass = g_pass, .ssid = g_ssid, .use_eth = USE_COMMM_ETHERNET, .use_wifi_ap = USE_COMMM_WIFI_AP, .use_wifi_sta = USE_COMMM_WIFI_STA};
    eth_start(config);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    float u = 0;           // Управляющий сигнал
    float r = 0;           // Ошибка
    int dir = 1;           // Направление

    g_command_queue = xQueueCreate(QUEUE_SIZE, COMMAND_MAX_SIZE);
    assert(g_command_queue != NULL);

    if (USE_COMMM_ETHERNET || USE_COMMM_WIFI_AP || USE_COMMM_WIFI_STA)
    {
        xTaskCreate(command_processing_task, "command_processor", 4096, NULL, 1, NULL);
        xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 1, NULL);
    }
    
    ledc_timer_config_t ledc_timer1 = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = DEFAULT_DUTY_RESOLUTION,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = min_freq,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer1));

    ledc_channel_config_t ledc_channel1 = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = PIN_STEP,
        .duty           = (int)(pow(2, bit - 1)),
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

    ESP_LOGI(TAG, "PID control task started");

    
    while (1)
    {
        signed int r = goal - s; 

        if (abs(r) > 2 && !g_pid_enabled)
        {
            ESP_LOGI(TAG, "Auto-return: was at target, now error=%d, restarting PID", abs(r));
            g_pid_enabled = true;
            //timer_paused = false;  
        }

        //ESP_LOGI(TAG, "s=%i, goal=%i, r=%f", s, goal, r);
        if (g_pid_enabled && abs(r) > 2) 
        {      
            if (timer_paused) 
            {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (int)(pow(2, bit - 1)));  
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                timer_paused = false;
                ESP_LOGI(TAG, "Timer resumed");
            }

            float p_term = Kp * r;
            
            
            u_integral += r;
            if (u_integral > 1000) u_integral = 1000;
            if (u_integral < -1000) u_integral = -1000;
            float i_term = Ki * u_integral;
            
           
            float d_term = Kd * (r - u_prev_error);
            u_prev_error = r;
            
           
            u = p_term + i_term + d_term;
            
            int u_max = 2000;
            if (u > u_max) u = u_max;
            if (u < -u_max) u = -u_max;

           
            dir = (u >= 0) ? 1 : 0;
            float frequency = abs(u);
            if (frequency < min_freq) frequency = min_freq;

            gpio_set_level(PIN_DIR, dir);
            
           
            ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, (uint32_t)frequency);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        }
        else
        {
            // Остановка ШИМ при достижении цели или выключении ПИД
            if (!timer_paused && abs(r) <= 2) {
                // Устанавливаем скважность 0% - нет импульсов, но таймер работает
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                timer_paused = true;
                g_pid_enabled = false;
                ESP_LOGI(TAG, "Timer paused (duty=0)");
            }
            
        }

        static int log_count = 0;
        if (log_count++ % 50 == 0) 
        {
            ESP_LOGI(TAG, "s=%i, goal=%i, err=%d\n", s, goal, r);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
 }
