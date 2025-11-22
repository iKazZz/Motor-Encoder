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

// Элементы ПИД
float pid_kp = DEFAULT_PID_KP;
float pid_ki = DEFAULT_PID_KI;
float pid_kd = DEFAULT_PID_KD;
double pid_u = 0;
double pid_up = 0;
double pid_ui = 0;
double pid_ud = 0;
int32_t pid_r = 0;
int32_t pid_r_prev = 0;

// Данные о положении энкодера
int32_t enc_goal = DEFAULT_ENC_GOAL;
int32_t enc_pos = 100;
int32_t enc_pos_prev = 0;
double enc_vel = 200;
double enc_angle = 0;
double enc_angle_prev = 0;
double time_count = 0;
double time_count_prev = 0;

//
int32_t enc_avg_pos = 0;
int32_t enc_avg_pos_start = 0;
double enc_avg_vel = 0;
double avg_time_count_start = 0;
double enc_avg_vel_th = DEFAULT_ENC_GOAL;
double enc_delta_vel_max = 100;
double enc_avg_freq = 0;

int timeout_counter = 0;
int timeout_counter_max = DEFAULT_TIMEOUT_COUNTER_MAX;
bool flag_auto_timeout_max = false;
bool flag_timeout = false;

// Параметры ШИМ
unsigned int pwm_freq = 0;
unsigned int pwm_freq_min = DEFAULT_PWM_FREQ_MIN;
unsigned int pwm_freq_max_static = DEFAULT_PWM_FREQ_MAX;
unsigned int pwm_freq_max_dynamic = DEFAULT_PWM_FREQ_MAX;
bool pwm_flag_paused = false;
int pwm_dir = 1;

// Счётчики
int log_counter = 0;
int log_counter_max = DEFAULT_LOG_COUNTER_MAX;
int telemetry_counter = 0;                   
int telemetry_counter_max = DEFAULT_TELEMETRY_COUNTER_MAX;
int avg_counter = 0;
int avg_counter_max = DEFAULT_TELEMETRY_COUNTER_MAX;

QueueHandle_t g_command_queue;

bool g_flag_send_telemetry = true;

struct sockaddr_storage g_last_cmd_source_addr; // TODO: mutex protect

gptimer_handle_t g_gptimer;

esp_err_t data(spi_device_handle_t spi, uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = 0;
    t.cmd = 0x01;
    t.addr = 0;
    t.length = len * 8;
    t.tx_buffer = NULL;
    t.rxlength = t.length;
    t.rx_buffer = data;
    t.user = 0;
    ret = spi_device_polling_transmit(spi, &t);
    // assert(ret == ESP_OK);
    return ret;
}

void append_telemetry_data(cJSON *json)
{
    cJSON_AddBoolToObject(json, "telemetry", true);
    cJSON_AddNumberToObject(json, "enc_pos", enc_pos);
    cJSON_AddNumberToObject(json, "enc_vel", enc_vel);
    cJSON_AddNumberToObject(json, "enc_avg_vel", enc_avg_vel);
    cJSON_AddNumberToObject(json, "enc_avg_vel_th", enc_avg_vel_th);
    cJSON_AddNumberToObject(json, "time_count", time_count);
    cJSON_AddNumberToObject(json, "timeout_counter", timeout_counter);
    cJSON_AddNumberToObject(json, "flag_timeout", flag_timeout);
    cJSON_AddNumberToObject(json, "avg_counter", avg_counter);
    cJSON_AddNumberToObject(json, "log_counter", log_counter);
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
    cJSON_AddStringToObject(json, STR_SSID, g_ssid);
    cJSON_AddStringToObject(json, STR_PASS, g_pass);
    cJSON_AddNumberToObject(json, STR_FLAG_SEND_TELEMETRY, g_flag_send_telemetry);

    cJSON_AddNumberToObject(json, STR_ENC_GOAL, enc_goal);
    cJSON_AddNumberToObject(json, STR_PID_KP, pid_kp);
    cJSON_AddNumberToObject(json, STR_PID_KI, pid_ki);
    cJSON_AddNumberToObject(json, STR_PID_KD, pid_kd);
    cJSON_AddNumberToObject(json, STR_PWM_FREQ_MIN, pwm_freq_min);
    cJSON_AddNumberToObject(json, STR_PWM_FREQ_MAX_STATIC, pwm_freq_max_static);
    cJSON_AddNumberToObject(json, STR_PWM_FREQ_MAX_DYNAMIC, pwm_freq_max_dynamic);
    cJSON_AddNumberToObject(json, STR_LOG_COUNTER_MAX, log_counter_max);
    cJSON_AddNumberToObject(json, STR_TELEMETRY_COUNTER_MAX, telemetry_counter_max);
    cJSON_AddNumberToObject(json, "timeout_counter_max", timeout_counter_max);


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
            
            if (!strcmp(param_name, STR_ENC_GOAL))
                enc_goal = subitem->valueint;
            if (!strcmp(param_name, STR_PID_KP))
                pid_kp = subitem->valuedouble;
            if (!strcmp(param_name, STR_PID_KI))
                pid_ki = subitem->valuedouble;
            if (!strcmp(param_name, STR_PID_KD))
                pid_kd = subitem->valuedouble;
            if (!strcmp(param_name, STR_PWM_FREQ_MIN))
                pwm_freq_min = subitem->valueint;
            if (!strcmp(param_name, STR_PWM_FREQ_MAX_STATIC))
                pwm_freq_max_static = subitem->valueint;
            if (!strcmp(param_name, STR_PWM_FREQ_MAX_DYNAMIC))
                pwm_freq_max_dynamic = subitem->valueint;
            if (!strcmp(param_name, STR_LOG_COUNTER_MAX))
            {
                log_counter_max = subitem->valueint;
                log_counter = 0;
            }
            if (!strcmp(param_name, STR_TELEMETRY_COUNTER_MAX))
            {
                telemetry_counter_max = subitem->valueint;
                telemetry_counter = 0;
            }
            if (!strcmp(param_name, "flag_auto_timeout_max"))
                flag_auto_timeout_max = subitem->valueint;
            if (!strcmp(param_name, "timeout_counter_max"))
                timeout_counter_max = subitem->valueint;

            if (!strcmp(param_name, STR_CMD_READ_FLASH) && subitem->valueint)
                nvs_read_config();
            if (!strcmp(param_name, STR_CMD_WRITE_FLASH) && subitem->valueint)
                nvs_write_config();

            if (!strcmp(param_name, "start_plot"))
            {
                g_flag_send_telemetry = true;
                ESP_LOGI(TAG, "Plot streaming started");
            }
            if (!strcmp(param_name, "stop_plot"))
            {
                g_flag_send_telemetry = false;
                ESP_LOGI(TAG, "Plot streaming stopped");
            }
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

void IRAM_ATTR sense_stop_isr(void *arg)
{
    BaseType_t flag_yield = 0;

    xTaskNotifyFromISR((TaskHandle_t)arg, 0, 0, &flag_yield);

    portYIELD_FROM_ISR(flag_yield);
}

void init_pins()
{
    gpio_reset_pin(PIN_PUL);
    gpio_set_direction(PIN_PUL, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_PUL, 0);

    gpio_reset_pin(PIN_DIR);
    gpio_set_direction(PIN_DIR, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_DIR, 0);

    // gpio_reset_pin(PIN_STOP);
    // gpio_set_direction(PIN_STOP, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(PIN_STOP, GPIO_PULLUP_ONLY);

    // gpio_reset_pin(PIN_LED);
    // gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    // gpio_set_level(PIN_LED, 1);

    // install gpio isr service
    gpio_install_isr_service(0);

    // hook isr handler for specific gpio pin
    // gpio_isr_handler_add(PIN_STOP, sense_stop_isr, xTaskGetCurrentTaskHandle());
    // gpio_set_intr_type(PIN_STOP, GPIO_INTR_ANYEDGE);
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

        uint8_t spi_test_buf[8] = "00000000";

void app_main(void)
{
    init_pins();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .max_transfer_sz = 1};
    spi_device_interface_config_t devcfg = {
        #ifdef CONFIG_OVERCLOCK
            .clock_speed_hz = 26 * 1000 * 1000,
        #else
            .clock_speed_hz = 10 * 1000 * 1000,
        #endif
        .mode = 1,
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

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

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

    // TickType_t xLastWakeTime = xTaskGetTickCount();

    ledc_timer_config_t ledc_timer1 = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = DEFAULT_DUTY_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = pwm_freq_min,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer1));

    ledc_channel_config_t ledc_channel1 = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_PUL,
        .duty = (int)(pow(2, DEFAULT_DUTY_RESOLUTION_BIT - 1)),
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

    ESP_LOGI(TAG, "PID control task started");

    while (1)
    {
        if (timeout_counter > timeout_counter_max)
        {
            flag_timeout = true;
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
            pwm_flag_paused = true;
            ESP_LOGI("AAAAA", "TIMEOUT");
        }
        ret = data(spi, spi_test_buf, sizeof(spi_test_buf));

        if (ret == ESP_OK)
        {
            int32_t spi_enc_count = ((uint32_t)spi_test_buf[1] << 16) + ((uint32_t)spi_test_buf[2] << 8) + (uint32_t)spi_test_buf[3];
            uint32_t spi_time_count = ((uint32_t)spi_test_buf[4] << 24) + ((uint32_t)spi_test_buf[5] << 16) + ((uint32_t)spi_test_buf[6] << 8) + (uint32_t)spi_test_buf[7];
            ESP_LOGI("MISO", "%02X %02X %02X %02X %02X %02X %02X %02X", spi_test_buf[0],spi_test_buf[1],spi_test_buf[2],spi_test_buf[3],spi_test_buf[4],spi_test_buf[5],spi_test_buf[6],spi_test_buf[7]);
            enc_pos_prev = enc_pos;
            enc_pos = (spi_enc_count - 1048576);
            time_count_prev = time_count;
            time_count = (double)spi_time_count * 20 / 1000000000;
            enc_angle_prev = enc_angle;
            enc_angle = (((double)spi_enc_count - 1048576) / 4) * 360 / 2048;
            
            ESP_LOGI("111", "1");
            if (time_count - time_count_prev != 0) enc_vel = (enc_pos - enc_pos_prev) / (time_count - time_count_prev);
            else enc_vel = 0;

            if (avg_counter++ >= avg_counter_max)
            {
                            ESP_LOGI("111", "2");

                if (time_count - avg_time_count_start != 0)
                {
                    enc_avg_vel = (enc_pos - enc_avg_pos_start) / (time_count - avg_time_count_start);
                    enc_avg_freq = enc_avg_vel_th;
                }
                else
                {
                    enc_avg_vel = 0;
                    enc_avg_freq = 0;
                }
                enc_avg_vel_th = ((double)8192 / 25000 * pwm_freq);
                enc_avg_vel_th = (pwm_dir == 1) ? enc_avg_vel_th : -enc_avg_vel_th;
                ///ESP_LOGI("ttt", "enc_pos = %i, enc_avg_pos_start = %i, enc_avg_vel = %.4f", enc_pos, enc_avg_pos_start, enc_avg_vel);
                //ESP_LOGI("yyy", "pwm_freq = %i, enc_avg_freq = %.4f, %.4f", pwm_freq, enc_avg_freq, (time_count - avg_time_count_start));
                avg_time_count_start = time_count;
                enc_avg_pos_start = enc_pos;
                
                //enc_avg_pos = 0;
                //enc_avg_vel = 0;
                //enc_avg_vel_th = 0;
                avg_counter = 0;
            }
            // ESP_LOGI("flag, enc_pos, angle, time_count, enc_vel", "%i %i %.2f %.2f %.2f", spi_test_buf[0] / 128, enc_pos, enc_angle, time_count, enc_vel);
        }

        pid_r = enc_goal - enc_pos;
        ESP_LOGI(TAG, "enc_pos=%i, enc_goal=%i, pid_r=%i", enc_pos, enc_goal, pid_r);
        if (!flag_timeout)
        {
            if (abs(pid_r) > 0)
            {
                if (pwm_flag_paused)
                {
                    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, (int)(pow(2, DEFAULT_DUTY_RESOLUTION_BIT - 1)));
                    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
                    pwm_flag_paused = false;
                    ESP_LOGI(TAG, "Timer resumed");
                }

                            ESP_LOGI("111", "3");

                pid_up = pid_kp * pid_r;
                pid_ui += pid_ki * pid_r;
                pid_ui = (pid_ui > 1000) ? 1000 : (pid_ui < -1000) ? -1000 : pid_ui;
                pid_ud = pid_kd * (pid_r - pid_r_prev);

                pid_u = pid_up + pid_ui + pid_ud;
                pid_r_prev = pid_r;

                pwm_dir = (pid_u >= 0) ? 1 : 0;
                pwm_freq = abs((int)pid_u);
                pwm_freq = (pwm_freq < pwm_freq_min) ? pwm_freq_min : pwm_freq;
                pwm_freq = (pwm_freq > pwm_freq_max_static) ? pwm_freq_max_static : pwm_freq;

                //enc_avg_pos += enc_pos;
                //enc_avg_vel += enc_pos - enc_pos_prev;
                ESP_LOGI("ddd", "enc_avg_vel_th = %.4f, freq = %i", enc_avg_vel_th, pwm_freq);

                gpio_set_level(PIN_DIR, pwm_dir);

                ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, pwm_freq);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
                timeout_counter++;
            }
            else if (!pwm_flag_paused)
            {
                // Устанавливаем скважность 0% - нет импульсов, но таймер работает
                ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
                ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
                pwm_flag_paused = true;
                ESP_LOGI(TAG, "Timer paused (duty=0)");
                pwm_freq = 0;
                timeout_counter = 0;
                avg_counter = 0;
            }
        }

        if (log_counter++ >= log_counter_max)
        {
            ESP_LOGI("MISO", "%02X %02X %02X %02X %02X %02X %02X %02X", spi_test_buf[0], spi_test_buf[1], 
                spi_test_buf[2], spi_test_buf[3], spi_test_buf[4], spi_test_buf[5], spi_test_buf[6], spi_test_buf[7]);
            ESP_LOGI(TAG, "enc_pos=%i, enc_goal=%i, err=%d\n", enc_pos, enc_goal, pid_r);
            log_counter = 0;
        }

        if (telemetry_counter++ >= telemetry_counter_max)
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
                            ESP_LOGW(TAG, "Failed to send telemetry: errno %d", errno);
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Telemetry sent: enc_pos=%d", enc_pos);
                        }
                        close(sock);
                    }
                    free(telemetry_to_send);

                }
                            ESP_LOGI("111", "4");

            }
            telemetry_counter = 0;
        }

        // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
