#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
// #include "driver/ledc.h"
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_cap.h"
#include "driver/twai.h"
#include "driver/pulse_cnt.h"
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
// #include "servosila_sc.h"
#include <cJSON.h>
#include <math.h>


#ifndef max
    #define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
    #define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

bool flag_send_telemetry = false;
bool current_telemetry = false;
QueueHandle_t telemetry_queue;

int num_command = 0;
int current_task = 0;
volatile uint32_t count = 0;

twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_35, TWAI_MODE_NORMAL);
twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// GPTimer

SemaphoreHandle_t gptimer_semaphore;

gptimer_config_t gptimer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 100000,
    .intr_priority = 0
};
gptimer_handle_t gptimer;

gptimer_alarm_config_t gptimer_dfc_alarm_config = {
    .reload_count = 0,
    .alarm_count = 10,
    .flags.auto_reload_on_alarm = true
};

gptimer_alarm_config_t gptimer_esc_alarm_config = {
    .reload_count = 0,
    .alarm_count = 15,
    .flags.auto_reload_on_alarm = true
};

bool gptimer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t xHigherPriorityTaskWoke = pdFALSE;
    xSemaphoreGiveFromISR(gptimer_semaphore, &xHigherPriorityTaskWoke);
    // count++;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoke);

    return false;
}

gptimer_event_callbacks_t gptimer_callback_group = {
    .on_alarm = gptimer_callback
};

// DFC

int dfc_duty_arr[3] = {0, 0, 0};
float dfc_el_phi_deg = 0;
float dfc_el_phi_rad = 0;
float dfc_el_freq = 4;
int dfc_bias = 0;
int dfc_dir = 1;
float dfc_speed = 1;

typedef struct dfc_uvw_coord {
    float u;
    float v;
    float w;
} dfc_uvw_coord_t;

typedef struct dfc_ab_coord {
    float alpha;
    float beta;
} dfc_ab_coord_t;

typedef struct dfc_dq_coord {
    float d;
    float q;
} dfc_dq_coord_t;

dfc_uvw_coord_t dfc_uvw_coord = {0, 0, 0};
dfc_ab_coord_t dfc_ab_coord = {0, 0};
dfc_dq_coord_t dfc_dq_coord = {1, 0};

// Pulse counter

volatile int pcnt_pos = 0;
volatile int pcnt_pos_prev = 0;

pcnt_unit_config_t pcnt_unit_config = {
    .low_limit = -24000,
    .high_limit = 24000,
    .flags.accum_count = true,
    .intr_priority = 2
};
pcnt_unit_handle_t pcnt_unit;

pcnt_chan_config_t pcnt_chan_a_config = {
    .edge_gpio_num = PIN_A,
    .level_gpio_num = PIN_B
};
pcnt_channel_handle_t pcnt_chan_a;

pcnt_chan_config_t pcnt_chan_b_config = {
    .edge_gpio_num = PIN_B,
    .level_gpio_num = PIN_A
};
pcnt_channel_handle_t pcnt_chan_b;

pcnt_glitch_filter_config_t pcnt_gf_config = {
    .max_glitch_ns = 1000
};

// MCPWM (PWM)

const uint32_t mcpwm_res = 40000000;
const int mcpwm_per = 1000;

int mcpwm_gen_pins[3] = {PIN_GHA, PIN_GHB, PIN_GHC};
mcpwm_timer_handle_t mcpwm_timer;
mcpwm_oper_handle_t mcpwm_operators[3];
mcpwm_cmpr_handle_t mcpwm_comparators[3];
mcpwm_gen_handle_t mcpwm_generators[3];

mcpwm_timer_config_t mcpwm_timer_config= {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = mcpwm_res,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
    .period_ticks = mcpwm_per,
    .intr_priority = 0,

    .flags.allow_pd = 0,
    .flags.update_period_on_empty = 1,
    .flags.update_period_on_sync = 0
};

mcpwm_operator_config_t mcpwm_oper_config = {
    .group_id = 0,
    .intr_priority = 0,

    .flags.update_gen_action_on_sync = 0,
    .flags.update_gen_action_on_tep = 0,
    .flags.update_gen_action_on_tez = 1
};

mcpwm_comparator_config_t mcpwm_cmpr_config = {
    .intr_priority = 0,

    .flags.update_cmp_on_sync = 0,
    .flags.update_cmp_on_tep = 0,
    .flags.update_cmp_on_tez = 0
};

mcpwm_generator_config_t mcpwm_gen_config = {
    .gen_gpio_num = 0,

    .flags.invert_pwm = 0,
    .flags.io_loop_back = 0,
    .flags.pull_down = 0,
    .flags.pull_up = 0
};

// MCPWM (Capture)

bool flag_mcpwm_cap_activated = false;
QueueHandle_t mcpwm_cap_queue;

typedef struct mcpwm_cap_data {
    int cur_pos;
    int prev_pos;
    int znak;
    uint32_t cur_tick;
    uint32_t prev_tick;
    uint32_t per;

} mcpwm_cap_data_t;

mcpwm_cap_timer_handle_t mcpwm_cap_timer;
mcpwm_cap_channel_handle_t mcpwm_cap_channel_A;
mcpwm_cap_channel_handle_t mcpwm_cap_channel_B;

mcpwm_capture_timer_config_t mcpwm_cap_timer_config = {
    .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
    .group_id = 0,
    .resolution_hz = 4 * mcpwm_res
};

mcpwm_capture_channel_config_t mcpwm_cap_channel_A_config = {
    .gpio_num = PIN_A,
    .prescale = 1,
    .intr_priority = 0,
    .flags.pull_up = true,
    .flags.pos_edge = true,
    .flags.neg_edge = true
};

mcpwm_capture_channel_config_t mcpwm_cap_channel_B_config = {
    .gpio_num = PIN_B,
    .prescale = 1,  
    .intr_priority = 0,
    .flags.pull_up = true,
    .flags.pos_edge = true,
    .flags.neg_edge = true
};

volatile mcpwm_cap_data_t mcpwm_cap_data = {0, 0, 0, 0};

bool aboba(mcpwm_cap_channel_handle_t chan, const mcpwm_capture_event_data_t *edata, void* userdata)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    mcpwm_cap_data.prev_pos = mcpwm_cap_data.cur_pos;
    pcnt_unit_get_count(pcnt_unit, &(mcpwm_cap_data.cur_pos));
    mcpwm_cap_data.prev_tick = mcpwm_cap_data.cur_tick;
    mcpwm_cap_data.cur_tick = edata->cap_value;

    mcpwm_cap_data.per = (mcpwm_cap_data.cur_tick - mcpwm_cap_data.prev_tick > 0) 
    ? mcpwm_cap_data.cur_tick - mcpwm_cap_data.prev_tick
    : UINT32_MAX - mcpwm_cap_data.prev_tick + mcpwm_cap_data.cur_tick;

    mcpwm_cap_data.znak = (mcpwm_cap_data.cur_pos > mcpwm_cap_data.prev_pos) ? 1 : -1;

    if ((current_task == 2) || (current_task == 3) || (current_task == 4)) xQueueOverwriteFromISR(mcpwm_cap_queue, &mcpwm_cap_data, &xHigherPriorityTaskWoken);
    if (current_telemetry) xQueueOverwriteFromISR(telemetry_queue, &mcpwm_cap_data, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return false;
}

mcpwm_capture_event_callbacks_t aaa = {
    .on_cap = aboba
};

// ESC

float esc_speed = 0;
float esc_goal = 1;
float esc_r = 0;
float esc_kp = 0.1;
float esc_ki = 0.1;
float esc_gamma = 0.8;
float esc_koef = 1;

float esc_up = 0;
float esc_ui = 0;
float esc_ui_max = 10;
float esc_u = 0;
float esc_u_max = 10;
int esc_dir = 1;

float esc_ki_max = 5;
float esc_ki_coef = -9.8;

float esc_avg = 0;
float esc_avg_prev = 0;

float speed_coef = 80000000 / 2400;

// Servo

int pid_pos = 0;
int servo_goal = 2400;
int servo_r = 0;
int servo_r_prev = 0;

float servo_kp = 1;
float servo_ki = 0;
float servo_kd = 0;

float servo_up = 0;
float servo_ui = 0;
float servo_ud = 0;
float servo_u = 0;

float servo_u_max = 8000;
float servo_ui_max = 100;
int servo_dir = 1;

float servo_speed_coef = 0.00025;

//

void nvs_read_config();
void nvs_write_config();
char* build_config_string(bool for_nvs);

static const char *TAG = "step_controller";

char g_ip_addr[64]  = DEFAULT_STATIC_IP_ADDR;
char g_ssid[64]     = DEFAULT_WIFI_STA_SSID;
char g_pass[64]     = DEFAULT_WIFI_STA_PASS;


mcpwm_cap_data_t telemetry_cap_data = {0, 0, 0, 0};

TaskHandle_t dfc_task_handle;
void dfc_task(void *pvParameters);
TaskHandle_t foc_task_handle;
void foc_task(void *pvParameters);
TaskHandle_t esc_task_handle;
void esc_task(void *pvParameters);
TaskHandle_t servo_task_handle;
void servo_task(void *pvParameters);
TaskHandle_t telemetry_task_handle;
void telemetry_task(void *pvParameters);
TaskHandle_t timer_task_handle;
void timer_task(void *pvParameters);

QueueHandle_t g_command_queue;
struct sockaddr_storage g_last_cmd_source_addr; // TODO: mutex protect
int g_last_sock;
t_command cmd;

char* build_telemetry_string()
{
    cJSON *json = cJSON_CreateObject();

    cJSON_AddBoolToObject(json, STR_TELEMETRY, flag_send_telemetry);
    cJSON_AddNumberToObject(json, "time", mcpwm_cap_data.cur_tick / 800000);
    if (telemetry_cap_data.per != 0) cJSON_AddNumberToObject(json, "speed", 
        speed_coef * (telemetry_cap_data.cur_pos - telemetry_cap_data.prev_pos) / telemetry_cap_data.per);
    else cJSON_AddNumberToObject(json, "speed", 0);
    cJSON_AddNumberToObject(json, "pos", telemetry_cap_data.cur_pos);
    cJSON_AddNumberToObject(json, "servo_ui", servo_ui);
    cJSON_AddNumberToObject(json, "esc_ui", esc_ui);
    cJSON_AddNumberToObject(json, "esc_goal", esc_goal);

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
    cJSON_AddNumberToObject(json, "esc_kp", esc_kp);
    cJSON_AddNumberToObject(json, "esc_ki", esc_ki);
    cJSON_AddNumberToObject(json, "esc_ki_max", esc_ki_max);
    cJSON_AddNumberToObject(json, "esc_ki_coef", esc_ki_coef);
    cJSON_AddNumberToObject(json, "esc_ui_max", esc_ui_max);
    cJSON_AddNumberToObject(json, "esc_goal", esc_goal);
    cJSON_AddNumberToObject(json, "servo_kp", servo_kp);
    cJSON_AddNumberToObject(json, "servo_ki", servo_ki);
    cJSON_AddNumberToObject(json, "servo_ui_max", servo_ui_max);
    cJSON_AddNumberToObject(json, "servo_goal", servo_goal);
    cJSON_AddNumberToObject(json, "servo_speed_coef", servo_speed_coef);
    cJSON_AddNumberToObject(json, "esc_gamma", esc_gamma);
    cJSON_AddNumberToObject(json, "esc_koef", esc_koef);
    cJSON_AddNumberToObject(json, "dfc_speed", dfc_speed);

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

            // ESP_LOGI(TAG, "Item %s", subitem->string);
            
            char *param_name = subitem->string;

            if (!strcmp(param_name, "cmd1")) 
            {
                if (!strcmp(subitem->valuestring, "stop")) num_command = -1;
                if (!strcmp(subitem->valuestring, "dfc")) num_command = 1;
                if (!strcmp(subitem->valuestring, "foc")) num_command = 2;
                if (!strcmp(subitem->valuestring, "esc")) num_command = 3;
                if (!strcmp(subitem->valuestring, "servo")) num_command = 4;
            }
            else if (!strcmp(param_name, "telemetry")) 
            {
                flag_send_telemetry = subitem->valueint;
            }
            else num_command = 0;
        }
        ESP_LOGI("Mine", "num_command = %d", num_command);

        if (num_command == 0)
        {
            for (int i=0; i < cJSON_GetArraySize(parsed_cmd); i++)
            {   
                cJSON *subitem = cJSON_GetArrayItem(parsed_cmd, i);

                ESP_LOGI(TAG, "Item %s", subitem->string);
                
                char *param_name = subitem->string;

                if (!strcmp(param_name, STR_IP_ADDR)) strncpy(g_ip_addr, subitem->valuestring, sizeof(g_ip_addr)-1);
                if (!strcmp(param_name, STR_SSID)) strncpy(g_ssid, subitem->valuestring, sizeof(g_ssid)-1);
                if (!strcmp(param_name, STR_PASS)) strncpy(g_pass, subitem->valuestring, sizeof(g_pass)-1);
                if (!strcmp(param_name, "esc_goal")) esc_goal = subitem->valuedouble;
                if (!strcmp(param_name, "esc_kp")) esc_kp = subitem->valuedouble;
                if (!strcmp(param_name, "esc_ki")) esc_ki = subitem->valuedouble;
                if (!strcmp(param_name, "esc_ki_max")) esc_ki_max = subitem->valuedouble;
                if (!strcmp(param_name, "esc_ki_coef")) esc_ki_coef = subitem->valuedouble;
                if (!strcmp(param_name, "esc_ui_max")) esc_ui_max = subitem->valuedouble;
                if (!strcmp(param_name, "servo_goal")) servo_goal = subitem->valuedouble;
                if (!strcmp(param_name, "servo_kp")) servo_kp = subitem->valuedouble;
                if (!strcmp(param_name, "servo_ki")) servo_ki = subitem->valuedouble;
                if (!strcmp(param_name, "servo_ui_max")) servo_ui_max = subitem->valuedouble;
                if (!strcmp(param_name, "servo_speed_coef")) servo_speed_coef = subitem->valuedouble;
                if (!strcmp(param_name, "esc_gamma")) esc_gamma = subitem->valuedouble;
                if (!strcmp(param_name, "esc_koef")) esc_koef = subitem->valuedouble;
                if (!strcmp(param_name, "dfc_speed")) dfc_speed = subitem->valuedouble;
                if (!strcmp(param_name, STR_CMD_READ_FLASH) && subitem->valueint) nvs_read_config();
                if (!strcmp(param_name, STR_CMD_WRITE_FLASH) && subitem->valueint) nvs_write_config();
            }
        }
        
    }

    cJSON_Delete(parsed_cmd);
}

void command_processing_task(void *pvParameters)
{
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
            g_last_sock = cmd.sock;
            parse_config_string(cmd.cmd);
            
            char *config_to_send = build_config_string(false);            
            if (config_to_send)
            {
                // ESP_LOGI(TAG, "Will send config %s", config_to_send);
                int err = sendto(cmd.sock, config_to_send, strlen(config_to_send), 0, (struct sockaddr *)&cmd.source_addr, sizeof(struct sockaddr));
                if (err < 0) 
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
            }
            free(config_to_send);

            if (!current_telemetry && flag_send_telemetry)
            {
                telemetry_queue = xQueueCreate(1, sizeof(mcpwm_cap_data_t));
                xTaskCreate(telemetry_task, "Telemetry", 4096, NULL, 1, &telemetry_task_handle);
                current_telemetry = true;
            }
            else if (current_telemetry && !flag_send_telemetry)
            {
                if (current_task == 0 || current_task == 1)
                {
                    ESP_ERROR_CHECK(mcpwm_capture_channel_disable(mcpwm_cap_channel_A));
                    ESP_ERROR_CHECK(mcpwm_capture_channel_disable(mcpwm_cap_channel_B));
                    ESP_ERROR_CHECK(mcpwm_capture_timer_stop(mcpwm_cap_timer));
                    flag_mcpwm_cap_activated = false;
                }
                vTaskDelete(telemetry_task_handle);
                current_telemetry = false;
                vQueueDelete(telemetry_queue);
                ESP_LOGI("Telemetry", "stopped");
            }

            if (current_task == 0)
            {
                if (num_command == 1)
                {
                    xTaskCreate(dfc_task, "Direct_Field_Control", 4096, NULL, 2, &dfc_task_handle);
                    current_task = 1;
                }
                else if (num_command == 2)
                {
                    mcpwm_cap_queue = xQueueCreate(1, sizeof(mcpwm_cap_data_t));
                    xTaskCreate(foc_task, "Field_Oriented_Control", 4096, NULL, 2, &foc_task_handle);
                    current_task = 2;
                }
                else if (num_command == 3)
                {
                    mcpwm_cap_queue = xQueueCreate(1, sizeof(mcpwm_cap_data_t));
                    xTaskCreate(esc_task, "Electrical_Speed_Control", 4096, NULL, 2, &esc_task_handle);
                    // xTaskCreate(timer_task, "Timer", 4096, NULL, 1, &timer_task_handle);
                    current_task = 3;
                }
                else if (num_command == 4)
                {
                    mcpwm_cap_queue = xQueueCreate(1, sizeof(mcpwm_cap_data_t));
                    xTaskCreate(servo_task, "Servo_Control", 4096, NULL, 2, &servo_task_handle);
                    current_task = 4;
                }
            }

            if (num_command == -1)
            {
                if (current_task == 1)
                {
                    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_STOP_EMPTY));
                    ESP_ERROR_CHECK(gptimer_stop(gptimer));
                    vTaskDelete(dfc_task_handle);
                    current_task = 0;
                }
                else if (current_task == 2)
                {
                    if (!current_telemetry)
                    {
                        ESP_ERROR_CHECK(mcpwm_capture_channel_disable(mcpwm_cap_channel_A));
                        ESP_ERROR_CHECK(mcpwm_capture_channel_disable(mcpwm_cap_channel_B));
                        ESP_ERROR_CHECK(mcpwm_capture_timer_stop(mcpwm_cap_timer));
                        flag_mcpwm_cap_activated = false;
                    }
                    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_STOP_EMPTY));
                    vTaskDelete(foc_task_handle);
                    current_task = 0;
                    vQueueDelete(mcpwm_cap_queue);
                }
                else if (current_task == 3)
                {
                    if (!current_telemetry)
                    {
                        ESP_ERROR_CHECK(mcpwm_capture_channel_disable(mcpwm_cap_channel_A));
                        ESP_ERROR_CHECK(mcpwm_capture_channel_disable(mcpwm_cap_channel_B));
                        ESP_ERROR_CHECK(mcpwm_capture_timer_stop(mcpwm_cap_timer));
                        flag_mcpwm_cap_activated = false;
                    }
                    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_STOP_EMPTY));
                    ESP_ERROR_CHECK(gptimer_stop(gptimer));
                    vTaskDelete(esc_task_handle);

                    // vTaskDelete(timer_task_handle);
                    current_task = 0;
                    vQueueDelete(mcpwm_cap_queue);
                }
                else if (current_task == 4)
                {
                    if (!current_telemetry)
                    {
                        ESP_ERROR_CHECK(mcpwm_capture_channel_disable(mcpwm_cap_channel_A));
                        ESP_ERROR_CHECK(mcpwm_capture_channel_disable(mcpwm_cap_channel_B));
                        ESP_ERROR_CHECK(mcpwm_capture_timer_stop(mcpwm_cap_timer));
                        flag_mcpwm_cap_activated = false;
                    }
                    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_STOP_EMPTY));
                    ESP_ERROR_CHECK(gptimer_stop(gptimer));

                    vTaskDelete(servo_task_handle);
                    current_task = 0;
                    vQueueDelete(mcpwm_cap_queue);
                }
            }
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

void foc_inverse_park_transform(float phi, dfc_dq_coord_t *dq, dfc_ab_coord_t *ab)
{
    ab->alpha = dq->d * cos(phi) - dq->q * sin(phi);
    ab->beta  = dq->q * cos(phi) + dq->d * sin(phi);
}

void foc_inverse_clark_transform(dfc_ab_coord_t *ab, dfc_uvw_coord_t *uvw)
{
    uvw->u = ab->alpha;
    uvw->v = (ab->beta * sqrt(3) - ab->alpha) / 2;
    uvw->w = -uvw->u - uvw->v; 
}

void twai_command_task(void *pvParameters)
{
    twai_message_t message = {
        .extd = 0,              // Standard Format message (11-bit ID)
        .rtr = 0,               // Send a data frame
        .ss = 1,                // Is single shot (won't retry on error or NACK)
        .self = 0,              // Not a self reception request
        .dlc_non_comp = 0,      // DLC is less than 8

        // Message ID and payload
        .identifier = 0,
        .data_length_code = 1,
        .data = {0} 
    };
    while (1)
    {
        if (twai_receive(&message, pdMS_TO_TICKS(2000)) != ESP_OK)
        {
            ESP_LOGE("TWAI", "Failed to receive");
        }
        else
        {
            if (message.identifier == 2 && message.data[0] == 111 && current_task == 0)
            {
                xTaskCreate(dfc_task, "Direct_Field_Control", 4096, NULL, 2, &dfc_task_handle);
                current_task = 1;
            }
            else if (message.identifier == 2 && message.data[0] == 125 && current_task == 1)
            {
                ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_STOP_EMPTY));
                ESP_ERROR_CHECK(gptimer_stop(gptimer));
                vTaskDelete(dfc_task_handle);
                current_task = 0;
            }
            else
            {
                ESP_LOGE("Twai", "Wrong Data");
            }
        }
    }
}

// Initializations

void pins_init()
{
    gpio_reset_pin(PIN_GHA);
    gpio_set_direction(PIN_GHA, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_GHA, 0);

    gpio_reset_pin(PIN_GHB);
    gpio_set_direction(PIN_GHB, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_GHB, 0);

    gpio_reset_pin(PIN_GHC);
    gpio_set_direction(PIN_GHC, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_GHC, 0);

    gpio_reset_pin(PIN_A);
    gpio_set_direction(PIN_A, GPIO_MODE_INPUT);
    gpio_input_enable(PIN_A);
    gpio_set_pull_mode(PIN_A, GPIO_PULLUP_ONLY);
    gpio_pullup_en(PIN_A);

    gpio_reset_pin(PIN_B);
    gpio_set_direction(PIN_B, GPIO_MODE_INPUT);
    gpio_input_enable(PIN_B);
    gpio_set_pull_mode(PIN_B, GPIO_PULLUP_ONLY);
    gpio_pullup_en(PIN_B);

    gpio_reset_pin(PIN_3V_1);
    gpio_set_direction(PIN_3V_1, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_3V_1, 1);

    // gpio_reset_pin(PIN_3V_2);
    // gpio_set_direction(PIN_3V_2, GPIO_MODE_OUTPUT);
    // gpio_set_level(PIN_3V_2, 1);
}

void pcnt_init()
{
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_config, &pcnt_unit));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &pcnt_chan_a_config, &pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &pcnt_chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &pcnt_gf_config));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, -24000));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, 24000));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

void mcpwm_pwm_init()
{
    ESP_ERROR_CHECK(mcpwm_new_timer(&mcpwm_timer_config, &mcpwm_timer));

    for (int i = 0; i < 3; i++)
    {
        ESP_ERROR_CHECK(mcpwm_new_operator(&mcpwm_oper_config, &mcpwm_operators[i]));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(mcpwm_operators[i], mcpwm_timer));

        ESP_ERROR_CHECK(mcpwm_new_comparator(mcpwm_operators[i], &mcpwm_cmpr_config, &mcpwm_comparators[i]));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_comparators[i], 0));

        mcpwm_gen_config.gen_gpio_num = mcpwm_gen_pins[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(mcpwm_operators[i], &mcpwm_gen_config, &mcpwm_generators[i]));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(mcpwm_generators[i], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mcpwm_comparators[i], MCPWM_GEN_ACTION_LOW)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(mcpwm_generators[i], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, mcpwm_comparators[i], MCPWM_GEN_ACTION_HIGH)));
    }

    ESP_ERROR_CHECK(mcpwm_timer_enable(mcpwm_timer));
}

void mcpwm_capture_init()
{
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&mcpwm_cap_timer_config, &mcpwm_cap_timer));
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(mcpwm_cap_timer, &mcpwm_cap_channel_A_config, &mcpwm_cap_channel_A));
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(mcpwm_cap_timer, &mcpwm_cap_channel_B_config, &mcpwm_cap_channel_B));

    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(mcpwm_cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(mcpwm_cap_channel_A, &aaa, NULL));
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(mcpwm_cap_channel_B, &aaa, NULL)); 
}

void gptimer_init()
{
    gptimer_semaphore = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(gptimer_new_timer(&gptimer_config, &gptimer));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &gptimer_esc_alarm_config));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &gptimer_callback_group, NULL));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
}

void calibration()
{
    ESP_LOGI("Calibration", "Started");
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_START_NO_STOP));

    dfc_el_phi_rad = 0;
    foc_inverse_park_transform(dfc_el_phi_rad, &dfc_dq_coord, &dfc_ab_coord);
    foc_inverse_clark_transform(&dfc_ab_coord, &dfc_uvw_coord);

    dfc_duty_arr[0] = (int)(mcpwm_per * (dfc_uvw_coord.u / 4 + 1.0 / 4));
    dfc_duty_arr[1] = (int)(mcpwm_per * (dfc_uvw_coord.v / 4 + 1.0 / 4));
    dfc_duty_arr[2] = (int)(mcpwm_per * (dfc_uvw_coord.w / 4 + 1.0 / 4));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_comparators[0], dfc_duty_arr[0]));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_comparators[1], dfc_duty_arr[1]));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_comparators[2], dfc_duty_arr[2]));

    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &dfc_bias));

    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_STOP_EMPTY));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_comparators[0], 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_comparators[1], 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_comparators[2], 0));
    ESP_LOGI("Calibration", "bias = %d", dfc_bias);
    ESP_LOGI("Calibration", "calibrated succesful");
}

// Command tasks

void dfc_task(void *pvParameters)
{
    ESP_LOGI("DFC", "Task started");
    dfc_dq_coord.d = 1;
    
    if (current_telemetry && !flag_mcpwm_cap_activated)
    {
        ESP_ERROR_CHECK(mcpwm_capture_timer_start(mcpwm_cap_timer));
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(mcpwm_cap_channel_A));
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(mcpwm_cap_channel_B));
        flag_mcpwm_cap_activated = true;
    }

    ESP_ERROR_CHECK(gptimer_start(gptimer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_START_NO_STOP));

    while (true)
    {
        if (xSemaphoreTake(gptimer_semaphore, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            dfc_el_phi_deg += dfc_speed * (360 * dfc_el_freq / 1000000 * 15);
            if(dfc_el_phi_deg >= 360)
            {
                dfc_el_phi_deg -= 360;
            }
            else if (dfc_el_phi_deg <= -360)
            {
                dfc_el_phi_deg += 360;
            }
            dfc_el_phi_rad = dfc_el_phi_deg * M_PI / 180;

            foc_inverse_park_transform(dfc_el_phi_rad, &dfc_dq_coord, &dfc_ab_coord);
            foc_inverse_clark_transform(&dfc_ab_coord, &dfc_uvw_coord);

            dfc_duty_arr[0] = (int)(mcpwm_per * (dfc_uvw_coord.u / 4 + 1.0 / 4));
            dfc_duty_arr[1] = (int)(mcpwm_per * (dfc_uvw_coord.v / 4 + 1.0 / 4));
            dfc_duty_arr[2] = (int)(mcpwm_per * (dfc_uvw_coord.w / 4 + 1.0 / 4));

            for (int i = 0; i < 3; i++)
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_comparators[i], dfc_duty_arr[i]));
            }
        }
        else
        {
            ESP_LOGI("dfc", "Error");
        }
    }
}

void foc_task(void *pvParameters)
{
    ESP_LOGI("FOC", "Task started");
    dfc_dq_coord.d = 1;
    dfc_dir = 1;

    calibration();
    if (!flag_mcpwm_cap_activated)
    {
        ESP_ERROR_CHECK(mcpwm_capture_timer_start(mcpwm_cap_timer));
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(mcpwm_cap_channel_A));
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(mcpwm_cap_channel_B));
        flag_mcpwm_cap_activated = true;
    }
    mcpwm_cap_data_t foc_cap_data = {0, 0, 0, 0};

    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_START_NO_STOP));

    while (true)
    {
        if (xQueueReceive(mcpwm_cap_queue, &foc_cap_data, pdMS_TO_TICKS(100)) != pdPASS)
        {
            pcnt_unit_get_count(pcnt_unit, &(foc_cap_data.cur_pos));
        }

        dfc_el_phi_deg = (float)(foc_cap_data.cur_pos - dfc_bias) / 2400 * 360 * 14 + dfc_dir * 90;
        if(dfc_el_phi_deg >= 360)
        {
            dfc_el_phi_deg -= 360;
        }
        else if (dfc_el_phi_deg <= -360)
        {
            dfc_el_phi_deg += 360;
        }
        dfc_el_phi_rad = dfc_el_phi_deg * M_PI / 180;

        foc_inverse_park_transform(dfc_el_phi_rad, &dfc_dq_coord, &dfc_ab_coord);
        foc_inverse_clark_transform(&dfc_ab_coord, &dfc_uvw_coord);

        dfc_duty_arr[0] = (int)(mcpwm_per * (dfc_uvw_coord.u / 4 + 1.0 / 4));
        dfc_duty_arr[1] = (int)(mcpwm_per * (dfc_uvw_coord.v / 4 + 1.0 / 4));
        dfc_duty_arr[2] = (int)(mcpwm_per * (dfc_uvw_coord.w / 4 + 1.0 / 4));

        for (int i = 0; i < 3; i++)
        {
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_comparators[i], dfc_duty_arr[i]));
        }
    }
}

void esc_task(void *pvParameters)
{
    ESP_LOGI("ESC", "Task started");
    TickType_t xLastTake = 0;

    // ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &gptimer_esc_alarm_config));
    
    calibration();
    if (!flag_mcpwm_cap_activated)
    {
        ESP_ERROR_CHECK(mcpwm_capture_timer_start(mcpwm_cap_timer));
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(mcpwm_cap_channel_A));
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(mcpwm_cap_channel_B));
        flag_mcpwm_cap_activated = true;
    }
    mcpwm_cap_data_t esc_cap_data = {0, 0, 0, 0};

    ESP_ERROR_CHECK(gptimer_start(gptimer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_START_NO_STOP));

    while (true)
    {
        if (xSemaphoreTake(gptimer_semaphore, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            if (pdTICKS_TO_MS(xTaskGetTickCount() - xLastTake) < 1)
            {
                if (xQueueReceive(mcpwm_cap_queue, &esc_cap_data, 0) == pdPASS)
                {
                    if (esc_cap_data.per != 0)
                    {
                        esc_avg_prev = esc_avg;
                        esc_speed = speed_coef * (esc_cap_data.cur_pos - esc_cap_data.prev_pos) / esc_cap_data.per;
                        esc_avg = esc_avg_prev * (1.0 - esc_gamma) + esc_speed * esc_gamma;
                    }
                    xLastTake = xTaskGetTickCount();
                }
            }
            else
            {
                esc_avg_prev = esc_avg;
                esc_speed = 0;
                esc_avg = esc_avg_prev * (1.0 - esc_gamma) + esc_speed * esc_gamma;
                xLastTake = xTaskGetTickCount();
            }
            esc_r = esc_goal - esc_avg;
            esc_up = esc_kp * esc_r;

            esc_ui += esc_ki * esc_r;
            esc_ui = (esc_ui > esc_ui_max) ? esc_ui_max : (esc_ui < -esc_ui_max) ? -esc_ui_max : esc_ui;
            esc_u = esc_up + esc_ui;

            esc_dir = (esc_u > 0) ? 1 : -1;
            esc_u = (abs(esc_u) > esc_u_max) ? esc_u_max : abs(esc_u);

            if (esc_cap_data.znak != esc_dir)
            {
                dfc_dq_coord.d = esc_koef * esc_u / esc_u_max;
            }
            else
            {
                dfc_dq_coord.d = esc_u / esc_u_max;
            }

            dfc_el_phi_deg = (float)(esc_cap_data.cur_pos - dfc_bias) / 2400 * 360 * 14 + esc_dir * 90;
            if(dfc_el_phi_deg >= 360)
            {
                dfc_el_phi_deg -= 360;
            }
            else if (dfc_el_phi_deg <= -360)
            {
                dfc_el_phi_deg += 360;
            }
            dfc_el_phi_rad = dfc_el_phi_deg * M_PI / 180;

            foc_inverse_park_transform(dfc_el_phi_rad, &dfc_dq_coord, &dfc_ab_coord);
            foc_inverse_clark_transform(&dfc_ab_coord, &dfc_uvw_coord);

            dfc_duty_arr[0] = (int)(mcpwm_per * (dfc_uvw_coord.u / 4 + 1.0 / 4));
            dfc_duty_arr[1] = (int)(mcpwm_per * (dfc_uvw_coord.v / 4 + 1.0 / 4));
            dfc_duty_arr[2] = (int)(mcpwm_per * (dfc_uvw_coord.w / 4 + 1.0 / 4));

            for (int i = 0; i < 3; i++)
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_comparators[i], dfc_duty_arr[i]));
            }
            // count++;
        }
        else
        {
            // ESP_LOGE("ESC", "Error with taking gptimer_semaphore, %lu", count);
            // count = 0;
        }
    }
}

void servo_task(void *pvParameters)
{
    ESP_LOGI("Servo", "Task started");
    TickType_t xLastTake = 0;
    calibration();
    if (!flag_mcpwm_cap_activated)
    {
        ESP_ERROR_CHECK(mcpwm_capture_timer_start(mcpwm_cap_timer));
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(mcpwm_cap_channel_A));
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(mcpwm_cap_channel_B));
        flag_mcpwm_cap_activated = true;
    }
    mcpwm_cap_data_t servo_cap_data = {0, 0, 0, 0};

    ESP_ERROR_CHECK(gptimer_start(gptimer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_START_NO_STOP));

    while (true)
    {
        if (xSemaphoreTake(gptimer_semaphore, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            if (xQueueReceive(mcpwm_cap_queue, &servo_cap_data, 0) == pdPASS)
            {
                if (servo_cap_data.per != 0)
                {
                    esc_avg_prev = esc_avg;
                    esc_speed = speed_coef * (servo_cap_data.cur_pos - servo_cap_data.prev_pos) / servo_cap_data.per;
                    esc_avg = esc_avg_prev * (1.0 - esc_gamma) + esc_speed * esc_gamma;
                    servo_r = servo_goal - servo_cap_data.cur_pos;
                }
                xLastTake = xTaskGetTickCount();
            }
            else
            {
                if (pdTICKS_TO_MS(xTaskGetTickCount() - xLastTake) >= 1)
                {
                    esc_avg_prev = esc_avg;
                    esc_speed = 0;
                    esc_avg = esc_avg_prev * (1.0 - esc_gamma) + esc_speed * esc_gamma;
                    servo_r = servo_goal - servo_cap_data.cur_pos;
                    xLastTake = xTaskGetTickCount();
                }
            }
            servo_up = servo_kp * servo_r;

            servo_ui += servo_ki * servo_r;
            servo_ui = (servo_ui > servo_ui_max) ? servo_ui_max : (servo_ui < -servo_ui_max) ? -servo_ui_max : servo_ui;

            servo_u = servo_up + servo_ui;

            servo_u = (servo_u > servo_u_max) ? servo_u_max : (servo_u < -servo_u_max) ? -servo_u_max : servo_u;

            esc_goal = servo_speed_coef * servo_u;
            esc_r = esc_goal - esc_avg;
            esc_up = esc_kp * esc_r;

            esc_ui += esc_ki * esc_r;
            esc_ui = (esc_ui > esc_ui_max) ? esc_ui_max : (esc_ui < -esc_ui_max) ? -esc_ui_max : esc_ui;
            esc_u = esc_up + esc_ui;

            esc_dir = (esc_u > 0) ? 1 : -1;
            esc_u = (abs(esc_u) > esc_u_max) ? esc_u_max : abs(esc_u);

            if (servo_cap_data.znak != esc_dir)
            {
                dfc_dq_coord.d = esc_koef * esc_u / esc_u_max;
            }
            else
            {
                dfc_dq_coord.d = esc_u / esc_u_max;
            }

            dfc_el_phi_deg = (float)(servo_cap_data.cur_pos - dfc_bias) / 2400 * 360 * 14 + esc_dir * 90;
            if(dfc_el_phi_deg >= 360)
            {
                dfc_el_phi_deg -= 360;
            }
            else if (dfc_el_phi_deg <= -360)
            {
                dfc_el_phi_deg += 360;
            }
            dfc_el_phi_rad = dfc_el_phi_deg * M_PI / 180;

            foc_inverse_park_transform(dfc_el_phi_rad, &dfc_dq_coord, &dfc_ab_coord);
            foc_inverse_clark_transform(&dfc_ab_coord, &dfc_uvw_coord);

            dfc_duty_arr[0] = (int)(mcpwm_per * (dfc_uvw_coord.u / 4 + 1.0 / 4));
            dfc_duty_arr[1] = (int)(mcpwm_per * (dfc_uvw_coord.v / 4 + 1.0 / 4));
            dfc_duty_arr[2] = (int)(mcpwm_per * (dfc_uvw_coord.w / 4 + 1.0 / 4));

            for (int i = 0; i < 3; i++)
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_comparators[i], dfc_duty_arr[i]));
            }
            // count++;
        }
        else
        {
            // ESP_LOGE("ESC", "Error with taking gptimer_semaphore, %lu", count);
            // count = 0;
        }
    }
}

void telemetry_task(void *pvParameters)
{
    ESP_LOGI("Telemetry", "started");
    if (!flag_mcpwm_cap_activated)
    {
        ESP_ERROR_CHECK(mcpwm_capture_timer_start(mcpwm_cap_timer));
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(mcpwm_cap_channel_A));
        ESP_ERROR_CHECK(mcpwm_capture_channel_enable(mcpwm_cap_channel_B));
        flag_mcpwm_cap_activated = true;
    }

    while (1)
    {
        if (xQueueReceive(telemetry_queue, &telemetry_cap_data, 0) != pdPASS)
        {
            mcpwm_capture_channel_trigger_soft_catch(mcpwm_cap_channel_A);
        }
        char *telemetry_to_send = build_telemetry_string();            
        if (telemetry_to_send)
        {
            int err = sendto(cmd.sock, telemetry_to_send, strlen(telemetry_to_send), 0, (struct sockaddr *)&cmd.source_addr, sizeof(struct sockaddr));
            if (err < 0) 
            {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
            }
        }
        free(telemetry_to_send);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void timer_task(void *pvParameters)
{
    uint32_t cycle;
    TickType_t xLastWake = xTaskGetTickCount();

    while(1)
    {
        cycle = count;
        count = 0;
        ESP_LOGI("Timer", "%lu", cycle);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(1000));
    }
}

// Main

void app_main(void)
{
    pins_init();
    pcnt_init();
    mcpwm_pwm_init();
    mcpwm_capture_init();
    gptimer_init();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to install twai driver");
    }

    if (twai_start() != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to start twai");
    }

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

    t_eth_config eth_config = {
        .ip             = g_ip_addr, 
        .pass           = g_pass, 
        .ssid           = g_ssid, 
        .use_eth        = USE_COMMM_ETHERNET, 
        .use_wifi_ap    = USE_COMMM_WIFI_AP, 
        .use_wifi_sta   = USE_COMMM_WIFI_STA
    };
    eth_start(eth_config);

    g_command_queue = xQueueCreate(QUEUE_SIZE, COMMAND_MAX_SIZE);
    assert(g_command_queue != NULL);

    if (USE_COMMM_ETHERNET || USE_COMMM_WIFI_AP || USE_COMMM_WIFI_STA)
    {
        xTaskCreate(command_processing_task, "command_processor", 4096, NULL, 1, NULL);
        xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 1, NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    xTaskCreate(twai_command_task, "Twai", 4096, NULL, 1, NULL);
    t_command cmd;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
 }
