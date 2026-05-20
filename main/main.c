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
// #include "driver/twai.h"
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
#include "servosila_sc.h"
#include <cJSON.h>
#include <math.h>


#ifndef max
    #define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
    #define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#if (DUTY_RESOLUTION_BIT == 1) 
    #define DUTY_RESOLUTION LEDC_TIMER_1_BIT 
#elif (DUTY_RESOLUTION_BIT == 2) 
    #define DUTY_RESOLUTION LEDC_TIMER_2_BIT
#elif (DUTY_RESOLUTION_BIT == 3) 
    #define DUTY_RESOLUTION LEDC_TIMER_3_BIT
#elif (DUTY_RESOLUTION_BIT == 4) 
    #define DUTY_RESOLUTION LEDC_TIMER_4_BIT
#elif (DUTY_RESOLUTION_BIT == 5) 
    #define DUTY_RESOLUTION LEDC_TIMER_5_BIT
#elif (DUTY_RESOLUTION_BIT == 6) 
    #define DUTY_RESOLUTION LEDC_TIMER_6_BIT
#elif (DUTY_RESOLUTION_BIT == 7) 
    #define DUTY_RESOLUTION LEDC_TIMER_7_BIT
#elif (DUTY_RESOLUTION_BIT == 8) 
    #define DUTY_RESOLUTION LEDC_TIMER_8_BIT
#elif (DUTY_RESOLUTION_BIT == 9) 
    #define DUTY_RESOLUTION LEDC_TIMER_9_BIT
#elif (DUTY_RESOLUTION_BIT == 10) 
    #define DUTY_RESOLUTION LEDC_TIMER_10_BIT
#elif (DUTY_RESOLUTION_BIT == 11) 
    #define DUTY_RESOLUTION LEDC_TIMER_11_BIT
#elif (DUTY_RESOLUTION_BIT == 12) 
    #define DUTY_RESOLUTION LEDC_TIMER_12_BIT
#elif (DUTY_RESOLUTION_BIT == 13) 
    #define DUTY_RESOLUTION LEDC_TIMER_13_BIT
#elif (DUTY_RESOLUTION_BIT == 14) 
    #define DUTY_RESOLUTION LEDC_TIMER_14_BIT
#elif (DUTY_RESOLUTION_BIT == 15) 
    #define DUTY_RESOLUTION LEDC_TIMER_15_BIT
#elif (DUTY_RESOLUTION_BIT == 16) 
    #define DUTY_RESOLUTION LEDC_TIMER_16_BIT
#elif (DUTY_RESOLUTION_BIT == 17) 
    #define DUTY_RESOLUTION LEDC_TIMER_17_BIT
#elif (DUTY_RESOLUTION_BIT == 18) 
    #define DUTY_RESOLUTION LEDC_TIMER_18_BIT
#elif (DUTY_RESOLUTION_BIT == 19) 
    #define DUTY_RESOLUTION LEDC_TIMER_19_BIT
#elif (DUTY_RESOLUTION_BIT == 20) 
    #define DUTY_RESOLUTION LEDC_TIMER_20_BIT
#endif

// FOC

typedef struct foc_uvw_coord {
    float u;
    float v;
    float w;
} foc_uvw_coord_t;

typedef struct foc_ab_coord {
    float alpha;
    float beta;
} foc_ab_coord_t;

typedef struct foc_dq_coord {
    float d;
    float q;
} foc_dq_coord_t;

// Pulse counter

pcnt_unit_config_t pcnt_unit_config = {
    .low_limit = -10000,
    .high_limit = 10000
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

// MCPWM

const uint32_t mcpwm_res = 20000000;
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

// PID

float pid_kp = 0;
float pid_ki = 0.0;
float pid_kd = 0.0;

static float u_integral_max = 100; 
static float i_term_max = 100; 
signed int goal_pos = 0;

void nvs_read_config();
void nvs_write_config();
char* build_config_string(bool for_nvs);

static const char *TAG = "step_controller";

char g_ip_addr[64]  = DEFAULT_STATIC_IP_ADDR;
char g_ssid[64]     = DEFAULT_WIFI_STA_SSID;
char g_pass[64]     = DEFAULT_WIFI_STA_PASS;


signed int encoder_pos = 0;
unsigned int dead_zone = 0;


float p_term = 0;
float i_term = 0;
float d_term = 0;
int graph_count = 0;

int log_count = 0;
int time_count = 0;
static int time_count_max = 1000;
int telemetry_counter = 0;   
static bool timer_paused = false; 

QueueHandle_t g_command_queue;
TickType_t xLastWakeTime;

bool g_flag_send_telemetry = true;

volatile unsigned long g_pause_ms = DEFAULT_PAUSE;
volatile unsigned long g_calibration_timeout_ms = DEFAULT_CALIBRATION_TIMEOUT;

struct sockaddr_storage g_last_cmd_source_addr; // TODO: mutex protect

gptimer_handle_t g_gptimer;

void append_telemetry_data(cJSON *json)
{
    cJSON_AddStringToObject(json, STR_TELEMETRY, "true");
    //cJSON_AddNumberToObject(json, "graph_count", graph_count);
    cJSON_AddNumberToObject(json, "encoder_pos", encoder_pos);
    cJSON_AddNumberToObject(json, "time_count", time_count);
    // cJSON_AddNumberToObject(json, "u", u);
    // cJSON_AddNumberToObject(json, "duty_ratio", duty_ratio);

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
    cJSON_AddNumberToObject(json, "kp", pid_kp);
    cJSON_AddNumberToObject(json, "ki", pid_ki);
    cJSON_AddNumberToObject(json, "kd", pid_kd);
    cJSON_AddNumberToObject(json, "dead_zone", dead_zone);
    cJSON_AddNumberToObject(json, "time_count_max", time_count_max);
    cJSON_AddNumberToObject(json, "i_term_max", i_term_max);
    // cJSON_AddNumberToObject(json, "duty", duty);

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

void IRAM_ATTR sense_stop_isr(void *arg)
{
    BaseType_t flag_yield = 0;

    xTaskNotifyFromISR((TaskHandle_t)arg, 0, 0, &flag_yield);

    portYIELD_FROM_ISR(flag_yield);
}

void init_pins()
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

void foc_inverse_park_transform(float phi, foc_dq_coord_t *dq, foc_ab_coord_t *ab)
{
    ab->alpha = dq->d * cos(phi) - dq->q * sin(phi);
    ab->beta  = dq->q * cos(phi) + dq->d * sin(phi);
}

void foc_inverse_clark_transform(foc_ab_coord_t *ab, foc_uvw_coord_t *uvw)
{
    uvw->u = ab->alpha;
    uvw->v = (ab->beta * sqrt(3) - ab->alpha) / 2;
    uvw->w = -uvw->u - uvw->v; 
}

void pcnt_init()
{
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_config, &pcnt_unit));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &pcnt_chan_a_config, &pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &pcnt_chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &pcnt_gf_config));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

void mcpwm_init()
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
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mcpwm_timer, MCPWM_TIMER_START_NO_STOP));
}

void app_main(void)
{
    xLastWakeTime = xTaskGetTickCount();
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

    pcnt_init();

    mcpwm_init();

    int delay_ms = 15;
    int foc_duty_arr[3] = {0, 0, 0};
    float foc_el_phi_deg = 0;
    float foc_el_phi_rad = 0;
    float foc_el_freq = 4;

    foc_uvw_coord_t foc_uvw_coord = {0, 0, 0};
    foc_ab_coord_t foc_ab_coord = {0, 0};
    foc_dq_coord_t foc_dq_coord = {1, 0};

    int log_timer = 0;
    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();
        foc_el_phi_deg += (360 * foc_el_freq / 1000 * delay_ms);
        if(foc_el_phi_deg >= 360)
        {
            foc_el_phi_deg -= 360;
        }
        // foc_el_phi_deg = 0;
        foc_el_phi_rad = foc_el_phi_deg * M_PI / 180;
        // ESP_LOGI("FOC", "d = %.2f, q = %.2f", foc_dq_coord.d, foc_dq_coord.q);
        foc_inverse_park_transform(foc_el_phi_rad, &foc_dq_coord, &foc_ab_coord);
        // ESP_LOGI("FOC", "a = %.2f, b = %.2f", foc_ab_coord.alpha, foc_ab_coord.beta);
        foc_inverse_clark_transform(&foc_ab_coord, &foc_uvw_coord);
        // ESP_LOGI("FOC", "u = %.2f, v = %.2f, w = %.2f", foc_uvw_coord.u, foc_uvw_coord.v, foc_uvw_coord.w);

        foc_duty_arr[0] = (int)(mcpwm_per * (foc_uvw_coord.u / 4 + 1.0 / 4));
        foc_duty_arr[1] = (int)(mcpwm_per * (foc_uvw_coord.v / 4 + 1.0 / 4));
        foc_duty_arr[2] = (int)(mcpwm_per * (foc_uvw_coord.w / 4 + 1.0 / 4));

        for (int i = 0; i < 3; i++)
        {
            ESP_LOGI("ABC", "A = %d, B = %d, C = %d", foc_duty_arr[0], foc_duty_arr[1], foc_duty_arr[2]);
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(mcpwm_comparators[i], foc_duty_arr[i]));
        }

        int count;
        if (log_timer == 50)
        {
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &count));
            ESP_LOGI("Encoder", "count = %d", count);
            log_timer = 0;
        }
        else
        {
            log_timer++;
        }


        // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(delay_ms));
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
 }
