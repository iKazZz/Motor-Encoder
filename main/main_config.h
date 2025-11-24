#pragma once

#include "hal/gpio_types.h"
#include "driver/twai.h"

#define USE_COMMM_ETHERNET      (0)
#define USE_COMMM_WIFI_AP       (0)
#define USE_COMMM_WIFI_STA      (1)

#define QUEUE_SIZE              (1)
#define COMMAND_MAX_SIZE        (1024)

#define WIFI_AP_SSID            "StepperController"
#define WIFI_AP_PASS            ""
#define WIFI_AP_CHANNEL         (0)

#define DEFAULT_WIFI_STA_SSID   "Aud112"
// #define DEFAULT_WIFI_STA_SSID   "RT-GPON-F6ED"
// #define DEFAULT_WIFI_STA_SSID   "Nadya 5 g"

#define DEFAULT_WIFI_STA_PASS   "justapassword"
// #define DEFAULT_WIFI_STA_PASS   "yPYAEyYsUE"
// #define DEFAULT_WIFI_STA_PASS   "25049025"

#define DEFAULT_STATIC_IP_ADDR  "192.168.1.10"
// #define DEFAULT_STATIC_IP_ADDR  "192.168.0.10"
// #define DEFAULT_STATIC_IP_ADDR  "192.168.0.50"

#define STATIC_NETMASK_ADDR     "255.255.255.0"
#define STATIC_GW_ADDR          "192.168.1.1"
#define PORT                    (4242)

#define NVS_STORAGE             "stepper"
#define NVS_STR_CONFIG          "config"

#define STR_IP_ADDR             "ip_addr"
#define STR_SSID                "ssid"
#define STR_PASS                "password"

#define STR_FLAG_SEND_TELEMETRY "send_telemetry"
#define STR_TELEMETRY           "telemetry"

#define STR_CMD_READ_FLASH      "restore_from_flash"
#define STR_CMD_WRITE_FLASH     "save_to_flash"

#define PIN_PUL                 (5)
#define PIN_DIR                 (2)
//#define PIN_STOP                (15)
//#define PIN_LED                 (2)

#define HOST                    SPI2_HOST
#define PIN_NUM_MISO            (12)
#define PIN_NUM_MOSI            (13)
#define PIN_NUM_CLK             (14)
#define PIN_NUM_SS              (15)

#define DEFAULT_PID_KP          (1)
#define DEFAULT_PID_KI          (0)
#define DEFAULT_PID_KD          (0)

#define STR_PID_KP              "pid_kp"
#define STR_PID_KI              "pid_ki"
#define STR_PID_KD              "pid_kd"

#define GRAPH_ARRAY_SIZE        (200)

#define DEFAULT_DUTY_RESOLUTION_BIT     (4)
#define DEFAULT_TELEMETRY_COUNTER_MAX   (5) // 1 тик = 10 мс
#define DEFAULT_LOG_COUNTER_MAX         (200)
#define DEFAULT_PWM_FREQ_MIN            (200)
#define DEFAULT_PWM_FREQ_MAX            (7000)
#define DEFAULT_ENC_GOAL                (0)
#define DEFAULT_TIMEOUT_COUNTER_MAX     (10000)

#define STR_DUTY_RESOLUTION_BIT         "duty_resolution_bit"
#define STR_TELEMETRY_COUNTER_MAX       "telemetry_counter_max"
#define STR_LOG_COUNTER_MAX             "log_counter_max"
#define STR_PWM_FREQ_MIN                "pwm_freq_min"
#define STR_PWM_FREQ_MAX_STATIC         "pwm_freq_max_static"
#define STR_PWM_FREQ_MAX_DYNAMIC        "pwm_freq_max_dynamic"
#define STR_ENC_GOAL                    "enc_goal"

#if (DEFAULT_DUTY_RESOLUTION_BIT == 1) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_1_BIT 
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 2) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_2_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 3) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_3_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 4) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_4_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 5) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_5_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 6) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_6_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 7) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_7_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 8) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_8_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 9) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_9_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 10) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_10_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 11) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_11_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 12) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_12_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 13) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_13_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 14) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_14_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 15) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_15_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 16) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_16_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 17) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_17_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 18) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_18_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 19) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_19_BIT
#elif (DEFAULT_DUTY_RESOLUTION_BIT == 20) 
    #define DEFAULT_DUTY_RESOLUTION LEDC_TIMER_20_BIT
#endif