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
// #define STATIC_GW_ADDR          "192.168.0.10"
#define PORT                    (4242)

#define NVS_STORAGE             "stepper"
#define NVS_STR_CONFIG          "config"

#define STR_IP_ADDR             "ip_addr"
#define STR_SSID                "ssid"
#define STR_PASS                "password"

#define STR_STEP_PERIOD_FWD     "step_period_fwd_us"
#define STR_STEP_PERIOD_BWD     "step_period_bwd_us"
#define STR_STEPS               "steps"
#define STR_PAUSE               "pause_ms"
#define STR_CALIBRATION_TIMEOUT "calibration_timeout_ms"

#define STR_FLAG_SEND_TELEMETRY "send_telemetry"
#define STR_TELEMETRY           "telemetry"

#define STR_CMD_READ_FLASH      "restore_from_flash"
#define STR_CMD_WRITE_FLASH     "save_to_flash"

#define PIN_STEP                 (32)
#define PIN_DIR                  (33)

#define PIN_STOP                 (15)

#define PIN_LED                  (2)

#define PIN_A                    (25)
#define PIN_B                    (26)


#define DEFAULT_STEP_PERIOD_US  (1000)
#define DEFAULT_STEPS           (3000)
#define DEFAULT_PAUSE           (100)
#define DEFAULT_CALIBRATION_TIMEOUT (100)
