#pragma once

#include "hal/gpio_types.h"
#include "driver/twai.h"

#define USE_COMMM_ETHERNET      (0)
#define USE_COMMM_WIFI_AP       (0)
#define USE_COMMM_WIFI_STA      (1)

#define QUEUE_SIZE              (1)
#define COMMAND_MAX_SIZE        (1024)

#define WIFI_AP_SSID            "MicroMetron"
#define WIFI_AP_PASS            ""
#define WIFI_AP_CHANNEL         (0)

#define DEFAULT_WIFI_STA_SSID   "Aud112"
// #define DEFAULT_WIFI_STA_SSID   "RT-GPON-F6ED"
// #define DEFAULT_WIFI_STA_SSID   "Nadya 5 g"

#define DEFAULT_WIFI_STA_PASS   "justapassword"
// #define DEFAULT_WIFI_STA_PASS   "yPYAEyYsUE"
// #define DEFAULT_WIFI_STA_PASS   "25049025"

#define DEFAULT_STATIC_IP_ADDR  "192.168.1.11"
// #define DEFAULT_STATIC_IP_ADDR  "192.168.0.10"
// #define DEFAULT_STATIC_IP_ADDR  "192.168.0.50"

#define STATIC_NETMASK_ADDR     "255.255.255.0"
#define STATIC_GW_ADDR          "192.168.1.1"
#define PORT                    (4242)

#define NVS_STORAGE             "MiM"
#define NVS_STR_CONFIG          "config"

#define STR_IP_ADDR             "ip_addr"
#define STR_SSID                "ssid"
#define STR_PASS                "password"

#define STR_ESC_KP              "esc_kp"
#define STR_ESC_KI              "esc_ki"
#define STR_SERVO_KP            "servo_kp"
#define STR_SERVO_KI            "servo_ki"
#define STR_SERVO_KD            "servo_kd"
#define STR_CUR_MAX             "cur_max"



#define STR_FLAG_SEND_TELEMETRY "send_telemetry"
#define STR_TELEMETRY           "telemetry"

#define STR_CMD_READ_FLASH      "restore_from_flash"
#define STR_CMD_WRITE_FLASH     "save_to_flash"

#define PIN_LED                 (25)
#define PIN_SYNC                (18)

#define PIN_RELAY_EN            (26)
#define PIN_DRV_EN              (33)

#define PIN_PWM_A               (21)
#define PIN_PWM_B               (19)
#define PIN_PWM_C               (32)

#define PIN_ENC_A               (14)
#define PIN_ENC_B               (27)
#define PIN_ENC_Z               (13)

#define PIN_OPT_1               (39)
#define PIN_OPT_2               (36)

#define PIN_CAN_TX              (23)
#define PIN_CAN_RX              (22)


#define DUTY_RESOLUTION_BIT     (8)
#define DEFAULT_PAUSE           (100)
#define DEFAULT_CALIBRATION_TIMEOUT (100)

