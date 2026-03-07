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