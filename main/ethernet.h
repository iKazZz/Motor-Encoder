#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct 
{
    char use_eth;
    char use_wifi_ap;
    char use_wifi_sta;
    char *ip;
    char *ssid;
    char *pass;
} t_eth_config;

void eth_start(t_eth_config config);

#ifdef __cplusplus
}
#endif
