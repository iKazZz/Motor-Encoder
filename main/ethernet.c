#include "ethernet.h"
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "ethernet_init.h"
#include "main_config.h"
#include "sdkconfig.h"

static const char *TAG = "ethernet";

static char g_ip_addr[64] = "";

static void set_static_ip(esp_netif_t *netif)
{
    if (esp_netif_dhcpc_stop(netif) != ESP_OK) 
    {
        ESP_LOGI(TAG, "Failed to stop dhcp client");
    }

    esp_netif_ip_info_t ip;
    memset(&ip, 0 , sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(g_ip_addr);
    ip.netmask.addr = ipaddr_addr(STATIC_NETMASK_ADDR);
    ip.gw.addr = ipaddr_addr(STATIC_GW_ADDR);

    esp_err_t err = esp_netif_set_ip_info(netif, &ip);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "Failed to set ip info, %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", g_ip_addr, STATIC_NETMASK_ADDR, STATIC_GW_ADDR);
}

/** Event handler for Ethernet events */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        set_static_ip(arg);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    switch (event_id) 
    {
    case WIFI_EVENT_STA_START:
        ESP_LOGI(TAG, "Wifi STA starting");
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_CONNECTED:
        ESP_LOGI(TAG, "Wifi STA connected");
        set_static_ip(arg);
        break;
    case WIFI_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "Wifi AP connected");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        ESP_LOGI(TAG, "Wifi STA disconnected");
        esp_wifi_connect();
        break;
    case WIFI_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "Wifi AP disconnected");
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
}

void eth_start(t_eth_config config)
{
    uint8_t eth_port_cnt = 0;
    esp_eth_handle_t *eth_handles;
    esp_netif_t *sta_netif, *ap_netif;

    strncpy(g_ip_addr, config.ip, sizeof(g_ip_addr));
    
    if (config.use_eth)
    {
        // Initialize Ethernet driver
        ESP_ERROR_CHECK(eth_init(&eth_handles, &eth_port_cnt));
    }

    // Initialize TCP/IP network interface aka the esp-netif (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    if (config.use_wifi_ap)
    {
        // Initialize WiFi AP
        ap_netif = esp_netif_create_default_wifi_ap();
        assert(ap_netif != NULL);

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, ap_netif, NULL));
    }

    if (config.use_wifi_sta)
    {
        // Initialize WiFi STA
        sta_netif = esp_netif_create_default_wifi_sta();
        assert(sta_netif != NULL);

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, sta_netif, NULL));
    }

    if (config.use_eth)
    {
        esp_netif_t *eth_netifs[eth_port_cnt];
        esp_eth_netif_glue_handle_t eth_netif_glues[eth_port_cnt];

        // Create instance(s) of esp-netif for Ethernet(s)
        if (eth_port_cnt == 1) 
        {
            // Use ESP_NETIF_DEFAULT_ETH when just one Ethernet interface is used and you don't need to modify
            // default esp-netif configuration parameters.
            esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
            eth_netifs[0] = esp_netif_new(&cfg);
            eth_netif_glues[0] = esp_eth_new_netif_glue(eth_handles[0]);
            // Attach Ethernet driver to TCP/IP stack
            ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[0], eth_netif_glues[0]));
        } 
        else 
        {
            // Use ESP_NETIF_INHERENT_DEFAULT_ETH when multiple Ethernet interfaces are used and so you need to modify
            // esp-netif configuration parameters for each interface (name, priority, etc.).
            esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
            esp_netif_config_t cfg_spi = {
                .base = &esp_netif_config,
                .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
            };
            char if_key_str[10];
            char if_desc_str[10];
            char num_str[3];
            for (int i = 0; i < eth_port_cnt; i++) {
                itoa(i, num_str, 10);
                strcat(strcpy(if_key_str, "ETH_"), num_str);
                strcat(strcpy(if_desc_str, "eth"), num_str);
                esp_netif_config.if_key = if_key_str;
                esp_netif_config.if_desc = if_desc_str;
                esp_netif_config.route_prio -= i*5;
                eth_netifs[i] = esp_netif_new(&cfg_spi);
                eth_netif_glues[i] = esp_eth_new_netif_glue(eth_handles[0]);
                // Attach Ethernet driver to TCP/IP stack
                ESP_ERROR_CHECK(esp_netif_attach(eth_netifs[i], eth_netif_glues[i]));
            }
        }

        // Register ethernet event handers
        ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, eth_netifs[0]));
    }

    // Register IP event handers
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    if (config.use_eth)
    {
        // Start Ethernet driver state machine
        for (int i = 0; i < eth_port_cnt; i++) 
        {
            ESP_ERROR_CHECK(esp_eth_start(eth_handles[i]));
        }
    }

    if (config.use_wifi_ap)
    {
            wifi_config_t wifi_config = {
            .ap = {
                .ssid = WIFI_AP_SSID,
                .ssid_len = strlen(WIFI_AP_SSID),
                .channel = WIFI_AP_CHANNEL,
                .password = WIFI_AP_PASS,
                .max_connection = 1,
    #ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
                .authmode = WIFI_AUTH_WPA3_PSK,
                .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
    #else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
                .authmode = WIFI_AUTH_WPA2_PSK,
    #endif
                .pmf_cfg = {
                        .required = true,
                },
            },
        };

        if (strlen(WIFI_AP_PASS) == 0) 
        {
            wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        }

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

        ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
        esp_netif_ip_info_t ip;
        memset(&ip, 0 , sizeof(esp_netif_ip_info_t));
        ip.ip.addr = ipaddr_addr(config.ip);
        ip.netmask.addr = ipaddr_addr(STATIC_NETMASK_ADDR);
        ip.gw.addr = ipaddr_addr(STATIC_GW_ADDR);

        ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip));
        ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

        ESP_ERROR_CHECK(esp_wifi_start());
    }

    if (config.use_wifi_sta)
    {
        wifi_config_t wifi_config = {
            .sta = {
                .ssid = DEFAULT_WIFI_STA_SSID,
                .password = DEFAULT_WIFI_STA_PASS,
                /* Setting a password implies station will connect to all security modes including WEP/WPA.
                * However these modes are deprecated and not advisable to be used. In case your Access point
                * doesn't support WPA2, these mode can be enabled by commenting below line */
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,
                },
        };

        strncpy((char*)wifi_config.sta.ssid, config.ssid, sizeof(wifi_config.sta.ssid));
        strncpy((char*)wifi_config.sta.password, config.pass, sizeof(wifi_config.sta.password));

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
        ESP_ERROR_CHECK(esp_wifi_start() );
    }
}