#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "main_config.h"
#include "lwip/sockets.h"

typedef struct
{
    int sock;
    struct sockaddr_storage source_addr;
    char cmd[COMMAND_MAX_SIZE];
} t_command;

void udp_server_task(void *pvParameters);

#ifdef __cplusplus
}
#endif
