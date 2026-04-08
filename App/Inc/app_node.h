#ifndef APP_NODE_H
#define APP_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_dht11.h"
#include "bsp_esp01s.h"

typedef struct
{
    BSP_ESP01S_HandleTypeDef esp;
    BSP_DHT11_HandleTypeDef dht11;
    uint32_t lastTelemetryTick;
} APP_NodeContext;

HAL_StatusTypeDef APP_Node_Init(APP_NodeContext* node);
void APP_Node_Process(APP_NodeContext* node);

#ifdef __cplusplus
}
#endif

#endif /* APP_NODE_H */
