#ifndef APP_NODE_H
#define APP_NODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "bsp_dht11.h"
#include "bsp_esp01s.h"

typedef enum
{
    APP_POST_ITEM_DELAY = (1UL << 0),
    APP_POST_ITEM_RELAY = (1UL << 1),
    APP_POST_ITEM_MOTOR = (1UL << 2),
    APP_POST_ITEM_DHT11 = (1UL << 3),
    APP_POST_ITEM_ESP01S = (1UL << 4)
} APP_PostItem;

typedef enum
{
    APP_SELF_TEST_ITEM_RELAY1 = (1UL << 0),
    APP_SELF_TEST_ITEM_RELAY2 = (1UL << 1),
    APP_SELF_TEST_ITEM_RELAY3 = (1UL << 2)
} APP_SelfTestItem;

typedef struct
{
    uint32_t passMask;
    uint32_t failMask;
    uint8_t done;
} APP_TestResult;

typedef APP_TestResult APP_PostResult;
typedef APP_TestResult APP_SelfTestResult;

typedef struct
{
    BSP_ESP01S_HandleTypeDef esp;
    BSP_DHT11_HandleTypeDef dht11;
    APP_PostResult post;
    APP_SelfTestResult selfTest;
    uint8_t dht11Valid;
    uint8_t wifiConnected;
    uint8_t uploadEnabled;
    uint8_t fanDutyPercent;
    uint8_t uploadFailStreak;
    uint8_t lastUploadOk;
    uint16_t uploadOkCount;
    uint16_t uploadFailCount;
    uint32_t lastTelemetryTick;
} APP_NodeContext;

HAL_StatusTypeDef APP_Node_Init(APP_NodeContext* node);
void APP_Node_Process(APP_NodeContext* node);
void APP_Node_RunPowerOnSelfTest(APP_NodeContext* node);
void APP_Node_RunHardwareSelfTest(APP_NodeContext* node);
uint8_t APP_Node_IsPostPassed(const APP_NodeContext* node);
uint8_t APP_Node_IsSelfTestPassed(const APP_NodeContext* node);
const APP_PostResult* APP_Node_GetPostResult(const APP_NodeContext* node);
const APP_SelfTestResult* APP_Node_GetSelfTestResult(const APP_NodeContext* node);
void APP_Node_SetDebugUart(UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif /* APP_NODE_H */
