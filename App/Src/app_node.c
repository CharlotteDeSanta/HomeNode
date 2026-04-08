#include "app_node.h"
#include <string.h>
#include "bsp_delay.h"
#include "bsp_motor.h"
#include "bsp_relay.h"
#include "tim.h"
#include "usart.h"

HAL_StatusTypeDef APP_Node_Init(APP_NodeContext* node)
{
    HAL_StatusTypeDef status;

    if (node == 0)
    {
        return HAL_ERROR;
    }

    memset(node, 0, sizeof(*node));

    status = BSP_Delay_Init(&htim2);
    if (status != HAL_OK)
    {
        return status;
    }

    BSP_Relay_Init();

    status = BSP_Motor_Init(&htim3);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BSP_DHT11_Init(&node->dht11, GPIOA, GPIO_PIN_6);
    if (status != HAL_OK)
    {
        return status;
    }

    status = BSP_ESP01S_Init(&node->esp, &huart3, GPIOB, GPIO_PIN_8, GPIOB, GPIO_PIN_9);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_OK;
}

void APP_Node_Process(APP_NodeContext* node)
{
    if (node == 0)
    {
        return;
    }

    /* TODO: poll DHT11, update local state, and exchange frames with ESP-01S. */
}
