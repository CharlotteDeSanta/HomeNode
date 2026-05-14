#include "bsp_relay.h"

#define RELAY1_PORT GPIOB
#define RELAY1_PIN  GPIO_PIN_3
#define RELAY2_PORT GPIOB
#define RELAY2_PIN  GPIO_PIN_4
#define RELAY3_PORT GPIOB
#define RELAY3_PIN  GPIO_PIN_5
#define RELAY_ACTIVE_STATE   GPIO_PIN_SET
#define RELAY_INACTIVE_STATE GPIO_PIN_RESET

void BSP_Relay_Init(void)
{
    BSP_Relay_SetAll(0U);
}

HAL_StatusTypeDef BSP_Relay_Set(BSP_RelayChannel channel, uint8_t enable)
{
    GPIO_TypeDef* port;
    uint16_t pin;

    switch (channel)
    {
    case BSP_RELAY_1:
        port = RELAY1_PORT;
        pin = RELAY1_PIN;
        break;
    case BSP_RELAY_2:
        port = RELAY2_PORT;
        pin = RELAY2_PIN;
        break;
    case BSP_RELAY_3:
        port = RELAY3_PORT;
        pin = RELAY3_PIN;
        break;
    default:
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(port, pin, enable ? RELAY_ACTIVE_STATE : RELAY_INACTIVE_STATE);
    return HAL_OK;
}

void BSP_Relay_SetAll(uint8_t enable)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, enable ? RELAY_ACTIVE_STATE : RELAY_INACTIVE_STATE);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, enable ? RELAY_ACTIVE_STATE : RELAY_INACTIVE_STATE);
    HAL_GPIO_WritePin(RELAY3_PORT, RELAY3_PIN, enable ? RELAY_ACTIVE_STATE : RELAY_INACTIVE_STATE);
}
