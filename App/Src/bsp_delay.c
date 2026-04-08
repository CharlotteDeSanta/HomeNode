#include "bsp_delay.h"

static TIM_HandleTypeDef* s_delayTim = 0;

HAL_StatusTypeDef BSP_Delay_Init(TIM_HandleTypeDef* htim)
{
    if (htim == 0)
    {
        s_delayTim = 0;
        return HAL_ERROR;
    }

    s_delayTim = htim;
    if (HAL_TIM_Base_Start(s_delayTim) != HAL_OK)
    {
        s_delayTim = 0;
        return HAL_ERROR;
    }

    return HAL_OK;
}

void BSP_Delay_Us(uint16_t us)
{
    uint32_t timeoutMs;
    uint32_t lastProgressTick;
    uint16_t lastCounter;

    if (s_delayTim == 0)
    {
        return;
    }

    if (us == 0U)
    {
        return;
    }

    __HAL_TIM_SET_COUNTER(s_delayTim, 0);
    timeoutMs = ((uint32_t)us / 1000U) + 2U;
    lastProgressTick = HAL_GetTick();
    lastCounter = (uint16_t)__HAL_TIM_GET_COUNTER(s_delayTim);

    while (1)
    {
        uint16_t counter = (uint16_t)__HAL_TIM_GET_COUNTER(s_delayTim);
        if (counter >= us)
        {
            break;
        }

        if (counter != lastCounter)
        {
            lastCounter = counter;
            lastProgressTick = HAL_GetTick();
        }
        else if ((HAL_GetTick() - lastProgressTick) > timeoutMs)
        {
            break;
        }
    }
}

uint16_t BSP_Delay_GetCounter(void)
{
    if (s_delayTim == 0)
    {
        return 0;
    }

    return (uint16_t)__HAL_TIM_GET_COUNTER(s_delayTim);
}
