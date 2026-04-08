#include "bsp_esp01s.h"
#include <string.h>

HAL_StatusTypeDef BSP_ESP01S_Init(BSP_ESP01S_HandleTypeDef* esp,
                                  UART_HandleTypeDef* huart,
                                  GPIO_TypeDef* rstPort,
                                  uint16_t rstPin,
                                  GPIO_TypeDef* enPort,
                                  uint16_t enPin)
{
    if ((esp == 0) || (huart == 0) || (rstPort == 0) || (enPort == 0))
    {
        return HAL_ERROR;
    }

    esp->huart = huart;
    esp->rstPort = rstPort;
    esp->rstPin = rstPin;
    esp->enPort = enPort;
    esp->enPin = enPin;

    HAL_GPIO_WritePin(esp->rstPort, esp->rstPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(esp->enPort, esp->enPin, GPIO_PIN_SET);
    return HAL_OK;
}

void BSP_ESP01S_SetEnable(BSP_ESP01S_HandleTypeDef* esp, uint8_t enable)
{
    if (esp == 0)
    {
        return;
    }

    HAL_GPIO_WritePin(esp->enPort, esp->enPin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BSP_ESP01S_Reset(BSP_ESP01S_HandleTypeDef* esp)
{
    if (esp == 0)
    {
        return;
    }

    HAL_GPIO_WritePin(esp->rstPort, esp->rstPin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(esp->rstPort, esp->rstPin, GPIO_PIN_SET);
    HAL_Delay(200);
}

HAL_StatusTypeDef BSP_ESP01S_Send(BSP_ESP01S_HandleTypeDef* esp,
                                  const uint8_t* data,
                                  uint16_t length,
                                  uint32_t timeout)
{
    if ((esp == 0) || (esp->huart == 0) || (data == 0) || (length == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(esp->huart, (uint8_t*)data, length, timeout);
}

HAL_StatusTypeDef BSP_ESP01S_SendString(BSP_ESP01S_HandleTypeDef* esp,
                                        const char* text,
                                        uint32_t timeout)
{
    if (text == 0)
    {
        return HAL_ERROR;
    }

    return BSP_ESP01S_Send(esp, (const uint8_t*)text, (uint16_t)strlen(text), timeout);
}

HAL_StatusTypeDef BSP_ESP01S_Receive(BSP_ESP01S_HandleTypeDef* esp,
                                     uint8_t* data,
                                     uint16_t length,
                                     uint32_t timeout)
{
    if ((esp == 0) || (esp->huart == 0) || (data == 0) || (length == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive(esp->huart, data, length, timeout);
}
