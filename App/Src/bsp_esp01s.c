#include "bsp_esp01s.h"
#include <string.h>

#define BSP_ESP01S_RX_DMA_BUFFER_SIZE    512U

static uint8_t s_rxDmaBuffer[BSP_ESP01S_RX_DMA_BUFFER_SIZE];
static uint16_t s_rxReadIndex = 0U;
static uint8_t s_rxDmaStarted = 0U;

static uint16_t BSP_ESP01S_GetDmaWriteIndex(const UART_HandleTypeDef* huart)
{
    uint16_t remaining;
    uint16_t writeIndex;

    if ((huart == 0) || (huart->hdmarx == 0))
    {
        return s_rxReadIndex;
    }

    remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (remaining > BSP_ESP01S_RX_DMA_BUFFER_SIZE)
    {
        remaining = BSP_ESP01S_RX_DMA_BUFFER_SIZE;
    }

    writeIndex = (uint16_t)(BSP_ESP01S_RX_DMA_BUFFER_SIZE - remaining);
    if (writeIndex >= BSP_ESP01S_RX_DMA_BUFFER_SIZE)
    {
        writeIndex = 0U;
    }

    return writeIndex;
}

static uint8_t BSP_ESP01S_ReadDmaByte(BSP_ESP01S_HandleTypeDef* esp, uint8_t* data)
{
    uint16_t writeIndex;

    writeIndex = BSP_ESP01S_GetDmaWriteIndex(esp->huart);
    if (s_rxReadIndex == writeIndex)
    {
        return 0U;
    }

    *data = s_rxDmaBuffer[s_rxReadIndex];
    s_rxReadIndex++;
    if (s_rxReadIndex >= BSP_ESP01S_RX_DMA_BUFFER_SIZE)
    {
        s_rxReadIndex = 0U;
    }

    return 1U;
}

static HAL_StatusTypeDef BSP_ESP01S_StartRxDma(BSP_ESP01S_HandleTypeDef* esp)
{
    HAL_StatusTypeDef status;

    if ((esp == 0) || (esp->huart == 0) || (esp->huart->hdmarx == 0))
    {
        return HAL_ERROR;
    }

    if (esp->huart->RxState == HAL_UART_STATE_BUSY_RX)
    {
        s_rxDmaStarted = 1U;
        return HAL_OK;
    }

    s_rxReadIndex = 0U;
    memset(s_rxDmaBuffer, 0, sizeof(s_rxDmaBuffer));

    __HAL_UART_CLEAR_OREFLAG(esp->huart);
    status = HAL_UART_Receive_DMA(esp->huart, s_rxDmaBuffer, BSP_ESP01S_RX_DMA_BUFFER_SIZE);
    if (status == HAL_BUSY)
    {
        s_rxDmaStarted = 1U;
        return HAL_OK;
    }
    if (status != HAL_OK)
    {
        s_rxDmaStarted = 0U;
        return status;
    }

    s_rxDmaStarted = 1U;
    __HAL_DMA_DISABLE_IT(esp->huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(esp->huart->hdmarx, DMA_IT_TC);

    return HAL_OK;
}

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
    return BSP_ESP01S_StartRxDma(esp);
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
    uint16_t received = 0U;
    uint32_t startTick;

    if ((esp == 0) || (esp->huart == 0) || (data == 0) || (length == 0U))
    {
        return HAL_ERROR;
    }

    if (esp->huart->hdmarx == 0)
    {
        return HAL_UART_Receive(esp->huart, data, length, timeout);
    }

    if ((s_rxDmaStarted == 0U) || (esp->huart->RxState != HAL_UART_STATE_BUSY_RX))
    {
        if (BSP_ESP01S_StartRxDma(esp) != HAL_OK)
        {
            return HAL_UART_Receive(esp->huart, data, length, timeout);
        }
    }

    startTick = HAL_GetTick();
    while (received < length)
    {
        if (BSP_ESP01S_ReadDmaByte(esp, &data[received]) != 0U)
        {
            received++;
            continue;
        }

        if (timeout == 0U)
        {
            return HAL_TIMEOUT;
        }

        if ((timeout != HAL_MAX_DELAY) && ((HAL_GetTick() - startTick) >= timeout))
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}
