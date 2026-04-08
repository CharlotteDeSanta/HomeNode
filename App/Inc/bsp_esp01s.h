#ifndef BSP_ESP01S_H
#define BSP_ESP01S_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "usart.h"

typedef struct
{
    UART_HandleTypeDef* huart;
    GPIO_TypeDef* rstPort;
    uint16_t rstPin;
    GPIO_TypeDef* enPort;
    uint16_t enPin;
} BSP_ESP01S_HandleTypeDef;

HAL_StatusTypeDef BSP_ESP01S_Init(BSP_ESP01S_HandleTypeDef* esp,
                                  UART_HandleTypeDef* huart,
                                  GPIO_TypeDef* rstPort,
                                  uint16_t rstPin,
                                  GPIO_TypeDef* enPort,
                                  uint16_t enPin);
void BSP_ESP01S_SetEnable(BSP_ESP01S_HandleTypeDef* esp, uint8_t enable);
void BSP_ESP01S_Reset(BSP_ESP01S_HandleTypeDef* esp);
HAL_StatusTypeDef BSP_ESP01S_Send(BSP_ESP01S_HandleTypeDef* esp,
                                  const uint8_t* data,
                                  uint16_t length,
                                  uint32_t timeout);
HAL_StatusTypeDef BSP_ESP01S_SendString(BSP_ESP01S_HandleTypeDef* esp,
                                        const char* text,
                                        uint32_t timeout);
HAL_StatusTypeDef BSP_ESP01S_Receive(BSP_ESP01S_HandleTypeDef* esp,
                                     uint8_t* data,
                                     uint16_t length,
                                     uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ESP01S_H */
