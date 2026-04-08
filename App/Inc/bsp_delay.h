#ifndef BSP_DELAY_H
#define BSP_DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "tim.h"

HAL_StatusTypeDef BSP_Delay_Init(TIM_HandleTypeDef* htim);
void BSP_Delay_Us(uint16_t us);
uint16_t BSP_Delay_GetCounter(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DELAY_H */
