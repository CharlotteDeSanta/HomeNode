#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "tim.h"

typedef enum
{
    BSP_MOTOR_CHANNEL_A = 0,
    BSP_MOTOR_CHANNEL_B
} BSP_MotorChannel;

typedef enum
{
    BSP_MOTOR_STOP = 0,
    BSP_MOTOR_FORWARD,
    BSP_MOTOR_REVERSE
} BSP_MotorDirection;

HAL_StatusTypeDef BSP_Motor_Init(TIM_HandleTypeDef* htim);
void BSP_Motor_SetStandby(uint8_t enable);
HAL_StatusTypeDef BSP_Motor_Set(BSP_MotorChannel channel,
                                BSP_MotorDirection direction,
                                uint16_t pulse);
void BSP_Motor_StopAll(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_MOTOR_H */
