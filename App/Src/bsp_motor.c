#include "bsp_motor.h"
#include "main.h"

#define MOTOR_AIN1_PORT GPIOA
#define MOTOR_AIN1_PIN  GPIO_PIN_4
#define MOTOR_AIN2_PORT GPIOA
#define MOTOR_AIN2_PIN  GPIO_PIN_5
#define MOTOR_STBY_PORT GPIOA
#define MOTOR_STBY_PIN  GPIO_PIN_7
#define MOTOR_BIN1_PORT GPIOB
#define MOTOR_BIN1_PIN  GPIO_PIN_13
#define MOTOR_BIN2_PORT GPIOB
#define MOTOR_BIN2_PIN  GPIO_PIN_14

static TIM_HandleTypeDef* s_motorTim = 0;

static void motor_write_direction(BSP_MotorChannel channel, BSP_MotorDirection direction)
{
    GPIO_PinState pin1 = GPIO_PIN_RESET;
    GPIO_PinState pin2 = GPIO_PIN_RESET;

    if (direction == BSP_MOTOR_FORWARD)
    {
        pin1 = GPIO_PIN_SET;
        pin2 = GPIO_PIN_RESET;
    }
    else if (direction == BSP_MOTOR_REVERSE)
    {
        pin1 = GPIO_PIN_RESET;
        pin2 = GPIO_PIN_SET;
    }

    if (channel == BSP_MOTOR_CHANNEL_A)
    {
        HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, pin1);
        HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, pin2);
    }
    else
    {
        HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, pin1);
        HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, pin2);
    }
}

HAL_StatusTypeDef BSP_Motor_Init(TIM_HandleTypeDef* htim)
{
    HAL_StatusTypeDef status;

    if (htim == 0)
    {
        return HAL_ERROR;
    }

    s_motorTim = htim;

    status = HAL_TIM_PWM_Start(s_motorTim, TIM_CHANNEL_3);
    if (status != HAL_OK)
    {
        s_motorTim = 0;
        return status;
    }

    status = HAL_TIM_PWM_Start(s_motorTim, TIM_CHANNEL_4);
    if (status != HAL_OK)
    {
        (void)HAL_TIM_PWM_Stop(s_motorTim, TIM_CHANNEL_3);
        s_motorTim = 0;
        return status;
    }

    BSP_Motor_StopAll();
    BSP_Motor_SetStandby(1U);
    return HAL_OK;
}

void BSP_Motor_SetStandby(uint8_t enable)
{
    HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

HAL_StatusTypeDef BSP_Motor_Set(BSP_MotorChannel channel,
                                BSP_MotorDirection direction,
                                uint16_t pulse)
{
    uint32_t timChannel;

    if (s_motorTim == 0)
    {
        return HAL_ERROR;
    }

    if ((channel != BSP_MOTOR_CHANNEL_A) && (channel != BSP_MOTOR_CHANNEL_B))
    {
        return HAL_ERROR;
    }

    if ((direction != BSP_MOTOR_STOP) &&
        (direction != BSP_MOTOR_FORWARD) &&
        (direction != BSP_MOTOR_REVERSE))
    {
        return HAL_ERROR;
    }

    timChannel = (channel == BSP_MOTOR_CHANNEL_A) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
    motor_write_direction(channel, direction);
    __HAL_TIM_SET_COMPARE(s_motorTim, timChannel, pulse);
    return HAL_OK;
}

void BSP_Motor_StopAll(void)
{
    if (s_motorTim == 0)
    {
        return;
    }

    motor_write_direction(BSP_MOTOR_CHANNEL_A, BSP_MOTOR_STOP);
    motor_write_direction(BSP_MOTOR_CHANNEL_B, BSP_MOTOR_STOP);
    __HAL_TIM_SET_COMPARE(s_motorTim, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(s_motorTim, TIM_CHANNEL_4, 0U);
}
