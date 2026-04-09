#include "app_node.h"
#include <string.h>
#include "bsp_delay.h"
#include "bsp_motor.h"
#include "bsp_relay.h"
#include "tim.h"
#include "usart.h"

#define APP_BUTTON_COUNT       4U
#define APP_BUTTON_DEBOUNCE_MS 30U

typedef struct
{
    uint16_t pin;
    uint8_t stableState;
    uint8_t lastSample;
    uint32_t lastChangeTick;
} APP_ButtonState;

static APP_ButtonState s_buttons[APP_BUTTON_COUNT] =
{
    {GPIO_PIN_0, 0U, 0U, 0U},
    {GPIO_PIN_1, 0U, 0U, 0U},
    {GPIO_PIN_2, 0U, 0U, 0U},
    {GPIO_PIN_3, 0U, 0U, 0U}
};

static uint8_t s_relayState[3] = {0U, 0U, 0U};
static uint8_t s_fanEnabled = 0U;

static uint8_t app_read_button(uint16_t pin)
{
    return (HAL_GPIO_ReadPin(GPIOA, pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static uint8_t app_button_pressed_event(APP_ButtonState* button, uint32_t nowTick)
{
    uint8_t sample = app_read_button(button->pin);

    if (sample != button->lastSample)
    {
        button->lastSample = sample;
        button->lastChangeTick = nowTick;
    }

    if ((nowTick - button->lastChangeTick) >= APP_BUTTON_DEBOUNCE_MS)
    {
        if (button->stableState != sample)
        {
            uint8_t previous = button->stableState;
            button->stableState = sample;

            if ((previous == 0U) && (sample == 1U))
            {
                return 1U;
            }
        }
    }

    return 0U;
}

static void app_apply_relay_state(uint8_t relayIndex)
{
    (void)BSP_Relay_Set((BSP_RelayChannel)relayIndex, s_relayState[relayIndex]);
}

static void app_apply_fan_state(void)
{
    uint16_t pulse;

    if (s_fanEnabled == 0U)
    {
        (void)BSP_Motor_Set(BSP_MOTOR_CHANNEL_A, BSP_MOTOR_STOP, 0U);
        BSP_Motor_SetStandby(0U);
        return;
    }

    BSP_Motor_SetStandby(1U);
    pulse = (uint16_t)(((__HAL_TIM_GET_AUTORELOAD(&htim3) + 1U) * 60U) / 100U);
    if (pulse == 0U)
    {
        pulse = 1U;
    }

    (void)BSP_Motor_Set(BSP_MOTOR_CHANNEL_A, BSP_MOTOR_FORWARD, pulse);
}

HAL_StatusTypeDef APP_Node_Init(APP_NodeContext* node)
{
    HAL_StatusTypeDef status;
    uint32_t nowTick;
    uint32_t index;

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
    memset(s_relayState, 0, sizeof(s_relayState));
    s_fanEnabled = 0U;

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

    app_apply_fan_state();

    nowTick = HAL_GetTick();
    for (index = 0U; index < APP_BUTTON_COUNT; index++)
    {
        s_buttons[index].stableState = app_read_button(s_buttons[index].pin);
        s_buttons[index].lastSample = s_buttons[index].stableState;
        s_buttons[index].lastChangeTick = nowTick;
    }

    return HAL_OK;
}

void APP_Node_Process(APP_NodeContext* node)
{
    uint32_t nowTick;

    if (node == 0)
    {
        return;
    }

    nowTick = HAL_GetTick();

    if (app_button_pressed_event(&s_buttons[0], nowTick) != 0U)
    {
        s_relayState[0] ^= 1U;
        app_apply_relay_state(0U);
    }

    if (app_button_pressed_event(&s_buttons[1], nowTick) != 0U)
    {
        s_relayState[1] ^= 1U;
        app_apply_relay_state(1U);
    }

    if (app_button_pressed_event(&s_buttons[2], nowTick) != 0U)
    {
        s_relayState[2] ^= 1U;
        app_apply_relay_state(2U);
    }

    if (app_button_pressed_event(&s_buttons[3], nowTick) != 0U)
    {
        s_fanEnabled ^= 1U;
        app_apply_fan_state();
    }

    /* TODO: poll DHT11, update local state, and exchange frames with ESP-01S. */
}
