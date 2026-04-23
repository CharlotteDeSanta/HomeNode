#include "bsp_dht11.h"
#include "bsp_delay.h"

static void dht_pin_as_output(GPIO_TypeDef* port, uint16_t pin)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &gpio);
}

static void dht_pin_as_input(GPIO_TypeDef* port, uint16_t pin)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(port, &gpio);
}

static HAL_StatusTypeDef dht_wait_for_state(const BSP_DHT11_HandleTypeDef* dht,
                                            GPIO_PinState state,
                                            uint16_t timeoutUs)
{
    uint16_t startCounter;

    if ((dht == 0) || (dht->port == 0))
    {
        return HAL_ERROR;
    }

    startCounter = BSP_Delay_GetCounter();
    while (HAL_GPIO_ReadPin(dht->port, dht->pin) != state)
    {
        if ((uint16_t)(BSP_Delay_GetCounter() - startCounter) > timeoutUs)
        {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef dht_measure_high_pulse_us(const BSP_DHT11_HandleTypeDef* dht,
                                                   uint16_t* highUs,
                                                   uint16_t timeoutUs)
{
    uint16_t startCounter;
    uint16_t durationUs;

    if ((dht == 0) || (dht->port == 0) || (highUs == 0))
    {
        return HAL_ERROR;
    }

    if (dht_wait_for_state(dht, GPIO_PIN_SET, timeoutUs) != HAL_OK)
    {
        return HAL_TIMEOUT;
    }

    startCounter = BSP_Delay_GetCounter();
    while (HAL_GPIO_ReadPin(dht->port, dht->pin) == GPIO_PIN_SET)
    {
        durationUs = (uint16_t)(BSP_Delay_GetCounter() - startCounter);
        if (durationUs > timeoutUs)
        {
            return HAL_TIMEOUT;
        }
    }

    *highUs = (uint16_t)(BSP_Delay_GetCounter() - startCounter);
    return HAL_OK;
}

static void dht_send_start_signal(BSP_DHT11_HandleTypeDef* dht)
{
    dht_pin_as_output(dht->port, dht->pin);
    HAL_GPIO_WritePin(dht->port, dht->pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(dht->port, dht->pin, GPIO_PIN_SET);
    BSP_Delay_Us(30);
    dht_pin_as_input(dht->port, dht->pin);
}

HAL_StatusTypeDef BSP_DHT11_Init(BSP_DHT11_HandleTypeDef* dht,
                                 GPIO_TypeDef* port,
                                 uint16_t pin)
{
    if ((dht == 0) || (port == 0))
    {
        return HAL_ERROR;
    }

    dht->port = port;
    dht->pin = pin;
    dht->humidityInt = 0U;
    dht->humidityDec = 0U;
    dht->temperatureInt = 0U;
    dht->temperatureDec = 0U;

    dht_pin_as_output(port, pin);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    return HAL_OK;
}

HAL_StatusTypeDef BSP_DHT11_Read(BSP_DHT11_HandleTypeDef* dht)
{
    uint8_t data[5] = {0U, 0U, 0U, 0U, 0U};
    uint8_t bitIndex;
    uint8_t checksum;
    uint8_t byteIndex;
    uint16_t highUs;

    if ((dht == 0) || (dht->port == 0))
    {
        return HAL_ERROR;
    }

    dht_send_start_signal(dht);

    if (dht_wait_for_state(dht, GPIO_PIN_RESET, 100U) != HAL_OK)
    {
        return HAL_TIMEOUT;
    }

    if (dht_wait_for_state(dht, GPIO_PIN_SET, 100U) != HAL_OK)
    {
        return HAL_TIMEOUT;
    }

    if (dht_wait_for_state(dht, GPIO_PIN_RESET, 100U) != HAL_OK)
    {
        return HAL_TIMEOUT;
    }

    for (bitIndex = 0U; bitIndex < 40U; bitIndex++)
    {
        byteIndex = (uint8_t)(bitIndex / 8U);

        if (dht_measure_high_pulse_us(dht, &highUs, 100U) != HAL_OK)
        {
            return HAL_TIMEOUT;
        }

        data[byteIndex] <<= 1U;
        if (highUs > 45U)
        {
            data[byteIndex] |= 1U;
        }
    }

    checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (checksum != data[4])
    {
        return HAL_ERROR;
    }

    dht->humidityInt = data[0];
    dht->humidityDec = data[1];
    dht->temperatureInt = data[2];
    dht->temperatureDec = data[3];
    return HAL_OK;
}

float BSP_DHT11_GetTemperatureC(const BSP_DHT11_HandleTypeDef* dht)
{
    if (dht == 0)
    {
        return 0.0f;
    }

    return (float)dht->temperatureInt + ((float)dht->temperatureDec / 10.0f);
}

float BSP_DHT11_GetHumidityRH(const BSP_DHT11_HandleTypeDef* dht)
{
    if (dht == 0)
    {
        return 0.0f;
    }

    return (float)dht->humidityInt + ((float)dht->humidityDec / 10.0f);
}
