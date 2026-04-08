#include "bsp_dht11.h"

static void dht_pin_as_output(GPIO_TypeDef* port, uint16_t pin)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &gpio);
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
    (void)dht;
    /* TODO: implement DHT11 timing protocol with TIM2-based microsecond delay. */
    return HAL_ERROR;
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
