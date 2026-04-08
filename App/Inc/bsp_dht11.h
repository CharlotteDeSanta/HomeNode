#ifndef BSP_DHT11_H
#define BSP_DHT11_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t humidityInt;
    uint8_t humidityDec;
    uint8_t temperatureInt;
    uint8_t temperatureDec;
} BSP_DHT11_HandleTypeDef;

HAL_StatusTypeDef BSP_DHT11_Init(BSP_DHT11_HandleTypeDef* dht,
                                 GPIO_TypeDef* port,
                                 uint16_t pin);
HAL_StatusTypeDef BSP_DHT11_Read(BSP_DHT11_HandleTypeDef* dht);
float BSP_DHT11_GetTemperatureC(const BSP_DHT11_HandleTypeDef* dht);
float BSP_DHT11_GetHumidityRH(const BSP_DHT11_HandleTypeDef* dht);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DHT11_H */
