#ifndef BSP_RELAY_H
#define BSP_RELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

typedef enum
{
    BSP_RELAY_1 = 0,
    BSP_RELAY_2,
    BSP_RELAY_3
} BSP_RelayChannel;

void BSP_Relay_Init(void);
HAL_StatusTypeDef BSP_Relay_Set(BSP_RelayChannel channel, uint8_t enable);
void BSP_Relay_SetAll(uint8_t enable);

#ifdef __cplusplus
}
#endif

#endif /* BSP_RELAY_H */
