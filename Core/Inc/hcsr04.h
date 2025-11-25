/*
 * hcsr04.h
 *
 *  Created on: Nov 25, 2025
 *      Author: MONSTER
 */

#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_

#include "stm32f4xx_hal.h"

// Ölçüm sonucu için küçük bir struct
typedef struct
{
    float distance_m;          // metre cinsinden mesafe
    HAL_StatusTypeDef status;  // HAL_OK, HAL_TIMEOUT, HAL_ERROR
} HCSR04_Measurement_t;

// Başlatma (timer + trigger pini)
void HCSR04_Init(void);

// Ölçüm yap (tetikler, echo bitene kadar bloklar)
HCSR04_Measurement_t HCSR04_Measure(void);

#endif /* INC_HCSR04_H_ */
