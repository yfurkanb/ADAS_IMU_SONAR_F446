/*
 * hcsr04.c
 *
 *  Created on: Nov 25, 2025
 *      Author: MONSTER
 */



#include "hcsr04.h"
#include "main.h"   // HCSR04_TRIG_Pin / Port ve htim3 buradan gelecek

extern TIM_HandleTypeDef htim3;


static volatile uint32_t ic_val1 = 0;
static volatile uint32_t ic_val2 = 0;
static volatile uint8_t  is_first_capture = 0;
static volatile uint8_t  measurement_done = 0;
static volatile uint32_t last_pulse_ticks = 0;  // 1 MHz olduğundan 1 tick = 1 µs

// Timer ARR değerini convenience için makro olarak alalım
#define HCSR04_TIM_ARR   (htim3.Init.Period)

// 10 µs civarı beklemek için küçük bir NOP loop'u
static void HCSR04_DelayUs_Approx(uint16_t us)
{
    // Bu çok kaba bir delay; 84 MHz civarında çalışıyor varsayıyoruz.
    // Hassas olmasına gerek yok, sadece 10 µs'den büyük olsun yeter.
    for (uint32_t i = 0; i < us * 10; i++)
    {
        __NOP();
    }
}

void HCSR04_Init(void)
{
    // Trigger pinini LOW'a çek
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);

    // TIM3 Input Capture'ı interrupt ile başlat
    // (CH1 PA6 → Echo pini)
    if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
    {
        // Hata varsa burada breakpoint koyabilirsin
    }

    // İlk ölçüm için rising edge'ten başlamasını garanti et
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
}

HCSR04_Measurement_t HCSR04_Measure(void)
{
    HCSR04_Measurement_t result;
    result.distance_m = 0.0f;
    result.status = HAL_ERROR;

    // Durum bayraklarını sıfırla
    measurement_done = 0;
    is_first_capture = 0;

    // Timer sayacını sıfırla
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

    // 1) Trigger pulse (10 µs HIGH)
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
    HCSR04_DelayUs_Approx(15);  // ~15 µs kadar bekle
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);

    // 2) Echo pulse bitene kadar bekle (max ~40 ms)
    uint32_t start_ms = HAL_GetTick();
    while (!measurement_done)
    {
        if ((HAL_GetTick() - start_ms) > 40)
        {
            // Timeout: echo gelmedi
            result.status = HAL_TIMEOUT;
            return result;
        }
    }

    // 3) Ölçülen pulse genişliğinden mesafeyi hesapla
    uint32_t pulse_ticks = last_pulse_ticks;   // 1 tick = 1 µs (TIM3 1 MHz)
    float pulse_us = (float)pulse_ticks;

    // Ses hızı ~343 m/s → 0.0343 cm/µs → 0.000343 m/µs
    // Ses gidiş-geliş yaptığı için /2
    // distance_m = pulse_us * 0.000343 / 2
    float distance_m = pulse_us * 0.0001715f;  // 0.000343 / 2

    result.distance_m = distance_m;
    result.status = HAL_OK;
    return result;
}

// HAL'in zayıf (weak) tanımını override ediyoruz
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (is_first_capture == 0)
        {
            // Rising edge: başlangıç zamanı
            ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            is_first_capture = 1;

            // Sonraki edge'i falling yap
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
                                          TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else
        {
            // Falling edge: bitiş zamanı
            ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

            uint32_t diff_ticks;
            if (ic_val2 >= ic_val1)
            {
                diff_ticks = ic_val2 - ic_val1;
            }
            else
            {
                // Overflow durumu teorik olarak mümkün ama
                // 1 MHz ve 65ms period ile HC-SR04 için zaten pek olmaz.
                diff_ticks = (HCSR04_TIM_ARR - ic_val1) + ic_val2 + 1;
            }

            last_pulse_ticks = diff_ticks;
            measurement_done = 1;

            // Bir sonraki ölçüm için tekrar rising'e dön
            is_first_capture = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
                                          TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}
