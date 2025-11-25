/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "hcsr04.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
    float ax, ay, az;      // g cinsinden
    float gx, gy, gz;      // dps
    float pitch, roll;     // derece
    float vx;              // ileri yöndeki hız (m/s)
} MotionState_t;

typedef struct
{
    float distance_m;      // metre
    float ttc_s;           // Time-To-Collision (s)
} ObstacleState_t;

typedef enum
{
    SAFETY_STATE_SAFE = 0,
    SAFETY_STATE_WARNING,
    SAFETY_STATE_CRITICAL
} SafetyState_t;




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

MotionState_t   g_motion;
ObstacleState_t g_obstacle;
SafetyState_t   g_safety_state = SAFETY_STATE_SAFE;

// Zaman damgaları (HAL_GetTick kullanacağız)
uint32_t last_imu_ms       = 0;   // 50 Hz (20 ms)
uint32_t last_sonar_ms     = 0;   // 20 Hz (50 ms)
uint32_t last_ttc_ms       = 0;   // 20 Hz (50 ms)
uint32_t last_state_ms     = 0;   // 50 Hz (20 ms)
uint32_t last_telemetry_ms = 0;   // 10 Hz (100 ms)



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define HARD_BRAKE_ACC_G      (-0.5f)   // ileri eksende ani fren
#define IMPACT_ACC_G          (2.5f)    // çarpma ivmesi (toplam)

// Mesafe eşikleri (m cinsinden)
#define WARN_DISTANCE_M       (1.2f)
#define CRIT_DISTANCE_M       (0.5f)

// TTC eşikleri (saniye)
#define WARN_TTC_S            (1.0f)
#define CRIT_TTC_S            (0.5f)

// Basit low-pass için alpha
#define LPF_ALPHA             (0.2f)


void Task_IMU(float dt);
void Task_Sonar(void);
void Task_Compute_TTC(void);
void Task_Update_Safety_State(void);
void Task_Send_Telemetry(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  if (MPU6050_Init(&hi2c1) != HAL_OK)
    {
        const char err[] = "MPU6050 init FAILED\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)err, sizeof(err)-1, 100);
    }
    else
    {
        const char ok[] = "MPU6050 init OK\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t*)ok, sizeof(ok)-1, 100);
    }

    // HC-SR04 init (TIM3 Input Capture başlatır)
    HCSR04_Init();

    // Başlangıç değerlerini sıfırla
    memset(&g_motion,    0, sizeof(g_motion));
    memset(&g_obstacle,  0, sizeof(g_obstacle));

    g_obstacle.ttc_s      = 999.0f;
    g_safety_state        = SAFETY_STATE_SAFE;

    last_imu_ms           = HAL_GetTick();
    last_sonar_ms         = HAL_GetTick();
    last_ttc_ms           = HAL_GetTick();
    last_state_ms         = HAL_GetTick();
    last_telemetry_ms     = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    uint32_t now = HAL_GetTick();

	      // 50 Hz: IMU + açı + hız + crash detection
	      if ((now - last_imu_ms) >= 20)
	      {
	          float dt = (now - last_imu_ms) / 1000.0f;
	          last_imu_ms = now;
	          Task_IMU(dt);
	      }

	      // 20 Hz: Sonar ölçümü
	      if ((now - last_sonar_ms) >= 50)
	      {
	          last_sonar_ms = now;
	          Task_Sonar();
	      }

	      // 20 Hz: TTC hesabı
	      if ((now - last_ttc_ms) >= 50)
	      {
	          last_ttc_ms = now;
	          Task_Compute_TTC();
	      }

	      // 50 Hz: Safety state machine
	      if ((now - last_state_ms) >= 20)
	      {
	          last_state_ms = now;
	          Task_Update_Safety_State();
	      }

	      // 10 Hz: Telemetry
	      if ((now - last_telemetry_ms) >= 100)
	      {
	          last_telemetry_ms = now;
	          Task_Send_Telemetry();
	      }





  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HCSR04_TRIG_Pin */
  GPIO_InitStruct.Pin = HCSR04_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(HCSR04_TRIG_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Task_IMU(float dt)
{
    MPU6050_Proc_t imu;

    if (MPU6050_ReadProcessed(&hi2c1, dt, &imu) != HAL_OK)
        return;

    // g_motion'a kopyala
    g_motion.ax    = imu.ax_g;
    g_motion.ay    = imu.ay_g;
    g_motion.az    = imu.az_g;
    g_motion.gx    = imu.gx_dps;
    g_motion.gy    = imu.gy_dps;
    g_motion.gz    = imu.gz_dps;
    g_motion.pitch = imu.pitch_deg;
    g_motion.roll  = imu.roll_deg;

    // İleri eksen=ax kabul ederek hız entegrasyonu (çok kaba, demo amaçlı)
    float acc_forward_mps2 = g_motion.ax * 9.81f;
    g_motion.vx += acc_forward_mps2 * dt;

    // Hızı biraz sınırlayalım (kaçmaması için)
    if (g_motion.vx > 50.0f)  g_motion.vx = 50.0f;
    if (g_motion.vx < -50.0f) g_motion.vx = -50.0f;
}


void Task_Sonar(void)
{
    HCSR04_Measurement_t m = HCSR04_Measure();

    if (m.status == HAL_OK)
    {
        // Basit low-pass
        g_obstacle.distance_m =
            (1.0f - LPF_ALPHA) * g_obstacle.distance_m +
            LPF_ALPHA * m.distance_m;
    }
    else
    {
        // Ölçüm hatası → istersen distance'i değiştirme
        // g_obstacle.distance_m aynı kalsın
    }
}


void Task_Compute_TTC(void)
{
    float d = g_obstacle.distance_m;
    float v = g_motion.vx;

    if (d > 0.05f && v > 0.1f)
    {
        g_obstacle.ttc_s = d / v;
    }
    else
    {
        g_obstacle.ttc_s = 999.0f; // çok uzak veya hareket yok
    }
}


void Task_Update_Safety_State(void)
{
    SafetyState_t new_state = g_safety_state;

    float ax   = g_motion.ax;
    float d    = g_obstacle.distance_m;
    float ttc  = g_obstacle.ttc_s;

    // Toplam ivmeyi hesaplayalım (çarpma tespiti için)
    float acc_total_g = sqrtf(g_motion.ax*g_motion.ax +
                              g_motion.ay*g_motion.ay +
                              g_motion.az*g_motion.az);
    uint8_t impact = (acc_total_g > IMPACT_ACC_G);

    switch (g_safety_state)
    {
        case SAFETY_STATE_SAFE:
            if (impact || (d < CRIT_DISTANCE_M) || (ttc < CRIT_TTC_S))
            {
                new_state = SAFETY_STATE_CRITICAL;
            }
            else if ((d < WARN_DISTANCE_M) ||
                     (ttc < WARN_TTC_S) ||
                     (ax < HARD_BRAKE_ACC_G))
            {
                new_state = SAFETY_STATE_WARNING;
            }
            break;

        case SAFETY_STATE_WARNING:
            if (impact || (d < CRIT_DISTANCE_M) || (ttc < CRIT_TTC_S))
            {
                new_state = SAFETY_STATE_CRITICAL;
            }
            else if ((d > WARN_DISTANCE_M * 1.2f) &&
                     (ttc > WARN_TTC_S * 1.5f) &&
                     (ax > HARD_BRAKE_ACC_G * 0.5f))
            {
                new_state = SAFETY_STATE_SAFE;
            }
            break;

        case SAFETY_STATE_CRITICAL:
            // Çarpma bitmiş, mesafe açılmış, TTC rahatlamış ise WARNING'e döner
            if (!impact &&
                (d > WARN_DISTANCE_M * 1.5f) &&
                (ttc > WARN_TTC_S * 2.0f))
            {
                new_state = SAFETY_STATE_WARNING;
            }
            break;
    }

    g_safety_state = new_state;

    // LED ile görselleştirme (Nucleo'da PA5 LED)
    switch (g_safety_state)
    {
        case SAFETY_STATE_SAFE:
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // LED kapalı
            break;
        case SAFETY_STATE_WARNING:
            // Blink
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            break;
        case SAFETY_STATE_CRITICAL:
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);   // LED sürekli yanık
            break;
    }
}


void Task_Send_Telemetry(void)
{
    char buf[180];
    int len = snprintf(buf, sizeof(buf),
                       "AX:%.2f AY:%.2f AZ:%.2f | "
                       "Vx:%.2f m/s | "
                       "Dist:%.2f m TTC:%.2f s | "
                       "Pitch:%.1f Roll:%.1f | "
                       "State:%d\r\n",
                       g_motion.ax, g_motion.ay, g_motion.az,
                       g_motion.vx,
                       g_obstacle.distance_m, g_obstacle.ttc_s,
                       g_motion.pitch, g_motion.roll,
                       g_safety_state);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
    }
}

/* USER CODE END 4 */




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
