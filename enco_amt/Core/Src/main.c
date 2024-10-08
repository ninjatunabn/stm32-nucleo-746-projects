/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
int32_t encoderValue;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);

/* Main function */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();                 // Initialize the Hardware Abstraction Layer
    SystemClock_Config();        // Configure system clock
    MX_GPIO_Init();              // Initialize GPIO
    MX_TIM1_Init();              // Initialize TIM1 for encoder mode

    /* Start encoder interface for TIM1 */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);  // Start encoder interface on TIM1
//    char buffer[50];
//    uint32_t lastTime = 0;  // To track the last time the encoder was read
//    uint32_t interval = 10000;  // 10 seconds in milliseconds

    /* Infinite loop */
    while (1)
    {
//        uint32_t currentTime = HAL_GetTick();  // Get the current time in milliseconds
//        if (currentTime - lastTime >= interval) {
            // Read the current encoder value from TIM1
            encoderValue = __HAL_TIM_GET_COUNTER(&htim1);
//            sprintf(buffer, "Encoder Value: %ld\r\n", encoderValue);
            // Print the encoder value (you may use UART or another method to view the output)
//            printf("Encoder Value: %ld\r\n", encoderValue);
//            HAL_Delay(1000);

//            lastTime = currentTime;  // Update lastTime to the current time
        }
    }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 72;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 3;

    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

/* TIM1 Initialization Function */
static void MX_TIM1_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;  // No prescaler
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;  // Counter counts up
    htim1.Init.Period = 65535;  // Maximum value for encoder counter
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /* Configure TIM1 for encoder mode using both TI1 and TI2 */
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;  // Configure encoder mode using both TI1 and TI2

    /* TI1 configuration */
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;  // Capture on rising edge of TI1
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;  // TI1 is connected directly to input capture
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;  // No prescaler for TI1
    sConfig.IC1Filter = 0;  // No filter

    /* TI2 configuration */
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;  // Capture on rising edge of TI2
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;  // TI2 is connected directly to input capture
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;  // No prescaler for TI2
    sConfig.IC2Filter = 0;  // No filter

    /* Initialize TIM1 with the encoder configuration */
    if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    /* Master configuration (not used for this example) */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);
}

/* GPIO Initialization Function */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
}

/* Error Handler */
void Error_Handler(void)
{
    while (1) {}
}
