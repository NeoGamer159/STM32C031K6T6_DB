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
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADS1015_ADS1115.h"
#include "i2c-lcd.h"
#include "SHT31.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float temperature = 0;
static uint8_t fan_speed = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_OFF  0
#define PWM_25   25
#define PWM_50   50
#define PWM_75   75

#define T_ON_LOW        30.0f   // OFF -> 25 %
#define T_ON_MED        32.0f   // 25 % -> 50 %
#define T_ON_HIGH       35.0f   // 50 % -> 75 %

#define T_OFF_LOW       29.0f   // 25 % -> OFF
#define T_DOWN_TO_LOW   31.0f   // 50 % -> 25 %
#define T_DOWN_TO_MED   34.0f   // 75 % -> 50 %
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1;

/* USER CODE BEGIN PV */
ADS1xx5_I2C i2c;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void print_fixed(int32_t val, uint8_t frac_digits, char *out, size_t out_len)
{
    int32_t sign = (val < 0) ? -1 : 1; //Decide if num is negative or positive
    uint32_t u = (uint32_t)(sign < 0 ? -val : val); //Create negative or positive version of number
    uint32_t p10 = 1;
    for (uint8_t i = 0; i < frac_digits; ++i) p10 *= 10;

    uint32_t whole = u / p10;
    uint32_t frac  = u % p10;

    if (frac_digits == 0) {
        (void)snprintf(out, out_len, "%s%lu", (sign < 0) ? "-" : "", (unsigned long)whole);
        return;
    }
    char fmt[16];
    (void)snprintf(fmt, sizeof(fmt), "%%s%%lu.%%0%ulu", (unsigned)frac_digits);
    (void)snprintf(out, out_len, fmt, (sign < 0) ? "-" : "", (unsigned long)whole, (unsigned long)frac);
}

static int32_t to_fixed_round(float x, uint8_t frac_digits)
{
    // scale = 10^N
    int32_t scale = 1;
    for (uint8_t i = 0; i < frac_digits; ++i) scale *= 10;

    // Ruční zaokrouhlení bez <math.h>
    float scaled = x * (float)scale;
    if (scaled >= 0.0f) scaled += 0.5f; else scaled -= 0.5f;
    int32_t v = (int32_t)scaled;
    return v;
}

void Fan_Update(void)
{
    float temperature = SHT31_GetTemperature();

    switch (fan_speed)
    {
        case 0: // OFF
            if (temperature >= T_ON_LOW) {
                TIM3->CCR1 = PWM_25;
                lcd_put_cur(0, 15);
                lcd_send_string("L");
                fan_speed = 1;
            }
            break;

        case 1: // LOW (25 %)
            if (temperature >= T_ON_MED) {
                TIM3->CCR1 = PWM_50;
                fan_speed = 2;
            } else if (temperature <= T_OFF_LOW) {
                TIM3->CCR1 = PWM_OFF;
                fan_speed = 0;
            }
            break;

        case 2: // MED (50 %)
            if (temperature >= T_ON_HIGH) {
                TIM3->CCR1 = PWM_75;
                fan_speed = 3;
            } else if (temperature <= T_DOWN_TO_LOW) {
                TIM3->CCR1 = PWM_25;
                fan_speed = 1;
            }
            break;

        case 3: // HIGH (75 %)
            if (temperature <= T_DOWN_TO_MED) {
                TIM3->CCR1 = PWM_50;
                fan_speed = 2;
            }
            break;
    }
}
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* USER CODE BEGIN 2 */
  ADS1115(&i2c, &hi2c1, ADS_ADDR_GND);
  ADSsetGain(&i2c, GAIN_EIGHT);

  TIM3->CCR1 = 0;

  SHT31_Config(SHT31_ADDRESS_A, &hi2c1);

  lcd_init ();
  lcd_put_cur(0, 0);
  lcd_send_string ("PAN");
  lcd_put_cur(1, 0);
  lcd_send_string("TEMNOT");
  HAL_Delay(500);
  lcd_clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	  if(SHT31_GetData(SHT31_Periodic, SHT31_Medium, SHT31_NON_Stretch, SHT31_1) == SHT31_OK)
	  {
		  temperature = SHT31_GetTemperature();
		  char tempStr[16];
		  snprintf(tempStr, sizeof(tempStr), "Temp: %.2f C", temperature);
		  lcd_put_cur(1, 0);
		  lcd_send_string(tempStr);
	  }
	  int16_t adc1 = ADSreadADC_SingleEnded(&i2c, 1);
	  float current = ((float)adc1 * 0.512f) / (32768.0f * 0.1f);
	  char buffer[7];
	  sprintf(buffer, "%.3f", current);
	  lcd_put_cur(0, 0);
	  lcd_send_string(buffer);
	  HAL_Delay(100);
	  //adc2 = ADSreadADC_SingleEnded(&i2c, 2);
	  //adc3 = ADSreadADC_SingleEnded(&i2c, 3);
	  */
	  if(SHT31_GetData(SHT31_Periodic, SHT31_Medium, SHT31_NON_Stretch, SHT31_1) == SHT31_OK)
	  {
		  temperature = SHT31_GetTemperature();
		  char tempStr[16];
		  int32_t t_centi = to_fixed_round(temperature, 2);
		  print_fixed(t_centi, 2, tempStr, sizeof(tempStr));
		  lcd_put_cur(1, 0);
		  lcd_send_string(tempStr);
	  }

	  Fan_Update();

	  int16_t adc1 = ADSreadADC_SingleEnded(&i2c, 1);
	  float current = ((float)adc1 * 0.512f) / (32768.0f * 0.1f);

	  char iStr[16];
	  int32_t i_milli = to_fixed_round(current, 3);   // např. 0.123 A → 123

	  print_fixed(i_milli, 3, iStr, sizeof(iStr));
	  lcd_put_cur(0, 0);
	  lcd_send_string(iStr);
	  HAL_Delay(100);
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x0090194B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 40-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_1_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
