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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include "cirbuf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Declare Sample Frequency
float T_s;
float f_s;
float pi = 3.1415;

// Declare variable used for DMA transfers
uint32_t adc_val;

// Define circular buffers used for DSP.
#define CIRBUF_LEN 5
uint8_t cirbuf_idx = 0; // Index used for tracking current location in circular buffer
float cirbuf_mic[CIRBUF_LEN];
float cirbuf_y_hpf[CIRBUF_LEN];
float cirbuf_y_rect[CIRBUF_LEN];
float cirbuf_y_env[CIRBUF_LEN];
float cirbuf_y_hpf_beat[CIRBUF_LEN];

float cirbuf_intensity[CIRBUF_LEN];

uint32_t cirbuf_pwm;

uint32_t proc_time;

// Flag used to capture a number of samples for offline debug
#define DEBUG_CAPTURE 0

// Arrays to store recorded outputs for debugging
#if DEBUG_CAPTURE
#define DEBUG_NUM_SAMPLES 256
float x_capture[DEBUG_NUM_SAMPLES];
//float hpf_capture[DEBUG_NUM_SAMPLES];
//float y_capture[DEBUG_NUM_SAMPLES];
uint32_t pwm_capture[DEBUG_NUM_SAMPLES];
uint32_t proc_time_capture[DEBUG_NUM_SAMPLES];
uint32_t cnt_capture = 0;
#endif


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	// Get indexes of nearby values for difference equations
	uint8_t cirbuf_idx_minus_1 = ArrayIdxToCirBufIdx(cirbuf_idx, -1, CIRBUF_LEN);
//	uint8_t cirbuf_idx_minus_2 = ArrayIdxToCirBufIdx(cirbuf_idx, -2, CIRBUF_LEN);

	// Save ADC read to circular buffer
	cirbuf_mic[cirbuf_idx] = adc_val;

	// Apply HPF
	float w_n_hpf = 2 * pi * 30;
	float one_minus_alpha_hpf = f_s / (f_s + w_n_hpf);
	cirbuf_y_hpf[cirbuf_idx] = ApplyFirstDifferenceHPF(
			one_minus_alpha_hpf,
			cirbuf_mic[cirbuf_idx],
			cirbuf_mic[cirbuf_idx_minus_1],
			cirbuf_y_hpf[cirbuf_idx_minus_1]);

	// Rectify
	cirbuf_y_rect[cirbuf_idx] = abs(cirbuf_y_rect[cirbuf_idx]);

	// Apply Envelope with LPF
	float w_n_env = 2 * pi * 500;
	float alpha_env = w_n_env / (f_s + w_n_env);
	float one_minus_alpha_env = 1 - alpha_env;
	cirbuf_y_env[cirbuf_idx] = ApplyFirstDifferenceEnvelopeLPF(
			alpha_env,
			one_minus_alpha_env,
			cirbuf_y_rect[cirbuf_idx],
			cirbuf_y_env[cirbuf_idx_minus_1]);

	// Apply additional HPF for beat detect
	float w_n_hpf_beat = 2 * pi * 30;
	float one_minus_alpha_hpf_beat = f_s / (f_s + w_n_hpf);
	cirbuf_y_hpf_beat[cirbuf_idx] = ApplyFirstDifferenceHPF(
			one_minus_alpha_hpf_beat,
			cirbuf_y_env[cirbuf_idx],
			cirbuf_y_env[cirbuf_idx_minus_1],
			cirbuf_y_hpf_beat[cirbuf_idx_minus_1]);

	// Store final value
//	cirbuf_y[cirbuf_idx] = cirbuf_x[cirbuf_idx];
//	cirbuf_y[cirbuf_idx] = cirbuf_hpf[cirbuf_idx];
//	cirbuf_y[cirbuf_idx] = cirbuf_env[cirbuf_idx];

	// Set PWM duty cycle based on output signal
//	uint32_t pwm_ccr = ApplyGain(cirbuf_y[cirbuf_idx], pwm_gain, htim2.Init.Period);
//	uint32_t pwm_ccr = (uint32_t)cirbuf_y[cirbuf_idx] * pwm_gain;
//	htim2.Instance->CCR1 = pwm_ccr;
//	cirbuf_pwm[cirbuf_idx] = pwm_ccr;

//	// Toggle the Green LED
//	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	// Increment circular buffer index to store next read from ADC
//	cirbuf_idx = (cirbuf_idx + 1) % CIRBUF_LEN;

#if DEBUG_CAPTURE
	// Save current sample data to capture arrays
	x_capture[cnt_capture] = cirbuf_x[cirbuf_idx];
//	hpf_capture[cnt_capture] = cirbuf_hpf[cirbuf_idx];
//	y_capture[cnt_capture] = cirbuf_y[cirbuf_idx];
	pwm_capture[cnt_capture] = cirbuf_pwm[cirbuf_idx];

	// Increment capture sample counter
	cnt_capture += 1;
#endif

	// Save counter indicating how long it took to get here
	proc_time = htim2.Instance->CNT;

#if DEBUG_CAPTURE
	// Capture processing time as well
	proc_time_capture[cnt_capture] = proc_time;

	// If we have captured all the samples needed for this run, stop cycle and print info over serial
	if (cnt_capture >= DEBUG_NUM_SAMPLES)
	{
		HAL_ADC_Stop_DMA(&hadc1);
		for (uint32_t idx = 0; idx < DEBUG_NUM_SAMPLES; idx += 1){
			uint8_t row_text[30] = "";
			sprintf((char*)row_text, "%.3f,%lu,%lu\r\n", \
					x_capture[idx], pwm_capture[idx], (uint32_t)proc_time_capture[idx]);
			HAL_UART_Transmit(&huart2, row_text, sizeof(row_text), HAL_MAX_DELAY);
		}
		uint8_t end_msg[5] = "END\r\n";
		HAL_UART_Transmit(&huart2, end_msg, sizeof(end_msg), HAL_MAX_DELAY);
	}
#endif
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Define Sample Frequency
  T_s = (float)htim2.Init.Period / 64E6;
  f_s = 1 / T_s;

  // Define variables used for HPF
//  f_cutoff_hpf = 5000.0f;
//  w_cutoff_hpf = (f_cutoff_hpf * 2 * 3.1415);

  // Define constants used for envelope function
//  f_cutoff_env = 0.5f;
//  w_cutoff_env = f_cutoff_env * 2.0 * 3.1415;
//  env_coef1 = w_cutoff_env / (f_s + w_cutoff_env);
//  env_coef2 = f_s / (f_s + w_cutoff_env);

  // Define constants used for PWM generation
//  pwm_gain = 2;

  // calibrate ADC for better accuracy and start it w/ interrupt
  if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
                Error_Handler();



  // Start ADC
//  if(HAL_ADC_Start_IT(&hadc1) != HAL_OK)
//                Error_Handler();

  if(HAL_ADC_Start_DMA(&hadc1, &adc_val, 1) != HAL_OK)
                Error_Handler();



  // Start TIM
//  if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
//  if(HAL_TIM_Base_Start(&htim2) != HAL_OK)
  if(HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
                Error_Handler();




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  return 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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
