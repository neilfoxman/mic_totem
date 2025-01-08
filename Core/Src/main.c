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
#include <math.h>
#include "cirbuf.h"
#include "sm.h"

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

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const uint32_t adc1_isr_clear_mask =
	ADC_ISR_JQOVF |
	ADC_ISR_AWD3 |
	ADC_ISR_AWD2 |
	ADC_ISR_AWD1 |
	ADC_ISR_JEOS |
	ADC_ISR_JEOC |
	ADC_ISR_OVR |
	ADC_ISR_EOS |
	ADC_ISR_EOC;

// External variable used for tracking how long certain processes take to complete
uint32_t proc_time;

// OVR Event tracking variable
uint32_t ovr_cnt = 0;

// Define States

// Prototypes
void config_mic_s(Event evt);
void mic_s(Event evt);
void config_led_s(Event evt);
void led_s(Event evt);

// Declare Sample Frequency, To be calculated later by getting timer register value
float T_s;
float f_s;

// Declare ADC read variables
uint16_t adc_reads[5];
#define ADC_RANK_MIC 0
#define ADC_RANK_BRIGHTNESS 1
#define ADC_RANK_TRANSIENT_SENSITIVITY 2
#define ADC_RANK_TRANSIENT_INTENSITY_RATIO 3
#define ADC_RANK_INTENSITY_TAU 4

uint32_t mic_arr = 4000;
float intensity_tau_adc;
float tau_intensity_min = 0.01;
float tau_intensity_max = 2.0;
float tau_intensity;
float tau_over_T_intensity;
float intensity;
float transient_sensitivity_adc;
float transient_sensitivity;
float transient_thresh;

void config_mic_s(Event evt){
	switch(evt){
			case ENTER:
				// Disable led_s configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~
//				proc_time = TIM2->CNT;
				// Disable counter temporarily to configure peripherals
				LL_TIM_DisableCounter(TIM2);

				// Disable DMA requests triggering on update events
				LL_TIM_DisableDMAReq_UPDATE(TIM2);

				// Disable DMA
				LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
				LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
				SET_BIT(DMA1->IFCR, DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2);

				// Force reset counter using update event
				LL_TIM_GenerateEvent_UPDATE(TIM2);

				// Set CCR to 0 so outputting a low (reset) value
				LL_TIM_OC_SetCompareCH1(TIM2, 0);

				// Configure for mic_s ~~~~~~~~~~~~~~~~~~~~~~~~~~
				// Calculate time constant for intensity LPF
				intensity_tau_adc = (float)(4095 - adc_reads[ADC_RANK_INTENSITY_TAU]);
				tau_intensity = tau_intensity_min + ( (tau_intensity_max - tau_intensity_min) / (4095.0) ) * intensity_tau_adc;
				tau_over_T_intensity = tau_intensity / T_s;


				// Calculate Transient Threshold based on ADC read and intensity (threshold is scaled intensity value)
				transient_sensitivity_adc = (float)(4095 - adc_reads[ADC_RANK_TRANSIENT_SENSITIVITY]); // Make Numerator smaller to make min sensitivity less sensitive.
				transient_sensitivity = transient_sensitivity_adc * (40.0 / 4095.0);
				transient_thresh = intensity * transient_sensitivity;

				// Set timer ARR such that DSP can complete in one cycle
				LL_TIM_SetAutoReload(TIM2, mic_arr);

				// Clear any pending status registers
				CLEAR_REG(TIM2->SR);
				SET_BIT(DMA1->IFCR, DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2);

				// Reset DMA counter
				LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 5);
				while(LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1) != 5){}

				// Enable DMA
				LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

				// Enable ADC
				LL_ADC_Enable(ADC1);

				// Start ADC triggering on TRGO events
				LL_ADC_REG_StartConversion(ADC1);

				// Enable counter. Transition to next state occurs on the next update event
				LL_TIM_EnableCounter(TIM2);
				break;
			case TIM_UE:
				// Transition to next state
				transition(mic_s);
			default:
				break;
		}
}


// Define circular buffers used for DSP.
#define CIRBUF_LEN 255
uint8_t cirbuf_idx = 0; // Index used for tracking current location in circular buffers
float cirbuf_mic[CIRBUF_LEN];

// Incoming signal HPF
float cirbuf_y_hpf[CIRBUF_LEN];
float tau_over_T_y_hpf;

// Intensity signal (slow LPF envelope filter)
// Intensity needs to be done in float math due to high tau/T relative to x[n]
float cirbuf_intensity[CIRBUF_LEN];

// HPF for beat detect
float cirbuf_y_hpf_beat[CIRBUF_LEN];
float tau_over_T_y_hpf_beat;

// Rectified beat detect signal
float y_beat_abs;

// Transient detector counter
int32_t transient_present = 0;

// Number of mic samples before switch states to set LEDs
uint32_t num_samples_before_LED = 1000;
uint32_t num_samples_taken = 0;

void mic_s(Event evt){
	switch(evt){
		case ENTER:
			// Reset sample tracker
			num_samples_taken = 0;
			break;
		case ADC_OVR:
			ovr_cnt++; // Increment counter (for debugging)
			break;
		case MIC_DMA_COMPLETE:
			// Increment sample counter
			num_samples_taken++;

			// Process data
			// Get indexes of nearby values for difference equations
			uint8_t cirbuf_idx_minus_1 = ArrayIdxToCirBufIdx(cirbuf_idx, -1, CIRBUF_LEN);
//			uint8_t cirbuf_idx_minus_2 = ArrayIdxToCirBufIdx(cirbuf_idx, -2, CIRBUF_LEN);

			// Save new mic read every sample
			cirbuf_mic[cirbuf_idx] = (float)adc_reads[ADC_RANK_MIC];

			// Apply HPF
			cirbuf_y_hpf[cirbuf_idx] = ApplyFirstDifferenceHPF_float(
				cirbuf_y_hpf[cirbuf_idx_minus_1],
				cirbuf_mic[cirbuf_idx],
				cirbuf_mic[cirbuf_idx_minus_1],
				tau_over_T_y_hpf
			);

			// Determine Intensity
//			cirbuf_intensity[cirbuf_idx] = ApplyFirstDifferenceLPF_float(
			cirbuf_intensity[cirbuf_idx] = ApplyFirstDifferenceEnvelopeLPF_float(
				cirbuf_intensity[cirbuf_idx_minus_1],
				cirbuf_y_hpf[cirbuf_idx],
				tau_over_T_intensity
			);
			intensity = cirbuf_intensity[cirbuf_idx];

			// Beat Detect HPF
			cirbuf_y_hpf_beat[cirbuf_idx] = ApplyFirstDifferenceHPF_float(
				cirbuf_y_hpf_beat[cirbuf_idx_minus_1],
				cirbuf_mic[cirbuf_idx],
				cirbuf_mic[cirbuf_idx_minus_1],
				tau_over_T_y_hpf_beat
			);

			y_beat_abs = fabs(cirbuf_y_hpf_beat[cirbuf_idx]);

			// Beat detect threshold
			if (y_beat_abs > transient_thresh)
			{
				transient_present++;
			}

			// FINAL: Increment circular buffer index to store next read from ADC
			cirbuf_idx = (cirbuf_idx + 1) % CIRBUF_LEN;

			// Capture time at end of processing for setting good sample frequency
			proc_time = TIM2->CNT;

			if(num_samples_taken >= num_samples_before_LED){
				transition(config_led_s);
			}
			break;
		default:
			break;
	}
}

// LED output variables
#define NUM_LEDS 30
#define NUM_LED_CHANNELS 3
uint8_t led_vals[NUM_LEDS][NUM_LED_CHANNELS];

// Variables used when converting from bits to Timer CCR durations
#define NUM_CCRS (NUM_LEDS * NUM_LED_CHANNELS * 8)
uint32_t ccr_zero = 19;
uint32_t ccr_one = 38;
uint32_t ccr_sequence[NUM_CCRS];

uint32_t led_arr = 79;
//uint32_t led_arr = 1000;

uint32_t refresh_cntr = 0;

// Declare LED variables external from state machine for easier tracking/debugging
uint8_t transient_locations[NUM_LEDS];
float led_transient_component[NUM_LEDS];

// Each color channel intensity component oscillates at different frequencies and
// is dependent on measured intensity. Periods are measured in number of LED refresh cycles.
const float led_chan_periods[NUM_LED_CHANNELS] = {500, 600, 700};
uint32_t led_chan_periods_lcm;

float intensity_factor;
float led_intensity_component[NUM_LED_CHANNELS];
float transient_intensity_ratio;

float i_and_t;
float i_and_t_scaled;
float brightness_gain;

void config_led_s(Event evt){
    switch(evt){
		case ENTER:
			// Disable mic_s configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~
			// Disable counter temporarily to configure peripherals
			LL_TIM_DisableCounter(TIM2);

			// Stop ADC conversions (and therefore no DMA requests will be generated)
			LL_ADC_REG_StopConversion(ADC1);
			while (READ_BIT(ADC1->CR, ADC_CR_ADSTART) > 0){}

			// Disable ADC
//			LL_ADC_Disable(ADC1);
//			while (READ_BIT(ADC1->CR, ADC_CR_ADEN) > 0){}

			// Disable DMA and clear flags
			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
			LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
			SET_BIT(DMA1->IFCR, DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2);

			// Force reset counter using update event
			LL_TIM_GenerateEvent_UPDATE(TIM2);

			// Configure for led_s ~~~~~~~~~~~~~~~~~~~~~~~~~~
			// Update refresh counter, rollover on LCM of periods
			refresh_cntr = (refresh_cntr + 1) % led_chan_periods_lcm;

			// Fetch relevant values from ADC
			float brightness_gain_adc = (float)adc_reads[ADC_RANK_BRIGHTNESS];
			float transient_intensity_ratio_adc = (float)adc_reads[ADC_RANK_TRANSIENT_INTENSITY_RATIO];

			// Update transient component of LEDs based on previous value and decay rate
			for (uint8_t idx = 0; idx < NUM_LEDS; idx++){
				const float decay_factor = 0.9;
				led_transient_component[idx] = decay_factor * led_transient_component[idx];
			}

			// Define constant that compares how many refresh cycles need to occur before
			// transient pulse "moves" to the next LED.
			const uint8_t x_dot_over_refresh_rate = 1;

			// If rollover occurred based on x_dot_over_refresh_rate
			if (refresh_cntr % x_dot_over_refresh_rate == 0){
				// Shift transient impulses by 1
				for (uint8_t idx = NUM_LEDS - 1; idx > 0; idx--){
					transient_locations[idx] = transient_locations[idx-1];
				}

				// If a transient has occurred since the last time we checked
				if (transient_present > 0){
					// Start transient on first LED
					transient_locations[0] = 1;
				} else{
					transient_locations[0] = 0;
				}

				// Reset Transient counter
				transient_present = 0;

				// In any location where a transient has just moved, set led_transient_component to 1
				for (uint8_t idx = 0; idx < NUM_LEDS; idx++){
					if (transient_locations[idx] > 0){
						led_transient_component[idx] = 1.0f;
					}
				}
			}

			// Now calculate intensity component of LEDs
			for (uint8_t chan_idx = 0; chan_idx < NUM_LED_CHANNELS; chan_idx++){
				// Calculate led intensity component based on recorded intensity and max intensity
				// Max intensity estimated by experimentation (blowing on mic).
				// Goal is for intensity_factor to be between 0 and 1. Ceiling of 1 by design.
				intensity_factor = fminf(intensity, 1.0);
				led_intensity_component[chan_idx] = intensity_factor * ( sin((float)refresh_cntr / led_chan_periods[chan_idx]) + 1) * 0.5;
			}

			// Cycle through each LED and each color channel
			uint32_t ccr_idx = 0;
			for(int16_t led_idx = 0; led_idx < NUM_LEDS; led_idx++){
				for(uint8_t chan_idx = 0; chan_idx < NUM_LED_CHANNELS; chan_idx++){
					// Calculate channel color for this LED by scaling and combining intensity and transient component
					// Scale intensity and transient components so roughly between 0 and 255
					const float transient_intensity_ratio_scale = (1 / 4095.0); // Pre-calculate to speed up
					transient_intensity_ratio = transient_intensity_ratio_adc * transient_intensity_ratio_scale;
					i_and_t = 	( led_intensity_component[chan_idx] * (1.0-transient_intensity_ratio) ) +
								( led_transient_component[led_idx] * transient_intensity_ratio );

					// Calculate global brightness gain based on ADC read channel (potentiometer)
					const float brightness_gain_scale = 255.0 / 4095.0; // Pre-calculate to speed up
					brightness_gain = brightness_gain_adc * brightness_gain_scale;

					// Put all values together to determine intensity for this LED channel
					led_vals[led_idx][chan_idx] = (uint8_t)(fminf(brightness_gain * i_and_t, 255.0));

					// Generate array for Timer CCR values corresponding to each bit that will be written to LEDs
					for(int8_t bit_idx = 7; bit_idx >=0; bit_idx--){
						if((led_vals[led_idx][chan_idx] & (0b1 << bit_idx)) > 0){
							ccr_sequence[ccr_idx] = ccr_one;
						}
						else{
							ccr_sequence[ccr_idx] = ccr_zero;
						}
						ccr_idx++;
					}
				}
			}

			// Set timer ARR for duration to write one bit to LEDs
			LL_TIM_SetAutoReload(TIM2, led_arr);

			// Clear any pending status registers
			CLEAR_REG(TIM2->SR);
			SET_BIT(DMA1->IFCR, DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2);

			// Reset DMA counter
			LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, NUM_CCRS);

			// Enable DMA
			LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

			// Enable DMA requests triggering on timer update events
			LL_TIM_EnableDMAReq_UPDATE(TIM2);

			// Enable counter. State will transition on the next update event
			LL_TIM_EnableCounter(TIM2);
			break;
		case TIM_UE:
			// Transition to next state
			transition(led_s);
		default:
			break;
    }
}

void led_s(Event evt){
    switch(evt){
		case ENTER:
			// By this point, sampling the ADCs and running DSP is assumed to have kept
			// output low long enough for reset pulse. No need to wait for it explicitly.
			break;
		case LED_DMA_COMPLETE:
			// By this point, all LEDs have been written to.
			// Transition state
			transition(config_mic_s);
			break;
		default:
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	// Pre-calculate all constants used for DSP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  	// Define Sample Frequency
  	T_s = (float)mic_arr / 64E6;
  	f_s = 1.0 / T_s;

  	// Incoming signal HPF
  	float f_n_hpf = 1000;
  	tau_over_T_y_hpf = CalcTauOverTFromFloat(f_n_hpf, f_s);

  	// Intensity signal (slow LPF envelope filter)
  	tau_over_T_intensity = (tau_intensity_max / T_s); // Initial value, gets changed on the fly

  	// HPF for beat detect
  	float f_n_hpf_beat = 3000;
  	tau_over_T_y_hpf_beat = CalcTauOverTFromFloat(f_n_hpf_beat, f_s);

  	// Calculate LCM for LED periods
  	led_chan_periods_lcm = 0;
  	for (uint8_t chan_idx = 0; chan_idx < NUM_LED_CHANNELS; chan_idx++){
  		led_chan_periods_lcm += (uint32_t)led_chan_periods[chan_idx];
  	}


  	// Peripheral Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// Configure DMA for transfer from ADC to memory
  	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
  	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR);
  	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)adc_reads);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1); // Enable interrupt to be called when transaction complete


	// Configure DMA for transfer from memory to Timer CCR
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&TIM2->CCR1);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)ccr_sequence);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2); // Enable interrupt to be called when transaction complete

	// Calibrate ADC for better accuracy
    LL_ADC_StartCalibration(ADC1, LL_ADC_SINGLE_ENDED);
    while(LL_ADC_IsCalibrationOnGoing(ADC1)){
  	  LL_mDelay(100);
    }

    // Enable ADC (but don't start)
    LL_ADC_Enable(ADC1);
	LL_ADC_EnableIT_OVR(ADC1); // Enable Overrun interrupt

	// Configure TIM
	CLEAR_REG(TIM2->SR);
	LL_TIM_EnableIT_UPDATE(TIM2); // Enable update event interrupt
	LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1); // Enable Capture compare channel
	LL_TIM_CC_SetDMAReqTrigger(TIM2, LL_TIM_CCDMAREQUEST_UPDATE); // Set DMA Request source


	// Enter starting state ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	transition(config_mic_s);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(64000000);
  LL_SetSystemCoreClock(64000000);
  LL_RCC_SetADCClockSource(LL_RCC_ADC1_CLKSRC_PLL_DIV_1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /*
   * The ADC is configured to be triggered by the Timer2 TRGO event.
   * See STMF302xxx reference manual 15.3.18 for details on how the
   * EXTSEL bits are configured to achieve this.
   */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PC0   ------> ADC1_IN6
  PC1   ------> ADC1_IN7
  PA0   ------> ADC1_IN1
  PA1   ------> ADC1_IN2
  PA4   ------> ADC1_IN5
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

  /* ADC1 interrupt Init */
  NVIC_SetPriority(ADC1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(ADC1_IRQn);

  /* USER CODE BEGIN ADC1_Init 1 */


  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM2_TRGO;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_5RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_6);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_5, LL_ADC_CHANNEL_7);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SINGLE_ENDED);
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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 DMA Init */

  /* TIM2_UP Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_MEDIUM);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_WORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_WORD);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */
  /*
   * Code below is used to configure the timer.  The timer counter is in upcounting mode.
   *
   * When the Auto-reload value (Period value in this code) is reached, an update event is generated.
   * The configuration below also leverages the Master-Slave configuration so that the update
   * event also serves as an externally facing TRGO event.  That TRGO event is then used to
   * initiate ADC conversions.
   *
   * This configuration section also configures the timer for PWM. The lowest section sets
   * the polarity of the output on TIM_CHANNEL_1, and PWM duty cycle is set by the CCR value
   * which is modulated in the main code.
   */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 10000;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH1);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**TIM2 GPIO Configuration
  PA5   ------> TIM2_CH1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);

  /**/
  LL_GPIO_SetPinPull(B1_GPIO_Port, B1_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(B1_GPIO_Port, B1_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
  EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
