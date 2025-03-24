/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4

#include "fatfs_sd.h"
#include "i2c-lcd.h"
#include <arm_math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define __VREFANALOG_VOLTAGE__	3300
#define SAMPLING_RATE 4000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

enum{ ADC_BUFFER_LENGTH = 8192};//8192};
uint16_t ADCBuffer[ADC_BUFFER_LENGTH/*/(sizeof(uint32_t)/sizeof(uint16_t))*/] = {0};
int16_t ADCOutput[ADC_BUFFER_LENGTH] = {0};
uint32_t measurementNumber;

HAL_StatusTypeDef ADCStatus;

int high_privacy;

enum{ROBOT_FILTER_TAPS_NUM = 100, HIGH_PASS_FILTER_TAPS_NUM = 79, HIGH_PRIVACY_FILTER_TAPS_NUM = 79, BLOCK_SAMPLES_NUM=ADC_BUFFER_LENGTH/2};
arm_fir_instance_f32 h300_filter_settings, h500_filter_settings, robot_filter_settings;

arm_rfft_fast_instance_f32 fft_instance_S;
char raw_frequency_string[16] = "2000\t\n";
char robot_frequency_string[16] = "2000\t\n";

float32_t robot_taps[ROBOT_FILTER_TAPS_NUM] = {
		1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1,
		 1
};
float32_t h300_filter_taps[HIGH_PASS_FILTER_TAPS_NUM] = {
	0.08079731151294718,
	-0.05699332190930257,
	-0.03614592292168836,
	-0.02000690273267152,
	-0.007845346965587816,
	0.0013148162018981184,
	0.007370194875646592,
	0.011245274010723763,
	0.013032121792291426,
	0.013038141911222729,
	0.011538166530483581,
	0.00863213996693967,
	0.004754263244010325,
	0.000257267420613976,
	-0.004388062928655355,
	-0.008659548283908107,
	-0.01213450412784642,
	-0.014306039905882605,
	-0.014809755980614939,
	-0.013402874098491805,
	-0.010031506662339018,
	-0.004957697248216561,
	0.00137233719025885,
	0.008273047983331254,
	0.014927717571158268,
	0.02046933509686823,
	0.023985116370398078,
	0.024690024374717803,
	0.02196022088100341,
	0.015451040582011733,
	0.005181540050606731,
	-0.008501196798671071,
	-0.024871997202005242,
	-0.042923613617873266,
	-0.061429815713810315,
	-0.07902431910218481,
	-0.09438231636583465,
	-0.1062979699405022,
	-0.11383895711169938,
	0.8835754821844368,
	-0.11383895711169938,
	-0.1062979699405022,
	-0.09438231636583465,
	-0.07902431910218481,
	-0.061429815713810315,
	-0.042923613617873266,
	-0.024871997202005242,
	-0.008501196798671071,
	0.005181540050606731,
	0.015451040582011733,
	0.02196022088100341,
	0.024690024374717803,
	0.023985116370398078,
	0.02046933509686823,
	0.014927717571158268,
	0.008273047983331254,
	0.00137233719025885,
	-0.004957697248216561,
	-0.010031506662339018,
	-0.013402874098491805,
	-0.014809755980614939,
	-0.014306039905882605,
	-0.01213450412784642,
	-0.008659548283908107,
	-0.004388062928655355,
	0.000257267420613976,
	0.004754263244010325,
	0.00863213996693967,
	0.011538166530483581,
	0.013038141911222729,
	0.013032121792291426,
	0.011245274010723763,
	0.007370194875646592,
	0.0013148162018981184,
	-0.007845346965587816,
	-0.02000690273267152,
	-0.03614592292168836,
	-0.05699332190930257,
	0.08079731151294718
};

float32_t h500_filter_taps[HIGH_PRIVACY_FILTER_TAPS_NUM] = {
  -0.07516447075974335,
  0.06251255225290474,
  0.03439297597838555,
  0.013532398620701706,
  -0.00044894064803285063,
  -0.009285437316757177,
  -0.013647051567478221,
  -0.014430257518063025,
  -0.01239046209762652,
  -0.008259050602957467,
  -0.0029819825926192377,
  0.002586539725273842,
  0.007688943034281563,
  0.011491754485430177,
  0.013296364336483475,
  0.012716358166584698,
  0.009712064687021718,
  0.0046655206159693895,
  -0.0016546694641123086,
  -0.008190639118758674,
  -0.013716443794195766,
  -0.01707215399789964,
  -0.017367814661743235,
  -0.014139106769718754,
  -0.00753175593284382,
  0.0016277526795339908,
  0.011912958399976156,
  0.021464237043695086,
  0.028257106389250112,
  0.03041411109780532,
  0.026507564931453686,
  0.015833275027789986,
  -0.0014290015819656088,
  -0.024177981494162205,
  -0.05045861712889163,
  -0.07767713882175666,
  -0.10293270080793987,
  -0.1234025540007905,
  -0.13673224951231597,
  0.8586412582866778,
  -0.13673224951231597,
  -0.1234025540007905,
  -0.10293270080793987,
  -0.07767713882175666,
  -0.05045861712889163,
  -0.024177981494162205,
  -0.0014290015819656088,
  0.015833275027789986,
  0.026507564931453686,
  0.03041411109780532,
  0.028257106389250112,
  0.021464237043695086,
  0.011912958399976156,
  0.0016277526795339908,
  -0.00753175593284382,
  -0.014139106769718754,
  -0.017367814661743235,
  -0.01707215399789964,
  -0.013716443794195766,
  -0.008190639118758674,
  -0.0016546694641123086,
  0.0046655206159693895,
  0.009712064687021718,
  0.012716358166584698,
  0.013296364336483475,
  0.011491754485430177,
  0.007688943034281563,
  0.002586539725273842,
  -0.0029819825926192377,
  -0.008259050602957467,
  -0.01239046209762652,
  -0.014430257518063025,
  -0.013647051567478221,
  -0.009285437316757177,
  -0.00044894064803285063,
  0.013532398620701706,
  0.03439297597838555,
  0.06251255225290474,
  -0.07516447075974335
};


float32_t h300_fir_state[BLOCK_SAMPLES_NUM + HIGH_PASS_FILTER_TAPS_NUM - 1];
//float32_t h500_fir_state[BLOCK_SAMPLES_NUM + HIGH_PRIVACY_FILTER_TAPS_NUM - 1];
float32_t robot_fir_state[BLOCK_SAMPLES_NUM + ROBOT_FILTER_TAPS_NUM - 1];

FATFS fs;  // file system
FIL unfiltered_file;
char unfiltered_fp[] = "Unfiltered_Data.bin";
FIL filtered_file;
char filtered_fp[] = "Filtered_Data.bin";
FILINFO fno;
FRESULT fresult;  // result

UINT unfiltered_bw, filtered_bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* Mount SD card*/
  fresult = f_mount(&fs, "/", 1);
  if (fresult != FR_OK)
	  Error_Handler();

  f_open(&unfiltered_file, unfiltered_fp, FA_CREATE_ALWAYS);
  f_close(&unfiltered_file);

  f_open(&filtered_file, filtered_fp, FA_CREATE_ALWAYS);
  f_close(&filtered_file);

  arm_fir_init_f32(&h300_filter_settings, HIGH_PASS_FILTER_TAPS_NUM, h300_filter_taps, h300_fir_state, BLOCK_SAMPLES_NUM);
  //arm_fir_init_f32(&h500_filter_settings, HIGH_PRIVACY_FILTER_TAPS_NUM, h500_filter_taps, h500_fir_state, BLOCK_SAMPLES_NUM);
  arm_fir_init_f32(&robot_filter_settings, ROBOT_FILTER_TAPS_NUM, robot_taps, robot_fir_state, BLOCK_SAMPLES_NUM);
  arm_rfft_fast_init_f32(&fft_instance_S, BLOCK_SAMPLES_NUM);

  lcd_init();

  //if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  	//  Error_Handler();

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) ADCBuffer, ADC_BUFFER_LENGTH) != HAL_OK)
	  Error_Handler();

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
	  Error_Handler();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  //lcd_send_string(frequency_string);
	  //HAL_Delay(20);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 230400;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void analyse_signal(int16_t* src, uint32_t N, char* frequency_string) {
	float32_t floatBuffer[N];
	float32_t outputBuffer[N];

	// Convert to float array
	for(uint32_t i = 0; i < N; i++) {floatBuffer[i] = (float32_t) src[i];}

	/*
	 * 	[in]	S			points to an arm_rfft_fast_instance_f32 structure
		[in]	p			points to input buffer (Source buffer is modified by this function.)
		[in]	pOut		points to output buffer
		[in]	ifftFlag   	0: RFFT or 1: RIFFT
	 *
	 * The function supports lengths of [32, 64, 128, ..., 4096] samples
	 */
	arm_rfft_fast_f32(&fft_instance_S, floatBuffer, outputBuffer, 0);

	/*
	 * According to documentation, outputBuffer[0] represents the DC offset and outputBuffer[1] is always zero.
	 * Otherwise each pair (e.g. outputBuffer[2], outputBuffer[3]) is the real and imaginary component respectively
	 */
	// So this might be needed?
	//outputBuffer[0] = 0;

	outputBuffer[0] = 0;

	// Compute magnitude of each element in output buffer
	float32_t magnitudeBuffer[N];
	arm_cmplx_mag_f32(outputBuffer, magnitudeBuffer, N/2);

	// Find maximum magnitude and corresponding index
	float32_t maxValue;
	uint32_t maxIndex;
	arm_max_f32(magnitudeBuffer, N/2, &maxValue, &maxIndex);

	// Calculate frequency of maximum magnitude
	uint16_t freq = (maxIndex * SAMPLING_RATE) / N;

	//arm_sort_f32(&sort_instance_S, magnitudeBuffer);

	sprintf(frequency_string,"%-16hu",freq);

	//return freq;
}


void process_signal(int16_t* dest, uint16_t* src, uint32_t N){

	//memcpy(dest,src,N*sizeof(uint16_t));

	float32_t filterBuffer[N];
	float32_t outputBuffer[N];

	for(uint32_t i = 0; i < N; i++) {filterBuffer[i] = (float32_t) src[i];}

	arm_fir_f32(&h300_filter_settings,filterBuffer,outputBuffer,N);

	if (high_privacy) {
		float32_t robotBuffer[N];

		arm_fir_f32(&robot_filter_settings,outputBuffer,robotBuffer,N);
		memcpy(outputBuffer,robotBuffer,N*sizeof(float32_t));
		//arm_fir_f32(&h500_filter_settings,robotBuffer,outputBuffer,N);
	}

	for(uint32_t i = 0; i < N; i++) {dest[i] = (int16_t) outputBuffer[i];}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

   // do nothing here
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* AdcHandle) {

	process_signal(ADCOutput,ADCBuffer,BLOCK_SAMPLES_NUM);
	measurementNumber += BLOCK_SAMPLES_NUM;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) ADCOutput, BLOCK_SAMPLES_NUM * sizeof(int16_t));
	fresult = f_open(&filtered_file, filtered_fp, FA_OPEN_APPEND | FA_WRITE);
	fresult = f_write(&filtered_file,ADCOutput, BLOCK_SAMPLES_NUM * sizeof(int16_t),&filtered_bw);
	fresult = f_close(&filtered_file);
	fresult = f_open(&unfiltered_file, unfiltered_fp, FA_OPEN_APPEND | FA_WRITE);
	fresult = f_write(&unfiltered_file,ADCBuffer, BLOCK_SAMPLES_NUM * sizeof(uint16_t),&unfiltered_bw);
	fresult = f_close(&unfiltered_file);

	analyse_signal(ADCBuffer, BLOCK_SAMPLES_NUM, raw_frequency_string);
	lcd_set_line(1, raw_frequency_string);
	analyse_signal(ADCOutput, BLOCK_SAMPLES_NUM, robot_frequency_string);
	lcd_set_line(2, robot_frequency_string);

	//HAL_UART_Transmit(&huart2, (uint8_t*) ADCOutput, ADC_BUFFER_LENGTH/2 * sizeof(int16_t),HAL_MAX_DELAY);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
	//if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
	//	Error_Handler();

	//flag = 1;
	process_signal(ADCOutput + BLOCK_SAMPLES_NUM,ADCBuffer + ADC_BUFFER_LENGTH/2,BLOCK_SAMPLES_NUM);
	measurementNumber += BLOCK_SAMPLES_NUM;
	HAL_UART_Transmit_DMA(&huart2, (uint8_t*) (ADCOutput + ADC_BUFFER_LENGTH/2), BLOCK_SAMPLES_NUM * sizeof(int16_t));
	fresult = f_open(&filtered_file, filtered_fp, FA_OPEN_APPEND | FA_WRITE);
	fresult = f_write(&filtered_file,ADCOutput + ADC_BUFFER_LENGTH/2, BLOCK_SAMPLES_NUM * sizeof(int16_t),&filtered_bw);
	fresult = f_close(&filtered_file);
	fresult = f_open(&unfiltered_file, unfiltered_fp, FA_OPEN_APPEND | FA_WRITE);
	fresult = f_write(&unfiltered_file,ADCBuffer + ADC_BUFFER_LENGTH/2, BLOCK_SAMPLES_NUM * sizeof(uint16_t),&unfiltered_bw);
	fresult = f_close(&unfiltered_file);

	analyse_signal(ADCBuffer + ADC_BUFFER_LENGTH/2, BLOCK_SAMPLES_NUM, raw_frequency_string);
	lcd_set_line(1, raw_frequency_string);
	analyse_signal(ADCOutput + ADC_BUFFER_LENGTH/2, BLOCK_SAMPLES_NUM, robot_frequency_string);
	lcd_set_line(2, robot_frequency_string);

	//HAL_UART_Transmit(&huart2, (uint8_t*) (ADCOutput + ADC_BUFFER_LENGTH/2), ADC_BUFFER_LENGTH/2 * sizeof(int16_t),HAL_MAX_DELAY);
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	high_privacy = 1 - high_privacy;
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim) {
}

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
