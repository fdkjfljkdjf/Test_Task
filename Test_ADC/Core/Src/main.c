/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARR_LOG 8         // размер массив для логирования
#define ADC_CHANNELS_NUM 9 // количестов каналов АЦП
#define tV_25   1.34f      // напряжение (в вольтах) на датчике при температуре 25 °C.
#define tSlope  0.0043f    // изменение напряжения (в вольтах) при изменении температуры на градус.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

uint16_t adcData[ADC_CHANNELS_NUM]; // массив данных с ADC
char transmit_str[80] = {0,};       // массив для записи и передачи времени, напряжения и температуры
uint8_t flag_adc = 0;               // флаг ADC
uint8_t flag_tim = 0;               // флаг TIMER

unsigned int second = 0;            // переменная, сохраняющая секунды
float average_voltage = 0;          // значение напряжения, усредненное по 8-и каналам
float temp = 0;                     // значение температуры
unsigned int measurement_number = 0;// номер измерения

uint8_t flag_receive = 0;           // флаг RECIVER
char command_code;                  // код команды

unsigned int arr_second_log[ARR_LOG] ={0,}; // массив секунд
float arr_voltage_log[ARR_LOG] = {0,};      // массив напряжений
float arr_temp_log[ARR_LOG] = {0,};         // массив температуры

unsigned int arr_second_log_wrn[ARR_LOG] ={0,};  // массив времени предупреждения
unsigned int arr_number_log_wrn[ARR_LOG] = {0,}; // массив номеров измерений с предупреждением
float arr_temp_log_wrn[ARR_LOG] = {0,};          // массив превышенной температуры

//uint8_t buff_1[7] = "1 get\r\n";
//uint8_t buff_2[7] = "2 get\r\n";
//uint8_t buff_3[7] = "3 get\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void array_shift_int(unsigned int arr[]);
void array_shift_float(float arr[]);
void logging(const unsigned int number_log, const unsigned int second_log, const float voltage_log, const float temp_log);
void logging_wrn(const unsigned int number_log, const unsigned int second_log, const float temp_log);
void print_log_inf_usb(unsigned int number_log);
void print_log_wrn_usb();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
	  flag_adc = 1;
  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1)
{
	if(htim1->Instance == TIM1)
	{
		flag_tim = 1;
		++second;
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
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, ADC_CHANNELS_NUM);
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(second < 5) {}
  while (1)
  {
	  if(flag_tim == 1)
	  {
      	++measurement_number;
      	sprintf(transmit_str, "%u \t %.3f V\t %.2f T\t\r\n", measurement_number, average_voltage, temp);
      	logging(measurement_number, second, average_voltage, temp);
      	if(measurement_number < 10){
      		CDC_Transmit_FS((uint8_t*)transmit_str, 23);
      	} else if (measurement_number >= 10 && measurement_number < 100){
      		CDC_Transmit_FS((uint8_t*)transmit_str, 24);
      	} else if(measurement_number >= 100 && measurement_number < 1000){
      		CDC_Transmit_FS((uint8_t*)transmit_str, 25);
      	} else {
      		CDC_Transmit_FS((uint8_t*)transmit_str, 26);
      	}
      	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, ADC_CHANNELS_NUM);
      	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      	flag_tim = 0;
	  }

	  if(flag_adc == 1)
	  {
		  average_voltage = 0;
		  temp = 0;
		  for (uint8_t i = 0; i < ADC_CHANNELS_NUM; i++)
		  {
			  if(i == 8){
				  temp = ((adcData[i] * 3.3) / 4095);
				  temp = ((tV_25 - temp) / tSlope) + 25;
				  if(temp > 28){
					  logging_wrn(measurement_number, second, temp);
				  }
			  } else {
				  average_voltage += ((adcData[i] * 3.3) / 4095);
			  }
		  }
		  average_voltage /= 8;
		  flag_adc = 0;
	  }

	  if(flag_receive == 1){

		  switch(command_code)
		  {
		  case '1':
			  print_log_inf_usb(measurement_number);
			  break;
		  case '2':
			  print_log_wrn_usb();
			  break;
		  }
	      flag_receive = 0;
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 9;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void array_shift_int(unsigned int arr[]){
    for(int i = ARR_LOG - 1; i > 0; --i){
        arr[i] = arr[i - 1];
    }
}
void array_shift_float(float arr[]){
    for(int i = ARR_LOG - 1; i > 0; --i){
        arr[i] = arr[i - 1];
    }
}

void logging_wrn(const unsigned int number_log, const unsigned int second_log, const float temp_log)
{
	array_shift_int(arr_number_log_wrn);
	array_shift_int(arr_second_log_wrn);
	array_shift_float(arr_temp_log_wrn);

	arr_number_log_wrn[0] = number_log;
	arr_second_log_wrn[0] = second_log;
	arr_temp_log_wrn[0] = temp_log;
}
void logging(const unsigned int number_log, const unsigned int second_log, const float voltage_log, const float temp_log)
{

	array_shift_int(arr_second_log);
	array_shift_float(arr_voltage_log);
	array_shift_float(arr_temp_log);

	arr_second_log[0] = second_log;
	arr_voltage_log[0] = voltage_log;
	arr_temp_log[0] = temp_log;
}

void print_log_inf_usb(unsigned int number_log){

	sprintf(transmit_str, "number: %u\t%u s\t%.2f V\t%.2f T\r\n"
			"number: %u\t%u s\t%.2f V\t%.2f T\r\n",
			number_log, arr_second_log[0], arr_voltage_log[0], arr_temp_log[0],
			number_log - 1, arr_second_log[1], arr_voltage_log[1], arr_temp_log[1]);
	if(number_log < 10){
		CDC_Transmit_FS((uint8_t*)transmit_str, 61);
	} else if (number_log >= 10 && number_log < 100){
		CDC_Transmit_FS((uint8_t*)transmit_str, 63);
	} else {
		CDC_Transmit_FS((uint8_t*)transmit_str, 68);
	}
	sprintf(transmit_str, "number: %u\t%u s\t%.2f V\t%.2f T\r\n"
			"number: %u\t%u s\t%.2f V\t%.2f T\r\n",
			number_log - 2, arr_second_log[2], arr_voltage_log[2], arr_temp_log[2],
			number_log - 3, arr_second_log[3], arr_voltage_log[3], arr_temp_log[3]);
	if(number_log < 10){
		CDC_Transmit_FS((uint8_t*)transmit_str, 61);
	} else if (number_log >= 10 && number_log < 100){
		CDC_Transmit_FS((uint8_t*)transmit_str, 63);
	} else {
		CDC_Transmit_FS((uint8_t*)transmit_str, 68);
	}
	sprintf(transmit_str, "number: %u\t%u s\t%.2f V\t%.2f T\r\n"
			"number: %u\t%u s\t%.2f V\t%.2f T\r\n",
			number_log - 4, arr_second_log[4], arr_voltage_log[4], arr_temp_log[4],
			number_log - 5, arr_second_log[5], arr_voltage_log[5], arr_temp_log[5]);
	if(number_log < 10){
		CDC_Transmit_FS((uint8_t*)transmit_str, 61);
	} else if (number_log >= 10 && number_log < 100){
		CDC_Transmit_FS((uint8_t*)transmit_str, 63);
	} else {
		CDC_Transmit_FS((uint8_t*)transmit_str, 68);
	}
	sprintf(transmit_str, "number: %u\t%u s\t%.2f V\t%.2f T\r\n"
			"number: %u\t%u s\t%.2f V\t%.2f T\r\n",
			number_log - 6, arr_second_log[6], arr_voltage_log[6], arr_temp_log[6],
			number_log - 7, arr_second_log[7], arr_voltage_log[7], arr_temp_log[7]);
	if(number_log < 10){
		CDC_Transmit_FS((uint8_t*)transmit_str, 61);
	} else if (number_log >= 10 && number_log < 100){
		CDC_Transmit_FS((uint8_t*)transmit_str, 63);
	} else {
		CDC_Transmit_FS((uint8_t*)transmit_str, 68);
	}
}


void print_log_wrn_usb(){
	sprintf(transmit_str, "number_wrn: %u\t%u s\t%.2f T\r\n"
			"number_wrn: %u\t%u s\t%.2f T\r\n",
			arr_number_log_wrn[0], arr_second_log_wrn[0], arr_temp_log_wrn[0],
			arr_number_log_wrn[1], arr_second_log_wrn[1], arr_temp_log_wrn[1]);
	if(arr_number_log_wrn[0] < 10){
		CDC_Transmit_FS((uint8_t*)transmit_str, 55);
	} else if (arr_number_log_wrn[0] >= 10 && arr_number_log_wrn[0] < 100){
		CDC_Transmit_FS((uint8_t*)transmit_str, 57);
	} else {
		CDC_Transmit_FS((uint8_t*)transmit_str, 62);
	}
	sprintf(transmit_str, "number_wrn: %u\t%u s\t%.2f T\r\n"
			"number_wrn: %u\t%u s\t%.2f T\r\n",
			arr_number_log_wrn[2], arr_second_log_wrn[2], arr_temp_log_wrn[2],
			arr_number_log_wrn[3], arr_second_log_wrn[3], arr_temp_log_wrn[3]);
	if(arr_number_log_wrn[0] < 10){
		CDC_Transmit_FS((uint8_t*)transmit_str, 55);
	} else if (arr_number_log_wrn[0] >= 10 && arr_number_log_wrn[0] < 100){
		CDC_Transmit_FS((uint8_t*)transmit_str, 57);
	} else {
		CDC_Transmit_FS((uint8_t*)transmit_str, 62);
	}
	sprintf(transmit_str, "number_wrn: %u\t%u s\t%.2f T\r\n"
			"number_wrn: %u\t%u s\t%.2f T\r\n",
			arr_number_log_wrn[4], arr_second_log_wrn[4], arr_temp_log_wrn[4],
			arr_number_log_wrn[5], arr_second_log_wrn[5], arr_temp_log_wrn[5]);
	if(arr_number_log_wrn[0] < 10){
		CDC_Transmit_FS((uint8_t*)transmit_str, 55);
	} else if (arr_number_log_wrn[0] >= 10 && arr_number_log_wrn[0] < 100){
		CDC_Transmit_FS((uint8_t*)transmit_str, 57);
	} else {
		CDC_Transmit_FS((uint8_t*)transmit_str, 62);
	}
	sprintf(transmit_str, "number_wrn: %u\t%u s\t%.2f T\r\n"
			"number_wrn: %u\t%u s\t%.2f T\r\n",
			arr_number_log_wrn[6], arr_second_log_wrn[6], arr_temp_log_wrn[6],
			arr_number_log_wrn[7], arr_second_log_wrn[7], arr_temp_log_wrn[7]);
	if(arr_number_log_wrn[0] < 10){
		CDC_Transmit_FS((uint8_t*)transmit_str, 55);
	} else if (arr_number_log_wrn[0] >= 10 && arr_number_log_wrn[0] < 100){
		CDC_Transmit_FS((uint8_t*)transmit_str, 57);
	} else {
		CDC_Transmit_FS((uint8_t*)transmit_str, 62);
	}
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
