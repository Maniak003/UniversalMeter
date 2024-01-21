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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t CO2Counter, CO2Interval, scint_counter = 0, scint_timer, battaryLevel = 0 ;
uint16_t CO2;
sSensorMeasurement_t messuremetData;
char text1306[18];
uint16_t ze08_value;
//uint8_t Data_Buffer[9];
uint8_t PM25_buffer[32];
float temperature, pressure, humidity, bataryValue = 0, soundPress = 48.2f, MAX44009_lux = 1150;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC3_Init(void);
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	//HAL_Delay(10);
	//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  uint16_t serialBufer[3];
  serialBufer[0] = 0;
  serialBufer[1] = 0;
  serialBufer[2] = 0;

  ST7735_Init();
  ST7735_FillScreen(ST7735_BLACK);

  ST7735_WriteString(0, 0, "SCD41 init  ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
  moduleReinit();
  enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);  // Write: C4 0 D:3F 0 D:86 0
  HAL_Delay(500);
  if (getSerialNumber(serialBufer)) { // Write: C4 0 D:36 0 D:82 0 Read: C5 0 D:EA 0 D:AE 0 D:FA 0 D:B7 0 D:07 0 D:29 0 D:3B 0 D:BF Write: C4 0 D:21 0 D:B1 0
	ST7735_WriteString(96, 0, "Ok", Font_7x10, ST7735_GREEN, ST7735_BLACK);
  } else {
	ST7735_WriteString(96, 0, "Fail", Font_7x10, ST7735_RED, ST7735_BLACK);
  }
  enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);

  ST7735_WriteString(0, 11, "BME280 init ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
  BME280_Init();
  ST7735_WriteString(96, 11, "Ok", Font_7x10, ST7735_GREEN, ST7735_BLACK);

  uint32_t vvv = AGS02MA_getFirmwareVersion();
  sprintf(text1306, "VER:%lu  ", vvv);
  ST7735_WriteString(0, 44, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);

  HAL_Delay(500);
  ST7735_FillScreen(ST7735_BLACK);
  CO2Interval = MEAS_CO2_INTERVAL2;
  CO2Counter = HAL_GetTick();
  scint_counter = 0;
  scint_timer = CO2Counter;

  MAX44009_Init(&MAX44009_PORT);
  MAX44009_ContinuousMode(1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	if (HAL_GetTick() - CO2Counter > CO2Interval) {
	  		CO2Counter = HAL_GetTick();
	  		readMeasurement(&messuremetData);
			if (messuremetData.CO2ppm > 0) {
				CO2 = messuremetData.CO2ppm;
				CO2Interval = MEAS_CO2_INTERVAL2;
			} else {
				CO2Interval = MEAS_CO2_INTERVAL1;
			}
			sprintf(text1306, "CO2:%dppm  ", CO2);
			if (CO2 < CO2_NOMINAL) {
				ST7735_WriteString(0, 0, text1306, Font_7x10, ST7735_GREEN, ST7735_BLACK);
			} else if (CO2 < CO2_MAXIMUM) {
				ST7735_WriteString(0, 0, text1306, Font_7x10, ST7735_YELLOW, ST7735_BLACK);
			} else {
				ST7735_WriteString(0, 0, text1306, Font_7x10, ST7735_RED, ST7735_BLACK);
			}
	  	}

	  	//memset(text1306, 0, sizeof(text1306));
	  	if (HAL_GetTick() - scint_timer > 0 ) {
			float mmm = (float) scint_counter / (float) (HAL_GetTick() - scint_timer) * 1000.0f;
			sprintf(text1306, "cps:%.2f %.1f%% ", mmm , 300.f / sqrt((float) scint_counter));
			if (mmm < NORMAL_LEVEL) {
				ST7735_WriteString(0, 11, text1306, Font_7x10, ST7735_GREEN, ST7735_BLACK);
			} else if (mmm < WARN_LEVEL) {
				ST7735_WriteString(0, 11, text1306, Font_7x10, ST7735_YELLOW, ST7735_BLACK);
			} else if (mmm < CRIT_LEVEL) {
				ST7735_WriteString(0, 11, text1306, Font_7x10, ST7735_RED, ST7735_BLACK);
			} else {
				ST7735_WriteString(0, 11, text1306, Font_7x10, ST7735_MAGENTA, ST7735_BLACK);
			}
		}

		temperature = BME280_ReadTemperature();
		pressure = BME280_ReadPressure() * 0.00750063755419211f; //0.00750063755419211
		humidity = BME280_ReadHumidity();
		sprintf(text1306, "T:%.0fC P:%.0f H:%.0f%% ", temperature, pressure, humidity);
		ST7735_WriteString(0, 22, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);

		/* TVOC
		 *  Норма содержания летучих веществ в воздухе - до 0.5 mg/m3
		 *  0.8 mg/m3 -- предельное значение.
		 */
		if (HAL_GetTick() > TVOC_TIMEOUT) {
			uint32_t tvoc = AGS02MA_getTVOC();
			sprintf(text1306, "TVOC:%luppb   ", tvoc);
			if (tvoc < NORMAL_TVOC_LEVEL) {
				ST7735_WriteString(0, 33, text1306, Font_7x10, ST7735_GREEN, ST7735_BLACK);
			} else if (tvoc < WARN_TVOC_LEVEL) {
				ST7735_WriteString(0, 33, text1306, Font_7x10, ST7735_YELLOW, ST7735_BLACK);
			} else {
				ST7735_WriteString(0, 33, text1306, Font_7x10, ST7735_RED, ST7735_BLACK);
			}
		} else {
			uint16_t tmout = (TVOC_TIMEOUT - HAL_GetTick()) / 1000;
			sprintf(text1306, "TVOC:WrmUP(%u)  ", tmout);
			ST7735_WriteString(0, 33, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);
		}

		/* ZE08
		 * https://gazoanalizators.ru/converter/
		 * 1 ppm формальдегида CH2O равен 1.24577 мг/м3
		 * Средне-суточная ПДК концентрация -- 0.003 мг/м3 0.0024ppm
		 * Разовая ПДК 0,035 мг/м3 -- 0.02803ppm
		 * */

		if((ze08_value = ZE08_readData()) != 0xFFFF) {
			/*
			//ze08_value = ZE08_readData(Data_Buffer, sizeof(Data_Buffer));

			for (int iii = 0; iii < sizeof(Data_Buffer) - 3; iii++) {
				sprintf(text1306, "%2x", Data_Buffer[iii]);
				ST7735_WriteString(iii * 16, 44, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);
			}*/
			sprintf(text1306, "CH2O:%uppb     ", ze08_value);
			if (ze08_value < NORMAL_CH2O) {
				ST7735_WriteString(0, 44, text1306, Font_7x10, ST7735_GREEN, ST7735_BLACK);
			} else if (ze08_value < WARN_CH2O) {
				ST7735_WriteString(0, 44, text1306, Font_7x10, ST7735_YELLOW, ST7735_BLACK);
			} else {
				ST7735_WriteString(0, 44, text1306, Font_7x10, ST7735_RED, ST7735_BLACK);
			}
			//	ST7735_WriteString(0, 44, "CH2O: N/A     ", Font_7x10, ST7735_WHITE, ST7735_BLACK);
			//}
		}

		/* PM25*/

		if (PM25_GetData(PM25_buffer, sizeof(PM25_buffer))) {
/*
		for (int iii = 0; iii < sizeof(PM25_buffer); iii++) {
			sprintf(text1306, "%2x", PM25_buffer[iii]);
			ST7735_WriteString(iii * 16, 55, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);
		}
*/
			uint16_t part03  = PM25_buffer[16] << 8 | PM25_buffer[17];
			uint16_t part05  = PM25_buffer[18] << 8 | PM25_buffer[19];
			uint16_t part10  = PM25_buffer[20] << 8 | PM25_buffer[21];
			uint16_t part25  = PM25_buffer[22] << 8 | PM25_buffer[23];
			uint16_t part50  = PM25_buffer[24] << 8 | PM25_buffer[25];
			uint16_t part100 = PM25_buffer[26] << 8 | PM25_buffer[27];

			sprintf(text1306, "PM0.3 :%u  ", part03);
			ST7735_WriteString(0, 55, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);
			sprintf(text1306, "PM0.5 :%u  ", part05);
			ST7735_WriteString(0, 66, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);
			sprintf(text1306, "PM1.0 :%u  ", part10);
			ST7735_WriteString(0, 77, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);
			sprintf(text1306, "PM2.5 :%u  ", part25);
			ST7735_WriteString(0, 88, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);
			sprintf(text1306, "PM5.0 :%u  ", part50);
			ST7735_WriteString(0, 99, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);
			sprintf(text1306, "PM10  :%u  ", part100);
			ST7735_WriteString(0, 110, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);
		}

		/* Напряжение источника питания */
		bataryValue = (float) battaryLevel / 4096 * ADC_L_TUNE;
		if (bataryValue > 3) {
			sprintf(text1306, "%.2f", bataryValue);
		} else {
			sprintf(text1306, "BatL");
		}
		ST7735_WriteString(98, 0, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);
		HAL_ADC_Start_DMA(&hadc3, &battaryLevel, 1);


		/* Звуковое давление */
		sprintf(text1306, "Sound :%0.1f db", soundPress);
		ST7735_WriteString(0, 121, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);

		/* Освещенность */
		if(MAX44009_OK == MAX44009_ReadLightHighResolution(&MAX44009_lux)) {
			sprintf(text1306, "Illum :%0.1f lux   ", MAX44009_lux);
		} else {
			sprintf(text1306, "Illum :%0.0f lux   ", 0.0f);
		}
		ST7735_WriteString(0, 132, text1306, Font_7x10, ST7735_WHITE, ST7735_BLACK);

		HAL_Delay(1000);
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
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
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hi2c1.Init.Timing = 0x3010BEFF;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim4, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_RX;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ST7735S_RESET_Pin|ST7735S_CS_Pin|ST7735S_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ST7735S_RESET_Pin ST7735S_CS_Pin ST7735S_DC_Pin */
  GPIO_InitStruct.Pin = ST7735S_RESET_Pin|ST7735S_CS_Pin|ST7735S_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
