/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
#include "GFX_FUNCTIONS.h"
#include <ST7735_fonts.h>
#include "SCD41.h"
#include <stdio.h>
#include "BME280.h"
#include "AGS02MA.h"
#include "ZE08.h"
#include "PM25.h"
#include "max44009.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ST7735S_RESET_Pin GPIO_PIN_3
#define ST7735S_RESET_GPIO_Port GPIOA
#define ST7735S_CS_Pin GPIO_PIN_4
#define ST7735S_CS_GPIO_Port GPIOA
#define ST7735S_DC_Pin GPIO_PIN_6
#define ST7735S_DC_GPIO_Port GPIOA
#define BAT_Pin GPIO_PIN_1
#define BAT_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MEAS_CO2_INTERVAL1 1000
#define MEAS_CO2_INTERVAL2 5000
#define CO2_NOMINAL 800.0f
#define CO2_MAXIMUM 1000.0f
#define NORMAL_LEVEL 30.0f
#define WARN_LEVEL 50.0f
#define CRIT_LEVEL 100.0f
#define DIV_PULSE_LED 10
#define ADC_L_TUNE 5.70f
extern uint32_t scint_counter;
extern float bataryValue;
extern ADC_HandleTypeDef hadc3;

#define NORMAL_TVOC_LEVEL 100
#define WARN_TVOC_LEVEL 1000
#define TVOC_TIMEOUT 120000

#define NORMAL_CH2O 3 //2.4f
#define WARN_CH2O 28 //28.03f
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
