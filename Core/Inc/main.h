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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rollon.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PERIOD_T1 415
#define HPERIOD_T1 (PERIOD_T1/2)
#define HPERIOD_PWR ((PERIOD_T1/10)*9)
#define LED_FLT_Pin GPIO_PIN_5
#define LED_FLT_GPIO_Port GPIOE
#define LED_STATUS_Pin GPIO_PIN_6
#define LED_STATUS_GPIO_Port GPIOE
#define AN_VBUS_Pin GPIO_PIN_0
#define AN_VBUS_GPIO_Port GPIOA
#define AN_TEMP_Pin GPIO_PIN_3
#define AN_TEMP_GPIO_Port GPIOA
#define AN_ISEN_Pin GPIO_PIN_4
#define AN_ISEN_GPIO_Port GPIOA
#define WLAN_IRQ_Pin GPIO_PIN_0
#define WLAN_IRQ_GPIO_Port GPIOB
#define PWM_PGEN_Pin GPIO_PIN_9
#define PWM_PGEN_GPIO_Port GPIOE
#define PWM_FGEN_Pin GPIO_PIN_11
#define PWM_FGEN_GPIO_Port GPIOE
#define WLAN_RST_Pin GPIO_PIN_13
#define WLAN_RST_GPIO_Port GPIOE
#define WLAN_EN_Pin GPIO_PIN_14
#define WLAN_EN_GPIO_Port GPIOE
#define ETH_RXERR_Pin GPIO_PIN_14
#define ETH_RXERR_GPIO_Port GPIOB
#define UART5_RE_Pin GPIO_PIN_8
#define UART5_RE_GPIO_Port GPIOC
#define EEPROM_SS_Pin GPIO_PIN_15
#define EEPROM_SS_GPIO_Port GPIOA
#define EEPROM_MOSI_Pin GPIO_PIN_5
#define EEPROM_MOSI_GPIO_Port GPIOB
#define BME_SDA_Pin GPIO_PIN_7
#define BME_SDA_GPIO_Port GPIOB
#define BME_SCL_Pin GPIO_PIN_8
#define BME_SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
