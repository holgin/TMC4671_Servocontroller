/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define FAN_PWM_Pin GPIO_PIN_13
#define FAN_PWM_GPIO_Port GPIOC
#define FAN_SPEED_FB_Pin GPIO_PIN_14
#define FAN_SPEED_FB_GPIO_Port GPIOC
#define OVP_INPUT_Pin GPIO_PIN_15
#define OVP_INPUT_GPIO_Port GPIOC
#define OK_DRV_EN_Pin GPIO_PIN_0
#define OK_DRV_EN_GPIO_Port GPIOC
#define EN_OK_Pin GPIO_PIN_1
#define EN_OK_GPIO_Port GPIOC
#define OCD_INPUT_Pin GPIO_PIN_2
#define OCD_INPUT_GPIO_Port GPIOC
#define SOFT_START_RELAY_Pin GPIO_PIN_3
#define SOFT_START_RELAY_GPIO_Port GPIOC
#define USART2_RE_Pin GPIO_PIN_0
#define USART2_RE_GPIO_Port GPIOA
#define DEBUG_LED_Pin GPIO_PIN_4
#define DEBUG_LED_GPIO_Port GPIOF
#define NTC_HS2_Pin GPIO_PIN_4
#define NTC_HS2_GPIO_Port GPIOA
#define NTC_HS1_Pin GPIO_PIN_5
#define NTC_HS1_GPIO_Port GPIOA
#define NTC_PCB_Pin GPIO_PIN_6
#define NTC_PCB_GPIO_Port GPIOA
#define TMC_OK_LED_Pin GPIO_PIN_7
#define TMC_OK_LED_GPIO_Port GPIOA
#define DCLINK_MEAS_Pin GPIO_PIN_4
#define DCLINK_MEAS_GPIO_Port GPIOC
#define GPIO1_DIP_Pin GPIO_PIN_5
#define GPIO1_DIP_GPIO_Port GPIOC
#define GPIO2_CTRL_Pin GPIO_PIN_0
#define GPIO2_CTRL_GPIO_Port GPIOB
#define STO1_in_Pin GPIO_PIN_1
#define STO1_in_GPIO_Port GPIOB
#define STO2_in_Pin GPIO_PIN_2
#define STO2_in_GPIO_Port GPIOB
#define EM_BRAKE_Pin GPIO_PIN_11
#define EM_BRAKE_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_12
#define SPI2_NSS_GPIO_Port GPIOB
#define ENC2_SELECT_Pin GPIO_PIN_6
#define ENC2_SELECT_GPIO_Port GPIOC
#define FAULT_LED_Pin GPIO_PIN_7
#define FAULT_LED_GPIO_Port GPIOC
#define RST_TMC_Pin GPIO_PIN_11
#define RST_TMC_GPIO_Port GPIOA
#define EN_TMC_Pin GPIO_PIN_12
#define EN_TMC_GPIO_Port GPIOA
#define MAIN_ENABLE_Pin GPIO_PIN_5
#define MAIN_ENABLE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
