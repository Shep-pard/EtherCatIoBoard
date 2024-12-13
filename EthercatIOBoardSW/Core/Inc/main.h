/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOA
#define ESC_GPIO_Pin_RSTN_Pin GPIO_PIN_15
#define ESC_GPIO_Pin_RSTN_GPIO_Port GPIOB
#define ENC1_B_DIR_Pin GPIO_PIN_8
#define ENC1_B_DIR_GPIO_Port GPIOD
#define ENC2_B_DIR_Pin GPIO_PIN_9
#define ENC2_B_DIR_GPIO_Port GPIOD
#define ENC3_B_DIR_Pin GPIO_PIN_10
#define ENC3_B_DIR_GPIO_Port GPIOD
#define ENC4_B_DIR_Pin GPIO_PIN_11
#define ENC4_B_DIR_GPIO_Port GPIOD
#define ENC1_Z_Data_Pin GPIO_PIN_0
#define ENC1_Z_Data_GPIO_Port GPIOD
#define ENC1_Z_Data_EXTI_IRQn EXTI0_IRQn
#define ENC2_Z_Data_Pin GPIO_PIN_1
#define ENC2_Z_Data_GPIO_Port GPIOD
#define ENC2_Z_Data_EXTI_IRQn EXTI1_IRQn
#define ENC3_Z_Data_Pin GPIO_PIN_2
#define ENC3_Z_Data_GPIO_Port GPIOD
#define ENC3_Z_Data_EXTI_IRQn EXTI2_IRQn
#define ENC4_Z_Data_Pin GPIO_PIN_3
#define ENC4_Z_Data_GPIO_Port GPIOD
#define ENC4_Z_Data_EXTI_IRQn EXTI3_IRQn
#define ENC1_A_DIR_Pin GPIO_PIN_4
#define ENC1_A_DIR_GPIO_Port GPIOD
#define ENC2_A_DIR_Pin GPIO_PIN_5
#define ENC2_A_DIR_GPIO_Port GPIOD
#define ENC3_A_DIR_Pin GPIO_PIN_6
#define ENC3_A_DIR_GPIO_Port GPIOD
#define ENC4_A_DIR_Pin GPIO_PIN_7
#define ENC4_A_DIR_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
