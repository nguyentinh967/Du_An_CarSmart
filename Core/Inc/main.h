/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define S3_Pin GPIO_PIN_13
#define S3_GPIO_Port GPIOC
#define S2_Pin GPIO_PIN_14
#define S2_GPIO_Port GPIOC
#define S1_Pin GPIO_PIN_15
#define S1_GPIO_Port GPIOC
#define In1_Pin GPIO_PIN_1
#define In1_GPIO_Port GPIOA
#define In2_Pin GPIO_PIN_2
#define In2_GPIO_Port GPIOA
#define In3_Pin GPIO_PIN_3
#define In3_GPIO_Port GPIOA
#define In4_Pin GPIO_PIN_4
#define In4_GPIO_Port GPIOA
#define S4_Pin GPIO_PIN_5
#define S4_GPIO_Port GPIOA
#define S5_Pin GPIO_PIN_0
#define S5_GPIO_Port GPIOB
#define CLB_Line_Pin GPIO_PIN_1
#define CLB_Line_GPIO_Port GPIOB
#define Near_Line_Pin GPIO_PIN_10
#define Near_Line_GPIO_Port GPIOB
#define Trigger_Pin GPIO_PIN_9
#define Trigger_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
