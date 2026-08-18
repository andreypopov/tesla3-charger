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
#define PIN_CHG_Pin GPIO_PIN_13
#define PIN_CHG_GPIO_Port GPIOC
#define PIN_DCDC_Pin GPIO_PIN_14
#define PIN_DCDC_GPIO_Port GPIOC
#define PIN_PilotDown_Pin GPIO_PIN_1
#define PIN_PilotDown_GPIO_Port GPIOD
#define PIN_Pilot_Pin GPIO_PIN_0
#define PIN_Pilot_GPIO_Port GPIOA
#define PIN_Proximity_Pin GPIO_PIN_1
#define PIN_Proximity_GPIO_Port GPIOA
#define PIN_Temp1_Pin GPIO_PIN_2
#define PIN_Temp1_GPIO_Port GPIOA
#define PIN_Temp2_Pin GPIO_PIN_3
#define PIN_Temp2_GPIO_Port GPIOA
#define PIN_Res_Pin GPIO_PIN_4
#define PIN_Res_GPIO_Port GPIOA
#define PIN_DC_Pin GPIO_PIN_6
#define PIN_DC_GPIO_Port GPIOA
#define PIN_CS_Pin GPIO_PIN_0
#define PIN_CS_GPIO_Port GPIOB
#define PIN_Led_Pin GPIO_PIN_10
#define PIN_Led_GPIO_Port GPIOB
#define PIN_In2_Pin GPIO_PIN_12
#define PIN_In2_GPIO_Port GPIOB
#define PIN_In1_Pin GPIO_PIN_13
#define PIN_In1_GPIO_Port GPIOB
#define PIN_Out2_Pin GPIO_PIN_14
#define PIN_Out2_GPIO_Port GPIOB
#define PIN_Out1_Pin GPIO_PIN_15
#define PIN_Out1_GPIO_Port GPIOB
#define PIN_RS_Pin GPIO_PIN_8
#define PIN_RS_GPIO_Port GPIOA
#define PIN_Buzzer_Pin GPIO_PIN_15
#define PIN_Buzzer_GPIO_Port GPIOA
#define PIN_Sb_up_Pin GPIO_PIN_3
#define PIN_Sb_up_GPIO_Port GPIOB
#define PIN_Sb_down_Pin GPIO_PIN_4
#define PIN_Sb_down_GPIO_Port GPIOB
#define PIN_Sb_left_Pin GPIO_PIN_5
#define PIN_Sb_left_GPIO_Port GPIOB
#define PIN_Sb_right_Pin GPIO_PIN_6
#define PIN_Sb_right_GPIO_Port GPIOB
#define PIN_Sb_ok_Pin GPIO_PIN_7
#define PIN_Sb_ok_GPIO_Port GPIOB
#define PIN_PCS_Pin GPIO_PIN_8
#define PIN_PCS_GPIO_Port GPIOB
#define PIN_power_on_Pin GPIO_PIN_9
#define PIN_power_on_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
