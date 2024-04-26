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
#include "stm32g4xx_hal.h"
#include <arm_math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define _U_DC 208
#define _SQRT3 1.73205081
#define _PIdiv3 1.04719755


extern float32_t U_alpha, U_beta;
extern float32_t ThetaV, ThetaC;
extern float32_t counterfrequency;
extern float32_t U_max;
extern float32_t sinevalue,cosinevalue;
extern float32_t DFreq;
extern uint16_t *switchtime[3];
extern uint16_t adrT;
extern uint16_t SVM[180];
extern uint16_t currVec;
extern uint16_t hasStarted;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LEDPin_Pin GPIO_PIN_5
#define LEDPin_GPIO_Port GPIOA
#define Start_Pin GPIO_PIN_8
#define Start_GPIO_Port GPIOC
#define Short_Pin GPIO_PIN_8
#define Short_GPIO_Port GPIOA
#define Short_EXTI_IRQn EXTI9_5_IRQn
#define V_c_Pin GPIO_PIN_9
#define V_c_GPIO_Port GPIOA
#define V_c_EXTI_IRQn EXTI9_5_IRQn
#define V_b_Pin GPIO_PIN_10
#define V_b_GPIO_Port GPIOA
#define V_b_EXTI_IRQn EXTI15_10_IRQn
#define V_a_Pin GPIO_PIN_11
#define V_a_GPIO_Port GPIOA
#define V_a_EXTI_IRQn EXTI15_10_IRQn
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define LAA1_Pin GPIO_PIN_4
#define LAA1_GPIO_Port GPIOB
#define LAA2_Pin GPIO_PIN_5
#define LAA2_GPIO_Port GPIOB
#define LBB1_Pin GPIO_PIN_6
#define LBB1_GPIO_Port GPIOB
#define LBB2_Pin GPIO_PIN_7
#define LBB2_GPIO_Port GPIOB
#define LCC1_Pin GPIO_PIN_8
#define LCC1_GPIO_Port GPIOB
#define LCC2_Pin GPIO_PIN_9
#define LCC2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
