/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct{
	uint8_t rsp_buf_av;
	uint8_t nano_buf_av;
	uint8_t misc_buf_av;
	uint8_t pending_cmds;
} kim_status_t;

extern kim_status_t kim_bufs_status;

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
#define NRESET_UHF_Pin GPIO_PIN_3
#define NRESET_UHF_GPIO_Port GPIOA
#define SPI_NSS_Pin GPIO_PIN_4
#define SPI_NSS_GPIO_Port GPIOA
#define SPI_NSS_EXTI_IRQn EXTI4_IRQn
#define BUSY_SB_Pin GPIO_PIN_0
#define BUSY_SB_GPIO_Port GPIOB
#define NRESET_SB_Pin GPIO_PIN_1
#define NRESET_SB_GPIO_Port GPIOB
#define IRQ_SB_Pin GPIO_PIN_2
#define IRQ_SB_GPIO_Port GPIOB
#define IRQ_SB_EXTI_IRQn EXTI2_IRQn
#define NSS_SB_Pin GPIO_PIN_12
#define NSS_SB_GPIO_Port GPIOB
#define NSS_UHF_Pin GPIO_PIN_13
#define NSS_UHF_GPIO_Port GPIOB
#define BUSY_UHF_Pin GPIO_PIN_11
#define BUSY_UHF_GPIO_Port GPIOA
#define IRQ_UHF_Pin GPIO_PIN_12
#define IRQ_UHF_GPIO_Port GPIOA
#define IRQ_UHF_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
