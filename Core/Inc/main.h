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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum {
	CANFD,
	RS485,
} Interface;

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
#define GR_EN_Pin GPIO_PIN_13
#define GR_EN_GPIO_Port GPIOC
#define GR_D0_Pin GPIO_PIN_14
#define GR_D0_GPIO_Port GPIOC
#define GR_D1_Pin GPIO_PIN_15
#define GR_D1_GPIO_Port GPIOC
#define GR_D2_Pin GPIO_PIN_0
#define GR_D2_GPIO_Port GPIOA
#define GR_D3_Pin GPIO_PIN_4
#define GR_D3_GPIO_Port GPIOA
#define F_CS_Pin GPIO_PIN_4
#define F_CS_GPIO_Port GPIOC
#define M_CS_Pin GPIO_PIN_0
#define M_CS_GPIO_Port GPIOB
#define GR_D4_Pin GPIO_PIN_1
#define GR_D4_GPIO_Port GPIOB
#define GR_D5_Pin GPIO_PIN_2
#define GR_D5_GPIO_Port GPIOB
#define DIAG_1_Pin GPIO_PIN_10
#define DIAG_1_GPIO_Port GPIOB
#define DIAG_2_Pin GPIO_PIN_11
#define DIAG_2_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define CUR_SENSE_FAULT_Pin GPIO_PIN_6
#define CUR_SENSE_FAULT_GPIO_Port GPIOC
#define F_EN_Pin GPIO_PIN_8
#define F_EN_GPIO_Port GPIOA
#define F_STATUS_Pin GPIO_PIN_9
#define F_STATUS_GPIO_Port GPIOA
#define M_FAULT_Pin GPIO_PIN_10
#define M_FAULT_GPIO_Port GPIOA
#define VDS_0_Pin GPIO_PIN_15
#define VDS_0_GPIO_Port GPIOA
#define VDS_18_Pin GPIO_PIN_10
#define VDS_18_GPIO_Port GPIOC
#define VDS_75_Pin GPIO_PIN_11
#define VDS_75_GPIO_Port GPIOC
#define IDRIVE_0_Pin GPIO_PIN_3
#define IDRIVE_0_GPIO_Port GPIOB
#define IDRIVE_18_Pin GPIO_PIN_4
#define IDRIVE_18_GPIO_Port GPIOB
#define IDRIVE_75_Pin GPIO_PIN_5
#define IDRIVE_75_GPIO_Port GPIOB
#define MODE_0_Pin GPIO_PIN_6
#define MODE_0_GPIO_Port GPIOB
#define MODE_45_Pin GPIO_PIN_7
#define MODE_45_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

uint32_t calculateCrc(uint8_t *data, uint16_t length);
void startDfu();
void doRs485Test();
void doFdCanTest();

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
