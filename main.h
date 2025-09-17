/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "defines.h"
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
void delay300ns();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VAC_HARDWARE_VERSION_TEXT       "K549-4"
#define VAC_HARDWARE_VERSION            4
#define VAC_PROGRAM_VERSION             13

/* USER CODE BEGIN Private defines */

typedef union {
  uint32_t data;
  struct {
    uint32_t oneMs      : 1;
    uint32_t twoMs      : 1;
    uint32_t threeMs    : 1;
    uint32_t fourMs     : 1;
    uint32_t fiveMs     : 1;
    uint32_t sixMs      : 1;
    uint32_t sevenMs    : 1;
    uint32_t eightMs    : 1;
    uint32_t nineMs     : 1;
    uint32_t tenMs      : 1;                            // 10
    uint32_t twentyMs   : 1;
    uint32_t thirtyMs   : 1;
    uint32_t fourtyMs   : 1;
    uint32_t fiftyMs    : 1;
    uint32_t sixtyMs    : 1;
    uint32_t seventyMs  : 1;
    uint32_t eightyMs   : 1;
    uint32_t ninetyMs   : 1;
    uint32_t hundredMs  : 1;
    uint32_t oneSecond  : 1;                            // 20
    uint32_t appRun     : 1;                            // 21
    uint32_t b22        : 1;                            // 22
    uint32_t b23        : 1;                            // 23
    uint32_t b24        : 1;                            // 24
    uint32_t b25        : 1;                            // 25
    uint32_t b26        : 1;                            // 26
    uint32_t b27        : 1;                            // 27
    uint32_t b28        : 1;                            // 28
    uint32_t b29        : 1;                            // 29
    uint32_t b30        : 1;                            // 30
    uint32_t b31        : 1;                            // 31
    uint32_t b32        : 1;                            // 32
  };
} flagTimer_t;
extern flagTimer_t flagTimer;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
