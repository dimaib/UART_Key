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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "function.h"

#define DELAY_SEND						300															//задержка после получени€ команды от сервера на ответное сообщение

#define MAX_SIZE							200

#define FLASH_SPEED_ON				300															//скорость мигани€ светодиода, если зар€дка включена
#define FLASH_SPEED_OFF				600															//скорость мигани€ светодиода, если зар€дка выключена

#define OFF_CRC								1																//1-выключить 0-включить проверку crc при приеме через UART

#define SPEED_FLASH						400															//скорость мигани€ светодиодной лентой

extern uint8_t lock;                                          //состо€ние треккера. «аблокирован или нет
extern uint8_t perecent_accum;                                //переменна€ дл€ хранени€ процента зар€да аккумул€тора
extern uint8_t speed;                                         //переменна€ дл€ хранени€ скорости движени€

extern uint8_t pwm_red;																				//значение PWM дл€ красного цвета
extern uint8_t pwm_green;																			//значение PWM дл€ зеленого цвета
extern uint8_t pwm_blue;																			//значение PWM дл€ синего цвета
extern uint8_t pwm_inc;																				//инкремент PWM

extern uint32_t uart_baud;																		//переменна€ дл€ настройки скорости uart при включении

extern uint8_t status;																				//переменна€ дл€ хранени€ статуса
extern uint8_t command;																				//команды дл€ передачи в main

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
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
