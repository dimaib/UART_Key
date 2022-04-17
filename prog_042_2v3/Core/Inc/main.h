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
#define PACKET_START					0x5A																																				//старт посылки
#define PACKET_END						0xFF																																				//конец посылки
#define PACKET_SIZE						11																																					//размер пакета, байт
#define DELAY_SEND						300																																					//задержка после получения команды от сервера на ответное сообщение

//0x5A, 0xA5, 0x02, 0x3D, 0x20, 0x02, 0x71, 0x01, 0x00, 0x2C, 0xFF

//#define MSG_ON_1							{0x5A, 0xA5, 0x01, 0x3D, 0x20, 0x02, 0x71, 0x01, 0x2D, 0xFF}								//команда 1 для подачи зарядки и включение магнита
//#define MSG_ON_2							{0x5A, 0xA5, 0x01, 0x3E, 0x20, 0x02, 0x58, 0x01, 0x45, 0xFF}								//команда 2 для подачи зарядки и включение магнита
#define MSG_ON_1							{0x5A, 0xA5, 0x02, 0x3D, 0x20, 0x02, 0x71, 0x01, 0x00, 0x2C, 0xFF}								//команда 1 для подачи зарядки и включение магнита
#define MSG_ON_2							{0x5A, 0xA5, 0x02, 0x3D, 0x20, 0x02, 0x71, 0x01, 0x00, 0x2C, 0xFF}								//команда 2 для подачи зарядки и включение магнита

#define FLASH_SPEED_ON				300																																					//скорость мигания светодиода, если зарядка включена
#define FLASH_SPEED_OFF				600																																					//скорость мигания светодиода, если зарядка выключена


#define CLEAR_BUF							for(uint8_t i=0;i<PACKET_SIZE;i++) buf_rx[i]=0															//очищаем буфер приема

#define OPEN									GPIOA->ODR|=(1<<2)
#define CLOSE									GPIOA->ODR&=~(1<<2)

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
#define load1_on_Pin GPIO_PIN_1
#define load1_on_GPIO_Port GPIOF
#define is_charger_Pin GPIO_PIN_0
#define is_charger_GPIO_Port GPIOA
#define debug_led_Pin GPIO_PIN_1
#define debug_led_GPIO_Port GPIOA
#define magnit_on_Pin GPIO_PIN_2
#define magnit_on_GPIO_Port GPIOA
#define baudrate_Pin GPIO_PIN_3
#define baudrate_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
