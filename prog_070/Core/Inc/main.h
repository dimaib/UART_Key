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
#define PACKET_SIZE						10																																					//размер пакета, байт
#define DELAY_SEND						300																																					//задержка после получения команды от сервера на ответное сообщение

//#define MSG_OPEN							{0x5A, 0xA5, 0x01, 0x20, 0x3F, 0x02, 0x1A, 0x01, 0x82, 0xFF}								//команда для открытия
#define MSG_OPEN							{0x5A, 0xA5, 0x01, 0x3D, 0x20, 0x02, 0x70, 0x01, 0x2E, 0xFF}								//команда для открытия

#define MSG_OPEN_TX						{0x5A, 0xA5, 0x01, 0x3F, 0x20, 0x05, 0x1A, 0x00, 0x80, 0xFF}								//возврат на команду открытие
#define MSG_STATUS						{0x5A, 0xA5, 0x01, 0x20, 0x3F, 0x01, 0x1C, 0x01, 0x81, 0xFF}								//команда запрос статуса
#define MSG_STATUS_CLOUSE			{0x5A, 0xA5, 0x01, 0x3F, 0x20, 0x04, 0x1C, 0x00, 0x7F, 0xFF}								//возврат статуса - закрыт
#define MSG_STATUS_OPEN				{0x5A, 0xA5, 0x01, 0x3F, 0x20, 0x04, 0x1C, 0x01, 0x7E, 0xFF}								//возврат статуса - открыт
#define MSG_LIMITSWITCH				{0x5A, 0xA5, 0x01, 0x3F, 0x20, 0xC4, 0x1C, 0x00, 0xBF, 0xFE}								//возврат срабатывания концевика

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
#define load_1_Pin GPIO_PIN_1
#define load_1_GPIO_Port GPIOF
#define led_debug_Pin GPIO_PIN_1
#define led_debug_GPIO_Port GPIOA
#define magnit_Pin GPIO_PIN_2
#define magnit_GPIO_Port GPIOA
#define switch_baud_Pin GPIO_PIN_3
#define switch_baud_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
