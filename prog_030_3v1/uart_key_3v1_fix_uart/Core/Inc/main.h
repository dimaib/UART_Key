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

#define DELAY_SEND						300															//�������� ����� ��������� ������� �� ������� �� �������� ���������

#define MAX_SIZE							200

#define FLASH_SPEED_ON				300															//�������� ������� ����������, ���� ������� ��������
#define FLASH_SPEED_OFF				600															//�������� ������� ����������, ���� ������� ���������

#define OFF_CRC								1																//1-��������� 0-�������� �������� crc ��� ������ ����� UART

#define SPEED_FLASH						400															//�������� ������� ������������ ������

extern uint8_t lock;                                          //��������� ��������. ������������ ��� ���
extern uint8_t perecent_accum;                                //���������� ��� �������� �������� ������ ������������
extern uint8_t speed;                                         //���������� ��� �������� �������� ��������

extern uint8_t pwm_red;																				//�������� PWM ��� �������� �����
extern uint8_t pwm_green;																			//�������� PWM ��� �������� �����
extern uint8_t pwm_blue;																			//�������� PWM ��� ������ �����
extern uint8_t pwm_inc;																				//��������� PWM

extern uint32_t uart_baud;																		//���������� ��� ��������� �������� uart ��� ���������

extern uint8_t status;																				//���������� ��� �������� �������
extern uint8_t command;																				//������� ��� �������� � main

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
