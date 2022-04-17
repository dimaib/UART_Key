/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//массивы команд и ответов
uint8_t msg_open[]=MSG_OPEN;
uint8_t msg_open_tx[]=MSG_OPEN_TX;
uint8_t msg_status[]=MSG_STATUS;
uint8_t msg_status_close[]=MSG_STATUS_CLOUSE;
uint8_t msg_status_open[]=MSG_STATUS_OPEN;
uint8_t msg_limitswitch[]=MSG_LIMITSWITCH;
//массивы команд и ответов

uint8_t rx_item=0;																					//номер принятого символа
uint8_t buf_rx[PACKET_SIZE]={0};														//буфер для приема
extern uint8_t status;																			//переменная для хранения статуса
extern uint8_t command;																			//команды для передачи в main

uint8_t comparison(uint8_t *arr1,uint8_t *arr2)												//проверка равенства массивов
{
    for(uint8_t i=0;i<PACKET_SIZE;i++) if(arr1[i]!=arr2[i]) return 0;
    return 1;
}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	/*Прием данных*/
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	if(USART1->ISR & USART_ISR_RXNE){ 																								//Если прерывание вызвано по приему USART2
		uint8_t dat_rx=USART1->RDR;																											//считать принятый символ
		if(dat_rx==PACKET_START){																												//если пришел стартовый байт
			CLEAR_BUF;
			buf_rx[0]=PACKET_START;
			rx_item=1;
		}else if(dat_rx==PACKET_END&&rx_item==PACKET_SIZE-1){														//если пришел завершающий байт и кол-во принятых байт равно размеру пакета. Значит, что прием завершен, пора сравнивать 
			buf_rx[rx_item]=dat_rx;
			if(comparison(msg_open,buf_rx)){																							//если пришла команда на открытие
				command|=(1<<0);																														//команда на открытие закрытие замка
			}else if(comparison(msg_status,buf_rx)){																			//если пришла команда на запрос статуса
				command|=(1<<1);																														//команда на запрос статуса
			}
			CLEAR_BUF;
			rx_item=0;
		}else if(dat_rx==PACKET_END&&rx_item<PACKET_SIZE-1){														//если завершающий байт пришел досрочно
			CLEAR_BUF;
			rx_item=0;
		}else if(rx_item>PACKET_SIZE-1){																								//если приняли больше байт чем надо
			CLEAR_BUF;
			rx_item=0;
		}else if(rx_item){																															//принимает последующий байт посылки
			buf_rx[rx_item]=dat_rx;
			rx_item++;
		}
	}
  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
