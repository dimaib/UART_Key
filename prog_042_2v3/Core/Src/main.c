/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE BEGIN 0 */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t status=0;																																										//переменная для хранения статуса
uint8_t command=0;																																									//команды для передачи в main
uint32_t uart_baud=115200;																																					//скорость UART

uint16_t del_val=600;

extern uint8_t msg_on_1[];																																					//массив команды 1
extern uint8_t msg_on_2[];																																					//массив команды 2


void command_shell()																																									//функция для обработки команд в переменной command
{
	if(command){
		if(command&(1<<0)){																																								//команда открыть закрыть
			command&=~(1<<0);
			HAL_Delay(DELAY_SEND);
			status^=(1<<0);
			GPIOA->ODR&=~(1<<2);															//выключить магнит
			GPIOF->ODR&=~(1<<1);															//выключить зарядку
			del_val=FLASH_SPEED_OFF;
		}
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(300);
	//после пересборки проекта в кубе, требуется в инициализацию юарта вставить переменную uart_baud, вместо скорости
	if(!(GPIOA->IDR&(1<<3))) {																																					//если перемычка запаянна, то скорость 9600 иначе 115200. 
		//uart_baud=9600;
		//MX_USART1_UART_Init();
	}
	USART1->CR1 = USART_CR1_UE;
	NVIC_EnableIRQ(USART1_IRQn); USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;        //разрешение прерывания по приему данных UART1
  
	GPIOA->ODR&=~(1<<2);
	GPIOF->ODR&=~(1<<1);
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t del_toggle=0;
	uint8_t flag_zu=0;
	uint16_t delay_repeat_zu=0;														//для предотвращения повторной зарядки
	
	while (1)
  {
		command_shell();																		//обработка команд
		
		del_toggle++;																				//сквозная задержка для мигания отладочным светодиодом
		if(del_toggle>del_val){															//задержка на 600мс
			del_toggle=0;
			GPIOA->ODR^=(1<<1);																//мигаем
			HAL_Delay(10);																		//мигаем
			GPIOA->ODR^=(1<<1);																//мигаем
		}
		HAL_Delay(1);
		
		//сквозная задержка на повторное включение
		if(delay_repeat_zu) {
			if(!(GPIOA->IDR&(1<<0))) delay_repeat_zu=0;				//если пришло питание когда считает данный счетчик, то сбрасываем его и начинаем счет заного
			delay_repeat_zu++;																//сквозной счетчик для предотвращения повторного включения
		}
		if(delay_repeat_zu>=30000) delay_repeat_zu=0;				//если счетчик досчитал до 10с, то выключаем счетчик
		//сквозная задержка на повторное включение
		
		if(!(GPIOA->IDR&(1<<0))&&del_val>=FLASH_SPEED_OFF&&!flag_zu&&!delay_repeat_zu){ //если пришло питание от зарядки
			HAL_Delay(1000);
			if(!(GPIOA->IDR&(1<<0))){
				GPIOA->ODR|=(1<<2);															//включить магнит
				GPIOF->ODR|=(1<<1);															//включить зарядку
				del_val=FLASH_SPEED_ON;
				flag_zu=1;
			}
		}else if((GPIOA->IDR&(1<<0))&&flag_zu){							//предотвращение повторной зарядки, если команда пришла, а самокат не сняли
			HAL_Delay(1000);
			if(GPIOA->IDR&(1<<0)){
				flag_zu=0;
				delay_repeat_zu++;
			}
		}
		
		//if(GPIOA->IDR&(1<<0)) GPIOA->ODR|=(1<<1); else GPIOA->ODR&=~(1<<1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
