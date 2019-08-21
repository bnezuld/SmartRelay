/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f1xx_it.h"
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
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

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
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
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
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	debouncer = 0;
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	HAL_TIM_Base_Stop_IT(&htim4);

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if(debouncer == 0)
	{
		debouncer = 1;

		int read_10 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);
		int read_11 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);
		int read_12 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);
		int read_15 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

		if(read_10)
		{
			timerState++;
			if(timerState > type)
			{
				timerState = tic;
				currectTimerState++;
				if(currectTimerState > 1)
				{
					currectTimerState = 0;
					currentTimer++;
					if(currentTimer >= timerCount)
					{
						currentTimer = 0;
					}
				}
			}
		}
		if(read_11)
			{
				timerState--;
				if(timerState < tic)
				{
					timerState = tic;
					currectTimerState++;
					if(currectTimerState > 1)
					{
						currectTimerState = 0;
						currentTimer++;
						if(currentTimer >= timerCount)
						{
							currentTimer = 0;
						}
					}
				}
			}
		if(read_12)
		{
			if(timerState == tic)
			{
				timersTic[currectTimerState][currentTimer]++;
			}else if(timerState == type)
			{
				timersType[currectTimerState][currentTimer]++;
			}
		}

		if(read_15)
		{
			if(timerState == tic)
			{
				timersTic[currectTimerState][currentTimer]--;
			}else if(timerState == type)
			{
				timersType[currectTimerState][currentTimer]--;
			}
		}

		if(read_15 || read_12 || read_11 || read_10)
		{
			displayChange = true;
		}

		TIM4->ARR = GetDesiredPeriod(.2,TIM4->PSC);

	}

	HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_3);
  int read = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3);
  if(read == 1)//timer set when on(relay off)
  {
	  float offset = 1;
	  /*switch(timersType[0][0]) {
	  	  	  case centi:
	  	  		offset = .01;
	  	  		TIM7->PSC = 100;
	  	  		  break;
	  	  	  case seconds:
	  	  		offset = 1;
	  	  		TIM7->PSC = 1000;
	  	  		  break;
	  	  	  case minutes:
	  	  		TIM7->PSC = 10000;
	  	  		offset = 60;
	  	  		  break;
	  	  	  case hours:
	  	  		TIM7->PSC = 65535;
	  	  		offset = 60 * 60;
	  	  		  break;
	  	  }*/
	  if(timersType[0][0] == centi)
	  {
		  offset = .01;
		  TIM7->PSC = 10000;
	  }else if(timersType[0][0] == minutes)
	  {
		  offset = 60;
		  TIM7->PSC = 10000;
	  }else if(timersType[0][0] == hours)
	  {
		  offset = 60 * 60;
		  TIM7->PSC = 65535;
	  }else
	  {
		  offset = 1;
		  TIM7->PSC = 10000;
	  }
	  TIM7->ARR = GetDesiredPeriod(timersTic[0][0] * offset,TIM7->PSC);
	  TIM7->EGR = TIM_EGR_UG;//seems to only be needed when changing prescaler(PSC)
  }else//timer set for off(relay on)
  {
	  float offset = 1;
	  if(timersType[1][0] == centi)
	  	  {
	  		  offset = .01;
	  		  TIM7->PSC = 10000;
	  	  }else if(timersType[1][0] == minutes)
	  	  {
	  		  offset = 60;
	  		  TIM7->PSC = 10000;
	  	  }else if(timersType[1][0] == hours)
	  	  {
	  		  offset = 60 * 60;
	  		  TIM7->PSC = 65535;
	  	  }else
	  	  {
	  		  offset = 1;
	  		  TIM7->PSC = 10000;
	  	  }
	  TIM7->ARR = GetDesiredPeriod(timersTic[1][0] * offset,TIM7->PSC);
	  TIM7->EGR = TIM_EGR_UG;//seems to only be needed when changing prescaler(PSC)
  }
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
