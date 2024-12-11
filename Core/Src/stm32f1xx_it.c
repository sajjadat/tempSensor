/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#define LCD_Width      128.0
#define LCD_High       64.0
#define MIN_TEMP       0.0
#define MAX_TEMP       50.0
#define BARR_Width_MAX  120.0 		 
#define BARR_High      15.0

extern float freq,dt,temp,tempslow;
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
void ShowBArr_temp(void);
void Show_temp(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
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
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(Button_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
	uint16_t period,pw;
    static uint8_t i=0 ;
	static float dt_arr[8];
	
	period=TIM1->CCR1;
	pw=TIM1->CCR2;
	
	freq=24000000.0/(float)period;
	dt=(float)pw/(float)period;
//	temp= -1.4*dt*dt+ 214.56*dt - 68.60; 
	// 8 temp averaging
  dt_arr[i]=dt;	
	i++;
	if(i>7)
	{
		temp = 0;
		for(uint8_t j=0;j<8;j++)
		{
			temp+=dt_arr[j];
		}
		temp /= 8;
		temp= -1.43*temp*temp+ 214.56*temp - 68.60;	
		i=0;			
  }
  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	static uint8_t k = 0;
	k++;
	if(k==10)
	{
		tempslow=temp;
		k=0;	
	}
	ssd1306_Fill(Black);
	ShowBArr_temp();
	Show_temp();
	ssd1306_UpdateScreen();	
	HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void ShowBArr_temp(void)
{
	char Buff[5];
	float TEMP;
	TEMP=temp;
	if(TEMP<=MIN_TEMP)
	{
	TEMP=MIN_TEMP;		
	}
	else if(TEMP>=MAX_TEMP)
	{
	TEMP=MAX_TEMP;		
	}
	
	uint8_t Rectangle_X1 = ((LCD_Width-BARR_Width_MAX)/2) ; 
	uint8_t Rectangle_Y1 = BARR_High ; 
	uint8_t Rectangle_X2 = Rectangle_X1+BARR_Width_MAX ; 
	uint8_t Rectangle_Y2 = 0; 
	
	ssd1306_FillRectangle((int)Rectangle_X1 ,Rectangle_Y1,(Rectangle_X1 +(uint8_t)((TEMP-MIN_TEMP)*(BARR_Width_MAX/(MAX_TEMP-MIN_TEMP)))),Rectangle_Y2,White);
	ssd1306_DrawRectangle(Rectangle_X1 ,Rectangle_Y1, Rectangle_X2 , Rectangle_Y2 , White);
	ssd1306_SetCursor(Rectangle_X1 ,(BARR_High + 2));
  snprintf(Buff, sizeof(Buff), "%i",(int)MIN_TEMP);
	ssd1306_WriteString(Buff,Font_7x10,White);
	ssd1306_SetCursor((BARR_Width_MAX + (LCD_Width-BARR_Width_MAX)/2)-10,(BARR_High + 2));
  snprintf(Buff, sizeof(Buff), "%i",(int)MAX_TEMP);
	ssd1306_WriteString(Buff,Font_7x10,White);
}


void Show_temp(void)
{
	char Buff[16];
	ssd1306_SetCursor(35,40);
	snprintf(Buff, sizeof(Buff), "%2.2f C",tempslow);
	ssd1306_WriteString(Buff,Font_11x18,White);
	ssd1306_DrawCircle(94,41,3,White);
}
/* USER CODE END 1 */
