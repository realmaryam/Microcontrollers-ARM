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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "defines.h"
#include "tm_stm32f4_gpio.h"
#include "tm_stm32f4_keypad.h"
#include "LCD16x2.h"
#include "LCD16x2_cfg.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint32_t t_open =1;
uint32_t counter= 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

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
	
	SystemInit();
	TM_KEYPAD_Init(TM_KEYPAD_Type_Large);
	LCD_Init();
  LCD_Clear();
	HAL_SYSTICK_Config(1);
  

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
	
	TM_KEYPAD_Button_t Keypad_Button;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (HAL_GPIO_ReadPin(GPIOB,GPIO_Pin_1)==GPIO_PIN_RESET)										// checking pin B1
		{
			LCD_Clear();
			LCD_Set_Cursor(1,1);
			LCD_Write_String("Moisture: Wet");
		}
		if (HAL_GPIO_ReadPin(GPIOB,GPIO_Pin_1)==GPIO_PIN_SET & Keypad_Button != TM_KEYPAD_Button_A) 	
		{																																					// moisture is dry and keypad A not pressed
			counter=0;
			t_open=1;
			while(counter < 60000  & Keypad_Button != TM_KEYPAD_Button_A)						//t_open is 1 so it is 60000 milisecounds
			{
				HAL_SYSTICK_Callback();																								// this intrrupt adds counter 1 each time
				HAL_GPIO_WritePin(GPIOB,GPIO_Pin_2,GPIO_PIN_SET);
				LCD_Clear();
				LCD_Set_Cursor(1,1);
				LCD_Write_String("Moisture: Dry");
				LCD_Set_Cursor(2,1);
				LCD_Write_String("Open Time: 1 Min");
			}
			HAL_GPIO_WritePin(GPIOB,GPIO_Pin_2,GPIO_PIN_RESET);											//end of open time or key A is pressed
		}
		if(Keypad_Button == TM_KEYPAD_Button_A)																		//key A is pressed
		{
			LCD_Clear();
			LCD_Set_Cursor(1,1);
			LCD_Write_String("Choose Time from");
			LCD_Set_Cursor(2,1);
			LCD_Write_String("{1,2,3} Min:");
			HAL_SYSTICK_Callback();
			while(Keypad_Button != TM_KEYPAD_Button_1 & Keypad_Button != TM_KEYPAD_Button_2 & Keypad_Button != TM_KEYPAD_Button_3 )
			{
				HAL_SYSTICK_Callback();																								//waiting for keys 1,2,3 to be pressed
			}																																				//one of them is pressed. change in lcd and t_open value
			LCD_Clear();
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_Pin_1)==GPIO_PIN_RESET)
			{
				LCD_Set_Cursor(1,1);
				LCD_Write_String("Moisture: Wet");
			}
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_Pin_1)==GPIO_PIN_SET)
			{
				LCD_Set_Cursor(1,1);
				LCD_Write_String("Moisture: dry");
			}
			LCD_Set_Cursor(2,1);
			if(Keypad_Button == TM_KEYPAD_Button_1)
			{
				t_open=1;
				LCD_Write_String("Open Time: 1 Min");
			}
			if(Keypad_Button == TM_KEYPAD_Button_2)
			{
				t_open=2;
				LCD_Write_String("Open Time: 2 Min");
			}
			if(Keypad_Button == TM_KEYPAD_Button_3)
			{
				t_open=3;
				LCD_Write_String("Open Time: 3 Min");
			}
			counter =0;																															//set counter to count to 60000 ms each 1 mins
			while(counter < 60000 * t_open)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_Pin_2,GPIO_PIN_SET);
				HAL_SYSTICK_Callback();
				if(counter>60000 & counter < 120000)
				{
					LCD_Set_Cursor(2,1);
					LCD_Write_String("Open Time: 2 Min");
				}			
				if(counter< 60000)
				{
					LCD_Set_Cursor(2,1);
					LCD_Write_String("Open Time: 1 Min");
				}
				if (Keypad_Button == TM_KEYPAD_Button_A)
					break;
			}
			HAL_GPIO_WritePin(GPIOB,GPIO_Pin_2,GPIO_PIN_RESET);				
		}
		
		 
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

}

void HAL_SYSTICK_Callback(void)
{
	TM_KEYPAD_Update();
	counter++ ;
	
}
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC2 PC3 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB3 PB4 PB5
                           PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
