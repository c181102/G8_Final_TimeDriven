/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "taskQueue.h"
#include "dht11.h"
#include "rgb.h"
#include "delay_timer.h"
#include "lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	DISPLAY_TEMP_HUMI,
	DISPLAY_TEMP,
	DISPLAY_HUMI,
} DisplayMode_t;

//typedef enum
//{
//	TASK_READ_DATA,
//	TASK_CONTROL_LED,
//	TASK_DISPLAY,
//	TASK_SEND_TO_COM,
//} TaskIndex_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_RX_LEN 13
#define RGB_COMMAND_LEN	12
#define DISPLAY_COMMAND_LEN 11
#define TIME_COMMAND_LEN 8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
RGB_t rgb;
DHT11_Sensor dht;
DHT11_Status dhtStatus;
TaskQueue_t queue;
LCD_I2C_Name lcd;

uint8_t red = 0, green = 0, blue = 0;
uint32_t P, t0;

uint8_t rxData[MAX_RX_LEN];
uint8_t rxDataIndex = 0;

DisplayMode_t DisplayMode = DISPLAY_TEMP_HUMI;
//TaskIndex_t TaskIndex;
pTaskFunction currentTask;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void vTask_HandleInterrupt();
void vTask_ReadData();
void vTask_ControlRgb();
void vTask_Display();
void vTask_SendToCom();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

pTaskFunction TaskArray[] = {vTask_ReadData, vTask_Display, vTask_SendToCom, vTask_ControlRgb, vTask_ControlRgb};
uint32_t TaskTime[] = {1000, 100, 100, 350, 350};

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

	return ch;
}

void vTask_HandleInterrupt()
{
	NVIC_DisableIRQ(USART1_IRQn);
	const uint8_t* numPart;
	uint8_t buffer[5];
	printf("\n");
	switch (strlen((const char*)rxData))
	{
		case RGB_COMMAND_LEN:
			if (rxData[0] == 'r' && rxData[1] == 'g' && rxData[2] == 'b')
			{
				numPart = rxData + 3;
				for (int i = 0; i < 3; i++)
				{
					strncpy(buffer, (const char*)(numPart + i * 3), 3);
					buffer[3] = '\0'; // Null-terminate the buffer

					switch (i)
					{
						case 0: red = (uint8_t)atoi((const char*)buffer); break;
						case 1: green = (uint8_t)atoi((const char*)buffer); break;
						case 2: blue = (uint8_t)atoi((const char*)buffer); break;
					}
				}
				printf("Change RGB color:\r\n");
				printf("RED = %d\r\nGREEN = %d\r\nBLUE = %d\r\n\n", red, green, blue);
			}
			else
			{
				printf("Error Command Syntax\r\n\n");
			}
			break;
		case DISPLAY_COMMAND_LEN:
			if (strcmp((const char*)rxData, "displaytemp") == 0)
			{
				DisplayMode = DISPLAY_TEMP;
				printf("Change Display Mode to DISPLAY_TEMP\r\n\n");
			}
			else if (strcmp((const char*)rxData, "displayhumi") == 0)
			{
				DisplayMode = DISPLAY_HUMI;
				printf("Change Display Mode to DISPLAY_HUMI\r\n\n");
			}
			else if (strcmp((const char*)rxData, "displayboth") == 0)
			{
				DisplayMode = DISPLAY_TEMP_HUMI;
				printf("Change Display Mode to DISPLAY_TEMP_HUMI\r\n\n");
			}
			else
			{
				printf("Error Command Syntax\r\n\n");
			}
			break;
		case TIME_COMMAND_LEN:
			if(rxData[0] == 't' && rxData[1] == 'i' && rxData[2] == 'm' && rxData[3] == 'e')
			{
				numPart = rxData + 4;
				strncpy(buffer, (const char*)numPart, 4);
				buffer[4] = '\0'; // Null-terminate the buffer
				TaskTime[0] = (uint32_t)atoi((const char*)buffer);
				printf("Change DHT11 Period to %d\r\n\n", TaskTime[0]);

			}
			else
			{
				printf("Error Command Syntax\r\n\n");
			}
			break;
		default:
			printf("Error Command Syntax\r\n\n");
			break;
	}

	NVIC_EnableIRQ(USART1_IRQn);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance)
	{
		if (rxData[rxDataIndex] == '@')
		{
			rxData[rxDataIndex] = '\0';
			Queue_PushFront(&queue, vTask_HandleInterrupt);
			rxDataIndex = 0;
		}
		else
		{
			rxDataIndex ++;
		}
	}
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxData[rxDataIndex], 1);
}

void vTask_ReadData()
{
	printf("vTask_ReadData IN: %ld\r\n", uwTick);

	NVIC_DisableIRQ(USART1_IRQn);
	dhtStatus = DHT11_GetData(&dht);
	NVIC_EnableIRQ(USART1_IRQn);

	switch(dhtStatus)
	{
		case DHT11_ERR_CHECKSUM:
			printf("DHT11 ERROR CHECKSUM\r\n");
			break;
		case DHT11_ERR_RESPONSE:
			printf("DHT11 ERROR RESPONSE\r\n");
			break;
		default:
			printf("Get Data from DHT11 successfully\r\n");
			break;
	}

	Queue_PushRear(&queue, vTask_ReadData);

	printf("vTask_ReadData OUT: %ld\r\n\n", uwTick);
}

void vTask_Display()
{
	char temp[18], humi[15];

	printf("vTask_Display IN: %ld\r\n", uwTick);

  	if (dhtStatus == DHT11_OK)
  	{
  	  	sprintf(temp, "Temperature: %.2f", dht.Temp);
  	  	sprintf(humi, "Humidity: %.2f", dht.Humi);

  	  	LCD_Clear(&lcd);

  	  	switch (DisplayMode)
  	  	{
  	  		case DISPLAY_TEMP:
  	  			LCD_SetCursor(&lcd, 0, 0);
  	  			LCD_WriteString(&lcd, temp);
  	  			break;
  	  		case DISPLAY_HUMI:
  	  			LCD_SetCursor(&lcd, 0, 0);
  	  			LCD_WriteString(&lcd, humi);
  	  			break;
  	  		default:
  	  			LCD_SetCursor(&lcd, 0, 0);
  	  			LCD_WriteString(&lcd, temp);
  	  			LCD_SetCursor(&lcd, 0, 1);
  	  			LCD_WriteString(&lcd, humi);
  	  			break;
  	  	}
  	}

  	Queue_PushRear(&queue, vTask_Display);
  	printf("vTask_Display OUT: %ld\r\n\n", uwTick);
}

void vTask_ControlRgb()
{
	printf("vTask_ControlRgb IN: %ld\r\n", uwTick);

	RGB_SetValue(&rgb, red++, green++, blue++);

	Queue_PushRear(&queue, vTask_ControlRgb);
	printf("vTask_ControlRgb OUT: %ld\r\n\n", uwTick);
}

void vTask_SendToCom()
{
	printf("vTask_SendToCom IN: %ld\r\n", uwTick);

	if (dhtStatus == DHT11_OK)
	{
		switch (DisplayMode)
		{
			case DISPLAY_HUMI:
				printf("Humidity: %.2f\r\n", dht.Humi);
				break;
			case DISPLAY_TEMP:
				printf("Temperature: %.2f\r\n", dht.Temp);
				break;
			case DISPLAY_TEMP_HUMI:
				printf("Temperature: %.2f\r\n", dht.Temp);
				printf("Humidity: %.2f\r\n", dht.Humi);
				break;
		}
	}

	Queue_PushRear(&queue, vTask_SendToCom);
	printf("vTask_SendToCom OUT: %ld\r\n\n", uwTick);
}
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  P = 1000;
  t0 = uwTick;
  uint32_t idx;

  printf("Start\r\n\n");

  DHT11_Init(&dht, DHT_GPIO_Port, DHT_Pin, &htim4);
  RGB_Init(&rgb, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3);
  Queue_Init(&queue, MAX_TASKS);
  LCD_Init(&lcd, &hi2c2, LDC_DEFAULT_ADDRESS, 20, 4);

  HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxData[rxDataIndex],  1);

  for (idx = 0; idx < sizeof(TaskArray)/sizeof(TaskArray[0]); idx ++)
  {
	  Queue_PushRear(&queue, TaskArray[idx]);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  for (idx = 0; idx < sizeof(TaskArray)/sizeof(TaskArray[0]); idx++)
	  {
		  if ((uwTick % TaskTime[idx] == t0 && queue.Task[queue.Front] == TaskArray[idx]))
		  {
			  currentTask = Queue_Pop(&queue);
			  currentTask();
		  }
		  else if (queue.Task[queue.Front] == vTask_HandleInterrupt)
		  {
			  currentTask = Queue_Pop(&queue);
			  currentTask();
			  break;
		  }
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 554-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT_Pin */
  GPIO_InitStruct.Pin = DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
