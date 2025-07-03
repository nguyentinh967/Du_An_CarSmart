/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "string.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t diff = 0;
uint8_t distance = 0;
uint8_t distance_left = 0;
uint8_t distance_right = 0;
uint8_t is_capture;
uint8_t Val_S1 = 0, Val_S2 = 0, Val_S3 = 0, Val_S4 = 0, Val_S5 = 0;
uint8_t Tx_data[10] = "Hello Anh"; // mang truyen du lieu
uint8_t Rx_data[5]; // buffer luu chuoi nhan dc

void Delay_us(uint16_t us)
{
	htim1.Instance->CNT = 0;
	HAL_TIM_Base_Start(&htim1);
	while(htim1.Instance->CNT < us);
	HAL_TIM_Base_Stop(&htim1);
}
void Sr04_Trigger()
{
	HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
	Delay_us(10);
	HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Channel == HAL_TIM_ACTIVE_CHANNEL_1) //kiem tra k�nh g�y ra ngat
		{
			if(is_capture == 0)  // neu gi� tri dau ti�n kh�ng duoc nam bat 
				{
					IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // luu gi� tri thoi gian cua lan lay mau dau
					is_capture = 1;
					__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING); // B�y gio thay doi cuc th�nh canh roi 
				}
			else if(is_capture == 1) // neu gi� tri dau ti�n d� duoc chup
				{
					IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // luu gi� tri thoi gian cua lan 2
					__HAL_TIM_SET_COUNTER(htim,0); // reset the counter
					if(IC_Val2 > IC_Val1)
						{
							diff = IC_Val2 - IC_Val1; // Thoi gian 
						}
					else if(IC_Val2 < IC_Val1)
						{
							diff = 0xffff + IC_Val2 - IC_Val1;
						}
					distance = diff * (.034/2);
					is_capture = 0;
					__HAL_TIM_SET_CAPTUREPOLARITY(htim,TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING); // dat cuc t�nh th�nh canh l�n 
					__HAL_TIM_DISABLE_IT(&htim1,TIM_IT_CC1);
				}	
		}
}
void Reset_Servo()
{
	htim2.Instance->CCR1 = 75;
}
void Servo_Left()
{
	htim2.Instance->CCR1 = 25;
	HAL_Delay(500);
	Sr04_Trigger();
	Reset_Servo();
}
void Servo_Right()
{
	htim2.Instance->CCR1 = 125;
	HAL_Delay(500);
	Sr04_Trigger();
	Reset_Servo();
}
void Forward()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 99);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 99);
	HAL_GPIO_WritePin(In1_GPIO_Port,In1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(In2_GPIO_Port,In2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(In3_GPIO_Port,In3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(In4_GPIO_Port,In4_Pin,GPIO_PIN_RESET);
}
void Back()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 99);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 99);
	HAL_GPIO_WritePin(In1_GPIO_Port,In1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(In2_GPIO_Port,In2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(In3_GPIO_Port,In3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(In4_GPIO_Port,In4_Pin,GPIO_PIN_SET);
}
void Right()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 50);
	HAL_GPIO_WritePin(In1_GPIO_Port,In1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(In2_GPIO_Port,In2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(In3_GPIO_Port,In3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(In4_GPIO_Port,In4_Pin,GPIO_PIN_SET);
}
void Left()
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 50);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 100);
	HAL_GPIO_WritePin(In1_GPIO_Port,In1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(In2_GPIO_Port,In2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(In3_GPIO_Port,In3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(In4_GPIO_Port,In4_Pin,GPIO_PIN_RESET);
}
void Stop()
{
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
}
void Mode_AUTO()
{
	Sr04_Trigger();
	HAL_Delay(100);
	if(distance > 50)
	{
		Forward();
	}
	else
	{
		Stop();
		Servo_Left();
		distance_left = distance;
		Servo_Right();
		distance_right = distance;
		if(distance_left < 30 && distance_right < 30)
		{
			Back();
			HAL_Delay(300);
		}
		else
			{
				if(distance_left < distance_right)
				{
					Right();
					HAL_Delay(300);
				}
				if(distance_left > distance_right)
				{
					Left();
					HAL_Delay(300);
				}
			}
	}
}
void Mode_Line()
{
	Val_S1 = HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin);
	Val_S2 = HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin);
	Val_S3 = HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin);
	Val_S4 = HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin);
	Val_S5 = HAL_GPIO_ReadPin(S5_GPIO_Port, S5_Pin);
	if(Val_S2 == 1 && Val_S3 == 0 && Val_S4 == 1){
		Forward();
	}
	if(Val_S2 == 0 && Val_S3 == 0 && Val_S4 == 1){
		Left();
	}
	if(Val_S2 == 0 && Val_S3 == 1 && Val_S4 == 1){
		Left();
	}
	if(Val_S2 == 1 && Val_S3 == 0 && Val_S4 == 0){
		Right();
	}
	if(Val_S2 == 1 && Val_S3 == 1 && Val_S4 == 0){
		Right();
	}
	if(Val_S2 == 0 && Val_S3 == 0 && Val_S4 == 0){
		Stop();
	}
}
void Mode_Bluetooth()
{
	//HAL_UART_Transmit(&huart1, Tx_data, sizeof(Tx_data), 1000);
	HAL_UART_Receive_IT(&huart1, Rx_data, 5);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Mode_Bluetooth();
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
// abcdefghjklm
  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 1440-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 14400-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart1.Init.BaudRate = 9600;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, In1_Pin|In2_Pin|In3_Pin|In4_Pin
                          |Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : S3_Pin S2_Pin S1_Pin */
  GPIO_InitStruct.Pin = S3_Pin|S2_Pin|S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : In1_Pin In2_Pin In3_Pin In4_Pin
                           Trigger_Pin */
  GPIO_InitStruct.Pin = In1_Pin|In2_Pin|In3_Pin|In4_Pin
                          |Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : S4_Pin */
  GPIO_InitStruct.Pin = S4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(S4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S5_Pin Near_Line_Pin */
  GPIO_InitStruct.Pin = S5_Pin|Near_Line_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLB_Line_Pin */
  GPIO_InitStruct.Pin = CLB_Line_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(CLB_Line_GPIO_Port, &GPIO_InitStruct);

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
