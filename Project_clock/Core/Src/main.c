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
  * Created by Miguel A. Murillo
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint16_t pin[12];
	GPIO_TypeDef * PIO[12];
}GP;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
volatile uint8_t buffer = 0, L = 0;
volatile uint8_t milis = 0;
volatile uint8_t counterM = 1;
volatile uint16_t counterS = 0;
volatile uint16_t counterH = 12;
volatile uint16_t service = 0;
volatile uint32_t reference = 0;
uint8_t ha = 12, ma = 3;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint16_t pulse();
void setLeds(GP *GP);
void timeSet(uint16_t number);
void runMode(GP *PIO);
void setMode(GP *PIO);
void alarmMode(GP *PIO);
void alarm(GP *PIO);
void change(GP *PIO, uint8_t flag);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		buffer++;
		milis++;
		service++;
		if(milis == 10){
			counterS++;
			reference++;
			milis = 0;
		}
		if(counterS == 300){
			counterM++;
			L=0;
			counterS = 0;
		}
		if(counterM == 13){
			counterH++;
			counterM = 1;
		}
		if(counterH == 13)
			counterH = 1;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	GP PIO;
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  setLeds(&PIO);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch((uint16_t)pulse()){
	  case 0:
		  runMode(&PIO);
		  break;
	  case 1:
		  setMode(&PIO);
	  	  break;
	  case 2:
		  alarmMode(&PIO);
	 	  break;
 	  default:
 		  break;
	  	  }
	  if((ha == counterH) && (ma == counterM) && (L == 0))
		  alarm(&PIO);
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|LED1_Pin|LED2_Pin|LED3_Pin
                          |LED4_Pin|LED9_Pin|LED10_Pin|LED11_Pin
                          |LED12_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED5_Pin|LED6_Pin|LED7_Pin|LED8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin
                           LED9_Pin LED10_Pin LED11_Pin LED12_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED9_Pin|LED10_Pin|LED11_Pin|LED12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED5_Pin LED6_Pin LED7_Pin LED8_Pin */
  GPIO_InitStruct.Pin = LED5_Pin|LED6_Pin|LED7_Pin|LED8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void setLeds(GP *GP){
	GP->PIO[0] = LED1_GPIO_Port;
	GP->PIO[1] = LED2_GPIO_Port;
	GP->PIO[2] = LED3_GPIO_Port;
	GP->PIO[3] = LED4_GPIO_Port;
	GP->PIO[4] = LED5_GPIO_Port;
	GP->PIO[5] = LED6_GPIO_Port;
	GP->PIO[6] = LED7_GPIO_Port;
	GP->PIO[7] = LED8_GPIO_Port;
	GP->PIO[8] = LED9_GPIO_Port;
	GP->PIO[9] = LED10_GPIO_Port;
	GP->PIO[10]= LED11_GPIO_Port;
	GP->PIO[11]= LED12_GPIO_Port;
	GP->pin[0] = LED1_Pin;
	GP->pin[1] = LED2_Pin;
	GP->pin[2] = LED3_Pin;
	GP->pin[3] = LED4_Pin;
	GP->pin[4] = LED5_Pin;
	GP->pin[5] = LED6_Pin;
	GP->pin[6] = LED7_Pin;
	GP->pin[7] = LED8_Pin;
	GP->pin[8] = LED9_Pin;
	GP->pin[9] = LED10_Pin;
	GP->pin[10]= LED11_Pin;
	GP->pin[11]= LED12_Pin;
}
void runMode(GP *PIO){
	service = 0;
	do{
		HAL_GPIO_WritePin(PIO->PIO[counterH-1], PIO->pin[counterH-1], GPIO_PIN_SET);
		HAL_GPIO_WritePin(PIO->PIO[counterM-1], PIO->pin[counterM-1], GPIO_PIN_SET);
		timeSet(2);
		HAL_GPIO_WritePin(PIO->PIO[counterM-1], PIO->pin[counterM-1], GPIO_PIN_RESET);
		timeSet(2);
	}while(service < 16);
	HAL_GPIO_WritePin(PIO->PIO[counterH-1], PIO->pin[counterH-1], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(PIO->PIO[counterM-1], PIO->pin[counterM-1], GPIO_PIN_RESET);
}
void timeSet(uint16_t number){
	for(buffer=0;buffer<=number;){
	}
}
uint16_t pulse(){
	uint32_t time = 0;
	reference = 0;
	if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 1){
		time=reference;
		while((HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin))==1)
			timeSet(10);
		if((reference-time) <= 2 )
			return 0;
		else if((reference-time) >= 10)
			return 1;
		else if (((reference-time)>=6) && ((reference-time)<=7))
			return 2;
		return 3;
	}
	return 3;
}
void setMode(GP *PIO){
	service = 0;
	do{
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		timeSet(3);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		timeSet(3);
	}while(service < 15);
	change(PIO,1);
}
void change(GP *PIO, uint8_t flag){
	uint8_t select = 0, j = 0;
	if(flag == 1){
		while(!j){
			HAL_GPIO_WritePin(PIO->PIO[counterH-1], PIO->pin[counterH-1], GPIO_PIN_SET);
			select = pulse();
			if(select == 0){
				HAL_GPIO_WritePin(PIO->PIO[counterH-1], PIO->pin[counterH-1], GPIO_PIN_RESET);
				HAL_GPIO_WritePin(PIO->PIO[counterH], PIO->pin[counterH], GPIO_PIN_SET);
				counterH++;
			}else if (select == 2){
				j++;
				HAL_GPIO_WritePin(PIO->PIO[counterH-1], PIO->pin[counterH-1], GPIO_PIN_RESET);
			}
			HAL_GPIO_WritePin(PIO->PIO[counterH-1], PIO->pin[counterH-1], GPIO_PIN_RESET);
		}
		j=0;
		service = 0;
		do{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			timeSet(3);
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			timeSet(3);
		}while(service < 20);
		while(!j){
			HAL_GPIO_WritePin(PIO->PIO[counterM-1], PIO->pin[counterM-1], GPIO_PIN_SET);
			select = pulse();
			if(select == 0){
				HAL_GPIO_WritePin(PIO->PIO[counterM-1], PIO->pin[counterM-1], GPIO_PIN_RESET);
				HAL_GPIO_WritePin(PIO->PIO[counterM], PIO->pin[counterM], GPIO_PIN_SET);
				counterM++;
			}else if(select == 2){
				j++;
				service = 0;
				HAL_GPIO_WritePin(PIO->PIO[counterM-1], PIO->pin[counterM-1], GPIO_PIN_RESET);
				do{
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
				}while(service < 20);
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			}
			HAL_GPIO_WritePin(PIO->PIO[counterM-1], PIO->pin[counterM-1], GPIO_PIN_RESET);
		}
	}else{
		j=0;
		while(!j){
			if(ha == 13)
				ha = 1;
			HAL_GPIO_WritePin(PIO->PIO[ha-1], PIO->pin[ha-1], GPIO_PIN_SET);
			select = pulse();
			if(select == 0){
				HAL_GPIO_WritePin(PIO->PIO[ha-1], PIO->pin[ha-1], GPIO_PIN_RESET);
				HAL_GPIO_WritePin(PIO->PIO[ha], PIO->pin[ha], GPIO_PIN_SET);
				ha++;
			}else if (select == 2){
				j++;
				HAL_GPIO_WritePin(PIO->PIO[ha-1], PIO->pin[ha-1], GPIO_PIN_RESET);
			}
			HAL_GPIO_WritePin(PIO->PIO[ha-1], PIO->pin[ha-1], GPIO_PIN_RESET);
		}
		j=0;
		service = 0;
		do{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			timeSet(3);
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			timeSet(3);
		}while(service < 20);
		while(!j){
			if(ma == 13)
				ma = 1;
			HAL_GPIO_WritePin(PIO->PIO[ma-1], PIO->pin[ma-1], GPIO_PIN_SET);
			select = pulse();
			if(select == 0){
				HAL_GPIO_WritePin(PIO->PIO[ma-1], PIO->pin[ma-1], GPIO_PIN_RESET);
				HAL_GPIO_WritePin(PIO->PIO[ma], PIO->pin[ma], GPIO_PIN_SET);
				ma++;
			}else if(select == 2){
				j++;
				service = 0;
				HAL_GPIO_WritePin(PIO->PIO[ma-1], PIO->pin[ma-1], GPIO_PIN_RESET);
				do{
					HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
				}while(service < 20);
				HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			}
			HAL_GPIO_WritePin(PIO->PIO[ma-1], PIO->pin[ma-1], GPIO_PIN_RESET);
		}
	}
}
void alarmMode(GP *PIO){
	service = 0;
	do{
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		timeSet(3);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		timeSet(3);
	}while(service < 20);
	change(PIO, 2);
}
void alarm(GP *PIO){
	while(!HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)){
		service = 0;
		//do{
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			timeSet(1);
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			timeSet(1);
		//}while(service < 20);
	}
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	L++;
}
/*		for(i=0;i<12;i++)
	 HAL_GPIO_WritePin(ports->ports[i], ports->pos[i], GPIO_PIN_SET);
for(i=0;i<12;i++)
	HAL_GPIO_WritePin(ports->ports[i], ports->pos[i], GPIO_PIN_RESET);*/
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
