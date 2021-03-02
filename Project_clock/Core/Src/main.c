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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	GPIO_TypeDef * ports[12];
	uint16_t pos[12];
}PORTS;
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
volatile uint8_t buffer = 0;
volatile uint8_t milis = 0;
volatile uint8_t counterM = 1;
volatile uint16_t counterS = 0;
volatile uint16_t counterH = 12;
volatile uint16_t service = 0;
volatile uint32_t globalTime = 0;
uint8_t ha = 12,ma = 2;
int i,j;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int pulseTime();
void setLeds(PORTS *PORTS);
void timeSet(int number);
void runMode(PORTS *ports);
void setMode(PORTS *ports);
void alarmMode(PORTS *ports);
void alarm(PORTS *ports);
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
			globalTime++;
			milis = 0;
		}
		if(counterS == 300){
			counterM++;
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
	PORTS ports;
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
  setLeds(&ports);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch((int)pulseTime()){
	  case 0:
		  runMode(&ports);
		  break;
	  case 1:
		  setMode(&ports);
	  	  break;
	  case 2:
		  alarmMode(&ports);
	 	  break;
 	  default:
 		  break;
	  	  }
	  if((ha == counterH) && (ma == counterM))
		  alarm(&ports);
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
void setLeds(PORTS *PORTS){
	PORTS->ports[0] = LED1_GPIO_Port;
	PORTS->ports[1] = LED2_GPIO_Port;
	PORTS->ports[2] = LED3_GPIO_Port;
	PORTS->ports[3] = LED4_GPIO_Port;
	PORTS->ports[4] = LED5_GPIO_Port;
	PORTS->ports[5] = LED6_GPIO_Port;
	PORTS->ports[6] = LED7_GPIO_Port;
	PORTS->ports[7] = LED8_GPIO_Port;
	PORTS->ports[8] = LED9_GPIO_Port;
	PORTS->ports[9] = LED10_GPIO_Port;
	PORTS->ports[10]= LED11_GPIO_Port;
	PORTS->ports[11]= LED12_GPIO_Port;
	PORTS->pos[0] = LED1_Pin;
	PORTS->pos[1] = LED2_Pin;
	PORTS->pos[2] = LED3_Pin;
	PORTS->pos[3] = LED4_Pin;
	PORTS->pos[4] = LED5_Pin;
	PORTS->pos[5] = LED6_Pin;
	PORTS->pos[6] = LED7_Pin;
	PORTS->pos[7] = LED8_Pin;
	PORTS->pos[8] = LED9_Pin;
	PORTS->pos[9] = LED10_Pin;
	PORTS->pos[10]= LED11_Pin;
	PORTS->pos[11]= LED12_Pin;
}
void runMode(PORTS *ports){
	uint32_t time = 0;
	service = 0;
	do{
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	}while(service < 20);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	if((HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin))==1)
		time++;
	while(time==1){
		service = 0;
		do{
			HAL_GPIO_WritePin(ports->ports[counterH-1], ports->pos[counterH-1], GPIO_PIN_SET);
			HAL_GPIO_WritePin(ports->ports[counterM-1], ports->pos[counterM-1], GPIO_PIN_SET);
			timeSet(2);
			HAL_GPIO_WritePin(ports->ports[counterM-1], ports->pos[counterM-1], GPIO_PIN_RESET);
			timeSet(2);
		}while(service < 16);
		HAL_GPIO_WritePin(ports->ports[counterH-1], ports->pos[counterH-1], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ports->ports[counterM-1], ports->pos[counterM-1], GPIO_PIN_RESET);
		if(service>1)
			time++;
	}
}
void timeSet(int number){
	for(buffer=0;buffer<=number;){
	}
}
int pulseTime(){
	uint32_t time = 0;
	globalTime = 0;
	if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 1){
		time=globalTime;
		while((HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin))==1)
			timeSet(10);
		if((globalTime-time) >= 10)
			return 1;
		else if (((globalTime-time)>5) && ((globalTime-time)<10))
			return 2;
		return 0;
	}
	return 0;

}
void setMode(PORTS *ports){
	for(i=0;i>12;i++){
		 HAL_GPIO_WritePin(ports->ports[i], ports->pos[i], GPIO_PIN_SET);
	}
	for(i=0;i>12;i++){
		 HAL_GPIO_WritePin(ports->ports[i], ports->pos[i], GPIO_PIN_RESET);
	}
}
void alarmMode(PORTS *ports){
	for(i=13;i<1;i--){
		 HAL_GPIO_WritePin(ports->ports[i-1], ports->pos[i-1], GPIO_PIN_SET);
	}
	for(i=0;i>12;i++){
		 HAL_GPIO_WritePin(ports->ports[i], ports->pos[i], GPIO_PIN_RESET);
	}
}
void alarm(PORTS *ports){
	while(!HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)){
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		for(i=0;i<12;i++)
			 HAL_GPIO_WritePin(ports->ports[i], ports->pos[i], GPIO_PIN_SET);
		for(i=0;i<12;i++)
			HAL_GPIO_WritePin(ports->ports[i], ports->pos[i], GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}
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
