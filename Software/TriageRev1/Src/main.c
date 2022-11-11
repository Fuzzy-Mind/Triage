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
#include "masimo.h"
#include "math.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define AD7171_RESOLUTION 	32767.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

MasimoPacket masimoPacket;

extern uint8_t incommingByte[1];
extern uint8_t receiveMasimo[64];
extern uint8_t state;
extern uint16_t spo2;
extern uint16_t pulseRate;
extern uint16_t perfusionIndex;

uint8_t espOut[32];

uint8_t pattern=0;
int adcData = 0;
float normalizeData = 0;
int resistor = 0;

static const float REF_RESISTOR = 10000.0;
float A = 0.001214095268; //A
float B = 0.0002351453496; //B
float C = 0.00000009101744098;  //C
float	T = 0;
uint16_t tempUint =0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int adcRead() {
	int data = 0;
	
	pattern = 0;

	while(1){
		data = 0;

		if(!HAL_GPIO_ReadPin(ADC_DATA_GPIO_Port, ADC_DATA_Pin)) {
			for (int bit = 23; bit >= 0; bit--) {
				HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port, ADC_SCLK_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port, ADC_SCLK_Pin, GPIO_PIN_SET);

				if(HAL_GPIO_ReadPin(ADC_DATA_GPIO_Port, ADC_DATA_Pin))
					data |= (1 << bit);
			}
		}

		if(((data & 0x1F) == 0x0D) && ((data & 0x80) == 0))
			break;

		HAL_Delay(1);
	}

	pattern = (uint8_t) (data & 0xFF);

	return (data >> 8) & (0x7FFF);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_UART_Receive_IT(&huart2, incommingByte, 1);
	
	for(int i=0; i<3; i++){	
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
	}
	
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);

  /* USER CODE END 2 */
	initPacket();
	
	checkStatus();
	HAL_Delay(1000);
	
	if(state == BOARD_LOCKED){
		requestBoardInfo();
		HAL_Delay(1000);
		
		unlockBoard();
		HAL_Delay(1000);
	}
	
	if(state == 'M'){
		programmingAck();
		HAL_Delay(1000);
	}
	
	HAL_GPIO_WritePin(TEMP_SEL1_GPIO_Port, TEMP_SEL1_Pin, GPIO_PIN_RESET);	// Skin 1 secildi
	HAL_GPIO_WritePin(TEMP_SEL2_GPIO_Port, TEMP_SEL2_Pin, GPIO_PIN_RESET);	// Skin 1 secildi
	HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port, ADC_SCLK_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		requestSpo2();
		HAL_Delay(120);
		requestPulseRate();
		HAL_Delay(120);
		requestPerfusionIndex();
		HAL_Delay(120);
		
		adcData = adcRead();
		if(pattern == 0x0D){
			normalizeData = (float)adcData/AD7171_RESOLUTION;
			resistor = normalizeData*REF_RESISTOR;
			if(resistor<100){
				T=0;	// Arizali cilt
				tempUint = 0;
			}
			else{
				T = 1/(A + (B*log(resistor)) + (C*pow(log(resistor),3)));
				T = T-273.15;
				tempUint = T*100;
			}
		}
		else{
			T=1; // Cilt takili degil.
			tempUint = 1;
		}
		
		espOut[0] = 0xA4;
		espOut[1] = spo2>>8;
		espOut[2] = spo2;
		espOut[3] = pulseRate>>8;
		espOut[4] = pulseRate;
		espOut[5] = perfusionIndex>>8;
		espOut[6] = perfusionIndex;
		espOut[7] = tempUint>>8;
		espOut[8] = tempUint;
		espOut[9] = 0x4A;
		
		HAL_UART_Transmit(&huart1, espOut, 10, 100);
		HAL_Delay(1000);
		memset(espOut, 0, 32);
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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|TEMP_SEL1_Pin|TEMP_SEL2_Pin|ADC_SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin TEMP_SEL1_Pin TEMP_SEL2_Pin ADC_SCLK_Pin */
  GPIO_InitStruct.Pin = LED_Pin|TEMP_SEL1_Pin|TEMP_SEL2_Pin|ADC_SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ADC_DATA_Pin */
  GPIO_InitStruct.Pin = ADC_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_DATA_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
