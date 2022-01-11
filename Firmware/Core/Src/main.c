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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "pid_controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AVERAGE_LENGTH 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint32_t currentMillis;
uint32_t previousMillis = 0;
uint32_t PIDInterval = 20;

extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t dataRxToggle;
uint8_t userSerialRXBuffer[APP_RX_DATA_SIZE];
uint8_t dataRxFlag = 0;

uint16_t ADC_Value[4] = { 0 };

uint16_t setPoint1MovingAverage[AVERAGE_LENGTH] = { 0 };
uint16_t feedback1MovingAverage[AVERAGE_LENGTH] = { 0 };
uint16_t setPoint2MovingAverage[AVERAGE_LENGTH] = { 0 };
uint16_t feedback2MovingAverage[AVERAGE_LENGTH] = { 0 };
uint8_t setPoint1Pos = 0;
uint8_t setPoint2Pos = 0;
uint8_t feedback1Pos = 0;
uint8_t feedback2Pos = 0;
uint32_t setPoint1Sum = 0;
uint32_t setPoint2Sum = 0;
uint32_t feedback1Sum = 0;
uint32_t feedback2Sum = 0;
uint16_t setPoint1Avg = 0;
uint16_t setPoint2Avg = 0;
uint16_t feedback1Avg = 0;
uint16_t feedback2Avg = 0;
int setPoint1Map = 0;
int setPoint2Map = 0;
int feedback1Map = 0;
int feedback2Map = 0;

float kP1 = 0.001;
float kI1 = 0;
float kD1 = 0;

float kP2 = 0.001;
float kI2 = 0;
float kD2 = 0;

float percentage = 0;
uint8_t displayToggle = 0;

float PID1_out;
float PID2_out;

PIDControl PID1;
PIDControl PID2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
uint16_t user_get_adc1(void);
uint16_t user_get_adc2(void);
uint16_t movingAvg(uint16_t *ptrArrNumbers, uint32_t *ptrSum, uint8_t *pos,
		uint8_t len, uint16_t nextNum);
int user_MAP(int x, int in_min, int in_max, int out_min, int out_max);
void processRx(PIDControl *pid1, PIDControl *pid2);
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
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	PIDInit(&PID1, kP1, kI1, kD1, 0.020, -8000, 8000, AUTOMATIC, DIRECT);
	PIDInit(&PID2, kP2, kI2, kD2, 0.020, -8000, 8000, AUTOMATIC, DIRECT);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		currentMillis = HAL_GetTick();
		if (currentMillis - previousMillis >= PIDInterval) { //start timed event
			previousMillis = currentMillis;

			// Get ADC value
			for (uint8_t i = 0; i < 2; i++) {
				//Store ADC values of channel 1, 2 and 3 respectively
				ADC_Value[i] = user_get_adc1();
			}
			for (uint8_t i = 2; i < 4; i++) {
				//Store ADC values of channel 1, 2 and 3 respectively
				ADC_Value[i] = user_get_adc2();
			}

			setPoint1Map = user_MAP(movingAvg(setPoint1MovingAverage, &setPoint1Sum,
					&setPoint1Pos, AVERAGE_LENGTH, ADC_Value[0]), 0, 2800, -7000, 7000);
			feedback1Map = user_MAP(movingAvg(feedback1MovingAverage, &feedback1Sum,
								&feedback1Pos, AVERAGE_LENGTH, ADC_Value[1]), 700, 1900, -8000, 8000);


			setPoint2Map = user_MAP(movingAvg(setPoint2MovingAverage, &setPoint2Sum,
								&setPoint2Pos, AVERAGE_LENGTH, ADC_Value[2]), 0, 2800, -7000, 7000);
			feedback2Map = user_MAP(movingAvg(feedback2MovingAverage, &feedback2Sum,
								&feedback2Pos, AVERAGE_LENGTH, ADC_Value[3]), 700, 1900, -8000, 8000);

			//setPoint2Map = setPoint1Map;

			PIDSetpointSet(&PID1, setPoint1Map);
			PIDInputSet(&PID1, feedback1Map);

			PIDSetpointSet(&PID2, setPoint2Map);
			PIDInputSet(&PID2, feedback2Map);



			if(!HAL_GPIO_ReadPin(MASTER_EN_GPIO_Port, MASTER_EN_Pin)){

				PIDModeSet(&PID1, AUTOMATIC);
				PIDModeSet(&PID2, AUTOMATIC);

				PIDCompute(&PID1);
				PIDCompute(&PID2);

				PID1_out = (PIDOutputGet(&PID1) * percentage);
				PID2_out = (PIDOutputGet(&PID2) * percentage);

				HAL_GPIO_WritePin (GPIOB, EN2_Pin, 1);
				HAL_GPIO_WritePin (GPIOB, EN1_Pin, 1);
			}else {

				PIDModeSet(&PID1, MANUAL);
				PIDModeSet(&PID2, MANUAL);

				PIDCompute(&PID1);
				PIDCompute(&PID2);

				PID1_out = 0;
				PID2_out = 0;

				HAL_GPIO_WritePin (GPIOB, EN2_Pin, 0);
				HAL_GPIO_WritePin (GPIOB, EN1_Pin, 0);
			}

			if (PID1_out >= 0) {
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PID1_out);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
			      }
			else if (PID1_out < 0) {
			        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
			        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, abs(PID1_out));
		    }
			if (PID2_out >= 0) {
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PID2_out);
				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
			      }
			else if (PID2_out < 0) {
			        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
			        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, abs(PID2_out));
		    }


			char buffer[40];
			if(displayToggle == 1){
				sprintf(buffer, "%d , %d , %d , %d\r\n", setPoint2Map, feedback2Map, (int)PID2_out, (displayToggle<<7));
			}else{
				sprintf(buffer, "%d , %d , %d , %d\r\n", setPoint1Map, feedback1Map, (int)PID1_out, displayToggle);
			}
			CDC_Transmit_FS((uint8_t*) buffer, strlen(buffer));
			//USBD_CDC_SetRxBuffer(&hUsbDeviceFS, serialRXBuffer);
			//CDC_Transmit_FS(userSerialRXBuffer, strlen(userSerialRXBuffer));
			processRx(&PID1, &PID2);

		}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = ENABLE;
  hadc2.Init.NbrOfDiscConversion = 1;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7999;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STS_R_Pin|STS_G_Pin|STS_B_Pin|EN2_Pin
                          |EN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MASTER_DIR_Pin MASTER_EN_Pin */
  GPIO_InitStruct.Pin = MASTER_DIR_Pin|MASTER_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STS_R_Pin STS_G_Pin STS_B_Pin EN2_Pin
                           EN1_Pin */
  GPIO_InitStruct.Pin = STS_R_Pin|STS_G_Pin|STS_B_Pin|EN2_Pin
                          |EN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint16_t user_get_adc1() {
	//Turn ADC1 on
	HAL_ADC_Start(&hadc1);
	//Wait for ADC conversion to complete, the timeout is 100ms
	HAL_ADC_PollForConversion(&hadc1, 100);
	//Judge whether ADC conversion is successful
	if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) {
		//Read value
		return HAL_ADC_GetValue(&hadc1);
	}
	return 0;
}
uint16_t user_get_adc2() {
	//Turn ADC1 on
	HAL_ADC_Start(&hadc2);
	//Wait for ADC conversion to complete, the timeout is 100ms
	HAL_ADC_PollForConversion(&hadc2, 100);
	//Judge whether ADC conversion is successful
	if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc2), HAL_ADC_STATE_REG_EOC)) {
		//Read value
		return HAL_ADC_GetValue(&hadc2);
	}
	return 0;
}
uint16_t movingAvg(uint16_t *ptrArrNumbers, uint32_t *ptrSum, uint8_t *pos,
		uint8_t len, uint16_t nextNum) {

	//Subtract the oldest number from the prev sum, add the new number
	*ptrSum = *ptrSum - ptrArrNumbers[*pos] + nextNum;
	//Assign the nextNum to the position in the array
	ptrArrNumbers[*pos] = nextNum;
	//increment the position
	*pos = *pos + 1;
	if (*pos >= len) {
		*pos = 0;
	}
	//return the average
	return *ptrSum / len;
}

void processRx(PIDControl *pid1, PIDControl *pid2) {
	if (dataRxFlag != dataRxToggle) {
		float kP1, kI1, kD1;
		// Returns first token
		char *token = strtok(userSerialRXBuffer, ",");
		if(token[0] == 'L'){
			token = strtok(NULL, ",");
			kP1 = (float)atof(token);
			token = strtok(NULL, ",");
			kI1 = (float)atof(token);
			token = strtok(NULL, ",");
			kD1 = (float)atof(token);

			if(kP1 >= 0 && kD1 >= 0 && kI1 >= 0){
				PIDTuningsSet(pid1, kP1, kI1, kD1);
			}
			HAL_GPIO_TogglePin (GPIOB, STS_R_Pin);
		}else if(token[0] == 'R'){
			token = strtok(NULL, ",");
			kP1 = (float)atof(token);
			token = strtok(NULL, ",");
			kI1 = (float)atof(token);
			token = strtok(NULL, ",");
			kD1 = (float)atof(token);

			if(kP1 >= 0 && kD1 >= 0 && kI1 >= 0){
				PIDTuningsSet(pid2, kP1, kI1, kD1);
		}
			HAL_GPIO_TogglePin (GPIOB, STS_G_Pin);
		}else if(token[0] == 'G'){
			token = strtok(NULL, ",");
			percentage = (float)atof(token);
			HAL_GPIO_TogglePin (GPIOB, STS_B_Pin);
		}else if(token[0] == 'D'){
			token = strtok(NULL, ",");
			if(token[0] == 'L'){
				displayToggle = 0;
			}else if(token[0] == 'R'){
				displayToggle = 1;
			}
			HAL_GPIO_TogglePin (GPIOB, STS_B_Pin);
		}
		dataRxFlag = dataRxToggle;
	}

}
int user_MAP(int x, int in_min, int in_max, int out_min, int out_max){

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
	while (1) {
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
