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
#include "vl53l1_api.h"
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VL53L1_ADDRESS 0x52
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t TxBuf[50]; //USART1 debug buffer

VL53L1_DEV vl53l1_c;
FixPoint1616_t signalLimit = 16384;    //Signal,related to reflected amplitude
//0.25*65536
FixPoint1616_t sigmaLimit = 1179648;     //Sigmal, related to distance mm
//18*65536

uint32_t timingBudget = 200000;         //When the ranging mode is set to timed ranging, user has to define the period of time
								//between two consecutive measurements.
uint8_t preRangeVcselPeriod = 14;  //VCSEL pulse cycle
uint8_t finalRangeVcselPeriod = 10;//VCSEL pulse cycle period
//high accuracy

uint32_t distance;
VL53L1_RangingMeasurementData_t result_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vl53l1_Init(VL53L1_DEV pDev,uint8_t I2C_Addr, I2C_HandleTypeDef *I2CHandle)
{
	pDev->I2cDevAddr = I2C_Addr;
	pDev->I2cHandle = I2CHandle;
	pDev->comms_type=1;
	pDev->comms_speed_khz = 400;
	/*** VL53L1X Initialization ***/
	VL53L1_WaitDeviceBooted(pDev);
	HAL_Delay(2);
	VL53L1_DataInit(pDev);
	HAL_Delay(2);
	VL53L1_StaticInit(pDev);
	HAL_Delay(2);
	VL53L1_SetDistanceMode(pDev, VL53L1_DISTANCEMODE_LONG);
	HAL_Delay(2);
}
void vl53l1_ParamInit(VL53L1_DEV pDev,FixPoint1616_t sigmaLimit, FixPoint1616_t signalLimit,FixPoint1616_t timingBudget)
{
	VL53L1_SetLimitCheckEnable(pDev,VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE,1);//sigma--standard deviation, enable SIGMA limit check

	VL53L1_SetLimitCheckEnable(pDev,VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,1);//signal--amplitude of the signal-
																																													//-reflected. enable signal rate limit check
	HAL_Delay(2);
	VL53L1_SetLimitCheckValue(pDev,VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE,sigmaLimit);//set SIGMA limit

	HAL_Delay(2);
	VL53L1_SetLimitCheckValue(pDev,VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,signalLimit);//set signal rate limit

	VL53L1_SetMeasurementTimingBudgetMicroSeconds(pDev,timingBudget);//set the max interval for a whole diatance test

	HAL_Delay(2);
	VL53L1_SetInterMeasurementPeriodMilliSeconds(pDev, 300);

	HAL_Delay(2);
	VL53L1_StartMeasurement(pDev);
}
void getDistance(VL53L1_DEV pDev)
{
	 VL53L1_WaitMeasurementDataReady(pDev);
	 VL53L1_GetRangingMeasurementData(pDev, &result_data);
	 distance = result_data.RangeMilliMeter;
	 VL53L1_ClearInterruptAndStartMeasurement(pDev);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /*GPIO setup vl53l1 init*/
  HAL_GPIO_WritePin(XShut1_GPIO_Port, XShut1_Pin, GPIO_PIN_RESET);
  HAL_Delay(2); // 2ms reset time
  HAL_GPIO_WritePin(XShut1_GPIO_Port, XShut1_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  /*GPIO setup vl53l1 init*/
  vl53l1_Init(vl53l1_c,VL53L1_ADDRESS,&hi2c1);
  vl53l1_ParamInit(vl53l1_c, sigmaLimit, signalLimit, timingBudget);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  getDistance(vl53l1_c);
	  //debug oled check distance value
	  memset(TxBuf,0,sizeof(TxBuf)); //clear buffer before write
	  sprintf((char*)TxBuf,"%lu",distance);
	  HAL_UART_Transmit(&huart1,TxBuf,sizeof(TxBuf), 10);
	  HAL_Delay(500);
	  //----- Phan code test debug UART gui den man hinh OLED -----
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  HAL_GPIO_WritePin(GPIO0_GPIO_Port, GPIO0_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(XShut0_GPIO_Port, XShut0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(XShut1_GPIO_Port, XShut1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : GPIO0_Pin */
  GPIO_InitStruct.Pin = GPIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : XShut0_Pin */
  GPIO_InitStruct.Pin = XShut0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(XShut0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : XShut1_Pin */
  GPIO_InitStruct.Pin = XShut1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(XShut1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO1_Pin */
  GPIO_InitStruct.Pin = GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIO1_GPIO_Port, &GPIO_InitStruct);

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
