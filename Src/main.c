/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "mainApp.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "bmp280.h"
#include "math.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//********************************
// Be sure to have proper 3.3vlt power supply
// ADC and DHT22 consume lots of lower 
//********************************
//===============================
//			DHT		      	STM32f103
//-------------------------------
//		DATA OUT					PC15
//      VCC             3.3vlt
//      GND             Gnd
//===============================
//     Touch 					STM32F103
// -------------------------------
//     Output	 					PC14
//================================
// (gnd)---LDR----|----10k---3.3
//								PA0
//--------------------------------
//      ADC						STM32F103
//--------------------------------
//			out						   PA0
//      LDR Pin					 GND
//			10k pin					 Vcc(3.3)
//================================
//     UART           STM32f103
//--------------------------------
//      RX               PA9
//      TX							 PA10
//================================
//	  BMP280      STM32F103
// 	   VCC          3.3vlt
//   	 Gnd          Gnd
//   	 SCL          B6
//   	 SDA          B7
//********************************

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t sum, RH, TEMP;
uint8_t check = 0;
float t,r;
bool touch ;
uint16_t adcResult = 0;
float altitude ,pressure, p, temperature, humidity;

uint16_t size;
uint8_t Data[25];
float lumen;

//**************************************************
//char buf_temp[100],buf_humid[100];
char buff[100];
//char transmit_char[100];
//**************************************************



void delay_us (uint16_t us);
void set_gpio_input (void);
void set_gpio_output(void);
void DHT22_start (void);
void check_response (void);
uint8_t read_data (void);
long map(long x, long in_min, long in_max, long out_min, long out_max);

GPIO_InitTypeDef GPIO_InitStruct;
BMP280_HandleTypedef bmp280;


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim1);
	HAL_UART_Transmit(&huart1, (uint8_t *)"STARTING SENSORS READING\r\n",sizeof("STARTING SENSORS READING\r\n"),100);
	HAL_Delay(1000);
	
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;
	
	HAL_Delay(500);
	HAL_UART_Transmit(&huart1,(uint8_t *) " Trying BMP280 initialization\r\n",sizeof(" Trying BMP280 initialization\r\n"), 100);
	while (!(bmp280_init(&bmp280, &bmp280.params))) 
		{
			HAL_UART_Transmit(&huart1, (uint8_t*)".", sizeof("."), 1000);
			bmp280_init_default_params(&bmp280.params);
			HAL_Delay(90);
		}
	bool bme280p = bmp280.id == BMP280_CHIP_ID;
	size = sprintf((char *)Data, "BMP280: found %s\r\n", bme280p ? "BME280" : "BMP280");
	HAL_UART_Transmit(&huart1, Data, size, 1000);

		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//====================DHT22====================================		
		DHT22_start ();
		check_response();
		
		Rh_byte1 = read_data ();
		Rh_byte2 = read_data ();
		Temp_byte1 = read_data ();
		Temp_byte2 = read_data ();
		sum = read_data();
	
		TEMP = ((Temp_byte1<<8)|Temp_byte2);
		RH = ((Rh_byte1<<8)|Rh_byte2);
		
   	t=TEMP*0.1;
	  r=RH*0.1;
		
//===============================================================
//=============================ADC DATA=======================
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 100);
 
				adcResult = HAL_ADC_GetValue(&hadc1);
 
				HAL_ADC_Stop(&hadc1);
				lumen =  map(adcResult, 3300.00 , 700.00, 0.0 , 100.0);
//=============================================================
				HAL_Delay(100);
//==========================Read BMP280 Data===================
		if (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) 
			{
				pressure = 0;
				altitude = 0;
			
//			//size = sprintf((char *)Data,
//					"Temperature/pressure reading failed\n");
//			//HAL_UART_Transmit(&huart1, Data, size, 1000);
//			//HAL_Delay(100);
				}
		else
			{
				p = pressure/100;
				altitude = 44330.0*(1-pow(p/1013.25, 1/5.255));
			}
		//  size = sprintf((char *)Data,"\nPressure: %.2f Pa, Temperature: %.2f C , Altitude: %.2f m\r\n",
		//		pressure, temperature , altitude);
		//size = sprintf((char *)Data,"\nPressure: %.2f Pa, Temperature: %.2f C \r\n", 
		//		pressure, temperature);
		//HAL_UART_Transmit(&huart1, Data, size, 1000);
//		if (bme280p) {
//			//size = sprintf((char *)Data,", Humidity: %.2f\n", humidity);
//			//HAL_UART_Transmit(&huart1, Data, size, 1000);
//		}

//		else {
//			size = sprintf((char *)Data, "\n");
//			HAL_UART_Transmit(&huart1, Data, size, 1000);
//		}
		HAL_Delay(500);
//========================================================================
//====================================Touch read==========================
			touch = 	HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14);
//========================================================================			
			int len = sprintf(buff," sw :%s lum :%.2f T :%.2f H :%.2f P: %.2f Alt: %.2f\r\n", (touch ? "ON" : "OFF"),lumen ,t,r,pressure,altitude);
			HAL_UART_Transmit(&huart1,(uint8_t *) buff, len, 100);
			HAL_Delay(500);
			

	
	// mainApp();
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Reset_Pin|nIrq_Pin|DIO1_Pin|SPI_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : RED_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 DHT22_pin_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14|DHT22_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Reset_Pin nIrq_Pin DIO1_Pin SPI_NSS_Pin */
  GPIO_InitStruct.Pin = Reset_Pin|nIrq_Pin|DIO1_Pin|SPI_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void set_gpio_output (void)
{
	/*Configure GPIO pin output: PC15 */
  GPIO_InitStruct.Pin = DHT22_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT22_pin_GPIO_Port, &GPIO_InitStruct);
}

void set_gpio_input (void)
{
	/*Configure GPIO pin input: PC15 */
  GPIO_InitStruct.Pin = DHT22_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT22_pin_GPIO_Port, &GPIO_InitStruct);
}


void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);   //set the counter value to zero
	while(__HAL_TIM_GET_COUNTER(&htim1) < us); //wait for the counter to reach the us input in the parameter
}

void DHT22_start (void)
{
	set_gpio_output ();  // set the pin as output
	HAL_GPIO_WritePin (DHT22_pin_GPIO_Port, DHT22_pin_Pin ,GPIO_PIN_RESET);   // pull the pin low
	//delay_us(500);
	delay_us(500);   // wait for 500us
	HAL_GPIO_WritePin (DHT22_pin_GPIO_Port, GPIO_PIN_1,GPIO_PIN_SET);   // pull the pin high
	//delay_us(30);
	delay_us(30);   // wait for 30us
	set_gpio_input ();   // set as input
}

void check_response (void)
{
	delay_us(40);  // wait for 40us
	if (!(HAL_GPIO_ReadPin (DHT22_pin_GPIO_Port, DHT22_pin_Pin))) // if the pin is low
	{
		
		delay_us(80);   // wait for 80us
		if ((HAL_GPIO_ReadPin (DHT22_pin_GPIO_Port, DHT22_pin_Pin)))  check = 1;  // if the pin is high, response is ok
	}
	while((HAL_GPIO_ReadPin (DHT22_pin_GPIO_Port, DHT22_pin_Pin)));   // wait for the pin to go low
}


uint8_t read_data (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT22_pin_GPIO_Port, DHT22_pin_Pin)));   // wait for the pin to go high
		
		
		delay_us(40);   // wait for 40 us
		if ((HAL_GPIO_ReadPin (DHT22_pin_GPIO_Port, DHT22_pin_Pin)) == 0)   // if the pin is low 
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT22_pin_GPIO_Port, DHT22_pin_Pin)));  // wait for the pin to go low
	}
	return i;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
