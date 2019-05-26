
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

#include "tm2_stm32_nrf24l01.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

NRF24L01_config_TypeDef nrf_rx_cfg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void show_int8( int8_t );

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	  /* Data received and data for send */
	  uint8_t dataOut[32], dataIn[32], config_bytes[ 100 ];
	  //uint8_t cnt = 0;

	  /* NRF transmission status */
	  TM_NRF24L01_Transmit_Status_t transmissionStatus;

	  GPIO_PinState tmp_pin;

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
  MX_SPI1_Init();
  //MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  nrf_rx_cfg.CE_pin          = GPIO_PIN_1;
  nrf_rx_cfg.CE_port         = GPIOA;
  nrf_rx_cfg.CSN_pin         = GPIO_PIN_2;
  nrf_rx_cfg.CSN_port        = GPIOA;
  nrf_rx_cfg.SPI             = &hspi1;
  nrf_rx_cfg.radio_channel   = 15;
  nrf_rx_cfg.baud_rate       = TM_NRF24L01_DataRate_1M;
  nrf_rx_cfg.payload_len     = 1;
  nrf_rx_cfg.crc_len         = 1;
  nrf_rx_cfg.output_power    = TM_NRF24L01_OutputPower_0dBm;
  nrf_rx_cfg.rx_address[ 0 ] = 0x7E;
  nrf_rx_cfg.rx_address[ 1 ] = 0x7E;
  nrf_rx_cfg.rx_address[ 2 ] = 0x7E;
  nrf_rx_cfg.rx_address[ 3 ] = 0x7E;
  nrf_rx_cfg.rx_address[ 4 ] = 0x7E;
  nrf_rx_cfg.tx_address[ 0 ] = 0xE7;
  nrf_rx_cfg.tx_address[ 1 ] = 0xE7;
  nrf_rx_cfg.tx_address[ 2 ] = 0xE7;
  nrf_rx_cfg.tx_address[ 3 ] = 0xE7;
  nrf_rx_cfg.tx_address[ 4 ] = 0xE7;

  //turn ON LED's
  HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET );
  HAL_GPIO_WritePin( LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET );
  tm2_NRF24L01_Init( &nrf_rx_cfg );
  HAL_Delay( 2000 );
  tm2_NRF24L01_Clear_Interrupts( &nrf_rx_cfg );
  HAL_Delay( 100 );
  //turn off LED's
  HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET );
  HAL_GPIO_WritePin( LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET );
  HAL_Delay( 1000 );
//  dataIn[ 0 ] = 'Q';
//  tm2_NRF24L01_Transmit( &nrf_rx_cfg, dataIn );
//	/* Wait for data to be sent */
//	do {
//	  /* Get transmission status */
//	  transmissionStatus = tm2_NRF24L01_GetTransmissionStatus( &nrf_rx_cfg );
//	} while( transmissionStatus == TM_NRF24L01_Transmit_Status_Sending );
//  tm2_NRF24L01_PowerUpRx( &nrf_rx_cfg );

  tm2_NRF24L01_ReadConfig( &nrf_rx_cfg, config_bytes );
  show_int8( config_bytes[ 0 ]);
  if( config_bytes[ 0 ] == 0x7B ) { //NRF24L01_CONFIG
	  HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET );
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay( 100 );
	  if( tm2_NRF24L01_DataReady( &nrf_rx_cfg )) {
		/* Get data from NRF24L01+ */
		tm2_NRF24L01_GetData( &nrf_rx_cfg, dataIn );
		if( dataIn[ 0 ] == 'B' ) {
			HAL_GPIO_WritePin( LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET );
		}
		else if( dataIn[ 0 ] == 'b' ) {
			HAL_GPIO_WritePin( LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET );
		}
		else if( dataIn[ 0 ] == 'G' ) {
			HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET );
		}
		else if( dataIn[ 0 ] == 'g' ) {
			HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET );
		}
		tm2_NRF24L01_Transmit( &nrf_rx_cfg, dataIn );
		/* Wait for data to be sent */
		do {
		  /* Get transmission status */
		  transmissionStatus = tm2_NRF24L01_GetTransmissionStatus( &nrf_rx_cfg );
		} while( transmissionStatus == TM_NRF24L01_Transmit_Status_Sending );
		tm2_NRF24L01_PowerUpRx( &nrf_rx_cfg );
	  }

	  if( HAL_GPIO_ReadPin( B1_GPIO_Port, B1_Pin ) == GPIO_PIN_SET ) {
		  tmp_pin = HAL_GPIO_ReadPin( LD4_GPIO_Port, LD4_Pin );
		  HAL_GPIO_WritePin( LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET );
		  dataOut[ 0 ] = 'K';
		  tm2_NRF24L01_Transmit( &nrf_rx_cfg, dataOut );
		  HAL_Delay( 100 );
		  /* Wait for data to be sent */
		  do {
			  /* Get transmission status */
			  transmissionStatus = tm2_NRF24L01_GetTransmissionStatus( &nrf_rx_cfg );
		  } while( transmissionStatus == TM_NRF24L01_Transmit_Status_Sending );
		  tm2_NRF24L01_PowerUpRx( &nrf_rx_cfg );
		  HAL_Delay( 100 );
		  HAL_GPIO_WritePin( LD4_GPIO_Port, LD4_Pin, tmp_pin );
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CE_Pin|SPI1_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CE_Pin SPI1_CSN_Pin */
  GPIO_InitStruct.Pin = SPI1_CE_Pin|SPI1_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void show_int8( int8_t x ) {
	for( int loop = 0; loop < 8; loop++ ) {
	  HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET );
	  HAL_GPIO_WritePin( LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET );
	  HAL_Delay( 500 );
	  HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET );
	  if( x & (0x80 >> loop) ) {
		  HAL_GPIO_WritePin( LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET );
	  }
	  HAL_Delay( 500 );
	}
  HAL_GPIO_WritePin( LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET );
  HAL_GPIO_WritePin( LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET );
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
