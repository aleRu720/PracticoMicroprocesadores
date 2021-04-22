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

#include "mydelay.h"

#include "protocolo.h"

#include "debouncebuttonuser.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef union{
	struct{
		uint8_t STARTDATA: 1;
		uint8_t RAWDATAREADY: 1;
		uint8_t DATAREADY: 1;
		uint8_t BADVALUE: 1;
		uint8_t ACKPC: 1;
		uint8_t NOACKPC: 1;
		uint8_t SENDATAOK: 1;
		uint8_t RESERVE: 1;
	}flagBit;
	uint8_t resetFlags;
}uFlags;

static uFlags flags;

typedef enum{
	STANDBY,
	LEERSENSORES,
	CONVERTIRDATA,
	ENVIARDATA,
	LEERDATA
}eMefPpal;

static eMefPpal estadoPrograma;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TRUE 1
#define FALSE 0
#define MAXVALMV 3000
#define RESOL12B 4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

uint16_t valorADC[2], valorCovertAdc[2];
uint8_t indexADC;
delay_t timeOutTx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */

static void inicializaMef(void);

static void actualizaMef(void);

static void buttonState(delay_t *);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc->Instance == ADC1){
		if(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC)){
			__HAL_ADC_CLEAR_FLAG(hadc,ADC_FLAG_EOC);
			valorADC[indexADC++]=HAL_ADC_GetValue(hadc);
		}
		if(__HAL_ADC_GET_FLAG(hadc,ADC_FLAG_EOS)){
			indexADC=0;
			flags.flagBit.RAWDATAREADY=TRUE;
		}
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

	indexADC=0;

	delay_t hearbeat, debounceTime;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  indexADC=0;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_USART4_UART_Init();
  /* USER CODE BEGIN 2 */

  delayConfig(&hearbeat, 1000);

  delayConfig(&debounceTime, 40);

  delayConfig(&timeOutTx, 50);

  inicializaMef();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 //!Hearbeat, indica que el MCu está funcionando
	 if (delayRead(&hearbeat))
		 	 HAL_GPIO_TogglePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 //! Leer el estado del boton
	 buttonState(&debounceTime);

	 //! Actualizar la MEF principal
	 actualizaMef();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, signalE_Pin|LED_ROJO_Pin|LED_AZUL_Pin|LED_NARANJA_Pin
                          |LED_VERDE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : signalE_Pin LED_ROJO_Pin LED_AZUL_Pin LED_NARANJA_Pin
                           LED_VERDE_Pin */
  GPIO_InitStruct.Pin = signalE_Pin|LED_ROJO_Pin|LED_AZUL_Pin|LED_NARANJA_Pin
                          |LED_VERDE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : userButton_Pin */
  GPIO_InitStruct.Pin = userButton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(userButton_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void buttonState(delay_t *debounceTime){
	uint8_t boton;

	if (delayRead(debounceTime)){
		boton=estadoBoton(userButton_GPIO_Port, userButton_Pin);
		if(!boton && !flags.flagBit.STARTDATA){
				flags.flagBit.STARTDATA = TRUE;
		}
		if(!boton && flags.flagBit.STARTDATA && estadoPrograma!=STANDBY){
				inicializaMef();
		}
	}
}


static void inicializaMef(void){
	flags.resetFlags = FALSE;
	estadoPrograma=STANDBY;
	HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, FALSE);

}

static void actualizaMef(void){
static uint8_t timeOut;
	switch (estadoPrograma){
		case STANDBY:
			if(flags.flagBit.STARTDATA){
				estadoPrograma=LEERSENSORES;
				HAL_GPIO_WritePin(LED_AZUL_GPIO_Port, LED_AZUL_Pin, TRUE);
			}
		break;
		case LEERSENSORES:
			if(flags.flagBit.RAWDATAREADY){
				flags.flagBit.RAWDATAREADY=FALSE;
				estadoPrograma=CONVERTIRDATA;
			}
			else{
				if (indexADC==0)
					HAL_ADC_Start_IT(&hadc); //!< dispara la conversión del ADC
			}
		break;
		case CONVERTIRDATA:
			//!< Convierte los datos del canal 4 en valores de ingeniería
			valorCovertAdc[0]= (valorADC[0]* MAXVALMV)/RESOL12B;
			//!< Convierte los datos del canal 4 en valores de ingeniería
			valorCovertAdc[1]= (valorADC[1]* (MAXVALMV/10))/RESOL12B;
			estadoPrograma=ENVIARDATA;
			flags.flagBit.DATAREADY=TRUE;
			timeOut=0;
		break;
		case ENVIARDATA:
			if(delayRead(&timeOutTx)){
				timeOut++;
				if (timeOut >200){
					inicializaMef();
				}
			}
			if(flags.flagBit.DATAREADY){
				if(enviarDatos(valorCovertAdc))
					flags.flagBit.DATAREADY=FALSE;
			}else {
				flags.flagBit.SENDATAOK=leerAck();
				if(flags.flagBit.SENDATAOK){
					flags.flagBit.SENDATAOK=FALSE;
					estadoPrograma=LEERDATA;
					timeOut=0;
				}
			}
		break;
		case LEERDATA:
			if(delayRead(&timeOutTx)){
				timeOut++;
				if (timeOut >200){
						timeOut=0;
						estadoPrograma=ENVIARDATA;
						flags.flagBit.DATAREADY=TRUE;
				}
			}
			if(leerDatos()){
				flags.resetFlags=FALSE;
				flags.flagBit.STARTDATA=TRUE;
				estadoPrograma=LEERSENSORES;
			}

		break;
		default:
			flags.resetFlags = FALSE;
			estadoPrograma=STANDBY;
	}


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
