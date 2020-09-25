/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	VOID_COMMAND,
	RETRANSMIT_CAN_MESSAGE,
	STOP_CAN_INTERRUPTS
} CANCommandDecodeType;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_MESSAGE_SIZE			10					// количество символов в пакете данных UART
#define UART_MESSAGE_TIMEOUT		10					// таймаут UART. Выяснить, что это.
#define CAN_MESSAGE_SIZE			8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t CAN_previous_message_buffer[CAN_MESSAGE_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void uart_send_int32_t(UART_HandleTypeDef* uart_instance, int32_t value_to_send)
{
	char UART_message_char[UART_MESSAGE_SIZE];
	for (uint8_t i = 0; i<sizeof(UART_message_char); i++)				// инициализация массива
	{
		UART_message_char[i] = ' ';
	}
	sprintf(UART_message_char, "%d", (int)value_to_send);

	uint8_t UART_message_int[UART_MESSAGE_SIZE];						// создаём массив типа uint8_t и заполняем значениями из char-массива. Получаем массив с кодами символов в ASCII.
	for (uint8_t i = 0; i<sizeof(UART_message_int); i++)
	{
		UART_message_int[i] = UART_message_char[i];
	}
	HAL_UART_Transmit(uart_instance, &UART_message_int[0], UART_MESSAGE_SIZE, UART_MESSAGE_TIMEOUT);

	char line_break_char[] = "\r\n";									// создаём и посылаем массив с переносом строки и возвратом каретки (для читабельности)
	uint8_t line_break_int[2];
	for (uint8_t i = 0; i<sizeof(line_break_int); i++)
	{
		line_break_int[i] = line_break_char[i];
	}
	HAL_UART_Transmit(uart_instance, &line_break_int[0], sizeof(line_break_int), UART_MESSAGE_TIMEOUT);
}

void CAN_previous_message_buffer_init(uint8_t CAN_message_size, uint8_t* CAN_previous_message_buffer_pointer)
{
	for (int i = 0; i < CAN_message_size; i++)
	{
		*(CAN_previous_message_buffer_pointer + i) = 0;								// инициализируем нулями глобальный буфер для хранения предыдущего сообщения CAN
	}
}

_Bool check_CAN_buffer (FDCAN_HandleTypeDef *hfdcan, uint8_t* CAN_previous_message_buffer_pointer)
{
	FDCAN_RxHeaderTypeDef rx_header = CAN_rx_header_get();							// получаем наш собственный заголовок для CAN-сообщения
	uint8_t rx_data_buffer[CAN_MESSAGE_SIZE];										// создаём массив, в который будет помещено новое CAN-сообщение
	for (int i = 0; i < sizeof(rx_data_buffer); i++)
	{
		rx_data_buffer[i] = 0;														// инициализируем созданный массив нулями (ЭТО ВАЖНО!!! �?наче в считанном CAN-сообщении появится мусор)
	}
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data_buffer);		// считываем содержимое FIFO-буфера CAN, помещаем в созданный массив

	_Bool CAN_message_changed = 0;													// флаг, сообщающий, что новое содержимое FIFO-буфера CAN отличается от предыдущего
	for (int i = 0; i < CAN_MESSAGE_SIZE; i++)
	{
		if (rx_data_buffer[i] != *(CAN_previous_message_buffer_pointer + i))		// если считанное содержимое CAN-сообщения отличается от предыдущего
		{
			*(CAN_previous_message_buffer_pointer + i) = rx_data_buffer[i];			// записываем в буфер для старого сообщения новое сообщение
			CAN_message_changed = 1;												// выставляем флаг, что пришло новое сообщение
		}
	}

	if (CAN_message_changed)														// если пришло новое сообщение
	{
		CAN_message_decode(rx_data_buffer);							// запускаем расшифровку сообщения
	}

	return CAN_message_changed;
}

void CAN_message_decode(uint8_t* rx_data_buffer_pointer)
{
	CANCommandDecodeType recieved_command = *rx_data_buffer_pointer;				// в нулевом элементе сообщения содержится номер команды
	if (recieved_command == RETRANSMIT_CAN_MESSAGE)										// если номер команды соответствует команде "изменить координату мотора"
	{
		uint8_t test_data[8] = {0, 1, 2, 3, 0, 0, 0, 0};
		CAN_test_transmit(&hfdcan1, test_data);
		HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
	}
	if (recieved_command == STOP_CAN_INTERRUPTS)										// если номер команды соответствует команде "изменить координату мотора"
	{
		uint8_t test_data[8] = {0, 1, 2, 3, 0, 0, 0, 0};
		CAN_test_transmit(&hfdcan1, test_data);
		HAL_FDCAN_DeactivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE);
	}
}

FDCAN_RxHeaderTypeDef CAN_rx_header_get(void)
{
	FDCAN_RxHeaderTypeDef tx_header;

	tx_header.Identifier = 0x0;
	tx_header.IdType = FDCAN_STANDARD_ID;
	tx_header.RxFrameType = FDCAN_DATA_FRAME;
	tx_header.DataLength = FDCAN_DLC_BYTES_8;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.RxTimestamp = 0xFFFF;
	tx_header.FilterIndex = 0;
	tx_header.IsFilterMatchingFrame = 0;

	return tx_header;
}

FDCAN_TxHeaderTypeDef CAN_tx_header_get (void)
{
	FDCAN_TxHeaderTypeDef tx_header;

	tx_header.Identifier = 0x0;
	tx_header.IdType = FDCAN_EXTENDED_ID;
	tx_header.TxFrameType = FDCAN_DATA_FRAME;
	tx_header.DataLength = FDCAN_DLC_BYTES_8;
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;
	tx_header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
	tx_header.MessageMarker = 0;

	return tx_header;
}

void CAN_test_transmit(FDCAN_HandleTypeDef *hfdcan, uint8_t* data_to_send)
{
	FDCAN_TxHeaderTypeDef test_header = CAN_tx_header_get();
	uint8_t data_buffer[8];
	for (int i = 0; i < sizeof(data_buffer); i++)
	{
		data_buffer[i] = *(data_to_send + i);
	}
	//uint8_t test_data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &test_header, data_buffer);
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
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  CAN_previous_message_buffer_init((uint8_t)sizeof(CAN_previous_message_buffer), CAN_previous_message_buffer);

  //HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
  //HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_FIFO_EMPTY, FDCAN_TX_BUFFER0);
  HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0);
  //HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
  HAL_FDCAN_Start(&hfdcan1);
  //uint8_t test_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  //CAN_test_transmit(&hfdcan1, test_data);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	check_CAN_buffer(&hfdcan1, CAN_previous_message_buffer);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 3;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
