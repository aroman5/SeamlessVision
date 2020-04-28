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
#include "stm32fxxx_hal.h"

#include "tm_stm32_delay.h"
#include "tm_stm32_button.h"
#include "hx711.h"
#include "timer_delay_us.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

#define LOG_ENABLED 					0

//Report types
#define UART_DATA_TYPE_LOAD_CELL		0x00
#define UART_DATA_TYPE_BTN				0x01
#define UART_DATA_TYPE_TOUCH			0x02
#define UART_DATA_TYPE_TOUCH_RELEASE	0x03
#define UART_DATA_TYPE_VERSION			0x04

//Request types
#define UART_DATA_TYPE_VERSION_REQUEST	0x80
#define UART_DATA_TYPE_MOTOR_REQUEST	0x81

#define APP_VER							0x01

#define RXBUFFERSIZE        10	/* Size of Reception buffer */
#define TXBUFFERSIZE        50	/* Size of Transmit buffer */
#define PRINT_BUFFER_SIZE	100

/* Button pointer */
TM_BUTTON_t *BTN_HOME;
TM_BUTTON_t *BTN_SELECT;
TM_BUTTON_t *BTN_SELECT_DOWN;
TM_BUTTON_t *BTN_SELECT_UP;
TM_BUTTON_t *BTN_L_UP;
TM_BUTTON_t *BTN_L_DOWN;
TM_BUTTON_t *BTN_VOICE;
TM_BUTTON_t *BTN_R_DOWN;
TM_BUTTON_t *BTN_R_UP;
TM_BUTTON_t *BTN_SOS;

TM_BUTTON_t *BTN_TOUCH;

//Update Timer
TM_DELAY_Timer_t *SWTIM1;

uint8_t rx_uart_buffer[RXBUFFERSIZE] = { 0 }; /* Buffer used for reception */
uint8_t tx_uart_buffer[TXBUFFERSIZE] = { 0 }; /* Buffer used for transmit */
uint8_t tx_uart_buffer_size = 0;

static uint8_t uart_buffer[RXBUFFERSIZE] = { 0 };
static uint8_t uart_buffer_pos = 0;

HX711 HX711_sensor;
uint32_t HX711_val;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* Button callback function */
static void BUTTON_Callback(TM_BUTTON_t *ButtonPtr, TM_BUTTON_PressType_t PressType);
void SWTIM1_Callback(TM_DELAY_Timer_t *SWTIM, void *UserParameters);

void usart_process_data(const void *data, size_t len);
void request_data_check();
void send_app_ver();

void HAL_printf_valist(const char *fmt, va_list argp);
void HAL_printf(const char *fmt, ...);
static void logPrint(uint8_t level, const char *fmt, va_list argp);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Implement handler function */
static void BUTTON_Callback(TM_BUTTON_t *ButtonPtr, TM_BUTTON_PressType_t PressType) {
	uint8_t bytes[2];
	bytes[0] = ButtonPtr->idx;
	bytes[1] = PressType;

	UART_TX_DATA(UART_DATA_TYPE_BTN, bytes, sizeof(bytes));
	logI("BUTTON_Callback Button:%d PressType:%d\r\n", ButtonPtr->idx, PressType);

	HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);

	/* Normal press detected */
	if (PressType == TM_BUTTON_PressType_Normal) {
		if (ButtonPtr == BTN_HOME) {
			//HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
		}
	} else if (PressType == TM_BUTTON_PressType_Long) {
		//HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
	}
}

void usart_rx_check(void) {
	static size_t old_pos;
	size_t pos;

	/* Calculate current position in buffer */
	pos = RXBUFFERSIZE - huart1.hdmarx->Instance->CNDTR;
	if (pos != old_pos) { /* Check change in received data */
		if ((pos == 0 && rx_uart_buffer[RXBUFFERSIZE - 1] == 0x0A) || rx_uart_buffer[pos - 1] == 0x0A) {
			if (pos > old_pos) { /* Current position is over previous one */
				/* We are in "linear" mode */
				/* Process data directly by subtracting "pointers" */
				usart_process_data(&rx_uart_buffer[old_pos], pos - old_pos);
			} else {
				/* We are in "overflow" mode */
				/* First process data to the end of buffer */
				usart_process_data(&rx_uart_buffer[old_pos], ARRAY_LEN(rx_uart_buffer) - old_pos);
				/* Check and continue with beginning of buffer */
				if (pos > 0) {
					usart_process_data(&rx_uart_buffer[0], pos);
				}
			}

			old_pos = pos; /* Save current position as old */

			/* Check and manually update if we reached end of buffer */
			if (old_pos == ARRAY_LEN(rx_uart_buffer)) {
				old_pos = 0;
			}

			uart_buffer_pos = 0;
			request_data_check();
		}
	}
}

void usart_process_data(const void *data, size_t len) {
	const uint8_t *d = data;

	for (; len > 0; --len, ++d, uart_buffer_pos++) {
		uart_buffer[uart_buffer_pos] = *d;
	}
}

void request_data_check(void) {
	if (uart_buffer[0] == '$' || uart_buffer[1] == 'H') {
		switch (uart_buffer[2]) {
		case UART_DATA_TYPE_VERSION_REQUEST:
			send_app_ver();
			break;
		case UART_DATA_TYPE_MOTOR_REQUEST:
			if (uart_buffer[3] == 1) // Vibrating motor 1
					{
				HAL_GPIO_WritePin(M_1_GPIO_Port, M_1_Pin, uart_buffer[4]);
				HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, uart_buffer[4]);
			} else if (uart_buffer[3] == 2) // Vibrating motor 2
					{
				HAL_GPIO_WritePin(M_2_GPIO_Port, M_2_Pin, uart_buffer[4]);
				HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, uart_buffer[4]);
			}
			break;
		default:
			break;
		}
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* Init Load Cell A/D HX711 */
	HX711_sensor.gpioSck = HX711SCK_GPIO_Port;
	HX711_sensor.gpioData = HX711OUTPUT_GPIO_Port;
	HX711_sensor.pinSck = HX711SCK_Pin;
	HX711_sensor.pinData = HX711OUTPUT_Pin;
	HX711_sensor.offset = 0;
	HX711_sensor.gain = 1; // channel A, gain factor 128

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
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */

	logI("SeamlessVision Start!\r\n");

	/* Init Load Cell A/D HX711 */
	HX711_Init(HX711_sensor);

	/* Init delay */
	TM_DELAY_Init();

	//Set Load Cell frequency (ms)
#if LOG_ENABLED
	SWTIM1 = TM_DELAY_TimerCreate(500, 1, 1, SWTIM1_Callback, NULL);
#else
	SWTIM1 = TM_DELAY_TimerCreate(500, 1, 1, SWTIM1_Callback, NULL);
#endif

	/* Init button: idx, PORT, PIN, STATE when PRESSED */
	BTN_HOME = TM_BUTTON_Init(1, SW_HOME_GPIO_Port, SW_HOME_Pin, TM_BUTTON_PressType_Normal, BUTTON_Callback);
	BTN_SELECT = TM_BUTTON_Init(2, SW_SELECT_GPIO_Port, SW_SELECT_Pin, TM_BUTTON_PressType_Normal, BUTTON_Callback);
	BTN_SELECT_DOWN = TM_BUTTON_Init(3, SW_SELECT_DOWN_GPIO_Port, SW_SELECT_DOWN_Pin, TM_BUTTON_PressType_Normal, BUTTON_Callback);
	BTN_SELECT_UP = TM_BUTTON_Init(4, SW_SELECT_UP_GPIO_Port, SW_SELECT_UP_Pin, TM_BUTTON_PressType_Normal, BUTTON_Callback);
	BTN_L_UP = TM_BUTTON_Init(5, SW_L_UP_GPIO_Port, SW_L_UP_Pin, TM_BUTTON_PressType_Normal, BUTTON_Callback);
	BTN_L_DOWN = TM_BUTTON_Init(6, SW_L_DOWN_GPIO_Port, SW_L_DOWN_Pin, TM_BUTTON_PressType_Normal, BUTTON_Callback);
	BTN_VOICE = TM_BUTTON_Init(7, SW_VOICE_GPIO_Port, SW_VOICE_Pin, TM_BUTTON_PressType_Normal, BUTTON_Callback);
	BTN_R_DOWN = TM_BUTTON_Init(8, SW_R_DOWN_GPIO_Port, SW_R_DOWN_Pin, TM_BUTTON_PressType_Normal, BUTTON_Callback);
	BTN_R_UP = TM_BUTTON_Init(9, SW_R_UP_GPIO_Port, SW_R_UP_Pin, TM_BUTTON_PressType_Normal, BUTTON_Callback);
	BTN_SOS = TM_BUTTON_Init(10, SW_SOS_GPIO_Port, SW_SOS_Pin, TM_BUTTON_PressType_Normal, BUTTON_Callback);

	BTN_TOUCH = TM_BUTTON_Init(11, TOUCH_1_GPIO_Port, TOUCH_1_Pin, TM_BUTTON_PressType_Normal, BUTTON_Callback);

	/* Init all buttons */
	/* Set time how button is detected, 30 ms for normal press, 2000 ms for long press */
	TM_BUTTON_SetPressTime(BTN_HOME, 30, 2000);
	TM_BUTTON_SetPressTime(BTN_SELECT, 30, 2000);
	TM_BUTTON_SetPressTime(BTN_SELECT_DOWN, 30, 2000);
	TM_BUTTON_SetPressTime(BTN_SELECT_UP, 30, 2000);
	TM_BUTTON_SetPressTime(BTN_L_UP, 30, 2000);
	TM_BUTTON_SetPressTime(BTN_L_DOWN, 30, 2000);
	TM_BUTTON_SetPressTime(BTN_VOICE, 30, 2000);
	TM_BUTTON_SetPressTime(BTN_R_DOWN, 30, 2000);
	TM_BUTTON_SetPressTime(BTN_R_UP, 30, 2000);
	TM_BUTTON_SetPressTime(BTN_SOS, 30, 2000);

	TM_BUTTON_SetPressTime(BTN_TOUCH, 30, 2000);

	logI("Started successfully\r\n");
	HAL_UART_Receive_DMA(&huart1, (uint8_t*) rx_uart_buffer, RXBUFFERSIZE);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/* Update all buttons */
		TM_BUTTON_Update();
		usart_rx_check();
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* DMA1_Channel4_5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	/* USART1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00101D7C;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
	sSlaveConfig.InputTrigger = TIM_TS_ITR0;
	if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 38400;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, M_1_Pin | M_2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LED_1_Pin | LED_2_Pin | HX711SCK_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : M_1_Pin M_2_Pin */
	GPIO_InitStruct.Pin = M_1_Pin | M_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_1_Pin LED_2_Pin HX711SCK_Pin */
	GPIO_InitStruct.Pin = LED_1_Pin | LED_2_Pin | HX711SCK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : HX711OUTPUT_Pin SW_SELECT_UP_Pin SW_L_UP_Pin SW_SELECT_Pin
	 SW_SELECT_DOWN_Pin */
	GPIO_InitStruct.Pin = HX711OUTPUT_Pin | SW_SELECT_UP_Pin | SW_L_UP_Pin | SW_SELECT_Pin | SW_SELECT_DOWN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : SW_L_DOWN_Pin SW_VOICE_Pin SW_HOME_Pin SW_R_DOWN_Pin
	 SW_R_UP_Pin SW_SOS_Pin */
	GPIO_InitStruct.Pin = SW_L_DOWN_Pin | SW_VOICE_Pin | SW_HOME_Pin | SW_R_DOWN_Pin | SW_R_UP_Pin | SW_SOS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : IRQ_Pin */
	GPIO_InitStruct.Pin = IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : TOUCH_1_Pin */
	GPIO_InitStruct.Pin = TOUCH_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TOUCH_1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_3_Pin */
	GPIO_InitStruct.Pin = LED_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_3_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* Callback function for SWTIM1 */
void SWTIM1_Callback(TM_DELAY_Timer_t *SWTIM, void *UserParameters) {
	HX711_val = HX711_Value(HX711_sensor);
	//HX711_val = HX711_AverageValue(hx11, 5);

	logI("HX711 Val:%d\r\n", HX711_val);

	uint8_t *bytes = (uint8_t*) &HX711_val;
	/*
	 uint8_t bytes[4];
	 bytes[0] = (HX711_val >> 0)  & 0xFF;
	 bytes[1] = (HX711_val >> 8)  & 0xFF;
	 bytes[2] = (HX711_val >> 16) & 0xFF;
	 bytes[3] = (HX711_val >> 24) & 0xFF;
	 */

	UART_TX_DATA(UART_DATA_TYPE_LOAD_CELL, bytes, sizeof(bytes));
}

void UART_TX_DATA(uint8_t type, uint8_t *data, uint8_t size) {
#if LOG_ENABLED
	return;
#else
	uint8_t i, j;
	uint8_t tx_uart_size = size + 5;

	uint32_t uart_busy = 0;

	tx_uart_buffer[0] = '$';
	tx_uart_buffer[1] = 'H';
	tx_uart_buffer[2] = type;

	for (i = 3, j = 0; j < size; i++, j++)
		tx_uart_buffer[i] = data[j];

	tx_uart_buffer[tx_uart_size - 2] = '\r';
	tx_uart_buffer[tx_uart_size - 1] = '\n';

	while (HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY && HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_RX) {
		uart_busy++;
		if(uart_busy>10000)
			return;
	}

	if (HAL_UART_Transmit_DMA(&huart1, tx_uart_buffer, tx_uart_size) != HAL_OK) {
		Error_Handler();
	}
#endif
}

void send_app_ver() {
	uint8_t bytes[3];
	bytes[0] = APP_VER;
	bytes[1] = APP_VER;
	bytes[2] = APP_VER;

	UART_TX_DATA(UART_DATA_TYPE_VERSION, bytes, sizeof(bytes));
}

// Tx Transfer completed callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
	//HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
	//__NOP();
}

//Rx Transfer completed callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {

}

//UART error callbacks
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
	Error_Handler();
}

/**
 * @brief EXTI line detection callbacks
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_8) {

	}
}

void HAL_printf_valist(const char *fmt, va_list argp) {
	char string[PRINT_BUFFER_SIZE];
	if (vsprintf(string, fmt, argp) > 0) {
		if (HAL_UART_Transmit(&huart1, (uint8_t*) string, strlen(string), HAL_MAX_DELAY) != HAL_OK) {
			return;
			//Error_Handler();
		}
	} else {
		if (HAL_UART_Transmit(&huart1, (uint8_t*) "E - Print\n", 14, HAL_MAX_DELAY) != HAL_OK) {
			return;
			//Error_Handler();
		}
	}
}

/** Custom printf function, only translate to va_list arg HAL_UART.
 * @param *fmt String to print
 * @param ... Data
 */
void HAL_printf(const char *fmt, ...) {
	va_list argp;

	va_start(argp, fmt);
	HAL_printf_valist(fmt, argp);
	va_end(argp);
}

/** Generic LOG procedure
 * @param Log level
 * @param *fmt String to print
 * @param argp Parameters list
 */
static void logPrint(uint8_t level, const char *fmt, va_list argp) {
	HAL_printf("%c - ", level);
	HAL_printf_valist(fmt, argp);
}

/** LOG procedure - Info
 * @param *fmt String to print
 * @param ... Parameters list
 */
void logI(const char *fmt, ...) {
#if LOG_ENABLED
	va_list argp;

	va_start(argp, fmt);
	logPrint('I', fmt, argp);
	va_end(argp);
#else
	return;
#endif

}

/** LOG procedure - Error
 * @param *fmt String to print
 * @param .. Parameters list
 */
void logE(const char *fmt, ...) {
#if LOG_ENABLED
	va_list argp;

	va_start(argp, fmt);
	logPrint('E', fmt, argp);
	va_end(argp);
#else
	return;
#endif

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	for (uint8_t i = 0; i < 3; i++) {
		HAL_GPIO_TogglePin(LED_3_GPIO_Port, LED_3_Pin);
		HAL_Delay(1000);
	}

	NVIC_SystemReset();

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
