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
#include <image.h>
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "ssd1331.h"
#include "ugui.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim17;

osThreadId defaultTaskHandle;
osThreadId updateDisplayHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM17_Init(void);
void StartDefaultTask(void const *argument);
void StartUpdate(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int uxTopUsedPriority = configMAX_PRIORITIES - 1; // FreeRTOS/OCD debug workaround

volatile unsigned long ulHighFrequencyTimerTicks = 0;

unsigned long getRunTimeCounterValue(void)
{
	return ulHighFrequencyTimerTicks;
}

/* GUI structure */
UG_GUI gui;

/* Touch structure */
//static TP_STATE *TP_State;
/* Some defines */
#define MAX_OBJECTS        10
#define TOGGLE_GREEN_LED   HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//#define TOGGLE_RED_LED     GPIO_ToggleBits(GPIOG,GPIO_Pin_14);

/* Window 1 */
UG_WINDOW window_1;
UG_OBJECT obj_buff_wnd_1[MAX_OBJECTS];
UG_BUTTON button1_1;
UG_BUTTON button1_2;
UG_BUTTON button1_3;
UG_BUTTON button1_4;
UG_BUTTON button1_5;
UG_BUTTON button1_6;

/* Window 2 */
UG_WINDOW window_2;
UG_OBJECT obj_buff_wnd_2[MAX_OBJECTS];
UG_BUTTON button2_1;
UG_TEXTBOX textbox2_1;
UG_TEXTBOX textbox2_2;
UG_IMAGE image2_1;

/* Window 3 */
UG_WINDOW window_3;
UG_OBJECT obj_buff_wnd_3[MAX_OBJECTS];
UG_BUTTON button3_1;
UG_TEXTBOX textbox3_1;

/* FSM */
#define STATE_MAIN_MENU                0
#define STATE_BENCHMARK_RUN            1
#define STATE_BENCHMARK_RESULT         2
volatile UG_U32 state;
volatile UG_U32 next_state;

/* Benchmark */
volatile UG_U32 timer;
volatile UG_U32 hw_acc = 1;
char result_str[30];
UG_S16 xs, ys;
UG_S16 xe, ye;
UG_COLOR c;

/* Callback function for the main menu */
void window_1_callback(UG_MESSAGE *msg)
{
	if (msg->type == MSG_TYPE_OBJECT)
	{
		if (msg->id == OBJ_TYPE_BUTTON)
		{
			switch (msg->sub_id)
			{
			case BTN_ID_0: /* Toggle green LED */
			{
				TOGGLE_GREEN_LED
				;
				break;
			}
			case BTN_ID_1: /* Toggle red LED */
			{
//				TOGGLE_RED_LED
				;
				break;
			}
			case BTN_ID_2: /* Show uGUI info */
			{
				UG_WindowShow(&window_2);
				break;
			}
			case BTN_ID_3: /* Toggle hardware acceleration */
			{
				if (!hw_acc)
				{
					UG_ButtonSetForeColor(&window_1, BTN_ID_3, C_RED);
					UG_ButtonSetText(&window_1, BTN_ID_3, "HW_ACC\nOFF");
					UG_DriverEnable( DRIVER_DRAW_LINE);
					UG_DriverEnable( DRIVER_FILL_FRAME);
				}
				else
				{
					UG_ButtonSetForeColor(&window_1, BTN_ID_3, C_BLUE);
					UG_ButtonSetText(&window_1, BTN_ID_3, "HW_ACC\nON");
					UG_DriverDisable( DRIVER_DRAW_LINE);
					UG_DriverDisable( DRIVER_FILL_FRAME);
				}
				hw_acc = !hw_acc;
				break;
			}
			case BTN_ID_4: /* Start benchmark */
			{
				next_state = STATE_BENCHMARK_RUN;
				break;
			}
			case BTN_ID_5: /* Resize window */
			{
				static UG_U32 tog;

				if (!tog)
				{
					UG_WindowResize(&window_1, 0, 40, 239, 319 - 40);
				}
				else
				{
					UG_WindowResize(&window_1, 0, 0, 239, 319);
				}
				tog = !tog;
				break;
			}
			}
		}
	}
}

/* Callback function for the info window */
void window_2_callback(UG_MESSAGE *msg)
{
	if (msg->type == MSG_TYPE_OBJECT)
	{
		if (msg->id == OBJ_TYPE_BUTTON)
		{
			switch (msg->sub_id)
			{
			case BTN_ID_0:
			{
				UG_WindowHide(&window_2);
				break;
			}
			}
		}
	}
}

/* Callback function for the result window */
void window_3_callback(UG_MESSAGE *msg)
{
	if (msg->type == MSG_TYPE_OBJECT)
	{
		if (msg->id == OBJ_TYPE_BUTTON)
		{
			switch (msg->sub_id)
			{
			/* OK button */
			case BTN_ID_0:
			{
				UG_WindowShow(&window_1);
				break;
			}
			}
		}
	}
}

/* better rand() function */
UG_U32 randx(void)
{
	static UG_U32 z1 = 12345, z2 = 12345, z3 = 12345, z4 = 12345;
	UG_U32 b;
	b = ((z1 << 6) ^ z1) >> 13;
	z1 = ((z1 & 4294967294U) << 18) ^ b;
	b = ((z2 << 2) ^ z2) >> 27;
	z2 = ((z2 & 4294967288U) << 2) ^ b;
	b = ((z3 << 13) ^ z3) >> 21;
	z3 = ((z3 & 4294967280U) << 7) ^ b;
	b = ((z4 << 3) ^ z4) >> 12;
	z4 = ((z4 & 4294967168U) << 13) ^ b;
	return (z1 ^ z2 ^ z3 ^ z4);
}

/**
 * @brief  Tx Transfer completed callback.
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @retval None
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hspi);

	spi_event_handler();
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
	MX_DMA_Init();
	MX_LPUART1_UART_Init();
	MX_I2C2_Init();
	MX_SPI2_Init();
	MX_TIM17_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim17);

	/* Init uGUI */
	UG_Init(&gui, (void (*)(UG_S16, UG_S16, UG_COLOR)) pset, SSD1331_WIDTH, SSD1331_HEIGHT );

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of updateDisplay */
	osThreadDef(updateDisplay, StartUpdate, osPriorityNormal, 0, 256);
	updateDisplayHandle = osThreadCreate(osThread(updateDisplay), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 36;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1
			| RCC_PERIPHCLK_I2C2;
	PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
	PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x20B0D9FF;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void)
{

	/* USER CODE BEGIN LPUART1_Init 0 */

	/* USER CODE END LPUART1_Init 0 */

	/* USER CODE BEGIN LPUART1_Init 1 */

	/* USER CODE END LPUART1_Init 1 */
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = 115200;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN LPUART1_Init 2 */

	/* USER CODE END LPUART1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void)
{

	/* USER CODE BEGIN TIM17_Init 0 */

	/* USER CODE END TIM17_Init 0 */

	/* USER CODE BEGIN TIM17_Init 1 */

	/* USER CODE END TIM17_Init 1 */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 0;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 1440;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM17_Init 2 */

	/* USER CODE END TIM17_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, SSD1331_RST_Pin | SSD1331_DC_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SSD1331_RST_Pin SSD1331_DC_Pin */
	GPIO_InitStruct.Pin = SSD1331_RST_Pin | SSD1331_DC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
	/* USER CODE BEGIN 5 */

	ssd1331_init();

	/* Clear Screen */
	UG_FillScreen( C_BLACK);

	/* Create Window 1 */
	UG_WindowCreate(&window_1, obj_buff_wnd_1, MAX_OBJECTS, window_1_callback);
	UG_WindowSetTitleText(&window_1, "uGUI STM32F474RE");
	UG_WindowSetTitleTextFont(&window_1, &FONT_6X10);

	/* Create some Buttons */
	UG_ButtonCreate(&window_1, &button1_1, BTN_ID_0, 3, 3, 33, 20);
	UG_ButtonCreate(&window_1, &button1_2, BTN_ID_1, 3, 26, 33, 43);
	UG_ButtonCreate(&window_1, &button1_3, BTN_ID_2, 3, 50, 33, 67);
	UG_ButtonCreate(&window_1, &button1_4, BTN_ID_3, 40, 3,
			UG_WindowGetInnerWidth(&window_1) - 3, 20);
	UG_ButtonCreate(&window_1, &button1_5, BTN_ID_4, 40, 26,
			UG_WindowGetInnerWidth(&window_1) - 3, 43);
	UG_ButtonCreate(&window_1, &button1_6, BTN_ID_5, 40, 50,
			UG_WindowGetInnerWidth(&window_1) - 10, 67);

	/* Configure Button 1 */
	UG_ButtonSetFont(&window_1, BTN_ID_0, &FONT_4X6);
	UG_ButtonSetBackColor(&window_1, BTN_ID_0, C_GREEN);
	UG_ButtonSetText(&window_1, BTN_ID_0, "Green");
	/* Configure Button 2 */
	UG_ButtonSetFont(&window_1, BTN_ID_1, &FONT_4X6);
	UG_ButtonSetBackColor(&window_1, BTN_ID_1, C_RED);
	UG_ButtonSetText(&window_1, BTN_ID_1, "Red");
	/* Configure Button 3 */
	UG_ButtonSetFont(&window_1, BTN_ID_2, &FONT_4X6);
	UG_ButtonSetText(&window_1, BTN_ID_2, "About uGUI");
	UG_WindowShow(&window_1);
	/* Configure Button 4 */
	UG_ButtonSetFont(&window_1, BTN_ID_3, &FONT_4X6);
	UG_ButtonSetForeColor(&window_1, BTN_ID_3, C_RED);
	UG_ButtonSetText(&window_1, BTN_ID_3, "HW_ACC OFF");
	/* Configure Button 5 */
	UG_ButtonSetFont(&window_1, BTN_ID_4, &FONT_4X6);
	UG_ButtonSetText(&window_1, BTN_ID_4, "Start Benchmark");
	/* Configure Button 6 */
	UG_ButtonSetFont(&window_1, BTN_ID_5, &FONT_4X6);
	UG_ButtonSetText(&window_1, BTN_ID_5, "Resize Window");

	/* -------------------------------------------------------------------------------- */
	/* Create Window 2 (uGUI Info)                                                      */
	/* -------------------------------------------------------------------------------- */
	UG_WindowCreate(&window_2, obj_buff_wnd_2, MAX_OBJECTS, window_2_callback);
	UG_WindowSetTitleText(&window_2, "Info");
	UG_WindowSetTitleTextFont(&window_2, &FONT_6X10);
	UG_WindowResize(&window_2, 7, 13, 73, 93);

	/* Create Button 1 */
	UG_ButtonCreate(&window_2, &button2_1, BTN_ID_0, 7, 44,
			UG_WindowGetInnerWidth(&window_2) - 13, 63);
	UG_ButtonSetFont(&window_2, BTN_ID_0, &FONT_5X12);
	UG_ButtonSetText(&window_2, BTN_ID_0, "OK!");

	/* Create Textbox 1 */
	UG_TextboxCreate(&window_2, &textbox2_1, TXB_ID_0, 3, 3,
			UG_WindowGetInnerWidth(&window_2) - 3, 20);
	UG_TextboxSetFont(&window_2, TXB_ID_0, &FONT_5X8);
	UG_TextboxSetText(&window_2, TXB_ID_0, "uGUI v0.3");
	UG_TextboxSetAlignment(&window_2, TXB_ID_0, ALIGN_TOP_CENTER);

	/* Create Textbox 2 */
	UG_TextboxCreate(&window_2, &textbox2_2, TXB_ID_1, 3, 15,
			UG_WindowGetInnerWidth(&window_2) - 3, 42);
	UG_TextboxSetFont(&window_2, TXB_ID_1, &FONT_4X6);
	UG_TextboxSetText(&window_2, TXB_ID_1, "www.embeddedlightning.com");
	UG_TextboxSetAlignment(&window_2, TXB_ID_1, ALIGN_BOTTOM_CENTER);
	UG_TextboxSetForeColor(&window_2, TXB_ID_1, C_BLUE);
	UG_TextboxSetHSpace(&window_2, TXB_ID_1, 1);

	/* Create Image 1 */
	UG_ImageCreate(&window_2, &image2_1, IMG_ID_0,
			(UG_WindowGetInnerWidth(&window_2) >> 1) - (logo.width >> 1), 13, 0,
			0);
	UG_ImageSetBMP(&window_2, IMG_ID_0, &logo);

	/* -------------------------------------------------------------------------------- */
	/* Create Window 3 (Benchmark Result)                                               */
	/* -------------------------------------------------------------------------------- */
	UG_WindowCreate(&window_3, obj_buff_wnd_3, MAX_OBJECTS, window_3_callback);
	UG_WindowSetTitleText(&window_3, "Benchmark Result");
	UG_WindowSetTitleTextFont(&window_3, &FONT_8X12);
	UG_WindowResize(&window_3, 7, 30, 73, 77);

	/* Create Textbox 1 */
	UG_TextboxCreate(&window_3, &textbox3_1, TXB_ID_0, 2, 3,
			UG_WindowGetInnerWidth(&window_3) - 2, 20);
	UG_TextboxSetFont(&window_3, TXB_ID_0, &FONT_5X8);
	UG_TextboxSetText(&window_3, TXB_ID_0, "Result:\n99999 frm/sec");
	UG_TextboxSetAlignment(&window_3, TXB_ID_0, ALIGN_TOP_CENTER);

	/* Create Button 1 */
	UG_ButtonCreate(&window_3, &button3_1, BTN_ID_0, 13, 22,
			UG_WindowGetInnerWidth(&window_3) - 13, 40);
	UG_ButtonSetFont(&window_3, BTN_ID_0, &FONT_4X6);
	UG_ButtonSetText(&window_3, BTN_ID_0, "OK!");

	/* -------------------------------------------------------------------------------- */
	/* Start demo application                                                           */
	/* -------------------------------------------------------------------------------- */
	/* Show Window 1 */
	UG_WindowShow(&window_1);
	UG_WaitForUpdate();
	osDelay(2000);

	/* Initialize FSM */
	next_state = STATE_MAIN_MENU;
	state = !STATE_MAIN_MENU;

	/* Reset the frame counter */
	uint32_t frm_cnt = 0;

	TickType_t ticks = xTaskGetTickCount();

	/* Run the benchmark */
	while (frm_cnt < 1000)
	{
		xs = randx() % SSD1331_WIDTH;
		xe = randx() % SSD1331_WIDTH;
		ys = randx() % SSD1331_HEIGHT;
		ye = randx() % SSD1331_HEIGHT;
		c = randx() % 0xFFFFFF;
		UG_FillFrame(xs, ys, xe, ye, c);
		frm_cnt++;
		osDelay(30);
	}

	UG_WindowShow(&window_2);
	UG_WaitForUpdate();

	osDelay(2000);

	sprintf(result_str, "Result:\n%u frm/sec",
			1000000 / (xTaskGetTickCount() - ticks));
	UG_TextboxSetText(&window_3, TXB_ID_0, result_str);

	/* Show benchmark result */
	UG_WindowShow(&window_3);

	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUpdate */
/**
 * @brief Function implementing the updateDisplay thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUpdate */
void StartUpdate(void const *argument)
{
	/* USER CODE BEGIN StartUpdate */

	osDelay(100);

	/* Infinite loop */
	for (;;)
	{
		UG_Update();
		ssd1331_update();
		osDelay(30);
	}
	/* USER CODE END StartUpdate */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM17)
	{
		ulHighFrequencyTimerTicks++;
	}

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6)
	{
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

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
