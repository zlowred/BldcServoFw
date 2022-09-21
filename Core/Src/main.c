/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "usbd_cdc_if.h"

#include "config.h"
#include "acs711.h"
#include "tmc4671.h"
#include "drv8320h.h"
#include "gray_encoder.h"
#include "tmc/ic/TMC4671/TMC4671.h"
#include "eeprom_emul.h"
#include "interface.h"
#include "cli.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

FDCAN_HandleTypeDef hfdcan1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for usbRx */
osThreadId_t usbRxHandle;
uint32_t usbRxBuffer[ 128 ];
osStaticThreadDef_t usbRxControlBlock;
const osThreadAttr_t usbRx_attributes = {
  .name = "usbRx",
  .stack_mem = &usbRxBuffer[0],
  .stack_size = sizeof(usbRxBuffer),
  .cb_mem = &usbRxControlBlock,
  .cb_size = sizeof(usbRxControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for blinker */
osThreadId_t blinkerHandle;
uint32_t blinkerBuffer[ 128 ];
osStaticThreadDef_t blinkerControlBlock;
const osThreadAttr_t blinker_attributes = {
  .name = "blinker",
  .stack_mem = &blinkerBuffer[0],
  .stack_size = sizeof(blinkerBuffer),
  .cb_mem = &blinkerControlBlock,
  .cb_size = sizeof(blinkerControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for rs485Comm */
osThreadId_t rs485CommHandle;
uint32_t rs485CommBuffer[ 128 ];
osStaticThreadDef_t rs485CommControlBlock;
const osThreadAttr_t rs485Comm_attributes = {
  .name = "rs485Comm",
  .stack_mem = &rs485CommBuffer[0],
  .stack_size = sizeof(rs485CommBuffer),
  .cb_mem = &rs485CommControlBlock,
  .cb_size = sizeof(rs485CommControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for fdCanComm */
osThreadId_t fdCanCommHandle;
uint32_t fdCanCommBuffer[ 128 ];
osStaticThreadDef_t fdCanCommControlBlock;
const osThreadAttr_t fdCanComm_attributes = {
  .name = "fdCanComm",
  .stack_mem = &fdCanCommBuffer[0],
  .stack_size = sizeof(fdCanCommBuffer),
  .cb_mem = &fdCanCommControlBlock,
  .cb_size = sizeof(fdCanCommControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for spi2Comm */
osThreadId_t spi2CommHandle;
uint32_t spi2CommBuffer[ 128 ];
osStaticThreadDef_t spi2CommControlBlock;
const osThreadAttr_t spi2Comm_attributes = {
  .name = "spi2Comm",
  .stack_mem = &spi2CommBuffer[0],
  .stack_size = sizeof(spi2CommBuffer),
  .cb_mem = &spi2CommControlBlock,
  .cb_size = sizeof(spi2CommControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for usbRxCompleted */
osSemaphoreId_t usbRxCompletedHandle;
osStaticSemaphoreDef_t usbRxCompletedControlBlock;
const osSemaphoreAttr_t usbRxCompleted_attributes = {
  .name = "usbRxCompleted",
  .cb_mem = &usbRxCompletedControlBlock,
  .cb_size = sizeof(usbRxCompletedControlBlock),
};
/* Definitions for spi1TxCompleted */
osSemaphoreId_t spi1TxCompletedHandle;
osStaticSemaphoreDef_t spi1TxCompletedControlBlock;
const osSemaphoreAttr_t spi1TxCompleted_attributes = {
  .name = "spi1TxCompleted",
  .cb_mem = &spi1TxCompletedControlBlock,
  .cb_size = sizeof(spi1TxCompletedControlBlock),
};
/* Definitions for spi1RxCompleted */
osSemaphoreId_t spi1RxCompletedHandle;
osStaticSemaphoreDef_t spi1RxCompletedControlBlock;
const osSemaphoreAttr_t spi1RxCompleted_attributes = {
  .name = "spi1RxCompleted",
  .cb_mem = &spi1RxCompletedControlBlock,
  .cb_size = sizeof(spi1RxCompletedControlBlock),
};
/* Definitions for rs485RxCompleted */
osSemaphoreId_t rs485RxCompletedHandle;
osStaticSemaphoreDef_t rs485RxCompletedControlBlock;
const osSemaphoreAttr_t rs485RxCompleted_attributes = {
  .name = "rs485RxCompleted",
  .cb_mem = &rs485RxCompletedControlBlock,
  .cb_size = sizeof(rs485RxCompletedControlBlock),
};
/* Definitions for rs485TxCompleted */
osSemaphoreId_t rs485TxCompletedHandle;
osStaticSemaphoreDef_t rs485TxCompletedControlBlock;
const osSemaphoreAttr_t rs485TxCompleted_attributes = {
  .name = "rs485TxCompleted",
  .cb_mem = &rs485TxCompletedControlBlock,
  .cb_size = sizeof(rs485TxCompletedControlBlock),
};
/* Definitions for fdCanRxCompleted */
osSemaphoreId_t fdCanRxCompletedHandle;
osStaticSemaphoreDef_t fdCanRxCompletedControlBlock;
const osSemaphoreAttr_t fdCanRxCompleted_attributes = {
  .name = "fdCanRxCompleted",
  .cb_mem = &fdCanRxCompletedControlBlock,
  .cb_size = sizeof(fdCanRxCompletedControlBlock),
};
/* Definitions for spi2RxCompleted */
osSemaphoreId_t spi2RxCompletedHandle;
osStaticSemaphoreDef_t spi2RxCompletedControlBlock;
const osSemaphoreAttr_t spi2RxCompleted_attributes = {
  .name = "spi2RxCompleted",
  .cb_mem = &spi2RxCompletedControlBlock,
  .cb_size = sizeof(spi2RxCompletedControlBlock),
};
/* Definitions for spi2TxCompleted */
osSemaphoreId_t spi2TxCompletedHandle;
osStaticSemaphoreDef_t spi2TxCompletedControlBlock;
const osSemaphoreAttr_t spi2TxCompleted_attributes = {
  .name = "spi2TxCompleted",
  .cb_mem = &spi2TxCompletedControlBlock,
  .cb_size = sizeof(spi2TxCompletedControlBlock),
};
/* Definitions for fdCanTxCompleted */
osSemaphoreId_t fdCanTxCompletedHandle;
osStaticSemaphoreDef_t fdCanTxCompletedControlBlock;
const osSemaphoreAttr_t fdCanTxCompleted_attributes = {
  .name = "fdCanTxCompleted",
  .cb_mem = &fdCanTxCompletedControlBlock,
  .cb_size = sizeof(fdCanTxCompletedControlBlock),
};
/* USER CODE BEGIN PV */

extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t fdCanTxData[64];
uint8_t fdCanRxData[64];
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

uint8_t rs485TxData[64];
uint8_t rs485RxData[128];
uint16_t rs485RxDataPos;
uint16_t rs485RxDataLength;


uint8_t usbRxData[256];
uint32_t usbRxDataLength;
uint32_t usbRxDataPos;

volatile uint32_t isErasing;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
void handleUsbRx(void *argument);
void blink(void *argument);
void handleRs485(void *argument);
void handleFdCan(void *argument);
void handleSpi2(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * Function to perform jump to system memory boot from user application
 *
 * Call function when you want to jump to system memory
 */
void JumpToBootloader(void) {
	void (*SysMemBootJump)(void);

	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

	SysMemBootJump = (void (*)(void)) (*((uint32_t*) (0x1FFF0004)));
	__set_MSP(*(uint32_t*) (0x1FFF0000));

	SysMemBootJump();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	__HAL_RCC_PWR_CLK_ENABLE();
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0xDEAD) {
		HAL_PWR_EnableBkUpAccess();
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0);
		HAL_PWR_DisableBkUpAccess();
		JumpToBootloader();
	}

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* Enable and set FLASH Interrupt priority */
  /* FLASH interrupt is used for the purpose of pages clean up under interrupt */
  HAL_NVIC_SetPriority(FLASH_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FLASH_IRQn);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_USB_Device_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(F_CS_GPIO_Port, F_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(M_CS_GPIO_Port, M_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

	initDrv8320h();
	setTmc4671State(TMC4671_ENABLED);

	EE_Status ee_status = EE_OK;
	HAL_FLASH_Unlock();

  /* Set EEPROM emulation firmware to erase all potentially incompletely erased
     pages if the system came from an asynchronous reset. Conditional erase is
     safe to use if all Flash operations where completed before the system reset */
  if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == RESET) {
    /* System reset comes from a power-on reset: Forced Erase */
    /* Initialize EEPROM emulation driver (mandatory) */
    ee_status = EE_Init(EE_FORCED_ERASE);
    if(ee_status != EE_OK) {Error_Handler();}
  } else {
  	HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIAG_2_GPIO_Port, DIAG_2_Pin, GPIO_PIN_SET);
    /* Clear the Standby flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

    /* Check and Clear the Wakeup flag */
    if (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF2) != RESET) {
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
    }

    /* System reset comes from a STANDBY wakeup: Conditional Erase*/
    /* Initialize EEPROM emulation driver (mandatory) */
    ee_status = EE_Init(EE_CONDITIONAL_ERASE);
    if(ee_status != EE_OK) {Error_Handler();}
  }


  /* Wait any cleanup is completed before accessing flash again */
	while (isErasing) { }

	/* Lock the Flash Program Erase controller */
	HAL_FLASH_Lock();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of usbRxCompleted */
  usbRxCompletedHandle = osSemaphoreNew(1, 1, &usbRxCompleted_attributes);

  /* creation of spi1TxCompleted */
  spi1TxCompletedHandle = osSemaphoreNew(1, 1, &spi1TxCompleted_attributes);

  /* creation of spi1RxCompleted */
  spi1RxCompletedHandle = osSemaphoreNew(1, 1, &spi1RxCompleted_attributes);

  /* creation of rs485RxCompleted */
  rs485RxCompletedHandle = osSemaphoreNew(1, 1, &rs485RxCompleted_attributes);

  /* creation of rs485TxCompleted */
  rs485TxCompletedHandle = osSemaphoreNew(1, 1, &rs485TxCompleted_attributes);

  /* creation of fdCanRxCompleted */
  fdCanRxCompletedHandle = osSemaphoreNew(1, 1, &fdCanRxCompleted_attributes);

  /* creation of spi2RxCompleted */
  spi2RxCompletedHandle = osSemaphoreNew(1, 1, &spi2RxCompleted_attributes);

  /* creation of spi2TxCompleted */
  spi2TxCompletedHandle = osSemaphoreNew(1, 1, &spi2TxCompleted_attributes);

  /* creation of fdCanTxCompleted */
  fdCanTxCompletedHandle = osSemaphoreNew(1, 1, &fdCanTxCompleted_attributes);

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
  /* creation of usbRx */
  usbRxHandle = osThreadNew(handleUsbRx, NULL, &usbRx_attributes);

  /* creation of blinker */
  blinkerHandle = osThreadNew(blink, NULL, &blinker_attributes);

  /* creation of rs485Comm */
  rs485CommHandle = osThreadNew(handleRs485, NULL, &rs485Comm_attributes);

  /* creation of fdCanComm */
  fdCanCommHandle = osThreadNew(handleFdCan, NULL, &fdCanComm_attributes);

  /* creation of spi2Comm */
  spi2CommHandle = osThreadNew(handleSpi2, NULL, &spi2Comm_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 72;
  hfdcan1.Init.NominalTimeSeg2 = 71;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 9;
  hfdcan1.Init.DataTimeSeg2 = 8;
  hfdcan1.Init.StdFiltersNbr = 2;
  hfdcan1.Init.ExtFiltersNbr = 2;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x111;
	sFilterConfig.FilterID2 = 0x7FF;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Configure global filter on both FDCAN instances:
	 Filter all remote frames with STD and EXT ID
	 Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
	FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
		Error_Handler();
	}

	/* Activate Rx FIFO 0 new message notification on both FDCAN instances */
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0)
			!= HAL_OK) {
		Error_Handler();
	}

	/* Prepare Tx message Header */
	TxHeader.Identifier = 0x111;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_64;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = FDCAN_FD_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	/* Configure and enable Tx Delay Compensation, required for BRS mode.
	 TdcOffset default recommended value: DataTimeSeg1 * DataPrescaler
	 TdcFilter default recommended value: 0 */
	if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 9, 0) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	/* Start the FDCAN module on both FDCAN instances */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart2.Init.BaudRate = 16000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_8;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
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
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GR_EN_Pin|F_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, M_CS_Pin|DIAG_1_Pin|DIAG_2_Pin|SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(F_EN_GPIO_Port, F_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GR_EN_Pin F_CS_Pin */
  GPIO_InitStruct.Pin = GR_EN_Pin|F_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GR_D0_Pin GR_D1_Pin CUR_SENSE_FAULT_Pin VDS_18_Pin
                           VDS_75_Pin */
  GPIO_InitStruct.Pin = GR_D0_Pin|GR_D1_Pin|CUR_SENSE_FAULT_Pin|VDS_18_Pin
                          |VDS_75_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GR_D2_Pin GR_D3_Pin F_STATUS_Pin M_FAULT_Pin
                           VDS_0_Pin */
  GPIO_InitStruct.Pin = GR_D2_Pin|GR_D3_Pin|F_STATUS_Pin|M_FAULT_Pin
                          |VDS_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : M_CS_Pin DIAG_1_Pin DIAG_2_Pin SPI2_CS_Pin */
  GPIO_InitStruct.Pin = M_CS_Pin|DIAG_1_Pin|DIAG_2_Pin|SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GR_D4_Pin GR_D5_Pin IDRIVE_0_Pin IDRIVE_18_Pin
                           IDRIVE_75_Pin MODE_0_Pin MODE_45_Pin */
  GPIO_InitStruct.Pin = GR_D4_Pin|GR_D5_Pin|IDRIVE_0_Pin|IDRIVE_18_Pin
                          |IDRIVE_75_Pin|MODE_0_Pin|MODE_45_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : F_EN_Pin */
  GPIO_InitStruct.Pin = F_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(F_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue) {
  /* Call CleanUp callback when all requested pages have been erased */
  if (ReturnValue == 0xFFFFFFFF) {
    isErasing = 0;
  }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
		osSemaphoreRelease(spi1TxCompletedHandle);
	} else if (hspi->Instance == SPI2) {
		osSemaphoreRelease(spi2TxCompletedHandle);
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
		osSemaphoreRelease(spi1RxCompletedHandle);
	} else if (hspi->Instance == SPI2) {
		osSemaphoreRelease(spi2RxCompletedHandle);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
	if (huart->Instance == USART2) {
		printf("RX:%3d bytes\n", size);
		rs485RxDataLength = size;
		osSemaphoreRelease(rs485RxCompletedHandle);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		osSemaphoreRelease(rs485TxCompletedHandle);
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if (hfdcan->Instance == FDCAN1
			&& (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
		/* Retrieve Rx messages from RX FIFO0 */
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, fdCanRxData)
				== HAL_OK) {
			osSemaphoreRelease(fdCanRxCompletedHandle);
		}
	}
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan,
		uint32_t BufferIndexes) {
	if (hfdcan->Instance == FDCAN1) {
		osSemaphoreRelease(fdCanTxCompletedHandle);
	}
}

uint32_t calculateCrc(uint8_t *data, uint16_t length) {
	return HAL_CRC_Calculate(&hcrc, (uint32_t *)data, length);
}

int _write(int file, char *ptr, int len) {
	static uint8_t rc = USBD_OK;

	do {
		if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
			break;
		}
		rc = CDC_Transmit_FS((uint8_t *)ptr, len);
	} while (USBD_BUSY == rc);

	return len;
}

char nextChar() {
	while (usbRxDataPos >= usbRxDataLength) {
		USBD_CDC_ReceivePacket(&hUsbDeviceFS);
		if (osSemaphoreAcquire(usbRxCompletedHandle, 10) != osOK) {
			continue;
		}
		if (!usbRxDataLength) {
			continue;
		}
	}
	return usbRxData[usbRxDataPos++];
}

void startDfu() {
	osKernelLock();
	HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIAG_2_GPIO_Port, DIAG_2_Pin, GPIO_PIN_SET);
	HAL_PWR_EnableBkUpAccess();
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0xDEAD);
	HAL_PWR_DisableBkUpAccess();
	for (int i = 0; i < 10; i++) {
		HAL_Delay(50);
		HAL_GPIO_TogglePin(DIAG_1_GPIO_Port, DIAG_1_Pin);
		HAL_GPIO_TogglePin(DIAG_2_GPIO_Port, DIAG_2_Pin);
		HAL_Delay(200);
		HAL_GPIO_TogglePin(DIAG_1_GPIO_Port, DIAG_1_Pin);
		HAL_GPIO_TogglePin(DIAG_2_GPIO_Port, DIAG_2_Pin);
	}
	NVIC_SystemReset();
}

void doRs485Test() {
	printf("RS485 TX 1\n");
	memset(rs485TxData, 0, 60);
	sprintf((char*) rs485TxData, "$ 0x%08lX\n", xTaskGetTickCount());
	*((uint32_t *)&rs485TxData[60]) = calculateCrc(rs485TxData, 60);
	HAL_UART_Transmit_DMA(&huart2, rs485TxData, 32);
	osSemaphoreAcquire(rs485TxCompletedHandle, 10);
	printf("RS485 TX 2\n");
	osDelay(10);
	HAL_UART_Transmit_DMA(&huart2, &rs485TxData[32], 32);
	osSemaphoreAcquire(rs485TxCompletedHandle, 10);
	printf("RS485 TX 3\n");
	static uint8_t junk[4] = "junk";
	osDelay(10);
	HAL_UART_Transmit_DMA(&huart2, junk, 4);
	osSemaphoreAcquire(rs485TxCompletedHandle, 10);
	printf("RS485 TX 4\n");
}

void doFdCanTest() {
	sprintf((char*) fdCanTxData, "# %08lx\n", xTaskGetTickCount());
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, fdCanTxData)
			!= HAL_OK) {
		Error_Handler();
	}
	osSemaphoreAcquire(fdCanTxCompletedHandle, 10);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_handleUsbRx */
/**
 * @brief  Function implementing the usbRx thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_handleUsbRx */
void handleUsbRx(void *argument)
{
  /* USER CODE BEGIN 5 */

	/* Infinite loop */
	for (;;) {
		processCli(nextChar());
	}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_blink */
/**
 * @brief Function implementing the blinker thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_blink */
void blink(void *argument)
{
  /* USER CODE BEGIN blink */

	/* Infinite loop */
	for (;;) {
		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIAG_2_GPIO_Port, DIAG_2_Pin, GPIO_PIN_SET);
		osDelay(2000);
		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(DIAG_2_GPIO_Port, DIAG_2_Pin, GPIO_PIN_RESET);
		osDelay(1000);

		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIAG_2_GPIO_Port, DIAG_2_Pin, hasDrv8320hFailure());
		osDelay(100);
		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_RESET);
		osDelay(500);

		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIAG_2_GPIO_Port, DIAG_2_Pin, hasTmc4671Failure());
		osDelay(100);
		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_RESET);
		osDelay(100);
		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_SET);
		osDelay(100);
		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_RESET);
		osDelay(500);

		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIAG_2_GPIO_Port, DIAG_2_Pin, hasAcs711Failure());
		osDelay(100);
		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_RESET);
		osDelay(100);
		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_SET);
		osDelay(100);
		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_RESET);
		osDelay(100);
		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_SET);
		osDelay(100);
		HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_RESET);
		osDelay(500);
	}

  /* USER CODE END blink */
}

/* USER CODE BEGIN Header_handleRs485 */
/**
 * @brief Function implementing the rs485Comm thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_handleRs485 */
void handleRs485(void *argument)
{
  /* USER CODE BEGIN handleRs485 */
	/* Infinite loop */
	for (;;) {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, &rs485RxData[rs485RxDataPos], sizeof(rs485RxData) - rs485RxDataPos);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
		if (osSemaphoreAcquire(rs485RxCompletedHandle, 10) != osOK) {
			continue;
		}
		uint16_t newDataPos = rs485RxDataPos;
		for (;rs485RxDataLength > 0; rs485RxDataLength--) {
			newDataPos++;
			if (handleCommand(rs485RxData, newDataPos, RS485)) {
				memmove(rs485RxData, &rs485RxData[newDataPos], rs485RxDataLength);
				newDataPos = 0;
			}
		}
		if (newDataPos == 128) {
			memmove(rs485RxData, &rs485RxData[64], 64);
			newDataPos = 64;
		}
		printf("%3d -> %3d\n", rs485RxDataPos, newDataPos);
		rs485RxDataPos = newDataPos;
	}
  /* USER CODE END handleRs485 */
}

/* USER CODE BEGIN Header_handleFdCan */
/**
 * @brief Function implementing the fdCanComm thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_handleFdCan */
void handleFdCan(void *argument)
{
  /* USER CODE BEGIN handleFdCan */
	/* Infinite loop */
	for (;;) {
		if (osSemaphoreAcquire(fdCanRxCompletedHandle, 10) != osOK) {
			continue;
		}

		if (fdCanRxData[0] == '#') {
			memcpy(fdCanTxData, fdCanRxData, 64);
			fdCanTxData[0] = ']';
			if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, fdCanTxData)
					!= HAL_OK) {
				Error_Handler();
			}
			osSemaphoreAcquire(fdCanTxCompletedHandle, 10);
		} else {
			fdCanRxData[12] = 0;
			printf((char *)fdCanRxData);
		}
	}
  /* USER CODE END handleFdCan */
}

/* USER CODE BEGIN Header_handleSpi2 */
/**
 * @brief Function implementing the spi2Comm thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_handleSpi2 */
void handleSpi2(void *argument)
{
  /* USER CODE BEGIN handleSpi2 */
	/* Infinite loop */
	for (;;) {
		if (osSemaphoreAcquire(spi2RxCompletedHandle, 10) != osOK) {
			continue;
		}
	}
  /* USER CODE END handleSpi2 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
	__disable_irq();
	while (1) {
		for (int i = 0; i < 300000; i++) {
			HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DIAG_2_GPIO_Port, DIAG_2_Pin, GPIO_PIN_RESET);
		}
		for (int i = 0; i < 300000; i++) {
			HAL_GPIO_WritePin(DIAG_1_GPIO_Port, DIAG_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DIAG_2_GPIO_Port, DIAG_2_Pin, GPIO_PIN_SET);
		}
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
