/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "temp_sensor.h"
#include "load_cell.h"
#include "canHandler.h"
#include "flash_func.h"

extern uint32_t _app_addr;

#define RX_SETTING_MODE 0x18FFBBEB
#define TX_SETTING_MODE 0x18FFCCEB
#define TX_ERROR 0x18FFDDEB
#define OFFSET_PERIOD_TRANSMIT 1
#define SEMPLING_SENSDATA_LOAD_UNLOAD 20

#define MIN_OFFSET_SENS_DATA 0
#define MAX_OFFSET_SENS_DATA 32000
#define MIN_OFFSET_WEIGHT 1
#define MAX_OFFSET_WEIGHT 64000
#define SIZE_FLASH_WRITE_DATA 64
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	WORK, SETTING
} SwitchState;

typedef enum {
	WAITING = 0x10,
	ENTER_SETTING_MODE = 0x15,
	PERIOD_TRANSMIT = 0x16,
	CHANGE_CANID = 0x17,
	SET_WITHOUT_LOAD = 0x18,
	SET_WITH_LOAD = 0x19,
	SET_OFFSET_UNLOAD = 0x20,
	SET_OFFSET_LOAD = 0x21,
	SET_MAX_WEIGHT = 0x22,
	SET_COUNT_FLT_AVR_LOAD = 0x23,
	SET_DEFAULT_VALUES = 0x24,
	EXIT_SETTING_MODE = 0x25,
	ERROR_CMD = 0xFF
} CanCMD;

typedef enum {
	PER_10ms = 0x01, PER_20ms, PER_50ms, PER_100ms, PER_1000ms, PER_5000ms,
} EnumPeriodTransmitType;

#pragma pack(1)
typedef union {
	uint32_t CanID;
	struct {
		uint8_t SrcAdr;
		uint16_t PGN;
		uint8_t unused1 :3, Prioryty :3, unused2 :2;
	} ID_STRUCT;
} CAN_ID_UNION;

typedef union {
	struct {
		int32_t DataRaw;
		int16_t DataWeight;
		int16_t Temperature;
	} Data;
	uint8_t RawData[8];
} OutData;

typedef union {
	struct {
		uint8_t tempSRCHi;
		uint8_t tempSRCLow;
		uint8_t readFlashSensorData;
		uint8_t commLoadAmp;
	} Data;
	uint8_t RawData[8];
} strErrors;

typedef struct {
	uint8_t CMD;
	uint8_t unused[7];
} strSETTING_CMD;

typedef struct {
	uint8_t CMD;
	uint8_t TYPE_PERIOD;
	uint8_t unused[6];
} strSETTING_SET_PERIOD;

typedef struct {
	uint8_t CMD;
	uint8_t COUNT_FLT_AVR_LOAD;
	uint8_t unused[6];
} strSETTING_FLT_AVR;

typedef struct {
	uint8_t CMD;
	uint32_t CanID;
	uint8_t unused[3];
} strSETTING_SET_CANID;

typedef struct {
	uint8_t CMD;
	int16_t Offset;
	uint8_t unused[5];
} strSETTING_SET_OFFSET_SENS_DATA;

typedef struct {
	uint8_t CMD;
	uint16_t Weight;
	uint8_t unused[5];
} strSETTING_SET_WEIGHT;

typedef struct {
	CAN_ID_UNION TxCanID;
	uint8_t PeriodTransmitType;
	int16_t UnloadSensData;
	int16_t LoadSensData;
	uint16_t OffsetUnloadSensData;
	uint16_t OffsetLoadSensData;
	uint16_t WeightMax;
	uint8_t CountFltAvrLoad;
} strSensorData;

typedef union {
	struct {
		strSensorData SensData;
	} Data;
	uint8_t RawData[SIZE_FLASH_WRITE_DATA];
} strWriteFlashData;
#pragma pack()
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern uint32_t _app_addr;

#define RX_SETTING_MODE 0x18FFBBEB
#define TX_SETTING_MODE 0x18FFCCEB
#define TX_ERROR 0x18FFDDEB
#define OFFSET_PERIOD_TRANSMIT 1
#define SEMPLING_SENSDATA_LOAD_UNLOAD 20

#define MIN_OFFSET_SENS_DATA 0
#define MAX_OFFSET_SENS_DATA 32000
#define MIN_OFFSET_WEIGHT 1
#define MAX_OFFSET_WEIGHT 64000
#define SIZE_FLASH_WRITE_DATA 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float temperatureOut = 0;
int16_t outputLoadCell = 0;
uint32_t errCodeStrtCAN, errCodeTxCAN, errCodeRxCAN;

CAN_RxHeaderTypeDef pRxHeader;
uint8_t settingRxBuff[8] = { };
strSETTING_CMD settingTxBuff = { 0xFF, { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } };
strErrors sendErrors;

OutData outCanData;

SwitchState switchMain = WORK;
CanCMD switchCanCMD = WAITING;

uint16_t periodTransmit = 50;

strSensorData sensorData, oldSensorData;
uint8_t errorReadFlashSensorData;

uint16_t deltaCountTxCan, oldCountTxCan;
uint8_t flagSend;

uint8_t countWork;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t calculationWeight(int16_t rawSensData, strSensorData sensorData);
void writeSensorData(strSensorData *sensorData);
uint8_t readSensorData(strSensorData *sensorData);
uint16_t getPeriodTransmit(EnumPeriodTransmitType *periodType);
void setDefaultValues(strSensorData* sensorData);
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
	SCB->VTOR = (uint32_t) &_app_addr;
	__enable_irq();
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
  MX_ADC1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

	canSetHWFlt(CAN_EXT, 1, CAN_RX_FIFO0, RX_SETTING_MODE, 0xFFFFFFFF);
	errCodeStrtCAN = canStart();
	init_conversion_tempSensor();

	errorReadFlashSensorData = readSensorData(&sensorData);
	if (errorReadFlashSensorData) {
		setDefaultValues(&sensorData);
	}

	periodTransmit = getPeriodTransmit(&sensorData.PeriodTransmitType);
	setCountFltAvrLoad(sensorData.CountFltAvrLoad);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		countWork++;
		temperatureOut = proc_tempSensor();
		outputLoadCell = proc_hx711_getValue();

		outCanData.Data.DataRaw = outputLoadCell;
		outCanData.Data.Temperature = temperatureOut;

		flagSend = 0;
		deltaCountTxCan = HAL_GetTick() - oldCountTxCan;
		if (deltaCountTxCan > (periodTransmit - OFFSET_PERIOD_TRANSMIT)) {
			oldCountTxCan = HAL_GetTick();
			flagSend = 1;
		}

		switch (switchMain) {
		case WORK:
			outCanData.Data.DataWeight = calculationWeight(outputLoadCell, sensorData);

			if (flagSend) {
				errCodeTxCAN = canTXMessage(CAN_EXT, sensorData.TxCanID.CanID, 0, 8, outCanData.RawData);
			}

			errCodeRxCAN = canRxMessage(CAN_RX_FIFO0, &pRxHeader, settingRxBuff);
			if (errCodeRxCAN == HAL_OK) {
				if (pRxHeader.ExtId == RX_SETTING_MODE && ((strSETTING_CMD*) settingRxBuff)->CMD == ENTER_SETTING_MODE) {
					switchMain = SETTING;
					switchCanCMD = WAITING;
					settingTxBuff.CMD = ENTER_SETTING_MODE;
					canTXMessage(CAN_EXT, TX_SETTING_MODE, 0, 8, (uint8_t*) &settingTxBuff);
					oldSensorData = sensorData;
				}
			}
			break;
		case SETTING:
			errCodeRxCAN = canRxMessage(CAN_RX_FIFO0, &pRxHeader, settingRxBuff);
			if (errCodeRxCAN == HAL_OK) {
				if (pRxHeader.ExtId == RX_SETTING_MODE) {
					switchCanCMD = ((strSETTING_CMD*) settingRxBuff)->CMD;
				}
			}

			switch (switchCanCMD) {
			case WAITING:{

			}

				break;
			case EXIT_SETTING_MODE:{
				uint8_t writeFlag = sensorData.LoadSensData != oldSensorData.LoadSensData
						|| sensorData.OffsetLoadSensData != oldSensorData.OffsetLoadSensData
						|| sensorData.OffsetUnloadSensData != oldSensorData.OffsetUnloadSensData
						|| sensorData.PeriodTransmitType != oldSensorData.PeriodTransmitType
						|| sensorData.TxCanID.CanID != oldSensorData.TxCanID.CanID
						|| sensorData.UnloadSensData != oldSensorData.UnloadSensData
						|| sensorData.WeightMax != oldSensorData.WeightMax
						|| sensorData.CountFltAvrLoad != oldSensorData.CountFltAvrLoad;

				if (writeFlag) {
					writeSensorData(&sensorData);
				}
				switchMain = WORK;
				settingTxBuff.CMD = EXIT_SETTING_MODE;
			}
				break;
			case PERIOD_TRANSMIT:
				sensorData.PeriodTransmitType = ((strSETTING_SET_PERIOD*) settingRxBuff)->TYPE_PERIOD;

				periodTransmit = getPeriodTransmit(&sensorData.PeriodTransmitType);
				settingTxBuff.CMD = PERIOD_TRANSMIT;
				switchCanCMD = WAITING;
				break;
			case CHANGE_CANID:{
				uint32_t tempId = ((strSETTING_SET_CANID*) settingRxBuff)->CanID;
				sensorData.TxCanID.CanID = ((tempId & 0xFF) << 24) | ((tempId & 0xFF00) << 8) | ((tempId & 0xFF0000) >> 8) | ((tempId & 0xFF000000) >> 24);
				settingTxBuff.CMD = CHANGE_CANID;
				switchCanCMD = WAITING;
			}
				break;
			case SET_WITHOUT_LOAD:
				for (uint8_t i = 0; i < SEMPLING_SENSDATA_LOAD_UNLOAD; ++i) {
					sensorData.UnloadSensData = outCanData.Data.DataRaw;
				}

				settingTxBuff.CMD = SET_WITHOUT_LOAD;
				switchCanCMD = WAITING;
				break;
			case SET_WITH_LOAD:
				for (uint8_t i = 0; i < SEMPLING_SENSDATA_LOAD_UNLOAD; ++i) {
					sensorData.LoadSensData = outCanData.Data.DataRaw;
				}

				settingTxBuff.CMD = SET_WITH_LOAD;
				switchCanCMD = WAITING;
				break;
			case SET_OFFSET_LOAD: {
				int16_t offset = ((strSETTING_SET_OFFSET_SENS_DATA*) settingRxBuff)->Offset;
				if (offset > MIN_OFFSET_SENS_DATA && offset < MAX_OFFSET_SENS_DATA) {
					sensorData.OffsetLoadSensData = offset;
					settingTxBuff.CMD = SET_WITHOUT_LOAD;
				} else {
					settingTxBuff.CMD = ERROR_CMD;
				}

				switchCanCMD = WAITING;
			}
				break;
			case SET_OFFSET_UNLOAD: {
				int16_t offset = ((strSETTING_SET_OFFSET_SENS_DATA*) settingRxBuff)->Offset;
				if (offset > MIN_OFFSET_SENS_DATA && offset < MAX_OFFSET_SENS_DATA) {
					sensorData.OffsetUnloadSensData = offset;
					settingTxBuff.CMD = SET_WITH_LOAD;
				} else {
					settingTxBuff.CMD = ERROR_CMD;
				}

				switchCanCMD = WAITING;
			}
				break;
			case SET_MAX_WEIGHT: {
				uint16_t weight = ((strSETTING_SET_WEIGHT*) settingRxBuff)->Weight;
				if (weight > MIN_OFFSET_WEIGHT && weight < MAX_OFFSET_WEIGHT) {
					sensorData.WeightMax = weight;
					settingTxBuff.CMD = SET_WITH_LOAD;
				} else {
					settingTxBuff.CMD = ERROR_CMD;
				}

				settingTxBuff.CMD = SET_WITHOUT_LOAD;

				switchCanCMD = WAITING;
			}
				break;
			case SET_COUNT_FLT_AVR_LOAD: {
				uint8_t countFltAvrLoad = ((strSETTING_FLT_AVR*) settingRxBuff)->COUNT_FLT_AVR_LOAD;

				sensorData.CountFltAvrLoad = countFltAvrLoad;
				setCountFltAvrLoad(sensorData.CountFltAvrLoad);
				settingTxBuff.CMD = SET_COUNT_FLT_AVR_LOAD;
				switchCanCMD = WAITING;
			}
				break;
			case SET_DEFAULT_VALUES:{
				setDefaultValues(&sensorData);
				settingTxBuff.CMD = SET_DEFAULT_VALUES;
				switchCanCMD = WAITING;
			}
			break;
			default:
				settingTxBuff.CMD = ERROR_CMD;
				switchCanCMD = WAITING;
				break;
			}

			if (flagSend) {
				errCodeTxCAN = canTXMessage(CAN_EXT, TX_SETTING_MODE, 0, 8, (uint8_t*) &settingTxBuff);
			}
			break;
		}

		if (flagSend){
			uint8_t flagErrPresent = errorReadFlashSensorData || getTempFlagErrorsPresent() || getErrCommLoadAmp();
			if (flagErrPresent){
				sendErrors.Data.readFlashSensorData = errorReadFlashSensorData;
				sendErrors.Data.tempSRCHi = getTempErrSRCHi();
				sendErrors.Data.tempSRCLow = getTempErrSRCLow();
				sendErrors.Data.commLoadAmp = getErrCommLoadAmp();
				errCodeTxCAN = canTXMessage(CAN_EXT, TX_ERROR, 0, 8, sendErrors.RawData);
			}
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t calculationWeight(int16_t rawSensData, strSensorData sensorData) {
	int16_t deltaSensDataMax = sensorData.LoadSensData - sensorData.UnloadSensData;

	if (deltaSensDataMax < 0) {
		deltaSensDataMax = -deltaSensDataMax;
	}

	int16_t actDeltaSensData = rawSensData - sensorData.UnloadSensData;

	if (actDeltaSensData < 0) {
		actDeltaSensData = -actDeltaSensData;
	}

	int16_t tresholdMin = sensorData.OffsetUnloadSensData;
	int16_t tresholdMax = deltaSensDataMax - sensorData.OffsetLoadSensData;

	int16_t output = (int16_t) ((int32_t) sensorData.WeightMax * (actDeltaSensData - tresholdMin) / (tresholdMax - tresholdMin));

	if (output < 0) {
		output = 0;
	}

	return output;
}

void writeSensorData(strSensorData *sensorData) {
	strWriteFlashData writeSensData;
	writeSensData.Data.SensData = *sensorData;

	uint8_t *dataPointer = (uint8_t*) &writeSensData;
	uint8_t checkSumm = 0;

	for (uint32_t i = 0; i < (SIZE_FLASH_WRITE_DATA - 1); i++) {
		checkSumm += ((uint8_t*) dataPointer)[i] * 41381;
	}
	writeSensData.RawData[SIZE_FLASH_WRITE_DATA - 1] = checkSumm;

	flash_erase();
	flash_program(PARAMETERS_PAGE_ADDRESS, (uint32_t*) (&writeSensData), SIZE_FLASH_WRITE_DATA);
}

uint8_t readSensorData(strSensorData *sensorData) {
	strWriteFlashData *tmp = (strWriteFlashData*) PARAMETERS_PAGE_ADDRESS;
	uint8_t *dataPointer = (uint8_t*) PARAMETERS_PAGE_ADDRESS;
	uint8_t checkSumm = 0;

	for (uint32_t i = 0; i < (SIZE_FLASH_WRITE_DATA - 1); i++) {
		checkSumm += ((uint8_t*) dataPointer)[i] * 41381;
	}

	if (tmp->RawData[SIZE_FLASH_WRITE_DATA - 1] == checkSumm) {
		*sensorData = tmp->Data.SensData;
		return 0;
	} else {
		return 1;
	}
}

uint16_t getPeriodTransmit(EnumPeriodTransmitType *periodType) {
	switch (*periodType) {
	case PER_10ms:
		return 10;
		break;
	case PER_20ms:
		return 20;
		break;
	case PER_50ms:
		return 50;
		break;
	case PER_100ms:
		return 100;
		break;
	case PER_1000ms:
		return 1000;
		break;
	case PER_5000ms:
		return 5000;
		break;
	default:
		return 50;
		break;
	}
}

void setDefaultValues(strSensorData* sensorData){
	sensorData->TxCanID.CanID = 0x1CE75516;
	sensorData->WeightMax = 1000;
	sensorData->OffsetLoadSensData = 400;
	sensorData->OffsetUnloadSensData = 300;
	sensorData->LoadSensData = 8300;
	sensorData->UnloadSensData = -735;
	sensorData->CountFltAvrLoad = 2;
	sensorData->PeriodTransmitType = 1;
}

uint8_t countErr;
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
		countErr++;
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
