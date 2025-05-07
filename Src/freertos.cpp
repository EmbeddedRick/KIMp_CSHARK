/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include "radio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_PREFIX_SPARE_MSK1	0xC300//0b1100001100000000
#define CMD_PREFIX_SPARE_MSK2  	0xC030//0b1100000000110000
#define RSP_PREFIX_SPARE_MSK   	0xC3F0//0b110000111111110000
#define CMD_EXPECTED_VAL1      	0x4300
#define CMD_EXPECTED_VAL2      	0x4030
#define RSP_EXPECTED_VAL       	0x83C0
#define HEADER_LEN	(2)    	   	// To all subsystems, two 16bit header
// __________________KIM DEPENDANT_________________________ //
#define TCQUEUE_OBJECTS 			(5)  // Telecommand Queue
#define TMQUEUE_OBJECTS 			(10) // Telemetry Queue

#define MAX_TX_POWER_SBD			(13) // Maximum PWR for Freqs around 2.4GHz
#define MAX_TX_POWER_SUB2G4			(22) // Maximum PWR for Freqs under 2.4GHz

#define MSK_UHF_FLAGS				0x00000001U
#define MSK_SBD_FLAGS				0x00000010U

#define ID_FIELD_OFFSET				(11)
#define THIS_PILOT_ID				(105)
#define SIDE_PNL_ADDR				(20)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t msg_IRQSB[] = "An interrupt in UHF was detected \r\n";
uint8_t msg_IRQUHF[] = "An interrupt in SBand was detected \r\n";

/* BEACON message */
uint8_t beacon[] = "HelloThere from CSHARK";

uint32_t _2G4Hz = 2400000000; // 2.4GHz band
uint32_t _0G4Hz = 400000000;  // 400MHz band

LR11xx uhf(UHF_CONTEXT);
LR11xx sbd(SBAND_CONTEXT);

typedef enum{
	UHF_BAND = 0,
	S_BAND = 1,
} FrequencyBand_e;

typedef struct{
	FrequencyBand_e freq;
	uint8_t length;
	uint8_t buffer[255];
} telepkt_t;

enum REG_MAP_e{
  STATUS = 0,
  TC_REQ = 1,
  UHF_DLNK,
  SBAND_DLNK,
  HOUSEKPG,
  SCHEDULE,
};

typedef enum{
	CMD_READ = 0,
	CMD_WRITE = 1,
	CMD_ERROR = 2
} HEADER_CMD_e;

uint16_t* ptr = NULL;
uint8_t len;
uint8_t register_map;
uint8_t register_index;
uint16_t comd_token[2];
uint16_t resp_token[2];
HEADER_CMD_e header_state;
bool obc_cmdWrite_flag = false;

/* USER CODE END Variables */
/* Definitions for UHF_Task */
osThreadId_t UHF_TaskHandle;
const osThreadAttr_t UHF_Task_attributes = {
  .name = "UHF_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SBAND_Task */
osThreadId_t SBAND_TaskHandle;
const osThreadAttr_t SBAND_Task_attributes = {
  .name = "SBAND_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
osMessageQueueId_t TelemQueueHandle;
osMessageQueueId_t TelecmdQueueHandle;
osEventFlagsId_t frequencyFlagsHandle;
osMutexId_t spi_mutex;
void sendToSidePnl(uint8_t* pss_buf, uint8_t pss_length);
/* USER CODE END FunctionPrototypes */

void StartUhfTask(void *argument);
void StartSbandTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	/* creation of TelecmdQueue */
	TelecmdQueueHandle = osMessageQueueNew (TCQUEUE_OBJECTS, sizeof(telepkt_t),NULL);
	/* creation of UHFTelemQueue */
	TelemQueueHandle = osMessageQueueNew (TMQUEUE_OBJECTS, sizeof(telepkt_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UHF_Task */
  UHF_TaskHandle = osThreadNew(StartUhfTask, NULL, &UHF_Task_attributes);

  /* creation of SBAND_Task */
  SBAND_TaskHandle = osThreadNew(StartSbandTask, NULL, &SBAND_Task_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  frequencyFlagsHandle = osEventFlagsNew(NULL);
  /* USER CODE END RTOS_EVENTS */

}

// Add a main thread, this thread is used for beaconing and handle some
// other commands from OBC.

void StartUhfTask(void *argument)
{
	uhf.radio_init(LR11XX_WITH_LF_LP_PA, MAX_TX_POWER_SUB2G4, _0G4Hz);
	for(;;)
	{
		osEventFlagsWait(frequencyFlagsHandle, MSK_UHF_FLAGS, osFlagsWaitAny, osWaitForever);
		osEventFlagsClear(frequencyFlagsHandle, MSK_UHF_FLAGS);
		// check if the interrupt was asserted by and error, if it is continue
		osMutexAcquire(spi_mutex, osWaitForever); // take mutex
		uhf.radio_irq_callback();
		if(uhf.rx_done)
		{
			uhf.rx_done = false;
			/* Analyze the packet, for this follow the steps:
			 * 		1. Check CRC (optional, acceptable LoRa packets passed its own CRC already)
			 * 		2. Check SATELLITE ID field, if it doesn't match, deliver to Side Panel and continue
			 * 		3. If matches, add the message to telecommand queue
			 * */
			uint16_t sat_id = uhf.rx_buffer[ID_FIELD_OFFSET]<<8 || uhf.rx_buffer[ID_FIELD_OFFSET+1];
			if(sat_id != THIS_PILOT_ID){
				sendToSidePnl(uhf.rx_buffer, uhf.rx_buffer_lenght);
				memset(uhf.rx_buffer, 0, sizeof(uhf.rx_buffer));
				// ======================================
				uint8_t buf_sent[] = "ACK";
				uhf.radio_tx_custom(buf_sent, sizeof(buf_sent));		// Directly downlink, no queue
				continue;
			}
			telepkt_t valid_msg;
			valid_msg.freq = UHF_BAND;									// Working frequency
			valid_msg.length = uhf.rx_buffer_lenght; 					// Length of the packet payload
			memcpy(valid_msg.buffer, uhf.rx_buffer, valid_msg.length);	// Packet payload buffer
			osMessageQueuePut(TelecmdQueueHandle, &valid_msg, 0U, 0U);
			memset(uhf.rx_buffer,0, 255);
		}else if(uhf.tx_done)
		{
			uhf.tx_done = false;
		}
		osMutexRelease(spi_mutex); // free the mutex
		osDelay(1);
	}
}

void StartSbandTask(void *argument)
{
	sbd.radio_init(LR11XX_WITH_HF_PA, MAX_TX_POWER_SBD, _2G4Hz);
	for(;;)
	{
		osEventFlagsWait(frequencyFlagsHandle, MSK_SBD_FLAGS, osFlagsWaitAny, osWaitForever);
		osEventFlagsClear(frequencyFlagsHandle, MSK_SBD_FLAGS);
		// check if the interrupt was asserted by and error, if it is continue
		osMutexAcquire(spi_mutex, osWaitForever); // take mutex
		sbd.radio_irq_callback();
		if(sbd.rx_done)
		{
			sbd.rx_done = false;
			/* Analyze the packet, for this follow the steps:
			 * 		1. Check CRC (optional, acceptable LoRa packets passed its own CRC already)
			 * 		2. Check SATELLITE ID field, if it doesn't match, deliver to Side Panel and continue
			 * 		3. If matches, add the message to telecommand queue
			 * */
			uint16_t sat_id = sbd.rx_buffer[ID_FIELD_OFFSET]<<8 || sbd.rx_buffer[ID_FIELD_OFFSET+1];
			if(sat_id != THIS_PILOT_ID){
				sendToSidePnl(sbd.rx_buffer, sbd.rx_buffer_lenght);
				memset(sbd.rx_buffer, 0, sizeof(uhf.rx_buffer));
				// ======================================
				uint8_t buf_sent[] = "ACK";
				sbd.radio_tx_custom(buf_sent, sizeof(buf_sent));		// Directly downlink, no queue
				continue;
			}
			telepkt_t valid_msg;
			valid_msg.freq = S_BAND;									// Working frequency
			valid_msg.length = sbd.rx_buffer_lenght; 					// Length of the packet payload
			memcpy(valid_msg.buffer, sbd.rx_buffer, valid_msg.length);	// Packet payload buffer
			osMessageQueuePut(TelecmdQueueHandle, &valid_msg, 0U, 0U);
			memset(sbd.rx_buffer,0, 255);
		}else if(sbd.tx_done)
		{
			sbd.tx_done = false;
		}
		osMutexRelease(spi_mutex); // free the mutex
		osDelay(1);
	}
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == IRQ_UHF_Pin)
	{
		HAL_UART_Transmit(&huart1, msg_IRQUHF , sizeof(msg_IRQUHF), 100);
		osEventFlagsSet(frequencyFlagsHandle, MSK_UHF_FLAGS); // Set the thread in Ready state
	}else if(GPIO_Pin == IRQ_SB_Pin)
	{
		HAL_UART_Transmit(&huart1, msg_IRQSB , sizeof(msg_IRQSB), 100);
		osEventFlagsSet(frequencyFlagsHandle, MSK_SBD_FLAGS); // Set the thread in Ready state
	} else if(GPIO_Pin == SPI_NSS_Pin)
	{ // SPI_NSS is the CS pin as Falling detection interrupt
		HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)resp_token, (uint8_t*)comd_token, HEADER_LEN);
	}
}

HEADER_CMD_e analyzeCmdToken(uint16_t* buf)
{
  if((buf[0]&CMD_PREFIX_SPARE_MSK1)!=CMD_EXPECTED_VAL1
  || (buf[1]&CMD_PREFIX_SPARE_MSK2)!=CMD_EXPECTED_VAL2){
    resp_token[0] |=0x1000;//0b0001000000000000 -> Message error flag
    return CMD_ERROR;
  }
  if(!uhf.verifyChecksum(buf, 4, buf[1]&0x0F)){ // There are 4 bytes (two 16bit words)
    resp_token[0] |=0x1000;//0b0001000000000000 -> Message error flag
    return CMD_ERROR;
  }
  len = buf[0]&0xFF;                // Message length field
  register_map = 	(buf[0]>>10)&0x07; // Command token, ignore the R/W bit
  register_index = 	(buf[1]>>6)&0xFF;// Sub-address field
  // update flags reception correct!
  resp_token[0] &= 	~0x1000;//0b0001000000000000 -> Message error flag
  return static_cast<HEADER_CMD_e>((buf[0] >> 13) & 0x1);
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
	telepkt_t msg;
  if(!obc_cmdWrite_flag){
    HEADER_CMD_e action = analyzeCmdToken(comd_token);
    if(action == CMD_READ){
      if(register_map == TC_REQ){
    	  osMessageQueueGet(TelecmdQueueHandle, &msg, NULL, 0U);  	// Remember the buffer has the CRC included
    	  HAL_SPI_Transmit_DMA(&hspi1, msg.buffer, msg.length/2); 	// length is in BYTES, 1 word is 2 bytes
    }
    }else if(action == CMD_WRITE){
      ptr = (uint16_t*)malloc(len+1); // Sum 1 word for CRC
      HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)ptr, len+1);
      obc_cmdWrite_flag = true;
    }else{
        // Do something, probably nothing
    }
  }else{
	  // Here we write to KIM the downlink messages
    obc_cmdWrite_flag = false;
    if(register_map == UHF_DLNK){
    	telepkt_t valid_msg;
    	valid_msg.freq = UHF_BAND;						// Working frequency
    	valid_msg.length = len; 						// Length of the packet payload
    	memcpy(valid_msg.buffer,ptr, len);				// Packet payload buffer
    	osMessageQueuePut(TelemQueueHandle, &valid_msg, 0U, 0U);
    	free(ptr);
    }else if(register_map == SBAND_DLNK){
    	telepkt_t valid_msg;
    	valid_msg.freq = S_BAND;						// Working frequency
    	valid_msg.length = len; 						// Length of the packet payload
    	memcpy(valid_msg.buffer,ptr, len);				// Packet payload buffer
    	osMessageQueuePut(TelemQueueHandle, &valid_msg, 0U, 0U);
    	free(ptr);
    }
  }
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(ptr != NULL) free(ptr);
}

void sendToSidePnl(uint8_t* pss_buf, uint8_t pss_length){
	HAL_I2C_Master_Transmit(&hi2c1,SIDE_PNL_ADDR, pss_buf, pss_length, 1000);
}
/* USER CODE END Application */

