/*
 * radio.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: RicardoC
 */

#include "radio.h"

uint8_t end_tx_msg[] = "The end of transmition was detected \r\n";
uint8_t end_rx_msg[] = "The end of reception was detected \r\n";


/* Automatically send data definitions */
uint8_t tx_buffer[] = "Welcome to EMBeddedRick \r\n";


lr11xx_system_irq_mask_t radio_irq = LR11XX_SYSTEM_IRQ_NONE;

LR11xx::LR11xx(const char* CONTEXT){
	if(strcmp(CONTEXT, UHF_CONTEXT)==0){
		lr11xx_ctx.hspi = &hspi2;
		lr11xx_ctx.nss_port = NSS_UHF_GPIO_Port;
		lr11xx_ctx.rst_port = NRESET_UHF_GPIO_Port;
		lr11xx_ctx.bsy_port = BUSY_UHF_GPIO_Port;
		lr11xx_ctx.nss_pin = NSS_UHF_Pin;
		lr11xx_ctx.rst_pin = NRESET_UHF_Pin;
		lr11xx_ctx.bsy_pin = BUSY_UHF_Pin;
	}else if(strcmp(CONTEXT, SBAND_CONTEXT)==0){
		lr11xx_ctx.hspi = &hspi2;
		lr11xx_ctx.nss_port = NSS_SB_GPIO_Port;
		lr11xx_ctx.rst_port = NRESET_SB_GPIO_Port;
		lr11xx_ctx.bsy_port = BUSY_SB_GPIO_Port;
		lr11xx_ctx.nss_pin = NSS_SB_Pin;
		lr11xx_ctx.rst_pin = NRESET_SB_Pin;
		lr11xx_ctx.bsy_pin = BUSY_SB_Pin;
	}
}

/**
  * @brief DIO9 RF callback function (RF reception and transmission completion interrupt service function)
  * @param GPIO_Pin Pin number.
  * @retval None
  */
void LR11xx::radio_irq_callback( void )
{
	//lr11xx_system_irq_mask_t radio_irq = LR11XX_SYSTEM_IRQ_NONE;
	if(lr11xx_system_get_irq_status((void*)&lr11xx_ctx ,&radio_irq) != LR11XX_STATUS_OK )
	{
		HAL_UART_Transmit(&huart1, (const uint8_t *)"get_irq_status failed\r\n", sizeof("get_irq_status failed\r\n"), 100);
	}
	/* LR1121 Timeout judgment */
	if(( radio_irq & LR11XX_SYSTEM_IRQ_TIMEOUT ) == LR11XX_SYSTEM_IRQ_TIMEOUT)
	{
		/* LR1121 clear interrupt status*/
		lr11xx_system_clear_irq_status( (void*)&lr11xx_ctx, radio_irq );
		/* LR1121 Enter to receive */
		radio_rx();
	}
	else if(radio_irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR)
	{
		HAL_UART_Transmit(&huart1, (const uint8_t *)"header_error\r\n", sizeof("header_error\r\n"), 100);
		lr11xx_system_clear_irq_status( (void*)&lr11xx_ctx, radio_irq );
		radio_rx();
	}
	else if(radio_irq & LR11XX_SYSTEM_IRQ_CRC_ERROR)
	{
		HAL_UART_Transmit(&huart1, (const uint8_t *)"crc_error\r\n", sizeof("crc_error\r\n"), 100);
		lr11xx_system_clear_irq_status( (void*)&lr11xx_ctx, radio_irq );
		radio_rx();
	}
  /* LR1121 Reception completed */
	else if((radio_irq & LR11XX_SYSTEM_IRQ_RX_DONE) == LR11XX_SYSTEM_IRQ_RX_DONE)
  {
		 /* LR1121 Get received packet status and parameters */
		 lr11xx_radio_get_rx_buffer_status( (void*)&lr11xx_ctx, &rx_buffer_status );
		 rx_buffer_lenght = rx_buffer_status.pld_len_in_bytes;						// Your rx_buffer_len is here!!
		 /* LR1121 read FIFO Data received to rx_buffer*/
		 lr11xx_regmem_read_buffer8( (void*)&lr11xx_ctx, rx_buffer, 				// Your rx_buffer is here!!
									rx_buffer_status.buffer_start_pointer,
									rx_buffer_status.pld_len_in_bytes );
		 HAL_UART_Transmit(&huart1, end_rx_msg , sizeof(end_rx_msg), 100);
		 /* Debug interface */
		 //HAL_UART_Transmit(&huart1, rx_buffer , rx_buffer_status.pld_len_in_bytes, 100);
		 //memset(rx_buffer,0,sizeof(rx_buffer));

		 /* LR1121 clear interrupt status */
		 lr11xx_system_clear_irq_status( (void*)&lr11xx_ctx, radio_irq );
		 rx_done = true; // reception flag up
		 /* LR1121 Enter to receive */
		 radio_rx();
  }
	/* LR1121 Launch completed */
	else if((radio_irq & LR11XX_SYSTEM_IRQ_TX_DONE) == LR11XX_SYSTEM_IRQ_TX_DONE)
	{
		HAL_UART_Transmit(&huart1, end_tx_msg , sizeof(end_tx_msg), 100);
		/* LR1121 clear interrupt status */
		lr11xx_system_clear_irq_status( (void*)&lr11xx_ctx, radio_irq );
		//HAL_UART_Transmit(&huart1, (const uint8_t *)"TX DONE\r\n", sizeof("TX DONE\r\n"), 100);
		tx_done = true; // transmition flag up
		/* LR1121 Enter to receive */
		radio_rx();
	}
	/* LR1121 Other interrupt events*/
	else
	{
		radio_rx();
	}
}

void LR11xx::radio_init( lr11xx_pa_type_t pa_type, int8_t power, uint32_t freq)
{
	uint16_t errors;
	/* LR1121 reset */
  lr11xx_hal_reset((void*)&lr11xx_ctx);
	/* LR1121 wakeup */
	lr11xx_hal_wakeup((void*)&lr11xx_ctx);
	/* LR1121 进入standby */
  lr11xx_system_set_standby((void*)&lr11xx_ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);
	/* LR1121 配置模式DC-DC */
	lr11xx_system_set_reg_mode((void*)&lr11xx_ctx ,LR11XX_SYSTEM_REG_MODE_DCDC);
	/* LR1121 配置spi CRC 关闭 */
	lr11xx_system_enable_spi_crc((void*)&lr11xx_ctx,false);
	/* LR1121 配置LR1121 射频开关映射 */
	lr11xx_system_set_dio_as_rf_switch( (void*)&lr11xx_ctx , &system_rf_switch_cfg );
  /* LR1121 配置TCXO供电电压和检测超时 */
	lr11xx_system_set_tcxo_mode( (void*)&lr11xx_ctx, LR11XX_SYSTEM_TCXO_CTRL_1_8V, 320 );
	/* LR1121 配置低频(LF)时钟 */
	lr11xx_system_cfg_lfclk( (void*)&lr11xx_ctx, LR11XX_SYSTEM_LFCLK_XTAL, true );
	/* LR1121 清除错误状态 */
	lr11xx_system_clear_errors( (void*)&lr11xx_ctx );
	/* LR1121 系统校准 */
	lr11xx_system_calibrate( (void*)&lr11xx_ctx, 0x3F );
	/* LR1121 获取错误状态 */
	lr11xx_system_get_errors( (void*)&lr11xx_ctx, &errors );

	if(errors & LR11XX_SYSTEM_ERRORS_IMG_CALIB_MASK)
	{
		HAL_UART_Transmit(&huart1, (const uint8_t *)"calibrate err\r\n" ,sizeof("calibrate err\r\n") , 100);
	}
	/* LR1121 清除错误状态 */
	lr11xx_system_clear_errors( (void*)&lr11xx_ctx ) ;
	/* LR1121 清除irq标志 */
	lr11xx_system_clear_irq_status( (void*)&lr11xx_ctx, LR11XX_SYSTEM_IRQ_ALL_MASK );
	/* LR1121 配置射频数据调制类型（LoRa） */
	lr11xx_radio_set_pkt_type((void*)&lr11xx_ctx , LR11XX_RADIO_PKT_TYPE_LORA);
	/* SPI 读写检查 */
	lr11xx_radio_pkt_type_t spi_check;
	lr11xx_radio_get_pkt_type((void*)&lr11xx_ctx,&spi_check);
	if(spi_check != LR11XX_RADIO_PKT_TYPE_LORA)
	{
		HAL_UART_Transmit(&huart1, (const uint8_t *)"spi_check_err\r\n" ,sizeof("spi_check_err\r\n") , 100);
	}
	/* LR1121 配置射频数据包参数 */
	lr11xx_radio_set_lora_pkt_params( (void*)&lr11xx_ctx , &radio_pkt_params );
	/* LR1121 配置射频同步字参数 */
	lr11xx_radio_set_lora_sync_word((void*)&lr11xx_ctx,SYNC_WORD_NO_RADIO);
	/* LR1121 配置射频数据调制参数 */
	lr11xx_radio_set_lora_mod_params((void*)&lr11xx_ctx ,&mod_params);

	/* 2.4G LoRa 配置*/
	if(freq >= 2400000000)
	{
		/* LR1121 配置射频工作频点 */
		lr11xx_radio_set_rf_freq((void*)&lr11xx_ctx,freq);
		/* LR1121 获取射频发射配置参数 */
		lr11xx_get_tx_cfg( LR11XX_WITH_HF_PA, power, &output_params );
		/* LR1121 射频发射配置参数 */
		lr11xx_radio_set_pa_cfg( (void*)&lr11xx_ctx, &output_params.pa_cfg );
	}
	/* SUB-G LoRa 配置*/
	else
	{
		/* LR1121 配置射频工作频点 */
		lr11xx_radio_set_rf_freq((void*)&lr11xx_ctx,freq);
		/* 高功率PA 配置*/
		if(pa_type == LR11XX_WITH_LF_HP_PA)
		{
			/* LR1121 获取射频发射配置参数 */
			lr11xx_get_tx_cfg( LR11XX_WITH_LF_HP_PA, power, &output_params );
			/* LR1121 射频发射配置参数 */
			lr11xx_radio_set_pa_cfg( (void*)&lr11xx_ctx, &output_params.pa_cfg );
			lr11xx_radio_set_tx_params( (void*)&lr11xx_ctx ,power ,LR11XX_RADIO_RAMP_48_US);
		}
		/* 低功率PA 配置*/
		else
		{
			/* LR1121 获取射频发射配置参数 */
			lr11xx_get_tx_cfg( LR11XX_WITH_LF_LP_PA, power, &output_params );
			/* LR1121 射频发射配置参数 */
			lr11xx_radio_set_pa_cfg( (void*)&lr11xx_ctx, &output_params.pa_cfg );
		}
	}
	/* LR1121 模式自动退回配置 */
	lr11xx_radio_set_rx_tx_fallback_mode( (void*)&lr11xx_ctx,  LR11XX_RADIO_FALLBACK_STDBY_RC);
	/* LR1121 在接收中配置boost模式 允许增加~2dB的灵敏度 代价是~2mA更高的电流消耗在RX模式 默认开启*/
	lr11xx_radio_cfg_rx_boosted( (void*)&lr11xx_ctx , true );
}

/**
  * @brief 射频进入接收
  * @param None
  * @retval None
  */
void LR11xx::radio_rx( void )
{
	/* LR1121 配置射频数据包长度 */
	radio_pkt_params.pld_len_in_bytes = 255;
	/* LR1121 配置射频数据长度 */
	lr11xx_radio_set_lora_pkt_params( (void*)&lr11xx_ctx, &radio_pkt_params );
	/* LR1121 配置DIO9中断 */
	lr11xx_system_set_dio_irq_params((void*)&lr11xx_ctx,LR11XX_SYSTEM_IRQ_RX_DONE |
												LR11XX_SYSTEM_IRQ_TIMEOUT |
												LR11XX_SYSTEM_IRQ_HEADER_ERROR |
												LR11XX_SYSTEM_IRQ_CRC_ERROR ,
												LR11XX_SYSTEM_IRQ_NONE);
	/* LR1121 清除DIO9中断状态 */
	lr11xx_system_clear_irq_status( (void*)&lr11xx_ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
	/* LR1121 进入接收模式 */
	lr11xx_radio_set_rx((void*)&lr11xx_ctx, 0); // zero means: Set device in Rx mode until reception occurs.
}

/**
  * @brief 射频发送
  * @param None
  * @retval None
  */
void LR11xx::radio_tx_auto(void)
{
	/* LR1121 配置DIO9中断 */
	lr11xx_system_set_dio_irq_params((void*)&lr11xx_ctx, LR11XX_SYSTEM_IRQ_TX_DONE, LR11XX_SYSTEM_IRQ_NONE);
	/* LR1121 清除DIO9中断状态 */
	lr11xx_system_clear_irq_status( (void*)&lr11xx_ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
	/* LR1121 获取需要发射数据长度 */
	radio_pkt_params.pld_len_in_bytes = sizeof(tx_buffer);
	/* LR1121 配置发射数据长度 */
	lr11xx_radio_set_lora_pkt_params( (void*)&lr11xx_ctx, &radio_pkt_params );
	/* 写入数据到LR1121到FIFO */
	lr11xx_regmem_write_buffer8( (void*)&lr11xx_ctx , tx_buffer, sizeof(tx_buffer));
	/* LR1121 开始发送 */
	lr11xx_radio_set_tx((void*)&lr11xx_ctx,0); // zero means: Set device in Rx mode until reception occurs.
}

/**
  * @brief 射频自定义发送API
  * @param uint8_t *buffer：指向发送数据
  * @param uint8_t lenght： 数据长度
  * @retval None
  */
void LR11xx::radio_tx_custom(uint8_t *buffer, uint8_t lenght)
{
	/* LR1121 配置发射数据长度 */
	lr11xx_system_set_dio_irq_params((void*)&lr11xx_ctx,LR11XX_SYSTEM_IRQ_TX_DONE, LR11XX_SYSTEM_IRQ_NONE);
	/* LR1121 清除DIO9中断状态 */
	lr11xx_system_clear_irq_status( (void*)&lr11xx_ctx,  LR11XX_SYSTEM_IRQ_ALL_MASK);
	/* LR1121 获取需要发射数据长度 */
	radio_pkt_params.pld_len_in_bytes = lenght;
	/* LR1121 配置发射数据长度 */
	lr11xx_radio_set_lora_pkt_params( (void*)&lr11xx_ctx, &radio_pkt_params );
	/* LR1121 配置发射数据长度 */
	lr11xx_regmem_write_buffer8( (void*)&lr11xx_ctx , buffer, lenght);
	/* LR1121 开始发送 */
	lr11xx_radio_set_tx((void*)&lr11xx_ctx,0);
}

/**
  * @brief 射频进入低功耗
  * @param None
  * @retval None
  */
void LR11xx::radio_sleep(void)
{
	/* LR1121 进入standby */
	lr11xx_system_set_standby((void*)&lr11xx_ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);
	/* LR1121 配置休眠时DIO状态 */
	lr11xx_system_drive_dio_in_sleep_mode((void*)&lr11xx_ctx,true);
	/* LR1121 进入休眠模式 */
	lr11xx_system_set_sleep(NULL,sleep_cfgs,0);
}

/**
  * @brief 射频唤醒
  * @param None
  * @retval None
  */
void LR11xx::radio_wakeup(void)
{
	/* LR1121 休眠唤醒 */
	lr11xx_hal_wakeup((void*)&lr11xx_ctx);
}

uint16_t LR11xx::computeChecksum(uint16_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];  // XOR each byte
    }
    return checksum;
}

uint16_t LR11xx::verifyChecksum(uint16_t *data, size_t length, uint8_t chksm) {
    uint8_t computedChecksum = computeChecksum(data, length);
    return (computedChecksum ^ chksm) == 0;  // Should be 0 if correct
}

