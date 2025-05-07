/*
 * radio.h
 *
 *  Created on: Apr 24, 2025
 *      Author: RicardoC
 */

#ifndef KIM_SDK_LR211_DRIVER_RADIO_H_
#define KIM_SDK_LR211_DRIVER_RADIO_H_

#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "stdbool.h"

#include "lr11xx_hal.h"
#include "lr11xx_system.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_types.h"
#include "lr11xx_radio.h"
#include "event.h"
#include "string.h"

#define SYNC_WORD_NO_RADIO  0x12
#define WIFI_WORD_NO_RADIO  0x21

#define UHF_CONTEXT 	"LR11xx_UHF"
#define SBAND_CONTEXT 	"LR11xx_SB"
#define E80_CONTEXT		"LR1121"

#define IRQ_MASK                                                                          \
    ( LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT | \
      LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_FSK_LEN_ERROR )

class LR11xx{
	public:
		/* Receive data buffer */
		uint8_t rx_buffer[255] = {0};
		uint8_t rx_buffer_lenght = 0;
		bool tx_done = false;
		bool rx_done = false;
	public:
		LR11xx(const char* CONTEXT);
		void radio_tx_auto( void );
		void radio_tx_custom(uint8_t *buffer, uint8_t lenght);
		void radio_rx( void );
		void radio_sleep(void);
		void radio_wakeup(void);
		void radio_init( lr11xx_pa_type_t pa_type, int8_t power, uint32_t freq);
		void radio_inits( lr11xx_pa_type_t pa_type,
											int8_t power, uint32_t freq,
											lr11xx_radio_lora_sf_t sf,
											lr11xx_radio_lora_bw_t bw,
											lr11xx_radio_lora_cr_t cr,
											uint8_t ldros, uint8_t synword);
		void radio_init_up(void);
		void radio_irq_callback( void );

		uint16_t computeChecksum(uint16_t* data, size_t length);
		uint16_t verifyChecksum(uint16_t *data, size_t length, uint8_t chksm);
	private:
		// ===================
		lr11xx_radio_rx_buffer_status_t rx_buffer_status;
		//====================
		lr11xx_hal_context_t lr11xx_ctx;
		/* Used to store RF transmission parameters (PA, power, etc.) */
		lr11xx_bsp_tx_cfg_output_params_t  output_params;
		/* Supply current in power down mode */
		lr11xx_system_sleep_cfg_t sleep_cfgs =
		{
			.is_warm_start  = false,
			.is_rtc_timeout = false
		};
		// ====================RADIO DEPENDANT==================================
		lr11xx_system_rfswitch_cfg_t   system_rf_switch_cfg =
		{
			.enable  = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH | LR11XX_SYSTEM_RFSW2_HIGH | LR11XX_SYSTEM_RFSW3_HIGH,
			.standby = 0,
			.rx      = LR11XX_SYSTEM_RFSW1_HIGH,
			.tx      = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH,
			.tx_hp   = LR11XX_SYSTEM_RFSW0_HIGH,
			.tx_hf   = 0,
			.gnss    = LR11XX_SYSTEM_RFSW2_HIGH,
			.wifi    = LR11XX_SYSTEM_RFSW3_HIGH
		};
		/* RF parameter configuration */
		lr11xx_radio_pkt_params_lora_t  radio_pkt_params =
		{
			.preamble_len_in_symb = 8,
			.header_type = LR11XX_RADIO_LORA_PKT_EXPLICIT,
			.pld_len_in_bytes = 255,
			.crc = LR11XX_RADIO_LORA_CRC_ON,
			.iq = LR11XX_RADIO_LORA_IQ_STANDARD
		};
		/* LoRa Parameter configuration */
		lr11xx_radio_mod_params_lora_t  mod_params =
		{
			.sf   = LR11XX_RADIO_LORA_SF8,
			.bw   = LR11XX_RADIO_LORA_BW_125,
			.cr   = LR11XX_RADIO_LORA_CR_4_5,
			.ldro = 0
		};
};



#endif /* KIM_SDK_LR211_DRIVER_RADIO_H_ */
