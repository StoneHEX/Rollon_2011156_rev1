/*
 * rollon.h
 *
 *  Created on: May 4, 2023
 *      Author: fil
 */

#ifndef ROLLON_APP_ROLLON_H_
#define ROLLON_APP_ROLLON_H_

/*
* Protocol
* ----------
* 	Baudrate 500kb/sec
* 	1 Start, 1 Stop, No Parity, 8bit Data
*
* 	<'%'><B/L><reg_add><size><data><data><...><chksum>
*
* 	Meaning:
*
* 	<'%'>		: Start Message
* 	<'B'>		: BME280 registers
* 	<'L'>		: LSM6 registers
* 	<reg_addr>	: Reg start address
* 	<size>		: Data size
* 	<data>		: The data from sensor
* 	<chksum>	: Checksum , xor'ed bytes from start of message, 8 bit
*/

/* lwip
 *
increase segment size TCP_MSS to 1460 (this is max for ethernet)
increase PBUF_POOL_BUFSIZE to 1600 (mss+headers+etc) Send buffer (TCP_SND_BUF) will increase automatically.
and MEM_SIZE to 16000 (it seems too high but trimming is a time consuming procedure. Make it after, when project is operative)

*/

extern	ADC_HandleTypeDef hadc1;
extern	DMA_HandleTypeDef hdma_adc1;
extern	CRC_HandleTypeDef hcrc;
extern	I2C_HandleTypeDef hi2c1;
extern	RTC_HandleTypeDef hrtc;
extern	SPI_HandleTypeDef hspi1;
extern	SPI_HandleTypeDef hspi2;
extern	TIM_HandleTypeDef htim1;
extern	TIM_HandleTypeDef htim4;
extern	TIM_HandleTypeDef htim6;
extern	TIM_HandleTypeDef htim7;
extern	TIM_HandleTypeDef htim8;
extern	TIM_HandleTypeDef htim9;
extern	UART_HandleTypeDef huart5;
extern	UART_HandleTypeDef huart7;

#define	STATUS_PWM_TIM_CHANNEL	TIM_CHANNEL_2
#define	POWER_PWM_TIM			htim1
#define	POWER_PWM1_PGEN_CHANNEL	TIM_CHANNEL_1
#define	POWER_PWM2_FGEN_CHANNEL	TIM_CHANNEL_2
#define	ADC_MONITOR_TIMER		htim6
#define	ADC_MONITOR				hadc1
#define	ADC_MONITOR_COVERSIONS	4
#define	BME280_I2C				hi2c1
#define	TIM_TICK_10MSEC			htim7
#define	UART					huart5
#define	UART_LSM_PACKET_LEN		17
#define	UART_SENSOR_PACKET_LEN	12
#define	UART_BUF_LEN			64

#define	EEPROM_SPI_DEV			hspi1

#define	SENSOR_IS_LSM				'L'
#define	SENSOR_IS_BME				'B'
#define	LSM_PACKET_LEN				12
#define	BME_PACKET_LEN				8
#define	BUF_OFFSET_STRT				0
#define	BUF_OFFSET_SENSOR_TYPE		1
#define	BUF_OFFSET_SENSOR_REG_ADDR	2
#define	BUF_OFFSET_SENSOR_DATA_LEN	3
#define	BUF_OFFSET_SENSOR_DATA		4

enum RX_STATE_MACHINE
{
	WAIT_FOR_START_CHAR,
	GET_SENSOR_TYPE,
	GET_SENSOR_ADDRESS,
	GET_SENSOR_DATA_SIZE,
	GET_SENSOR_DATA,
	GET_DATA,
};

typedef struct _SystemTypeDef
{
	uint8_t			system_flags;
	uint8_t			tick_flags;
	uint8_t			tick_100ms;
	uint8_t			tick_500ms;
	uint8_t			tick_5Sec;
	uint16_t		analog_channels[ADC_MONITOR_COVERSIONS];
	uint16_t		pwr_stab_time;
	uint8_t			uart_flags;
	uint8_t			uart_buf_char;
	uint8_t			uart_index;
	uint8_t			uart_buf[UART_BUF_LEN];
	uint8_t			uart_sm;
	uint32_t		uart_rxerr;
	uint32_t		sensor_type;
	uint32_t		sensor_address;
	uint32_t		sensor_data_size;
	uint32_t		bme_pkt_complete;
	uint32_t		lsm_pkt_complete;
	uint8_t			uart_packet[UART_BUF_LEN];
	uint8_t			current_packet_len;
	uint8_t			uart_packet_cksum;
	uint8_t			ip_hh;
	uint8_t			ip_hl;
	uint8_t			ip_lh;
	uint8_t			ip_ll;
	uint8_t			lsm_data[LSM_PACKET_LEN];
	uint8_t			bme_data[BME_PACKET_LEN];
	float 			Temperature;
	float 			Pressure;
	float 			Humidity;
	uint16_t		an_vbus;
	uint16_t		an_temp;
	uint16_t		an_isen;
	uint16_t		an_itemp;
	uint16_t		max_an_isen;
}SystemTypeDef;

/* system_flags */
#define	SYSTEM_ADC_COMPLETE		0x40
#define	SYSTEM_ADC_HALF			0x20
#define	SYSTEM_ADC_ERROR		0x10
#define	SYSTEM_HTTPD_STARTED	0x01
/* uart_flags */
#define	SYSTEM_UART_ENABLED		0x80
#define	SYSTEM_UART_TX_RDY		0x08
#define	SYSTEM_UART_RX_RDY		0x04
#define	SYSTEM_UART_RX_ERR		0x02
#define	SYSTEM_BME280_PRESENT	0x01

/* tick_flags */
#define	TICK_10MSEC			0x01
#define	TICK_100MSEC		0x02
#define	TICK_500MSEC		0x04
#define	TICK_5SEC			0x08

#define	AN_VBUS		0
#define	AN_TEMP		1
#define	AN_ISEN		2
#define	AN_ITEMP	3

extern	SystemTypeDef	System;
extern	struct 	netif gnetif;

extern	void Rollon_Init(void);
extern	void Rollon_Loop(void);

#include "spi_eeprom.h"
#include "tcpServer.h"
extern	void httpd_init(void);

#endif /* ROLLON_APP_ROLLON_H_ */
