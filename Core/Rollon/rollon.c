/*
 * rollon.c
 *
 *  Created on: May 4, 2023
 *      Author: fil
 */

#include "main.h"
#include "lwip.h"
#include <string.h>

float Temperature, Pressure, Humidity;

__attribute__ ((aligned (4)))	SystemTypeDef				System;
uint8_t	ee_page0[EEPROM_PAGESIZE];

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if ( huart == &UART )
	{
		System.uart_flags |= SYSTEM_UART_TX_RDY;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if ( huart == &UART )
	{
		System.uart_flags |= SYSTEM_UART_RX_RDY;
	}
}
uint8_t clear_error;
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if ( huart == &UART )
	{
		clear_error = huart->Instance->RDR; // clear the error ?!
		System.uart_flags |= (SYSTEM_UART_RX_ERR | SYSTEM_UART_RX_RDY);
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	System.system_flags |= SYSTEM_ADC_COMPLETE;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	System.system_flags |= SYSTEM_ADC_HALF;
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	System.system_flags |= SYSTEM_ADC_ERROR;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if ( htim == &TIM_TICK_10MSEC )
	{
		HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_FLT_GPIO_Port, LED_FLT_Pin, GPIO_PIN_RESET);
		System.tick_flags |= TICK_10MSEC;
		System.tick_100ms ++;
		if ( System.tick_100ms == 10 )
		{
			System.tick_100ms = 0;
			System.tick_flags |= TICK_100MSEC;
			System.tick_500ms ++;
			if ( System.tick_500ms == 5 )
			{
				HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);

				System.tick_flags |= TICK_500MSEC;
				System.tick_500ms = 0;
				System.tick_5Sec++;
				if ( System.tick_5Sec == 5 )
				{
					System.tick_flags |= TICK_5SEC;
					System.tick_5Sec = 0;
				}
			}
		}
	}
}

#define	POWER_ON	1
#define	LINK_ON		1
#define	NET_ENABLE		1

void Rollon_Init(void)
{
	EEPROM_SPI_INIT(&EEPROM_SPI_DEV);
	EEPROM_SPI_ReadBuffer(ee_page0, 0, EEPROM_PAGESIZE);
#ifdef	NET_ENABLE
	while (!netif_is_up(&gnetif))
	{
		MX_LWIP_Process();
	}
	/*
	while(dhcp_supplied_address(&gnetif) != 1)
	{
		if ( dhcp_start(&gnetif) == 0)
				break;
	}
	*/
	tcp_server_init();

#endif

#ifdef	LINK_ON
	if ( HAL_TIM_PWM_Start(&POWER_PWM_TIM,POWER_PWM2_FGEN_CHANNEL) != 0 )
		HAL_GPIO_WritePin(LED_FLT_GPIO_Port, LED_FLT_Pin, GPIO_PIN_SET);
#endif

#ifdef	POWER_ON
	if ( HAL_TIM_PWM_Start(&POWER_PWM_TIM,POWER_PWM1_PGEN_CHANNEL) != 0 )
		HAL_GPIO_WritePin(LED_FLT_GPIO_Port, LED_FLT_Pin, GPIO_PIN_SET);
#endif
	HAL_TIM_Base_Start_IT(&TIM_TICK_10MSEC);
	HAL_ADC_Start_DMA(&ADC_MONITOR, (uint32_t* )System.analog_channels, ADC_MONITOR_COVERSIONS);
	System.uart_index = 0;
	UART_Start_Receive_IT(&UART, &System.uart_buf_char, 1);
}

uint8_t Rollon_AnalyzePacket(uint8_t packet_len)
{
uint8_t		i;
	System.uart_packet_cksum = 0;
	for(i=0;i<packet_len;i++)
	{
		System.uart_packet[i] = System.uart_buf[i];
		System.uart_packet_cksum ^= System.uart_packet[i];
	}
	return System.uart_packet_cksum;
}

void Rollon_Loop(void)
{
uint8_t i;

	if (( System.tick_flags & TICK_5SEC) == TICK_5SEC)
	{
		System.tick_flags &= ~TICK_5SEC;
	}
	if (( System.tick_flags & TICK_500MSEC) == TICK_500MSEC)
	{
		System.tick_flags &= ~TICK_500MSEC;
	}

	if (( System.system_flags & SYSTEM_ADC_COMPLETE) == SYSTEM_ADC_COMPLETE)
	{
		System.system_flags &= ~SYSTEM_ADC_COMPLETE;
		System.an_vbus = System.analog_channels[AN_VBUS];
		System.an_temp = System.analog_channels[AN_TEMP];
		System.an_isen = System.analog_channels[AN_ISEN];
	}

	if (( System.uart_flags & SYSTEM_UART_RX_RDY) == SYSTEM_UART_RX_RDY)
	{
		System.uart_flags &= ~SYSTEM_UART_RX_RDY;
		System.uart_buf[System.uart_index] = UART.Instance->RDR;
		UART_Start_Receive_IT(&UART, &System.uart_buf_char, 1);

		if (( System.uart_flags & SYSTEM_UART_RX_ERR) != SYSTEM_UART_RX_ERR)
		{
			switch ( System.uart_sm )
			{
			case	WAIT_FOR_START_CHAR	:
							System.uart_index = 0;
							if ( System.uart_buf[System.uart_index] == '%' )
							{
								System.uart_index ++;
								System.uart_sm = GET_SENSOR_TYPE;
							}
							break;
			case	GET_SENSOR_TYPE :
							if (( System.uart_buf[System.uart_index] == SENSOR_IS_BME) || ( System.uart_buf[System.uart_index] == SENSOR_IS_LSM))
							{
								System.sensor_type = System.uart_buf[System.uart_index];
								System.uart_index ++;
								System.uart_sm = GET_SENSOR_ADDRESS;
							}
							else
								System.uart_sm = WAIT_FOR_START_CHAR;
							break;
			case	GET_SENSOR_ADDRESS :
							System.sensor_address = System.uart_buf[System.uart_index];
							System.uart_index ++;
							System.uart_sm = GET_SENSOR_DATA_SIZE;
							break;
			case	GET_SENSOR_DATA_SIZE :
							System.sensor_data_size = System.uart_buf[System.uart_index];
							if ( System.sensor_type == SENSOR_IS_LSM )
							{
								if ( System.sensor_data_size != LSM_PACKET_LEN )
									System.uart_sm = WAIT_FOR_START_CHAR;
								else
								{
									System.uart_index ++;
									System.uart_sm = GET_SENSOR_DATA;
								}
							}
							else if ( System.sensor_type == SENSOR_IS_BME )
							{
								if ( System.sensor_data_size != BME_PACKET_LEN )
									System.uart_sm = WAIT_FOR_START_CHAR;
								else
								{
									System.uart_index ++;
									System.uart_sm = GET_SENSOR_DATA;
								}
							}
							else
							{
								System.uart_index ++;
								System.uart_sm = GET_SENSOR_DATA;
							}
							break;
			case	GET_SENSOR_DATA	:
							if ( System.uart_index > (System.sensor_data_size + BUF_OFFSET_SENSOR_DATA ) )
							{
								if ( System.sensor_type == SENSOR_IS_BME )
								{
									if ( Rollon_AnalyzePacket(System.uart_index) == 0 )
									{
										for(i=0;i<BME_PACKET_LEN;i++)
											System.bme_data[i] = System.uart_buf[i+BUF_OFFSET_SENSOR_DATA];
										System.bme_pkt_complete++;
									}
								}
								else if ( System.sensor_type == SENSOR_IS_LSM )
								{
									if ( Rollon_AnalyzePacket(System.uart_index) == 0 )
									{
										for(i=0;i<LSM_PACKET_LEN;i++)
											System.lsm_data[i] = System.uart_buf[i+BUF_OFFSET_SENSOR_DATA];
										System.lsm_pkt_complete++;
									}
								}
								else
									System.uart_sm = WAIT_FOR_START_CHAR;
							}
							System.uart_index ++;
							break;
			default:
				System.uart_sm = WAIT_FOR_START_CHAR;
			}
		}
		else
		{
			System.uart_flags &= ~SYSTEM_UART_RX_ERR;
			System.uart_rxerr++;
			System.uart_sm = 0;
			System.uart_index = 0;
		}
	}


#ifdef	NET_ENABLE
	MX_LWIP_Process();
	sys_check_timeouts();
	//ethernetif_input(&gnetif);
#endif
}
