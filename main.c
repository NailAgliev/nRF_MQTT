/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_power.h"
#include "nrf_serial.h"
#include "app_timer.h"
#include "app_scheduler.h"

#include "SEGGER_RTT.h"

#include "app_error.h"
#include "app_util.h"
#include "boards.h"

/** @file
 * @defgroup nrf_serial_example main.c
 * @{
 * @ingroup nrf_serial_example
 * @brief Example of @ref nrf_serial usage. Simple loopback.
 *
 */



#define SCHED_MAX_EVENT_DATA_SIZE 16
#define SCHED_QUEUE_SIZE 64
#define OP_QUEUES_SIZE          3
#define APP_TIMER_PRESCALER     NRF_SERIAL_APP_TIMER_PRESCALER

//#define 	APP_SCHED_EVENT_HEADER_SIZE   8

//#define 	APP_SCHED_BUF_SIZE(EVENT_SIZE, QUEUE_SIZE)   (((EVENT_SIZE) + APP_SCHED_EVENT_HEADER_SIZE) * ((QUEUE_SIZE) + 1))

char modem_data[128];

void (*p_func)(void *p_event_data, uint16_t event_size);

enum {
	AT,
	CFUN,
	CFUN_1,
	ATE,
	ATV,
	CMEE,
	CPIN_CHECK,
	CSQ_CHECK,
	CREG_CHECK,
	CIPSHUT,
	CGTT_CHECK,
	CIPQSEND,
	CIPRXGET,
	CSTT,
	CIICR,
	OK, //15
} modem_int_state;



static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}





static void slep_handler()
{
	SEGGER_RTT_printf(0, "got\r\n");
}



	
//	if (modem_data[0] == 0)
//	{
//	nrf_serial_read(p_event_data, &modem_data, sizeof(modem_data), NULL, 0);
//	}
//	else
//	{
//		while(false)
//		{
//			uint8_t i = 0;
//			if (modem_data[i] == 0)
//			{
//				nrf_serial_read(p_event_data, &modem_data[i], sizeof(modem_data), NULL, 0);
//				i = 0;
//				break;
//			}
//			else
//			{
//				i++;
//			}
//		}

static void event_handler(struct nrf_serial_s const * p_serial, nrf_serial_event_t event)
{
	switch (event)
	{
		case NRF_SERIAL_EVENT_RX_DATA:
		{
			app_sched_event_put(p_serial, sizeof(*p_serial), p_func);
		}
		default:
			break;
	}
}


NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
                      RX_PIN_NUMBER, TX_PIN_NUMBER,
                      RTS_PIN_NUMBER, CTS_PIN_NUMBER,
                      NRF_UART_HWFC_ENABLED, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_9600,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);

#define SERIAL_FIFO_TX_SIZE 32
#define SERIAL_FIFO_RX_SIZE 32

NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);


#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 1

NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);

NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ,
                      &serial_queues, &serial_buffs, event_handler, slep_handler);


NRF_SERIAL_UART_DEF(serial_uart, 0);



char* concat(char *s1, char *s2) 
		 {
        size_t len1 = strlen(s1);
        size_t len2 = strlen(s2);                      

        char *result = malloc(len1 + len2 + 1);

        memcpy(result, s1, len1);
        memcpy(result + len1, s2, len2 + 1);    

        return result;
    }

void at_write(char str[])
{
	
	char at[2] = "AT";
	char n[2] = "\n";
	char * data_tx = concat(at, str);
	data_tx = concat(data_tx, n);
	
	SEGGER_RTT_printf(0, "%s", data_tx);
	
	nrf_serial_write(&serial_uart, data_tx, sizeof(*data_tx), NULL, 0);
	
}





void modem_init()
{
	
	switch(modem_int_state)
	{
		case AT:  									 //Проверка доступен ли модуль
			{
				at_write("");
			}
		case CFUN:
			{
		
			}
		case CFUN_1: 								 //Рестарт модуля
			{
		
			}
		case ATE:										//No echo mode
			{
		
			}
		case ATV:										//Числовой формат ответов
			{
		
			}
		case CMEE:  							 	//Кодовый формат ошибок
			{
		
			}
		case CPIN_CHECK:						
			{
			
			}
		case CSQ_CHECK: 						//Проверка силы сигнала
			{
				
			}
		case CREG_CHECK:						//Проверка рекгестрации в сети
			{
				
			}
		case CIPSHUT:								//TCP restart
			{
				
			}
		case CGTT_CHECK:            //проверка готовности модуля для установления связи
			{
				
			}
		case CIPRXGET:							//Ручное получение полученных сообщений
			{
				
			}
		case CIPQSEND:							//Режим отправки без потверждения получения		
			{
				
			}
		case CSTT:									//Конфигурация APN
			{
				
			}
		case CIICR:									//Влючение GPRS
			{
				
			}			
	}
}



void serial_scheduled_ex (void *p_event_data, uint16_t event_size)
{
	if(modem_int_state < OK)
	{
		switch(modem_int_state)
	{
		case AT:  									 //Проверка доступен ли модуль
			{
				nrf_serial_read(p_event_data, &modem_data, sizeof(modem_data), NULL, 0);
				if(modem_data == 0)
				{
					modem_int_state = CFUN;
					modem_init();
				}
			}
		case CFUN:
			{
		
			}
		case CFUN_1: 								 //Рестарт модуля
			{
		
			}
		case ATE:										//No echo mode
			{
		
			}
		case ATV:										//Числовой формат ответов
			{
		
			}
		case CMEE:  							 	//Кодовый формат ошибок
			{
		
			}
		case CPIN_CHECK:						
			{
			
			}
		case CSQ_CHECK: 						//Проверка силы сигнала
			{
				
			}
		case CREG_CHECK:						//Проверка рекгестрации в сети
			{
				
			}
		case CIPSHUT:								//TCP restart
			{
				
			}
		case CGTT_CHECK:            //проверка готовности модуля для установления связи
			{
				
			}
		case CIPRXGET:							//Ручное получение полученных сообщений
			{
				
			}
		case CIPQSEND:							//Режим отправки без потверждения получения		
			{
				
			}
		case CSTT:									//Конфигурация APN
			{
				
			}
		case CIICR:									//Влючение GPRS
			{
				
			}			
	}
	}
}






int main(void)
{
    ret_code_t ret;
		
		p_func = &serial_scheduled_ex;
	
		scheduler_init();
	
	
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    ret = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(ret);

    nrf_drv_clock_lfclk_request(NULL);
    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    bsp_board_leds_init();
    bsp_board_buttons_init();

    ret = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
    APP_ERROR_CHECK(ret);

    static char tx_message[] = "Hello nrf_serial!\n\r";

    ret = nrf_serial_write(&serial_uart, tx_message, strlen(tx_message), NULL, 0);
    //SEGGER_RTT_printf(0, "%s", ret);
		//APP_ERROR_CHECK(ret);
		

    while (true)
    {
				app_sched_execute();
    }
}

/** @} */
