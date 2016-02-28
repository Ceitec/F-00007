/*
 * F-00007.c
 * Firmware pro ovládání pøes RS485 - > Ofuk
  * Created: 4.2.2016 15:28:42
 * Author : Lukas
 */ 


//#include <avr/iom32.h>
#include <avr/io.h>
//#include <avr/iom128a.h>
#include "inc/AllInit.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include "inc/common_defs.h"
#include "inc/defines.h"
#include "inc/timer.h"
#include "inc/uart_types.h"
#include "inc/uart_tri_0.h"
#include "inc/Tribus_types.h"
#include "inc/Tribus.h"
#include <stdlib.h>
// Standard Input/Output functions
#include <stdio.h>
#include <string.h>
#include "inc/Lib-all.h"



// Init pro Tribus
volatile byte timer0_flag = 0; // T = 10ms
byte led_timer = 0;



void send_data(void)
{
	uart0_put_data((byte *) &TB_bufOut);
}

//----------------------------------------------------------
ISR(TIMER1_CAPT_vect) {
	// T = 10ms
	
	timer0_flag = TRUE;
	
}

//----------------------------------------------------------
void process_timer_100Hz(void)
{
	if (timer0_flag) { // T = 10ms
		timer0_flag = false;
		uart0_ISR_timer();
		if (led_timer > 0) {
			led_timer--;
			if (led_timer == 0) {
				//PORTA ^= (1 << PINA6);
			}
		}
	}
}

void try_receive_data(void)
{
	byte i;
	byte *ptr;
	
	if (uart0_flags.data_received)
	{
		ptr = uart0_get_data_begin();
		for (i=0; i<9; i++)
		{
			TB_bufIn[i] = *ptr;
			ptr++;
		}
		uart0_get_data_end();
		uart0_flags.data_received = FALSE;
		if (TB_Read() == 0)
		{
			switch (TB_Decode())
			{
				case TB_CMD_OFUK:
					switch (TB_bufIn[TB_BUF_TYPE])
					{
						//info
						case OF_INFO:
							switch (TB_bufIn[TB_BUF_MOTOR])
							{
								//PORTB
								case 0x0B:
									switch (TB_Value)
									{
										case 0:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTB, PINB0));
											break;
										case 1:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTB, PINB1));
											break;
										case 2:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTB, PINB2));
											break;
										case 3:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTB, PINB3));
											break;
										case 4:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTB, PINB4));
											break;
										case 5:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTB, PINB5));
											break;
										case 6:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTB, PINB6));
											break;
										case 7:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTB, PINB7));
											break;
										default:
											TB_SendAck(TB_ERR_NOK, 0);
											break;
									}
								//PORTC
								case 0x0C:
									switch (TB_Value)
									{
										case 0:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTC, PINC0));
											break;
										case 1:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTC, PINC1));
											break;
										case 2:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTC, PINC2));
											break;
										case 3:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTC, PINC3));
											break;
										case 4:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTC, PINC4));
											break;
										case 5:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTC, PINC5));
											break;
										case 6:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTC, PINC6));
											break;
										case 7:
											TB_SendAck(TB_ERR_NOK, 0);
											break;
										default:
											TB_SendAck(TB_ERR_NOK, 0);
											break;
									}
								//PORTD
								case 0x0D:
									switch (TB_Value)
									{
										case 0:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTD, PIND0));
											break;
										case 1:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTD, PIND1));
											break;
										case 2:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTD, PIND2));
											break;
										case 3:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTD, PIND3));
											break;
										case 4:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTD, PIND4));
											break;
										case 5:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTD, PIND5));
											break;
										case 6:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTD, PIND6));
											break;
										case 7:
											TB_SendAck(TB_ERR_OK, Cteni_pinu(PORTD, PIND7));
											break;
										default:
											TB_SendAck(TB_ERR_NOK, 0);
											break;
									}
								default:
									TB_SendAck(TB_ERR_NOK, 0);
									break;
							}
							break;
							
						// Nastavení ofuku
						case OF_SETOUTPUT:
							// Pokud je Motor nastaven do 1 neboli true tak se zapne ofuk
							if (TB_bufIn[TB_BUF_MOTOR] == 1)
							{
								sbi(PORTC,PC4);
								TB_SendAck(TB_ERR_OK, 1);
							}
							else if(TB_bufIn[TB_BUF_MOTOR] == 0)
							{
								cbi(PORTC,PC4);
								TB_SendAck(TB_ERR_OK, 0);
							}
							else
							{
								cbi(PORTC,PC4);
								TB_SendAck(TB_ERR_NOK, TB_bufIn[TB_BUF_MOTOR]);
							}
						default:
							TB_SendAck(TB_ERR_NOK, 0);
							break;
					} 
					break;
			}
		}
	}
}


int main(void)
{
	//Nastavení Systemového enable pro RS485 pro UART0	
	DDRD |= (1 << DDD2);
	
	//Nastavení pinu PC4 jako výstupního pro ovládání ofuku
	DDRC |= (1 << DDC4);
		
	timer_init();
	
	uart0_init();
	TB_Callback_setBaud = &uart0_set_baud;
	TB_Callback_TX = &send_data;
	TB_Init((void*) 0x10); // addr in eeprom with settings
	
	sei();
	
    while(1)
    {
		
		process_timer_100Hz();
		uart0_process();
		try_receive_data();
    }
}


