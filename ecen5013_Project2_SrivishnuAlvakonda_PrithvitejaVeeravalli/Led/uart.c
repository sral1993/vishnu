/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "MKL25Z4.h"
#include "core_cmFunc.h"
#include "uart.h"
#include <stdio.h>
#include <led.h>



void UART0_IRQHandler()
{
	int i=0, value=0, intensity=0, Mod=32000;
		char input;

		while(1)
  {
		while(!(UART0_S1 & UART_S1_RDRF_MASK));
		input=UART0_D;
		while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
		UART0_D= input;
			if(input =='o')
			{
				LED_OFF();
			}
			if(input =='r')
			{
				LED_OFF();
				LED_RED(Mod);
			}
			if(input =='g')
			{
				LED_OFF();
				LED_GREEN(Mod);
		    }
			if(input =='b')
			{
			    LED_OFF();
			    LED_BLUE(Mod);
	        }
			if(input =='y')
			{
				LED_OFF();
				LED_YELLOW(Mod);
			}
			if(input =='v')
			{
				LED_OFF();
				LED_VIOLET(Mod);
			}
			if(input =='c')
			{
				LED_OFF();
				LED_CYAN(Mod);
			}
			if(input =='w')
			{
				LED_OFF();
				LED_WHITE(Mod);
			}

			if(input =='h')
			{
				while(!(UART0_S1 & UART_S1_RDRF_MASK));
				input=UART0_D;
				while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
				UART0_D= input;
				if(input == 'r')
				{
				if(intensity==10)
				{
					Mod=intensity;
					LED_OFF();
					LED_RED(Mod);
				}
				else
				{
					intensity=intensity+1;
					Mod=intensity;
					LED_OFF();
					LED_RED(Mod);
				}
				}
				if(input == 'g')
				{
				if(intensity==10)
				{
				    Mod=intensity;
					LED_OFF();
					LED_GREEN(Mod);
				}
				else
				{
				    intensity=intensity+1;
					Mod=intensity;
					LED_OFF();
					LED_GREEN(Mod);
				}
				}
				if(input == 'b')
				{
					if(intensity==10)
				{
					Mod=intensity;
				    LED_OFF();
					LED_BLUE(Mod);
				}
				    else
				{
					intensity=intensity+1;
					Mod=intensity;
					LED_OFF();
					LED_BLUE(Mod);
				}
				}
				if(input == 'y')
				{
					if(intensity==10)
				{
					Mod=intensity;
					LED_OFF();
					LED_YELLOW(Mod);
				}
					else
				{
					intensity=intensity+1;
					Mod=intensity;
					LED_OFF();
					LED_YELLOW(Mod);
				}
				}
				if(input == 'v')
				{
					if(intensity==10)
			    {
					Mod=intensity;
					LED_OFF();
					LED_VIOLET(Mod);
			    }
					else
			    {
					intensity=intensity+1;
				    Mod=intensity;
					LED_OFF();
					LED_VIOLET(Mod);
				}
				}
				if(input == 'c')
				{
					if(intensity==10)
				{
					Mod=intensity;
				    LED_OFF();
				    LED_CYAN(Mod);
				}
					else
				{
					intensity=intensity+1;
					Mod=intensity;
					LED_OFF();
					LED_CYAN(Mod);
				}
			    }
				if(input == 'w')
				{
					if(intensity==10)
				{
					Mod=intensity;
					LED_OFF();
					LED_WHITE(Mod);
				}
					else
				{
					intensity=intensity+1;
					Mod=intensity;
					LED_OFF();
					LED_WHITE(Mod);
				}
				}

			}
			if(input =='l')
			{
				while(!(UART0_S1 & UART_S1_RDRF_MASK));
						input=UART0_D;
						while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
						UART0_D= input;
						if(input == 'r')
										{
										if(intensity==0)
										{
											Mod=intensity;
											LED_OFF();
											LED_RED(Mod);
										}
										else
										{
											intensity=intensity-1;
											Mod=intensity;
											LED_OFF();
											LED_RED(Mod);
										}
										}
										if(input == 'g')
										{
										if(intensity==0)
										{
										    Mod=intensity;
											LED_OFF();
											LED_GREEN(Mod);
										}
										else
										{
										    intensity=intensity-1;
											Mod=intensity;
											LED_OFF();
											LED_GREEN(Mod);
										}
										}
										if(input == 'b')
										{
											if(intensity==0)
										{
											Mod=intensity;
										    LED_OFF();
											LED_BLUE(Mod);
										}
										    else
										{
											intensity=intensity-1;
											Mod=intensity;
											LED_OFF();
											LED_BLUE(Mod);
										}
										}
										if(input == 'y')
										{
											if(intensity==0)
										{
											Mod=intensity;
											LED_OFF();
											LED_YELLOW(Mod);
										}
											else
										{
											intensity=intensity-1;
											Mod=intensity;
											LED_OFF();
											LED_YELLOW(Mod);
										}
										}
										if(input == 'v')
										{
											if(intensity==0)
									    {
											Mod=intensity;
											LED_OFF();
											LED_VIOLET(Mod);
									    }
											else
									    {
											intensity=intensity-1;
										    Mod=intensity;
											LED_OFF();
											LED_VIOLET(Mod);
										}
										}
										if(input == 'c')
										{
											if(intensity==0)
										{
											Mod=intensity;
										    LED_OFF();
										    LED_CYAN(Mod);
										}
											else
										{
											intensity=intensity-1;
											Mod=intensity;
											LED_OFF();
											LED_CYAN(Mod);
										}
									    }
										if(input == 'w')
										{
											if(intensity==0)
										{
											Mod=intensity;
											LED_OFF();
											LED_WHITE(Mod);
										}
											else
										{
											intensity=intensity-1;
											Mod=intensity;
											LED_OFF();
											LED_WHITE(Mod);
										}
										}
			}
		}
}


void clock_configure()
    {
    	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;  //Clock for UART 0 is enabled

    	SIM_SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;
    	SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1); //Selecting MCGFLLClk for UART0

    	SIM_SCGC5|=SIM_SCGC5_PORTA_MASK;  //Clock for Port A is enabled
    }
void gpio_configure()
{
	PORTA_PCR2&=~PORT_PCR_MUX_MASK;
	PORTA_PCR2|=PORT_PCR_MUX(2);       //UART0 TX pin select

	PORTA_PCR1|=PORT_PCR_MUX(2);       //UART RX pin select

}
void uart0_init (uint32_t baudrate)
{

	    uint16_t BR;
		clock_configure();
		gpio_configure();


		UART0_C2 &= ~(UART0_C2_TE_MASK|UART0_C2_RE_MASK);     //UART0 TX and RX is disabled.
		UART0_C2 |= (UART_C2_RIE_MASK|UART_C2_TIE_MASK);
	    BR=DEFAULT_SYSTEM_CLOCK/(16*baudrate);
	    UART0_BDL = (uint8_t) BR;
	    UART0_BDH = (BR)>>baudrate_bh_shift;

	    UART0_C2 |= (UART0_C2_TE_MASK|UART0_C2_RE_MASK);
	    __enable_irq();
	    NVIC_EnableIRQ(UART0_IRQn);
 }





