/*
 * uart.c
 *
 *  Created on: Nov 7, 2016
 *      Author: Prithvi
 */

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


uint8_t *src,*dst;
	uint8_t height=10;
	uint8_t arr[100]={1,3,5,7,6,9,11,13,15,17};
	uint8_t b[100];
	int t,e;

typedef enum cmdi_t{
	R_on = 0x31,
	B_on=0x32,
	G_on=0x33,
	All_off=0x34,
	R_increase=0x35,
	R_decrease=0x36,
	G_increase=0x37,
	G_decrease=0x38,
	B_increase=0x39,
	B_decrease=0x62,
	do_memmove=0x64
}cmdi;

typedef struct CI_Msg_t{
	cmdi command;
	uint8_t data[8];
    uint8_t len;
    uint16_t checksum;

}CI_Msg;

CI_Msg x;

int8_t my_memmove(uint8_t * src,uint8_t *dst,uint8_t height)
{
int k=0;
src=src+length-1;
dst=dst+length-1;
//Moving data from last address in Source to last address of the destination
//This method make sure that the data transfers securely even when there is an overlap in //source and destination addresses.
while(k<length)
{
*dst=*src;
src--;
dst--;
k++;
}
return 0;
}

int add(t,e)
{
	int z;
	z=t+e;
	return z;
}
void UART0_IRQHandler()
{
	int i=0, value=0, intensity=0, Mod=0, light,h=0,m=0;
		int input;

		while(1)
 {
		while(!(UART0_S1 & UART_S1_RDRF_MASK));
		x.command=UART0_D;
		while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
		UART0_D= x.command;
		 while(!(UART0_S1 & UART_S1_RDRF_MASK));
		 x.len = UART0_D;
		 while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
		 UART0_D = x.len;
		 int f=0x30,y=0;
		 while(f<x.len)
		 {
			 while(!(UART0_S1 & UART_S1_RDRF_MASK));
			 x.data[y] = UART0_D;
			 while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
			 UART0_D = x.data[y];
			 y++;
			 f++;
		 }
        int ovfw=0;
        while(!(UART0_S1 & UART_S1_RDRF_MASK));
		x.checksum = UART0_D;
		while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
		UART0_D = x.checksum;

		while(!(UART0_S1 & UART_S1_RDRF_MASK));
		ovfw=UART0_D;
		while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
		UART0_D = ovfw;
		ovfw=ovfw<<8;
		x.checksum=x.checksum|ovfw;
		uint16_t syscheck=0;
		syscheck= x.command+x.len+x.data[0]+x.data[1] ;
        if(syscheck==x.checksum)
        {


		if(x.data[0]== 0x31)
		{
			Mod=32000;
		}
		else
		{
			Mod=0;
		}

		if(x.command == do_memmove)
		{
		if(x.data[1]== 0x68)
		{
		height=	8;
		src=(uint8_t *) arr;
	    dst= (uint8_t *)b;
	    //light= add(2,3);
	    my_memmove(src,dst,height);
        LED_OFF();
	    Mod=32000;
	    				LED_GREEN(Mod);
	    				while(m<1000000)
	    								{
	    									m++;
	    								}
	    				LED_OFF();


//		my_memmove(src,dst,height);
		//if(light == 0)
		//{
			/*while(h<10)
			{
				Mod=32000;
				LED_OFF();
				LED_RED(Mod);
				while(m<10000)
				{
					m++;
				}
				LED_OFF();
				LED_GREEN(Mod);
				while(m<10000)
								{
									m++;
								}
				LED_OFF();
				LED_BLUE(Mod);
				while(m<10000)
								{
									m++;
								}
				h++;
			}*/
		//}
		}
		}
		       if(x.command == R_on)
		       {
		    	   LED_OFF();

		    	   LED_RED(Mod);
		       }
		       if(x.command == B_on)
		       {
		    	   LED_OFF();

		    	   LED_BLUE(Mod);
		       }
		       if(x.command == G_on)
		       {
		    	   LED_OFF();

		    	   LED_GREEN(Mod);
		       }
               if(x.command == All_off)
               {
            	   LED_OFF();
               }
               if(x.command == R_increase)
               {
            	   if(intensity==10)
            	   	{
            	   	 Mod=intensity;
            	   	LED_OFF();
            	   	LED_RED(Mod);
            	   }
            	  else
            	   	{
            		  if(x.data[0]== 0x31)
            		  {
            			  intensity=intensity+1;
            		  }
               	   	 Mod=intensity;
            	   	 LED_OFF();
            	   	 LED_RED(Mod);
            	   	}
               }

               if(x.command == R_decrease)
               {
            	   if(intensity==0)
            	   	{
            	   	 Mod=intensity;
            	   	 LED_OFF();
            	   	 LED_RED(Mod);
            	   	}
            	   	else
            	   	{
            	   		if(x.data[0]== 0x31)
            	   		{
            	   			intensity=intensity-1;
            	   		}

            	   	Mod=intensity;
            	   	LED_OFF();
            	   	LED_RED(Mod);
            	   	}
               }
             if(x.command == G_increase)
                              {
                           	   if(intensity==10)
                           	   	{
                           	   	 Mod=intensity;
                           	   	 LED_OFF();
                           	   	 LED_GREEN(Mod);
                           	   	}
                           	   else
                           	   	{
                           		if(x.data[0]== 0x31)
                           		{
                           			intensity=intensity+1;
                           		}
                                 Mod=intensity;
                           	   	 LED_OFF();
                           	   	 LED_GREEN(Mod);
                           	   	}
                              }

                              if(x.command == G_decrease)
                              {
                           	   if(intensity==0)
                           	   	{
                           	   	 Mod=intensity;
                           	   	 LED_OFF();
                           	   	 LED_GREEN(Mod);
                           	   	}
                           	   	else
                           	   	{
                           	   	if(x.data[0]== 0x31)
                           	   	            	   		{
                           	   	            	   			intensity=intensity-1;
                           	   	            	   		}
                           	   	Mod=intensity;
                           	   	LED_OFF();
                           	   	LED_GREEN(Mod);
                           	   	}
                              }
                              if(x.command == B_increase)
                                             {
                                          	   if(intensity==10)
                                          	   	{
                                          	   	 Mod=intensity;
                                          	   	 LED_OFF();
                                          	   	 LED_BLUE(Mod);
                                          	   	}
                                          	   else
                                          	   	{
                                          		 if(x.data[0]== 0x31)
                                          		                            		{
                                          		                            			intensity=intensity+1;
                                          		                            		}
                                          	   	 Mod=intensity;
                                          	   	 LED_OFF();
                                          	   	 LED_BLUE(Mod);
                                          	   	}
                                             }

                                             if(x.command == B_decrease)
                                             {
                                          	   if(intensity==0)
                                          	   	{
                                          	   	 Mod=intensity;
                                          	   	 LED_OFF();
                                          	   	 LED_BLUE(Mod);
                                          	   	}
                                          	   	else
                                          	   	{
                                          	   	if(x.data[0]== 0x31)
                                          	   	            	   		{
                                          	   	            	   			intensity=intensity-1;
                                          	   	            	   		}
                                          	   	Mod=intensity;
                                          	   	LED_OFF();
                                          	   	LED_BLUE(Mod);
                                          	   	}
                                             }



        }

               }//while close


}//irq handler close



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



