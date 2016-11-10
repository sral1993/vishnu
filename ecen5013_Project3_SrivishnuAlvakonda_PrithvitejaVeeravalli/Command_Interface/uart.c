/*
 * uart.c
 *
 *  Created on: Nov 7, 2016
 *      Author: Prithvi
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
	B_decrease=0x40,
	do_memmove=0x41
}cmdi;

typedef struct CI_Msg_t{
	cmdi command;
	uint8_t data[8];
    uint8_t len;
    uint16_t checksum;

}CI_Msg;

CI_Msg x;

CI_Msg *cmd_ptr = &x;

int ovfw=0;

void Command_Inteface(CI_Msg *cmd_ptr);

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


void Command_Inteface(CI_Msg *cmd_ptr)
{
	while(!(UART0_S1 & UART_S1_RDRF_MASK));
	cmd_ptr->command=UART0_D;
	while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
	UART0_D= cmd_ptr->command;
	while(!(UART0_S1 & UART_S1_RDRF_MASK));
	cmd_ptr->len = UART0_D;
	while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
	UART0_D = cmd_ptr->len;
	int f=0x00,y=0;
	while(f<cmd_ptr->len)
			 {
				 while(!(UART0_S1 & UART_S1_RDRF_MASK));
				 cmd_ptr->data[y] = UART0_D;
				 while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
				 UART0_D = cmd_ptr->data[y];
				 y++;
				 f++;
			 }

	        while(!(UART0_S1 & UART_S1_RDRF_MASK));
	        cmd_ptr->checksum = UART0_D;
			while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
			UART0_D = cmd_ptr->checksum;
            if(ovfw==1)
            {
			while(!(UART0_S1 & UART_S1_RDRF_MASK));
			ovfw=UART0_D;
			while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
			UART0_D = ovfw;
			ovfw=ovfw<<8;
			cmd_ptr->checksum=cmd_ptr->checksum|ovfw;
            }

}
void UART0_IRQHandler()
{
	int i=0, value=0, intensity=0, Mod=0, light,h=0,m=0;
		int input;

		while(1)
 {

		Command_Inteface(cmd_ptr);
		int j=0;
		uint16_t syscheck=0;

		syscheck= cmd_ptr->command+cmd_ptr->len;
		while(j<cmd_ptr->len)
		{
		syscheck= syscheck+cmd_ptr->data[j];
		j++;
		}
        if(syscheck==cmd_ptr->checksum)
        {




		if(cmd_ptr->command == do_memmove)
		{

		height=	cmd_ptr->data[1];
		src=(uint8_t *) arr;
	    dst= (uint8_t *)b;

	    my_memmove(src,dst,height);
        LED_OFF();
	    Mod=32000;
	    				LED_GREEN(Mod);
	    				while(m<1000000)
	    								{
	    									m++;
	    								}
	    				LED_OFF();


		}

		if(cmd_ptr->data[0]== 0x31)
				{
					Mod=32000;
				}
				else
				{
					Mod=0;
				}

		       if(cmd_ptr->command == R_on)
		       {
		    	   LED_OFF();

		    	   LED_RED(Mod);
		       }
		       if(cmd_ptr->command == B_on)
		       {
		    	   LED_OFF();

		    	   LED_BLUE(Mod);
		       }
		       if(cmd_ptr->command == G_on)
		       {
		    	   LED_OFF();

		    	   LED_GREEN(Mod);
		       }
               if(cmd_ptr->command == All_off)
               {
            	   LED_OFF();
               }
               if(cmd_ptr->command == R_increase)
               {
            	   if(intensity==10)
            	   	{
            	   	 Mod=intensity;
            	   	LED_OFF();
            	   	LED_RED(Mod);
            	   }
            	  else
            	   	{
            		  if(cmd_ptr->data[0]== 0x31)
            		  {
            			  intensity=intensity+1;
            		  }
               	   	 Mod=intensity;
            	   	 LED_OFF();
            	   	 LED_RED(Mod);
            	   	}
               }

               if(cmd_ptr->command == R_decrease)
               {
            	   if(intensity==0)
            	   	{
            	   	 Mod=intensity;
            	   	 LED_OFF();
            	   	 LED_RED(Mod);
            	   	}
            	   	else
            	   	{
            	   		if(cmd_ptr->data[0]== 0x31)
            	   		{
            	   			intensity=intensity-1;
            	   		}

            	   	Mod=intensity;
            	   	LED_OFF();
            	   	LED_RED(Mod);
            	   	}
               }
             if(cmd_ptr->command == G_increase)
                              {
                           	   if(intensity==10)
                           	   	{
                           	   	 Mod=intensity;
                           	   	 LED_OFF();
                           	   	 LED_GREEN(Mod);
                           	   	}
                           	   else
                           	   	{
                           		if(cmd_ptr->data[0]== 0x31)
                           		{
                           			intensity=intensity+1;
                           		}
                                 Mod=intensity;
                           	   	 LED_OFF();
                           	   	 LED_GREEN(Mod);
                           	   	}
                              }

                              if(cmd_ptr->command == G_decrease)
                              {
                           	   if(intensity==0)
                           	   	{
                           	   	 Mod=intensity;
                           	   	 LED_OFF();
                           	   	 LED_GREEN(Mod);
                           	   	}
                           	   	else
                           	   	{
                           	   	if(cmd_ptr->data[0]== 0x31)
                           	   	            	   		{
                           	   	            	   			intensity=intensity-1;
                           	   	            	   		}
                           	   	Mod=intensity;
                           	   	LED_OFF();
                           	   	LED_GREEN(Mod);
                           	   	}
                              }
                              if(cmd_ptr->command == B_increase)
                                             {
                                          	   if(intensity==10)
                                          	   	{
                                          	   	 Mod=intensity;
                                          	   	 LED_OFF();
                                          	   	 LED_BLUE(Mod);
                                          	   	}
                                          	   else
                                          	   	{
                                          		 if(cmd_ptr->data[0]== 0x31)
                                          		                            		{
                                          		                            			intensity=intensity+1;
                                          		                            		}
                                          	   	 Mod=intensity;
                                          	   	 LED_OFF();
                                          	   	 LED_BLUE(Mod);
                                          	   	}
                                             }

                                             if(cmd_ptr->command == B_decrease)
                                             {
                                          	   if(intensity==0)
                                          	   	{
                                          	   	 Mod=intensity;
                                          	   	 LED_OFF();
                                          	   	 LED_BLUE(Mod);
                                          	   	}
                                          	   	else
                                          	   	{
                                          	   	if(cmd_ptr->data[0]== 0x31)
                                          	   	            	   		{
                                          	   	            	   			intensity=intensity-1;
                                          	   	            	   		}
                                          	   	Mod=intensity;
                                          	   	LED_OFF();
                                          	   	LED_BLUE(Mod);
                                          	   	}
                                             }



       } //syscheck

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



