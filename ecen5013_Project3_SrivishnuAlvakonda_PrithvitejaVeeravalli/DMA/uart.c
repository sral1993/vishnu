/*
 * uart.c
 *
 *  Created on: Oct 31, 2016
 *      Author: vishnu
 */
#include "MKL25Z4.h"
#include "core_cmFunc.h"
#include "uart.h"
#include <stdio.h>
#include <stdint.h>


/***************************************************************************
 * Begin UART0 functions
 **************************************************************************/
/********************************************************************/
/*
 * Initialize the uart for 8N1 operation, interrupts disabled, and
 * no hardware flow-control
 * Parameters:
 *  baud        uart baud rate
 */

char transmit_data[length] = "embedded";
char receive_data[length];
char *txstr = transmit_data;
char *rxstr = receive_data;
uint16_t index=1;
uint8_t transmit=1;


//This function is used to transmit one character of data from the UART
void putachar(char *string, uint16_t len)
{
	char alphabet = *string;
	UART0_D=alphabet;
    if(index<len)
    {
    	index++;
    	txstr++;
    }
    else
    {
    	txstr=transmit_data;
    	index=1;
    	transmit=0;
    }
}
//This function is used to receive a char of data from UART
void getachar()
{
	*rxstr = UART0_D;
	rxstr++;
}
//Interrupt Handling routine of UART0 which is used to manage transmit and receive action of UART0
void UART0_IRQHandler()
{
	if((UART0_S1 & UART0_S1_TDRE_MASK)&&(transmit))
	{
			putachar(txstr, length);
	}

    if(UART0_S1 & UART0_S1_RDRF_MASK)
	{
		getachar();
	}


}

//Configuring the clock of UART0 so that writes and reads to UART0 will happen
void clock_configure()
    {
    	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;  //Clock for UART 0 is enabled

    	SIM_SOPT2 &= ~SIM_SOPT2_UART0SRC_MASK;
    	SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1); //Selecting MCGFLLClk for UART0

    	SIM_SCGC5|=SIM_SCGC5_PORTA_MASK;  //Clock for Port A is enabled
    }
//Configuring or enabling the corresonding GPIO pins for UART RX and TX by selecting the right MUX i/p pins
void gpio_configure()
{
	PORTA_PCR2&=~PORT_PCR_MUX_MASK;
	PORTA_PCR2|=PORT_PCR_MUX(2);       //UART0 TX pin select

	PORTA_PCR1|=PORT_PCR_MUX(2);       //UART RX pin select

}
//This routine is used to setup the UART0 module of KL25Z FRDM Freescale Board
void uart0_init (uint32_t baudrate)
{

	    uint16_t BR;
		clock_configure();          //Configuring Clocks
		gpio_configure();           //Configuring GPIO


		UART0_C2 &= ~(UART0_C2_TE_MASK|UART0_C2_RE_MASK);     //Disabling UART0 RX and TX.
		UART0_C2 |= (UART_C2_RIE_MASK|UART_C2_TIE_MASK);      //Enabling RX and TX Interrupts
	    BR=DEFAULT_SYSTEM_CLOCK/(16*baudrate);                //Calculating the BR ratio
	    UART0_BDL = (uint8_t) BR;                             //Assigning lower 8 bits of BR to UART0_BDL
	    UART0_BDH = (BR)>>baudrate_bh_shift;                  //Assigning the more significant bits to UART0_BDH

	    UART0_C2 |= (UART0_C2_TE_MASK|UART0_C2_RE_MASK);      //Enabling TX and RX
	    __enable_irq();
	    NVIC_EnableIRQ(UART0_IRQn);
 }





