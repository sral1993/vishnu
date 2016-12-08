/*
 * uart.c
 *
 *  Created on: Nov 25, 2016
 *      Author: Vishnu
 */





#include "MKL25Z4.h"
//#include "core_cm4.h"
#include "core_cmFunc.h"
#include "uart.h"
#include <stdio.h>
/***************************************************************************
 * Begin UART0 functions
 **************************************************************************/
/********************************************************************/
/*
 * Initialize the uart for 8N1 operation, interrupts disabled, and
 * no hardware flow-control
 * Parameters:
 *  sysclk      uart module Clock in kHz(used to calculate baud)
 *  baud        uart baud rate
 */

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
    UART0_C2 &= ~(UART0_C2_TE_MASK|UART0_C2_RE_MASK);
    UART0_C2 |= UART_C2_RIE_MASK;
    BR=DEFAULT_SYSTEM_CLOCK/(clkdiv*baudrate);
   	UART0_BDL = (uint8_t) BR;
   	UART0_BDH = (BR)>>baudrate_bh_shift;
    UART0_C2 |= UART0_C2_TE_MASK|UART0_C2_RE_MASK;
    __enable_irq();
    NVIC_EnableIRQ(UART0_IRQn);

     }
