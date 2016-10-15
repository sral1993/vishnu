/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc
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
#include "circbuff.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>


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
#define length                10
char transmit_data[length] = "embedded";
char receive_data[length];
char *txstr = transmit_data;
char *rxstr = receive_data;
uint8_t rx_data;
uint16_t initial=1;
uint16_t receive=1;
uint16_t tx_buffer_empty=0;
char operation_tx_rx;

//This function is used to send a char of data from TX of UART0
void putachar()
{
    if(!(tx_buffer_empty))
    {
    UART0_D=(tx_buffer->head)->data;
    remove_item(tx_buffer); //This function is used to transmit the data loaded in the tx buffer through the TX of UART0
    if(tx_buffer->current_length>0)
    {

    }
    else
        tx_buffer_empty=1;
    }
}
//This function is used to get a char of data from RX of UART0
void getachar()
{
    if(rx_buffer->current_length<rx_buffer->max_length)
    {
    rx_data = UART0_D;
    add_item(rx_buffer, rx_data); //This function is used to take the keyed in element and store it in the RX buffer
    }
    else
        receive=0;
}
//This function is used to receive data from the serial terminal through the RX of UART 0 and print it by transmitting it to the serial terminal via transmit buffer
void UART0_IRQHandler()
{
    if((UART0_S1 & UART0_S1_RDRF_MASK)&&(receive))
    {
        getachar();
    }
    if((UART0_S1 & UART0_S1_TDRE_MASK)&&((!receive)))
    {
        if(initial==1)
        {
        while(tx_buffer->current_length<tx_buffer->max_length)
        {
            (tx_buffer->tail)->next_data=tx_buffer->tail+sizeof(declare_node);
            tx_buffer->tail=(tx_buffer->tail)->next_data;
            (tx_buffer->tail)->data=(rx_buffer->head)->data;
            remove_item(rx_buffer);
            tx_buffer->current_length++;
        }
        initial=0;
        }
        putachar();
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
		clock_configure();             //Configuring Clocks
		gpio_configure();              //Configuring GPIO


		UART0_C2 &= ~(UART0_C2_TE_MASK|UART0_C2_RE_MASK);     //Disabling UART0 RX and TX.
		UART0_C2 |= (UART_C2_RIE_MASK|UART_C2_TIE_MASK);      //Enabling RX and TX Interrupts
	    BR=DEFAULT_SYSTEM_CLOCK/(16*baudrate);                //Calculating the BR ratio
	    UART0_BDL = (uint8_t) BR;                             //Assigning lower 8 bits of BR to UART0_BDL
	    UART0_BDH = (BR)>>baudrate_bh_shift;                  //Assigning the more significant bits to UART0_BDH

	    UART0_C2 |= (UART0_C2_TE_MASK|UART0_C2_RE_MASK);      //Enabling TX and RX
	    __enable_irq();
	    NVIC_EnableIRQ(UART0_IRQn);
 }
