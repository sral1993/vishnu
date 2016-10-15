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

#ifndef SOURCES_UART_H_
#define SOURCES_UART_H_
#include<stdint.h>
#include<circbuff.h>

#define clkdiv    16
#define baudrate_bh_shift     0x0008
#define length                10

struct circular_buffer *tx_buffer;
struct circular_buffer *rx_buffer;
struct node declare_node;
struct node *circbuffer;


//Configuring the clock of UART0 so that writes and reads to UART0 will happen
void clock_configure();
//Configuring or enabling the corresonding GPIO pins for UART RX and TX by selecting the right MUX i/p pins
void gpio_configure();
//This routine is used to setup the UART0 module of KL25Z FRDM Freescale Board
void uart0_init(uint32_t baudrate);
//This function is used to transmit one character of data from the UART
void putachar();
//This function is used to receive a char of data from UART
void getachar();

#endif /* SOURCES_UART_H_ */
#ifndef CIRCBUFF_H
#define CIRCBUFF_H
#include<stdint.h>
#include<stdbool.h>

struct node
{
	uint8_t data;
	struct node *next_data;
};

/*Defining a structure called circular_buffer which is used to define all the characteristics necessary to define a circular buffer and showcase its state*/

struct circular_buffer
{
	struct node *buffer_start;
	struct node *buffer_end;
	struct node *head;
	struct node *tail;
	uint32_t current_length;
	uint32_t max_length;
};


void circ_buff_initialize(struct circular_buffer *buffer1, uint32_t max_length);
void circ_buff_destroy(struct circular_buffer *buffer);
/*Declaring a function buffer_full which is used to check whether the circular buffer is full or not*/
int buffer_full(struct circular_buffer *buffer1);
/*Declaring a function buffer_empty which is used to check whether the circular buffer is empty or not*/
int buffer_empty(struct circular_buffer *buffer1);
/*Declaring a function add_item which adds an item to the circular buffer i.e. the buffer whose address is being sent as the argument*/
void add_item(struct circular_buffer *buffer1, uint8_t item);
/*Declaring a function remove_item which removes an item from the circular buffer i.e. the buffer whose address is being sent as the argument*/
void remove_item(struct circular_buffer *buffer1);

/*Defining a structure called node which is used to store the individual elements of the circular buffer. That is each element of the buffer has
 a data field as well as the adress of the next element*/

/*Declaring an enumeration which is used to state of the circular buffer*/

typedef enum
{
bufferempty=0,
buffernotempty=1
}buffer_state;
#endif
