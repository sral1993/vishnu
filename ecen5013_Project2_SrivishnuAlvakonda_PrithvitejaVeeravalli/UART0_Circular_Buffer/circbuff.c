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


#include<stdio.h>
#include<stdint.h>
#include<stdlib.h>
#include "uart.h"
#include "MKL25Z4.h"
#include "circbuff.h"


struct circular_buffer buffer;
struct node declare_node;
struct node *circbuffer;

void circ_buff_initialize(struct circular_buffer *buffer, uint32_t max_length)
{
	 circbuffer=calloc(max_length, sizeof(declare_node));
	 buffer->max_length = max_length;
	 buffer->buffer_start = circbuffer;
	 buffer->buffer_end = buffer->buffer_start + sizeof(declare_node)*(buffer->max_length);
	 (buffer->buffer_end)->next_data = buffer->buffer_start;
	 buffer->current_length=0;

	 buffer->head = buffer->buffer_start;
	 buffer->tail = buffer->buffer_start - sizeof(declare_node);


}

void circ_buff_destroy(struct circular_buffer *buffer)
{
	free(circbuffer);
	free(buffer);

}


/*Declaring a function buffer_full which is used to check whether the circular buffer is full or not. It returns a value of 1 if the buffer is full and 0
if not full*/ 

int buffer_full(struct circular_buffer *buffer)
{
  /*Checking if the current_length of circular buffer is equal to the max_length of the circular buffer*/ 
	if(buffer->current_length==buffer->max_length)
	{
		printf("Circular Buffer is full");
		return 1;
	}
	else
		return 0;

}

/*Declaring a function buffer_empty which is used to check whether the circular buffer is empty or not. It returns a value of 1 if the buffer is empty and
o if the buffer is not empty*/

int buffer_empty(struct circular_buffer *buffer)
{
/*Checking if the current length of circular buffer is equal to zero.*/
	if(buffer->current_length==bufferempty)
	{
		printf("Circular Buffer is empty");
		return 1;
	}
	else
		return 0;
}

/*Declaring a function add_item which adds an item to the circular buffer i.e. the buffer whose address is being sent as the argument. If there is an 
overfill of data it does a wrap arund and it returns an error message and terminates the program*/

void add_item(struct circular_buffer *buffer, uint8_t item)
{
	struct node *temp;

/*If the current_length is less than the max_length than the tail is mooved to the node storing the new element and the data field of the new element is loaded with 
the value in variable item*/

	if(buffer->current_length!=buffer->max_length)
	{
		(buffer->tail)->next_data=buffer->tail+sizeof(declare_node);
		temp=(buffer->tail)->next_data;
		temp->data=item;
		buffer->tail=temp;
		buffer->current_length++;
	}

/*if the current_length is equal to the max_length than the head is moved to the 2nd oldest element and the new data item replaces the 1st oldest item.
After the wrap around the program is terminated showing an eerror message*/

	else
	{
		printf("Error:Overfill of Data and data has been wrapped around");
		temp=buffer->head;
		buffer->head=temp->next_data;
		temp->data=item;
		buffer->tail=temp;
		exit(1);

	}
}

/*Declaring a function remove_item which removes an item from the circular buffer i.e. the buffer whose address is being sent as the argument*/

void remove_item(struct circular_buffer *buffer)
{
	struct node *temp;

/*If the circular buffer is not empty the data item is removed from head and the head is moved to the next oldest element*/

	if(buffer->current_length!=bufferempty)
	{
		temp=buffer->head;
		buffer->head=temp->next_data;
		buffer->current_length--;
	}

/*If the circular buffer is empty data item can't be removed and hence it returns an error messag and terminates the current process making use of 
exit function*/
 
	else
	{
		printf("Error:Trying to extract data out of an empty circular buffer");
	    exit(1);
	}
}
