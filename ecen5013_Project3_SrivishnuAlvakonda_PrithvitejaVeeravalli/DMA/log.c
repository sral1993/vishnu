/*
 * log.c
 *
 *  Created on: Oct 31, 2016
 *      Author: Vishnu
 */

#include "log.h"
#include "uart.h"
#include "string.h"
#include "MKL25Z4.h"
#include "itoa.h"
static int i = 0;
int j=0;


void LOG(char *s,int l)
{
	uart0_init (38400);
		while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
		while(i<l)
		{
		UART0_D=*(s+i);
		i++;
		for(int k=0;k<700;k++);
		}
		i=0;
}
void LOG1(char *p,int l,int param,int data_type_size)
{

char par[l];
my_itoa(par,param ,10);
char* str=strcat(p,par);
LOG(str,strlen(str));
}
