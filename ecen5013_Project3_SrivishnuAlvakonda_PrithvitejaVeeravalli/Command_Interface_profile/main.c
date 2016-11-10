/*
 * main.c
 *
 *  Created on: Nov 09, 2016
 *      Author: Prithvi
 */

#include "MKL25Z4.h"
#include<stdint.h>
#include<string.h>
#include"log.h"
#include<math.h>
#include<stdlib.h>
#include "itoa.h"
#include "uart.h"




void start();
void TPM2_Setup();
int stop();
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

void TPM2_Setup()
{

	SIM_BASE_PTR->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	SIM_BASE_PTR->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	TPM2_BASE_PTR->SC = TPM_SC_CMOD(1) | TPM_SC_PS(0);
	TPM2_BASE_PTR->MOD = 32000;
	TPM2_BASE_PTR->CONTROLS[0].CnSC=TPM_CnSC_MSB_MASK|TPM_CnSC_ELSA_MASK;
	TPM2_BASE_PTR->CONTROLS[0].CnV=TPM2_BASE_PTR->MOD;
}

void start()
{
	TPM2_Setup();
}

int stop()
{
	TPM2_BASE_PTR->SC = TPM_SC_CMOD(0);
	int count=TPM2_CNT;
	int x=count/21;
	return x;
}

void main()
{

	uart0_init (38400);
	cmd_ptr->command=0x65;
	cmd_ptr->len=0x66;
	cmd_ptr->data[0]=0x67;
	cmd_ptr->checksum=0x68;
		start();


		while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
		UART0_D= cmd_ptr->command;
		while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
		UART0_D = cmd_ptr->len;
		while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
		UART0_D = cmd_ptr->data[0];
		while(!(UART0->S1 & UART_S1_TDRE_MASK) && !(UART0->S1 &UART_S1_TC_MASK));
		UART0_D = cmd_ptr->checksum;
		int time3=stop();
		char str2[]="                                                                                      time for one message of command interface in us :";
		int l2=strlen(str2);
		LOG1(str2,l2,time3,8);



}





