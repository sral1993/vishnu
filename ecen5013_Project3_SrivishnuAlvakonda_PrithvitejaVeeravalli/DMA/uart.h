/*
 * uart.h
 *
 *  Created on: Oct 31, 2016
 *      Author: Vishnu
 */

#ifndef SOURCES_UART_H_
#define SOURCES_UART_H_
#include<stdint.h>

#define clkdiv    16
#define baudrate_bh_shift     0x0008
#define length                10

//Configuring the clock of UART0 so that writes and reads to UART0 will happen
void clock_configure();
//Configuring or enabling the corresonding GPIO pins for UART RX and TX by selecting the right MUX i/p pins
void gpio_configure();
//This routine is used to setup the UART0 module of KL25Z FRDM Freescale Board
void uart0_init(uint32_t baudrate);
//This function is used to transmit one character of data from the UART
void putachar(char *str, uint16_t len);
//This function is used to receive a char of data from UART
void getachar();

#endif /* SOURCES_UART_H_ */
