/*
 * uart.h
 *
 *  Created on: Oct 12, 2016
 *      Author: Prithvi
 */

#ifndef SOURCES_UART_H_
#define SOURCES_UART_H_

#define clkdiv    16
#define baudrate_bh_shift   0x0008

void clock_configure();
void gpio_configure();
void uart0_init (uint32_t baud);
#endif /* SOURCES_UART_H_ */
