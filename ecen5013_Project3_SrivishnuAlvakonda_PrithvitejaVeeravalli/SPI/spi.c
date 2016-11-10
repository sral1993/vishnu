/*
 * spi.c
 *
 *  Created on: Nov 9, 2016
 *      Author: Prithvi
 */
#include "MKL25Z4.h"
#include "spi.h"
#include "nrf24.h"

void spi_clock_configure();
void spi_gpio_setup();

void spi_clock_configure()
{
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	   // Turn on SPI0 clock
	   SIM_SCGC4 |= SIM_SCGC4_SPI0_MASK;    // Set up ports for SPI0
}

void spi_gpio_setup()
{
	PORTD->PCR[0] |= PORT_PCR_MUX(1); // ss as gpio pin
	PORTD->PCR[1] |= PORT_PCR_MUX(2);
	PORTD->PCR[2] |= PORT_PCR_MUX(2);
	PORTD->PCR[3] |= PORT_PCR_MUX(2);    // Set port C, pin 8 data direction to output

	PTD_BASE_PTR->PDDR |= 1<<0;
}
void spi_init()
{
spi_clock_configure();

spi_gpio_setup();

   SPI0_C1 = SPI_C1_MSTR_MASK | SPI_C1_SPE_MASK;

   SPI_BR_REG(SPI0) = 0x03;
}

uint8_t spi_send(uint8_t command)
{
	uint8_t rx_ret;
    while(WAIT_SPTEF);
    SPI0_D = command;
    while(WAIT_SPRF);
    rx_ret = SPI0_D;

    return(rx_ret);

}

void nrf_write(uint8_t command, uint8_t *data, uint8_t length )
{
       uint8_t rx_ret = 0;

       NRF_CS_ENABLE();

       spi_send(command);

       while(length>0)
       {
       while(WAIT_SPTEF);
       SPI0_D = *data;
       while(WAIT_SPRF);
       rx_ret = SPI0_D;
       length--;
       data++;
       }
       NRF_CS_DISABLE();

}

void nrf_read(uint8_t command, uint8_t *read_data, uint8_t length)
{
       uint8_t rx_ret = 0;

       NRF_CS_ENABLE();

       spi_send(command);

       while(length>0)
       {
    	  *read_data = nrf_dummy();
    	  read_data++;
    	  length--;
       }
       NRF_CS_DISABLE();

}

void nrf_flush_tx()
{
	uint8_t rx_ret = 0;

	NRF_CS_ENABLE();

    spi_send(FLUSH_TX);

	NRF_CS_DISABLE();
}

void nrf_flush_rx()
{
	uint8_t rx_ret = 0;

	NRF_CS_ENABLE();

    spi_send(FLUSH_RX);

	NRF_CS_DISABLE();
}

void nRF_write_tx_payload(uint8_t *write_data )
{
	 uint8_t rx_ret = 0;
     uint8_t length = 32;
	 NRF_CS_ENABLE();

     spi_send(W_TX_PAYLOAD);

	 while(length>0)
	 {
	 while(WAIT_SPTEF);
	 SPI0_D = *write_data;
	 while(WAIT_SPRF);
	 rx_ret = SPI0_D;
	 length--;
	 write_data++;
	 }
	 NRF_CS_DISABLE();
}

uint8_t nrf_dummy(void)
{
    uint8_t rx_ret = 0;

    rx_ret = spi_send(NOP);

    return(rx_ret);
}

void nRF_set_tx_address(uint8_t *write_data)
{
	uint8_t rx_ret = 0;
	uint8_t length = 5;

	NRF_CS_ENABLE();

    spi_send(W_REGISTER | TX_ADDR);

	while(length>0)
	{
		while(WAIT_SPTEF);
		SPI0_D = *write_data;
		while(WAIT_SPRF);
		rx_ret = SPI0_D;
		length--;
		write_data++;

	}

	NRF_CS_DISABLE();

}
