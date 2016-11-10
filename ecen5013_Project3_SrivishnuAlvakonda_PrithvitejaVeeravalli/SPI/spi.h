/*
 * spi.h
 *
 *  Created on: Nov 9, 2016
 *      Author: Prithvi
 */

#ifndef SOURCES_SPI_H_
#define SOURCES_SPI_H_

#define NRF_CS_ENABLE() (PTD_BASE_PTR->PCOR |= 1<<0)
#define NRF_CS_DISABLE() (PTD_BASE_PTR->PSOR |= 1<<0)
#define WAIT_SPTEF ( !(SPI_S_REG(SPI0) & SPI_S_SPTEF_MASK))
#define WAIT_SPRF ( !(SPI_S_REG(SPI0) & SPI_S_SPRF_MASK))

void spi_init();
void nrf_write(uint8_t command, uint8_t *data, uint8_t length );
void nrf_read(uint8_t command, uint8_t *read_data, uint8_t length);
void nRF_set_tx_address(uint8_t *write_data);
void nRF_write_tx_payload(uint8_t *write_data);
void nrf_flush_tx();
void nrf_flush_rx();
void nRF_set_tx_address(uint8_t *read_data);
uint8_t nrf_dummy(void);
uint8_t spi_send(uint8_t command);



#endif /* SOURCES_SPI_H_ */
