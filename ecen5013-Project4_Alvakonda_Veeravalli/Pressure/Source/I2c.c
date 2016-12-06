/*
 * i2c.c
 *
 *  Created on: Nov 25, 2016
 *      Author: Vishnu
 */


#include "MKL25Z4.h"
#include "stdint.h"
#include "i2c.h"

void I2C_Clock_Configure()
{
	SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
}
void I2C_GPIO_Setup()
{
	PORTB_PCR0 |= PORT_PCR_MUX(2);
	PORTB_PCR1 |= PORT_PCR_MUX(2);
	PORTB_PCR3 |= PORT_PCR_MUX(1)|PORT_PCR_IRQC(0x1010);
}

void I2C_Init()
{
	I2C_Clock_Configure();
	I2C_GPIO_Setup();
	I2C0_F |= I2C_F_MULT(0)|I2C_F_ICR(0x1F);
	I2C0_C1 |= I2C_C1_IICEN_MASK;
}

void I2C_Write_Byte(uint8_t data)
{
	I2C0_D = data;
	while((I2C0_S & I2C_S_IICIF_MASK)==0);
	I2C0_S |= I2C_S_IICIF_MASK;
}

uint8_t I2C_Read_Byte()
{
	uint8_t received_byte;
	received_byte=I2C0_D;

	while((I2C0_S & I2C_S_IICIF_MASK)==0);
	I2C0_S |= I2C_S_IICIF_MASK;
	I2C0_C1 &= ~I2C_C1_MST_MASK;

	received_byte = I2C0_D;
	return received_byte;
}

uint8_t I2C_Read_Register(uint8_t slave_address, uint8_t register_address)
{
	uint8_t received_byte;

	I2C_Set_TX_Mode();
	I2C_Start();

	I2C_Write_Byte((slave_address<<slave_address_shift)|master_write);

	I2C_Write_Byte(register_address);

	I2C_ReStart();

	I2C_Write_Byte((slave_address<<slave_address_shift)|master_read);

	I2C_Set_RX_Mode();

	received_byte = I2C_Read_Byte();
	return received_byte;
}
void I2C_Write_Register(uint8_t slave_address, uint8_t register_address, uint8_t byte)
{
	I2C_Set_TX_Mode();
	I2C_Start();

	I2C_Write_Byte((slave_address<<slave_address_shift)|master_write);

	I2C_Write_Byte(register_address);

	I2C_Write_Byte(byte);

	I2C_Stop();
    I2C0_C1 &= ~I2C_C1_TX_MASK;
}
