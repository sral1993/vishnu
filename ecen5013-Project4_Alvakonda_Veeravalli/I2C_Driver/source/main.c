/*
 * main.c
 *
 *  Created on: Nov 25, 2016
 *      Author: Prithvi
 */
#include "MKL25Z4.h"
#include "stdint.h"
#include "i2c.h"

static int i = 0;


int main(void)
{
uint8_t register_read;
I2C_Init();
I2C_Write_Register(0x39, 0x82, 0x52);
register_read = I2C_Read_Register(0x39, 0x82);

}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
