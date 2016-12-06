/*
 * main.c
 *
 *  Created on: Nov 25, 2016
 *      Author: Prithvi
 */

#include "MKL25Z4.h"
#include "stdint.h"
#include "i2c.h"
#include "TSL2561.h"

static int i = 0;

int main(void)
{
	I2C_Init();
	TSL2561_Init();
	TSL2561_GPIO_Int_Enable();
	while(1);
//	I2C_Read_Register(TSL2561_slaveaddress, TSL2561_ADC0_Low_Reg );
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
