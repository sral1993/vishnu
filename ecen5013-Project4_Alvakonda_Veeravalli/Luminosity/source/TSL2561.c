/*
 * TSL2561.c
 *
 *  Created on: Nov 25, 2016
 *      Author: Prithvi
 */

/*
 * TSL2561.c
 *
 *  Created on: Nov 25, 2016
 *      Author: Prithvi
 */

#include "MKL25Z4.h"
#include "TSL2561.h"
#include "stdint.h"
#include "i2c.h"

void TSL2561_Init()
{
	I2CWriteRegister(TSL2561_slaveaddress, TSL2561_ctrl_addr, TSL2561_power_down);
	I2CWriteRegister(TSL2561_slaveaddress, TSL2561_ctrl_addr, TSL2561_power_up);
	I2CWriteRegister(TSL2561_slaveaddress, TSL2561_timing_addr, TSL2561_integration_time);
	I2CWriteRegister(TSL2561_slaveaddress, TSL2561_lowthresholdlow_addr, TSL2561_lowthresholdlow_byte);
	I2CWriteRegister(TSL2561_slaveaddress, TSL2561_lowthresholdhigh_addr, TSL2561_lowthresholdhigh_byte);
	I2CWriteRegister(TSL2561_slaveaddress, TSL2561_highthresholdlow_addr, TSL2561_highthresholdlow_byte);
	I2CWriteRegister(TSL2561_slaveaddress, TSL2561_highthresholdhigh_addr, TSL2561_highthresholdhigh_byte);
	I2CWriteRegister(TSL2561_slaveaddress, TSL2561_interruptctrlreg_addr, TSL2561_intcontrolregister_byte);
}

void TSL2561_GPIO_Int_Enable()
{
	PORTD_PCR1 = 0x00;
	PORTD_PCR1 |= PORT_PCR_MUX(1)|PORT_PCR_IRQC(0b1011);
	__enable_irq();
	NVIC_EnableIRQ(PORTD_IRQn);
}

