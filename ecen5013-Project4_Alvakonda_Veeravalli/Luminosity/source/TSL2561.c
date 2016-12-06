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
	I2C_Write_Register(TSL2561_slaveaddress, TSL2561_ctrl_addr, TSL2561_power_down);
	I2C_Write_Register(TSL2561_slaveaddress, TSL2561_ctrl_addr, TSL2561_power_up);
	I2C_Write_Register(TSL2561_slaveaddress, TSL2561_timing_addr, TSL2561_integration_time);
	I2C_Write_Register(TSL2561_slaveaddress, TSL2561_lowthresholdlow_addr, TSL2561_lowthresholdlow_byte);
	I2C_Write_Register(TSL2561_slaveaddress, TSL2561_lowthresholdhigh_addr, TSL2561_lowthresholdhigh_byte);
	I2C_Write_Register(TSL2561_slaveaddress, TSL2561_highthresholdlow_addr, TSL2561_highthresholdlow_byte);
	I2C_Write_Register(TSL2561_slaveaddress, TSL2561_highthresholdhigh_addr, TSL2561_highthresholdhigh_byte);
	I2C_Write_Register(TSL2561_slaveaddress, TSL2561_interruptctrlreg_addr, TSL2561_intcontrolregister_byte);
}

void TSL2561_GPIO_Int_Enable()
{
	PORTA_PCR1 = 0x00;
	PORTA_PCR1 |= PORT_PCR_MUX(1)|PORT_PCR_IRQC(10);
	__enable_irq();
	NVIC_EnableIRQ(PORTA_IRQn);
}
void PORTA_IRQHandler()
{
//	uint8_t ADC0_Reg_Low, ADC0_Reg_High;
	uint16_t ambience_reading;
	PORTA_PCR1 |= PORT_PCR_ISF_MASK;
	I2C_Read_Register(TSL2561_slaveaddress, TSL2561_ADC0_Low_Reg );
//	I2C_Read_Register(TSL2561_slaveaddress, TSL2561_ADC0_Low_Reg);
//	ADC0_Reg_High = I2C_Read_Register(TSL2561_slaveaddress, TSL2561_ADC0_High_Reg );
    ambience_reading = ADC0_Reg_High*256 + ADC0_Reg_Low;
}
