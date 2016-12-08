/*
 * main.c
 *
 *  Created on: Dec 5, 2016
 *      Author: Vishnu
 */
#include "MKL25Z4.h"
#include "led.h"
#include "i2c.h"
static int i = 0;
char data[6];
uint8_t f;
uint8_t CTRL_REG1_Data;
uint8_t Int_SourceSystem;
uint8_t Int_SourceTrans;


int main(void)
{
	Init_I2C();
	I2CWriteRegister(0x1D, 0x2A, 0x18);
	I2CWriteRegister(0x1D, 0x1D, 0x16);
	I2CWriteRegister(0x1D, 0x1F, 0x08);
	I2CWriteRegister(0x1D, 0x20, 0x05);
	I2CWriteRegister(0x1D, 0x2D, 0x20);
	I2CWriteRegister(0x1D, 0x2E, 0x20);
    CTRL_REG1_Data = I2CReadRegister(0x1D,0x2A); //Read out the contents of the register
	CTRL_REG1_Data|= 0x01; //Change the value in the register to Active Mode.
	I2CWriteRegister(0x1D, 0x2A,CTRL_REG1_Data);//Write in the updated value to put the device in
	                                            //Active Mode
	//f=I2CReadRegister(0x1D, 0x2A);
	//I2CReadMultiRegisters(0x1D, 0x01, 6, data);
}
void PORTA_IRQHandler(){
	PORTA_PCR14 |= PORT_PCR_ISF_MASK;
	Int_SourceSystem = I2CReadRegister(0x1D,0x0C);
	if ((Int_SourceSystem & 0x20)==0x20)
	{
		Blue(32000);
	//Perform an Action since Transient Flag has been set
	//Read the Transient to clear system interrupt and Transient
	Int_SourceTrans = I2CReadRegister(0x1D,0x1E);
	//Can parse out the data to perform a specific action based on the
	//axes or direction that made the condition true
	}
	//off(32000);
}
