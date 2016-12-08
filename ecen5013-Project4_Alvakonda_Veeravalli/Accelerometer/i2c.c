/*
 * i2c.c
 *
 *  Created on: Dec 7, 2016
 *      Author: Vishnu
 */
#include "i2c.h"
#include "MKL25Z4.h"
void Init_I2C(void)
{
  SIM_SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTE_MASK;// Turn on clock to PortA and Port E module
  SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;  // Turn on clock to I2C0 module
  PORTE_PCR24 = PORT_PCR_MUX(5);  // PTE24 pin is I2C0 SCL line
  PORTE_PCR25 = PORT_PCR_MUX(5);  // PTE25 pin is I2C0 SDA line
  /*
  	     	 * I2C0 is clocked by the bus clock, that is core/2.
  	     	 * For the MMA8451Q inertial sensor on the FRDM-25KLZ board the
  	     	 * maximum SCL frequency is 400 kHz.
  	     	 * Assuming PEE mode with core=48MHz, 400 kHz = 48MHz/2 / 60,
  	     	 * which means a prescaler (SCL divider) of 60.
  	     	 * According to table 38-41, I2C divider and hold values,
  	     	 * the closest SCL diver is 64 (375 kHz SCL), which is ICR value 0x12.
  	     	 * Alternatively, the SCL divider of 30 can be used (ICR=0x05) in combination
  	     	 * with a multiplicator of 2 (MULT=0x01).
  	     	 * A note states that ICR values lower than 0x10 might result in a varying
  	     	 * SCL divider (+/- 4). However the data sheet does not state anything
  	     	 * useful about that.
  	     	 */
  	     I2C0_F = I2C_F_MULT(0x00) | I2C_F_ICR(0x12); /* divide by 64 instead, so 375 kHz */
  I2C0_C1 = I2C_C1_IICEN_MASK;  /* enable the I2C module */
  PORTA_PCR14 |= (0|PORT_PCR_ISF_MASK|     // Clear the interrupt flag
  	     	                       PORT_PCR_MUX(0x1)|     // PTA14 is configured as GPIO
  	     	                       PORT_PCR_IRQC(0xA));   // PTA14 is configured for falling edge interrupts

  	     	        //Enable PORTA interrupt on NVIC
  	     	    __enable_irq();
  	     	   NVIC_EnableIRQ(PORTA_IRQn);
}
void I2C_StartTx (char SlaveID, char Mode)
{
  SlaveID = SlaveID << 1;
  SlaveID |= (Mode & 0x01);
  I2C_Start();
  I2C_WRITE(SlaveID);
}

void Pause(void){
    int n;
    for(n=1;n<50;n++) {
    }
}
char I2CReadRegister(char SlaveID, char RegisterAddress)
{
  char result;

  I2C_StartTx(SlaveID,WRITE);
  I2C_Wait();
  I2C_WRITE(RegisterAddress);
  I2C_Wait();
  I2C_RS();
  I2C_WRITE((SlaveID << 1) | READ);
  I2C_Wait();
  I2C_Rx();
  I2C_NACK();// Send NACK to tell dat dis is last transmission
  result = I2C0_D ; //In master receive mode, reading this register
                       // initiates receiving of the next byte of data.
  I2C_Wait();
  I2C_Stop(); // Send Stop
  result = I2C0_D ; // reading the register
  Pause();
  return result;
}

void I2CWriteRegister(char SlaveID, char RegisterAddress, char Data)
{
  I2C_StartTx(SlaveID,WRITE);
  I2C_Wait();
  I2C_WRITE(RegisterAddress);
  I2C_Wait();
  I2C_WRITE(Data);
  I2C_Wait();
  I2C_Stop();
  Pause();
}
void I2CReadMultiRegisters(char SlaveID, char RegisterAddress, char n, char * r)
{
  char i;

  I2C_StartTx(SlaveID,WRITE);
  I2C_Wait();
  I2C_WRITE(RegisterAddress);
  I2C_Wait();
  I2C_RS();
  I2C_WRITE((SlaveID << 1) | 0x01);
  I2C_Wait();
  I2C_Rx();
  I2C_ACK();
  i = I2C0_D;
  for(i=0;i<n-2;i++)
  {
    *r = I2C0_D;
    r++;
    I2C_Wait();
  }
  I2C_NACK();
  *r = I2C0_D;
  r++;
  I2C_Wait();
  I2C_Stop();
  *r = I2C0_D;
  Pause();
}
