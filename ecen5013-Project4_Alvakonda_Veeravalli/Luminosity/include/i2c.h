/*
 * i2c.h
 *
 *  Created on: Nov 25, 2016
 *      Author: Prithvi
 */

#ifndef SOURCES_I2C_H_
#define SOURCES_I2C_H_

#define WRITE                   0x00  /* Master write  */
#define READ                    0x01  /* Master read */
#define I2C_NACK()             I2C0_C1 |= I2C_C1_TXAK_MASK/*for sending NACK*/
#define I2C_ACK()              I2C0_C1 &= ~I2C_C1_TXAK_MASK/*for sending ACK*/
#define I2C_RS()               I2C0_C1 |= I2C_C1_RSTA_MASK/* for repeated start*/
#define I2C_Rx()               I2C0_C1 &= ~I2C_C1_TX_MASK/*I2C in RX mode*/
#define I2C_WRITE(data)        I2C0_D = data/*write to the data register*/
/*I2C in Tx Mode*/
#define I2C_Start()            I2C0_C1 |= I2C_C1_TX_MASK;\
                               I2C0_C1 |= I2C_C1_MST_MASK

#define I2C_Stop()             I2C0_C1 &= ~I2C_C1_MST_MASK;\
                               I2C0_C1 &= ~I2C_C1_TX_MASK

#define I2C_Wait()             while((I2C0_S & I2C_S_IICIF_MASK)==0) {} \
                               I2C0_S |= I2C_S_IICIF_MASK;/*wait till pending interrupts are cleared*/

void Init_I2C(void);
void I2C_StartTx(char SlaveID, char Mode);
void I2CWriteRegister(char SlaveID, char RegisterAddress, char Data);
char I2CReadRegister(char SlaveID, char RegisterAddress);
void I2CReadMultiRegisters(char SlaveID, char RegisterAddress, char n, char * r);




#endif /* SOURCES_I2C_H_ */

