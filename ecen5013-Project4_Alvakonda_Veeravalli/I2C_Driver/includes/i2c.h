/*
 * i2c.h
 *
 *  Created on: Nov 25, 2016
 *      Author: Prithvi
 */
 
 /*
 * i2c.h
 *
 *  Created on: Nov 24, 2016
 *      Author: prith
 */

#ifndef INCLUDES_I2C_H_
#define INCLUDES_I2C_H_

#define slave_address_shift   0x01
#define master_read  0x01
#define master_write 0x00

#define I2C_ReStart() I2C0_C1 |= I2C_C1_RSTA_MASK
#define I2C_Start()   I2C0_C1 |= I2C_C1_MST_MASK
#define I2C_Stop()    I2C0_C1 &= ~I2C_C1_MST_MASK
#define I2C_Set_TX_Mode() I2C0_C1 |= I2C_C1_TX_MASK
#define I2C_Set_RX_Mode() I2C0_C1 &= ~I2C_C1_TXAK_MASK;\
                          I2C0_C1 &= ~I2C_C1_TX_MASK

void I2C_Clock_Configure();
void I2C_GPIO_Setup();
void I2C_Init();
void I2C_Write_Byte(uint8_t data);
uint8_t I2C_Read_Byte();
uint8_t I2C_Read_Register(uint8_t slave_address, uint8_t register_address);
void I2C_Write_Register(uint8_t slave_address, uint8_t register_address, uint8_t byte);

#endif /* INCLUDES_I2C_H_ */
