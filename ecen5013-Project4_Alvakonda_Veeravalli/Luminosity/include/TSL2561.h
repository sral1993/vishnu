/*
 * lumos.h
 *
 *  Created on: November 25, 2016
 *      Author: Prithvi
 */

#ifndef TSL2561_H_
#define TSL2561_H_

        //#define TSL2561_enable                      true
        #define TSL2561_power_port                  gpioPortD
        #define TSL2561_power_pin                   0U
        #define TSL2561_interrupt_port              gpioPortD
        #define TSL2561_interrupt_pin               1U
        #define TSL2561_scl_port                    gpioPortC
        #define TSL2561_scl_pin                     5U
        #define TSL2561_sda_port                    gpioPortC
        #define TSL2561_sda_pin                     4U
        //#define TSL2561_GND_port                    gpioPortC
        //#define TSL2561_GND_pin                     6U

        #define I2C_FastMode                        400000
        #define TSL2561_ctrl_addr                   0x80
        #define TSL2561_timing_addr                 0x81
        #define TSL2561_lowthresholdlow_addr        0x82
        #define TSL2561_lowthresholdhigh_addr       0x83
        #define TSL2561_highthresholdlow_addr       0x84
        #define TSL2561_highthresholdhigh_addr      0x85
        #define TSL2561_interruptctrlreg_addr       0x86
        #define TSL2561_ADC0_Low_Reg                0xAC
        #define TSL2561_ADC0_High_Reg               0xAD
        #define TSL2561_power_down       0x00
        #define TSL2561_power_up         0x03
        #define TSL2561_integration_time 0x01
        #define TSL2561_lowthresholdlow_byte        0x0f
        #define TSL2561_lowthresholdhigh_byte       0x00
        #define TSL2561_highthresholdlow_byte       0x00
        #define TSL2561_highthresholdhigh_byte      0x08
        #define TSL2561_intcontrolregister_byte     0x14
        #define TSL2561_slaveaddress                0x39
        #define I2C_FLAG_WRITE                      0x0001
        #define I2C_FLAG_WRITE_READ                 0x0004
        #define TSL2561_period1                     0x0001
        #define TSL2561_period2                     0x0002
        #define TSL2561_period3                     0x0003

//        uint8_t I2C_tx_buffer[2]={0x00, 0x00};
//        uint8_t I2C_txaddr_buffer[1]={0x00};
//        uint8_t I2C_rx_buffer=0x00;
//        uint8_t I2C_period_number=1;
        uint8_t ADC0_Reg_Low, ADC0_Reg_High;


        void TSL2561_Init();
        void TSL2561_GPIO_Int_Enable();

#endif

