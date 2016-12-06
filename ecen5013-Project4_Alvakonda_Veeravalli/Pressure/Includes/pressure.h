/*
 * pressure.h
 *
 *  Created on: Nov 25, 2016
 *      Author: Vishnu
 */

#ifndef INCLUDES_PRESSURE_H_
#define INCLUDES_PRESSURE_H_


uint8_t byte_num;





//#define slave_address  0x76
#define control_reg 0xF4
#define config_reg  0xF5
#define status_reg  0xF3
#define temp_xlsb   0xFC
#define temp_lsb    0xFB
#define temp_msb	0xFA
#define press_xlsb	0xF9
#define press_lsb   0xF8
#define Pressure_MSB	0xF7
#define calib26		0xE1
#define calib41		0xF0
#define calib25		0xA1
#define calib00		0x88
#define reset		0xE0
#define id			0xD0

uint16_t req_address;

unsigned long int Raw_temperature,Raw_pressure;
signed long int t_fine;

uint8_t return_data;

uint16_t dig_T1;
 int16_t dig_T2;
 int16_t dig_T3;
uint16_t dig_P1;
 int16_t dig_P2;
 int16_t dig_P3;
 int16_t dig_P4;
 int16_t dig_P5;
 int16_t dig_P6;
 int16_t dig_P7;
 int16_t dig_P8;
 int16_t dig_P9;






void BME_Configuration();
void read_trimmed_data();
void read_rawdata();
void sensor_calibration();
signed long int calibration_T(signed long int adc_T);
unsigned long int calibration_P(signed long int adc_P);





#endif /* INCLUDES_PRESSURE_H_ */
