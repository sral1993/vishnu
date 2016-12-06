/*
 * pressure.c
 *
 *  Created on: Nov 25, 2016
 *      Author: Vishnu
 */

#include "MKL25Z4.h"
#include "stdint.h"
#include "pressure.h"
#include "i2c.h"

void BME_Configuration()
 {
	 uint8_t T_ovs = 1;             //Temperature oversampling x 1
	 uint8_t P_ovs = 1;             //Pressure oversampling x 1
	 uint8_t Mode = 3;               //Normal mode
	 uint8_t Standbytime = 5;               //Tstandby 1000ms
	 uint8_t IIR_Filter = 0;             //Filter off

     uint8_t control_reg_data;
     uint8_t config_reg_data;
	 control_reg_data = (T_ovs << 5) | (P_ovs << 2) | Mode;
	 config_reg_data = (Standbytime << 5) | (IIR_Filter << 2);

	 I2C_Write_Register(0x76, control_reg, control_reg_data);
	 I2C_Write_Register(0x76, config_reg, config_reg_data);
	 read_trimmed_data();

 }

 void read_trimmed_data()
 {
	  uint8_t data[32];
	  uint8_t i=0;
	  int a=calib00;
      while(i<24)
	 {
	 req_address = a;
	// data[i]= I2C_Read(1);
	 data[i]= I2C_Read_Register(0x76, req_address);
	 i++;
	 a++;
	 }

	dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
 }

 void read_rawdata()
 {


	 int i = 0;
	 uint32_t data[8];
	 int d=Pressure_MSB;

	 while(i<6)
	 {
		 req_address = d;
		 //data[i]=I2C_Read(1);
		 data[i]= I2C_Read_Register(0x76, req_address);
		 i++;
		 d++;
	 }
	 Raw_pressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
	 Raw_temperature = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
 }

 void sensor_calibration()
 {
	 double Final_Temp = 0.0, Final_Press = 0.0;
	 signed long int Calibrated_Temperature;
	 unsigned long int Calibrated_Pressure;
	 Calibrated_Temperature = calibration_T(Raw_temperature);
	 Calibrated_Pressure = calibration_P(Raw_pressure);
	 Final_Temp = (float)Calibrated_Temperature / 100.0;
	 Final_Press = (float)Calibrated_Pressure / 100.0;



 }



 signed long int calibration_T(signed long int adc_T)
 {
	 signed long int var1, var2, T;
	 var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
	 var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
	 t_fine = var1 + var2;
	 T = (t_fine * 5 + 128) >> 8;
	 return T;
 }

 unsigned long int calibration_P(signed long int adc_P)
 {
     signed long int var1, var2;
     unsigned long int P;
     var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
     var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
     var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
     var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
     var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
     var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
     if (var1 == 0)
     {
         return 0;
     }
     P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
     if(P<0x80000000)
     {
        P = (P << 1) / ((unsigned long int) var1);
     }
     else
     {
         P = (P / (unsigned long int)var1) * 2;
     }
     var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
     var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
     P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
     return P;
 }
