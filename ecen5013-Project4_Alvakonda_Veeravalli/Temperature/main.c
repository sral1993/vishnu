/*
 * main.c
 *
 *  Created on: Dec 06, 2016
 *      Author: Prithvi
 */


#include "MKL25Z4.h"
#include "log.h"
#include <string.h>

void ADC_Init(void)
{
	// Enable clocks
	SIM_SCGC6 |= SIM_SCGC6_ADC0_MASK;	// ADC 0 clock

	// Configure ADC
	ADC0_CFG1 = 0; // Reset register

	ADC0_CFG1 |= (ADC_CFG1_MODE(3)  |  	// 16 bits mode
				  ADC_CFG1_ADICLK(0)|	// Input Bus Clock (20-25 MHz out of reset (FEI mode))
				  ADC_CFG1_ADLPC_MASK |
				  ADC_CFG1_ADLSMP_MASK |
				  ADC_CFG1_ADIV(0)) ;	// Clock divide by 2 (10-12.5 MHz)

	ADC0_SC1A |= ADC_SC1_ADCH(26);
	ADC0_CFG2 |= ADC_CFG2_ADLSTS(3);
	ADC0_SC3 |= ADC_SC3_AVGS(3) | ADC_SC3_AVGE_MASK;
	ADC0_SC3 = 0; // Reset SC3
	ADC0_SC1A |= ADC_SC1_ADCH(9);
	//ADC0_SC1A |= ADC_SC1_ADCH(26) | ADC_SC1_DIFF_MASK; // Disable module


}


uint16_t TEMP_Read(void)
{
	//ADC0_SC1A = (ch & ADC_SC1_ADCH_MASK) | (ADC0_SC1A & (ADC_SC1_AIEN_MASK | ADC_SC1_DIFF_MASK));     // Write to SC1A to start conversion
	ADC0_SC1A = ADC_SC1_ADCH(26); //Write to SC1A to start conversion
	while(ADC0_SC2 & ADC_SC2_ADACT_MASK); 	 // Conversion in progress
	while(!(ADC0_SC1A & ADC_SC1_COCO_MASK)); // Run until the conversion is complete
	return ADC0_RA;
}

void main(void)
{
	int temp_1,temp_2,temp_3;
	ADC_Init();
	uint16_t temp;
	float temperature, temp1;
	int x=0;
	while(x<4)
	{
	temp = TEMP_Read();
	temp1 = (int)((3.3 / 65536) * temp * 1000);
	temperature = 25 - ((temp1 - 716) / 1620);
	int  y;

uint8_t str[]="Temperature is: ";
LOG1(str,strlen(str), temperature, sizeof(temperature));
	}

}

