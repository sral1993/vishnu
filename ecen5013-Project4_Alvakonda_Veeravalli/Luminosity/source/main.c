/*
 * main.c
 *
 *  Created on: Nov 25, 2016
 *      Author: Prithvi
 */

/*
 * main.c
 *
 *  Created on: Nov 25, 2016
 *      Author: Prithvi
 */

#include "MKL25Z4.h"
#include "stdint.h"
#include "i2c.h"
#include "TSL2561.h"
#include "led.h"

static int i = 0;

int main(void)
{
	uint8_t ambience_low;
	uint8_t ambience_high;
	uint16_t ambience;
	uint8_t dummy;
	reading = &dummy;
	Init_I2C();
	TSL2561_Init();
	TSL2561_GPIO_Int_Enable();
	while(1)
	{
	    I2CReadMultiRegisters(TSL2561_slaveaddress, TSL2561_ADC0_Low_Reg, 2, reading);
        ambience_low = *reading;
        reading++;
        ambience_high = *reading;
        ambience = ambience_high*256 + ambience_low;
        if(ambience < 0x000F)
        {
        	LED_RED(20);
        }
        else if(ambience > 0x0800)
        {
        	LED_RED(0);
        }
	}

}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

