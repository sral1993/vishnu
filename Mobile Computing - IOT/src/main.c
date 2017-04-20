#include "em_device.h"
#include "em_chip.h"
#include "em_int.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_acmp.h"
#include "em_timer.h"
#include "em_gpio.h"
#include <stdint.h>
#include <stdbool.h>
#include "em_dma.h"
#include "em_adc.h"
#include "dmactrl.h"
#include "dmactrl.c"
#include "em_i2c.h"
#include "em_leuart.h"
#include "lesense_letouch.h"
#include "lesense_letouch_config.h"
#include "circbuff.h"

#define EM0 0
#define EM1 1
#define EM2 2
#define EM3 3
#define EM4 4
#define LET0_Totalperiod           1.0                   // Total LED Blinking Time in Seconds
#define LET0_ON_Duty               0.004                  // ON DUTY Time in Seconds
#define LET0_LFXO_count            32768                 // As per requirements, from EM0 to EM2, LFXO is taken to provide clock source to LFA and
                                                         // LFXO operates at the frequency of 32.768Khz which is equivalent to 32768 counts per second
#define LET0_ULFRCO_count          1000                 // ULFRCO is the source for the Energy Mode 3
                                                         //  which has a frequency operating at 1Khz which is equivalent to a count of 1000
#define LET0_EM                    EM2                  // Current Energy mode of LETIMER0
#define LED0_port gpioPortE                              // The physical internal connection for led0 is through port gpioport E
#define LED0_pin 2U                                      // LED0_pin in the kit STK3600
#define LED1_port gpioPortE                              // The physical internal connection for led0 is through port gpioport E
#define LED1_pin 3U                                      // LED0_pin in the kit STK3600
#define sense_port gpioPortC                             // The physical internal connection for light sensor sensing is through port gpioport c
#define sense_pin  6                                     // Sense_pin in the kit STK3600
#define excite_port gpioPortD                            // The physical internal connection for light sensor exciting is through port gpioport d
#define excite_pin  6                                    // excite pin is through 6                              // excite pin is through 6
#define high_vdd_level 0x03                       // equivalent hexa decimal value of level 61
#define low_vdd_level 0x01                         // equivalent hexa decimal value of level 2

#define DMA_Initiate          1               // Initialize DMA
#define DMA_CHANNEL_ADC       0               // Initilaize DMA_CHANNEL_ADC with channel 0


DMA_CB_TypeDef cb;        // DMA call back structure


#define ADCSAMPLES                        400    // Number of ADC samples
volatile uint16_t ADC_DATA[ADCSAMPLES];
#define ADCSAMPLESPERSEC              75000      //Samples per second
int32_t average_temp=0;                          //Initialize average temperature=0
float Temp;

#define BME_slave_address 0x76
#define slave_address 0x1D
#define LEUART0_Port       gpioPortD
#define LEUART0_Pin        4
#define I2C1_port gpioPortD                  // i2c port
#define I2C1_pin  0                          // i2c pin
#define I2C1_intport gpioPortD               // i2c interrupt port
#define I2C1_intpin  1                       // i2c interrupt pin
#define Accelerometer_SDA_port gpioPortC                   // serial data line port
#define Accelerometer_SDA_pin 4                            // serial data line pin
#define Accelerometer_SCL_port gpioPortC                   // serial clock line port
#define Accelerometer_SCL_pin 5                            // serial clock line pin
#define BME_SDA_port gpioPortE                   // serial data line port
#define BME_SDA_pin 0                            // serial data line pin
#define BME_SCL_port gpioPortE                   // serial clock line port
#define BME_SCL_pin 1                            // serial clock line pin
#define Command_Reg 0x80
#define Write_Reg   0x00
#define Power_up    0x03
#define Timing_Reg  0x81
#define Integration_time 0x01
#define CTHLL 0x82
#define THLL  0x0f
#define CTHLH 0x83
#define THLH  0x00
#define CTHHL 0x84
#define THHL  0x00
#define CTHHH 0x85
#define THHH  0x08
#define Interrupt_Control 0x86
#define Int_persistence 0x14
unsigned int Peripheral_LEM[5];    // The Array of length 5(EM0,EM1,EM2,EM3,EM4) which is maximum possible Low energy mode plus 1 i.e (EM4 + 1) = (4 +1) = 5
float OSC_ratio=1;                 // initializing oscillator ratio
uint32_t Past_Ambience=1;       // Assuming initial Ambience level to be 1
uint32_t vdd_level=0x000000002;    // keeping initial reference vdd level to be at 2/63*vdd
uint8_t Pressure_I2C_tx_buffer[2] = {0x00, 0x00};
uint8_t Pressure_I2C_rx_buffer= {0x00};
uint8_t I2C_tx_buffer[2] = {0x00, 0x00};
uint8_t I2C_rx_buffer= {0x00};

uint16_t Sensor_light_LSB,Sensor_light_MSB,Sensor_light;
int Period=1;

uint8_t byte_num = 0;
uint8_t Atmel_led=0;
uint8_t *UART_temp_ptr;
uint8_t *UART_press_ptr;
uint16_t pressure;
uint16_t channels_touched = 0;
float sensitivity[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0};

int entry_over=0;
int leave_over=1;
int ambience;

#define Control_Reg1_Address			0x2A
#define Control_Reg1_value			0xFA		//Setting F_Read to 1 (8-bit mode)
#define AccMeter_MASK_ACTIVE				0x01

#define Control_Reg2_Address			0x2B
#define Control_Reg2_value			0x03

#define Control_Reg3_Address			0x2C
#define Control_Reg3_value			0x00

#define Control_Reg4_Address			0x2D	//Interrupt Enable Register
#define Control_Reg4_value			0x04	//Motion Detection Interrupt Enabled

#define Control_Reg5_Address			0x2E	//Interrupt Configuration Register
#define Control_Reg5_value			0x04	//Setting Motion Detection Interrupt to INT1 Pin

#define XYZ_Config_Address		0x0E
#define XYZ_Config_value			0x00	//Setting the dynamic range of 8-bit data to 2g; 0x01 for 4g; 0x02 for 8g

#define Motion_Config_Address		0x15
#define Motion_Config_value			0x78	//ELE = 0

#define Motion_Threshold_Address	0x17
#define Motion_Threshold_value		0xA0

#define Motion_Count_Address		0x18
#define AccMeter_MotionCount_Value			0x01	//Sets 641ms time to check if the motion is still being observed (debounce time)

#define AccMeter_SourceDetectionReg_Address		0x16 //Read Only
#define AccMeter_INT_SOURCE_Address				0x0C //Read Only



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


#define Circular_Buffer    true
#define UART_write true
struct circular_buffer *transmit_buffer;
struct circular_buffer buffer1;
uint8_t transmitbuffer[10];


uint8_t receive_buffer[10];
uint32_t status=1;

int j=0;

uint8_t tst[5];

static uint8_t capacitive_alert=0;
static uint8_t Accelerometer_alert=0;


 void BlockSleepMode(int em);
 void ClockManagementUnit_Configuration(void);
 void LET0_Configuration(void);
 void GPIO_Configuration(void);
 void ON_LED0();
 void OFF_LED0();
 void ON_LED1();
 void OFF_LED1();

 void Sleep_routine(void);
 void ACMP_Configuration();
 void UnBlockSleepMode(int em);
 void DMA_Callback(unsigned int channel, bool primary, void *user);
 void ADC_Configuration(void);
 void DMA_Configuration(void);
 float convertToCelsius(int32_t average_temp);
 void CPU_transfer();
 void Accelerometer_I2C_Configuration();
 void Pressure_I2C_Write();
 int Pressure_I2C_Read();
 void Accelerometer_I2C_Write();
 void Accelerometer_I2C_Read();
 void Accelerometer_Configuration(void);
 void Accelerometer_Enable(void);
 void Accelerometer_Disable(void);
 void BME_Configuration();
  void read_trimmed_data();
  void read_rawdata();
  void sensor_calibration();
  signed long int calibration_T(signed long int adc_T);
  unsigned long int calibration_P(signed long int adc_P);
  void BME_Enable();
  void BME_Disable();
  void entry_touch();
  void leave_touch();
  void LEUART0_Configuration(void);







  void ClockManagementUnit_Configuration(void)
   {
   CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
   CMU_OscillatorEnable(cmuOsc_ULFRCO, false, false);
   CMU_ClockEnable(cmuClock_HFPER, true);
   CMU_ClockEnable(cmuClock_HFLE, true);                  // High frequency Low Energy clock is enabled which is one of the inputs of LFA clock branch switch.
   CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
   //CMU_ClockEnable(cmuClock_LETIMER0, true);
   CMU_ClockEnable(cmuClock_GPIO, true);

   CMU_ClockEnable(cmuClock_ACMP0, true);

   //CMU_ClockEnable(cmuClock_DMA, true);
   //CMU_ClockEnable(cmuClock_ADC0, true);

   }




   // Function definition of GPIO_configuration: In order to blink the LED, the signals are sent through gpio LED0_port to LED0_pin
   void GPIO_Configuration(void)
   {
   	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, 0); /*Setting up the mode of the pin of port E which is used to drive led0*/

   	GPIO_PinModeSet(sense_port, sense_pin, gpioModeDisabled, 0);
   	GPIO_PinModeSet(excite_port, excite_pin, gpioModePushPull, 0);

   	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, 0);
   }




   //Function definition of LET0_Configuration: First LETIMER0 is initialized with comp0top enabled and repeat mode in free running state.
   //variables like interrupt flag, compo_value, comp1_value are declared

   void LET0_Configuration(void)
   {
	   CMU_ClockEnable(cmuClock_LETIMER0, true);
   int interrupt_Flag;
   int Comp0_value;
   int Comp1_value;
   LETIMER0->CTRL=0x00000200; // Comp0Top bit is set( LETIMER0_CTRL is a 32 bit register in which Comp0Top bit is set,
                              // so that counter value is loaded with Comp0_value everytime it underflows



   //  Setting up for Total LED Blinking time

   if (LET0_EM == EM3)
   {
   	Comp0_value = LET0_Totalperiod * LET0_ULFRCO_count * OSC_ratio ;      // Comp0_value is calculated which is time period*(counts per second) in EM3 mode
   }
   	else
   {
   		int LET0_prescaler;                                            // pre scaler is done in order to use LFXO clock for higher time periods

   		int i=0;
   		while((LET0_Totalperiod*LET0_LFXO_count/(1<<i))>65535)
   		{
   			i++;
   		}
   		LET0_prescaler= 1<<i;
   		int PRESCALER=i;
   		CMU->LFAPRESC0 &= 0xfffff0ff;                         // assigning pre scaler value to the register
   		CMU->LFAPRESC0 |= PRESCALER << 8;
   		Comp0_value = LET0_Totalperiod * (LET0_LFXO_count/LET0_prescaler);     //  The limit of LETIMER0 counter is 16 bits which is equivalent to 0 to 65535,
                                                             //  but, (LET0_totalperiod * LET0_LFXO_count)= 65536 which makes Comp0_value '0'
                                                             //  That's why (LET0_LFXO_count-1) is taken into consideration.
   }
   LETIMER0->CNT = Comp0_value;                              // Comp0_value is loaded to LETIMER0 counter. As comp0_top bit in LETIMER is enabled,
                                                             // counter value is loaded with comp0_value instead of FFFF every time it repeats


   LETIMER_CompareSet(LETIMER0,0,Comp0_value);               // CompareSet function initializes Comp0_value to Compare match register 0

   // Setting up for LED ON DUTY time

   if(LET0_EM == EM3)
   {
   Comp1_value = LET0_ON_Duty * LET0_ULFRCO_count * OSC_ratio;
   }
   else
   {
   	int LET0_prescaler;                                            // pre scaler is done in order to use LFXO clock for higher time periods

   	int i=0;
   	while((LET0_Totalperiod*LET0_LFXO_count/(1<<i))>65535)
   	{
   		i++;
   	}
   	LET0_prescaler= 1<<i;
   	int PRESCALER=i;
   	CMU->LFAPRESC0 &= 0xfffff0ff;                         // assigning pre scaler value to the register
   	CMU->LFAPRESC0 |= PRESCALER << 8;
   	Comp1_value = LET0_ON_Duty * (LET0_LFXO_count/LET0_prescaler);
   }
   LETIMER_CompareSet(LETIMER0,1,Comp1_value);               // CompareSet function initializes Comp1_value to Compare match register 1


   interrupt_Flag = LETIMER0->IF;
   LETIMER_IntClear(LETIMER0, interrupt_Flag);               // IntClear function clears the interrupt flags of LETIMER

   LETIMER_IntEnable(LETIMER0,(LETIMER_IEN_UF | LETIMER_IEN_COMP1) );   // IntEnable function enables the LETIMER Interrupt flag through any one of the enabled
                                                                        // interrupt flags of Underflow or Comp1

   BlockSleepMode(LET0_EM);                                  // block the sleep mode under the current energy mode of LETIMER0

   NVIC_EnableIRQ(LETIMER0_IRQn);                            // Enable the Letimer0 interrupts in Nested vector interrupt Controller

   }


   // Analog Comparator 0 is used  to sense the light from light sensor by exciting it for 4 ms and compares the sensed output with the reference inputs



   /*LETIMER0 interrupts handler that will perform the desired function on UF or COMP1 interrupt*/




   // function definition of ON_LED:  when this function is called, the pin 2U in GPIO portE is made set through PinOutSet
   void ON_LED0()
   {

   GPIO_PinOutSet(LED0_port,LED0_pin);
   }

   // function definition of OFF_LED: when this function is called, the pin 2U in GPIO portE is made clear through PinOut Clear

   void OFF_LED0()
   {
   GPIO_PinOutClear(LED0_port,LED0_pin);
   }

   void ON_LED1()
   {

   GPIO_PinOutSet(LED1_port,LED1_pin);
   }

   void OFF_LED1()
   {
   GPIO_PinOutClear(LED1_port,LED1_pin);
   }


   // Function definition of Sleep_routine: In order to save energy//
   // Depending on the Low energy modes of different peripherals,the sleep routine is operated to sleep in the highest possible energy mode//

   void Sleep_routine(void)
   {

	   if (Peripheral_LEM[EM0] > 0)
	   {
	   }
	   else if (Peripheral_LEM[EM1] > 0)
	   {
	   EMU_EnterEM1();
	   }
	   else if (Peripheral_LEM[EM2] > 0)
	   {
	   EMU_EnterEM2(true);
	   }
	   else if (Peripheral_LEM[EM3] > 0)
	   {
	   EMU_EnterEM3(true);
	   }
   }


    void BlockSleepMode(int em)
     {
     INT_Disable();                      // interrupt disable
     Peripheral_LEM[em]++;
     INT_Enable();                       // interrupt enable
     }

     void UnBlockSleepMode(int em)
     {
     INT_Disable();                      // interrupt disable
     if (Peripheral_LEM[em] >0)
   	  Peripheral_LEM[em]--;
     else Peripheral_LEM[em] = 0;
     INT_Enable();                       // interrupt enable

    }



     //DMA call back routine after DMA completes its transfer of 1000 samples.

     void DMA_Callback(unsigned int channel, bool primary, void *user)
     {
       (void) channel;
       (void) primary;
       (void) user;

       UnBlockSleepMode(EM1);
       CMU_ClockEnable(cmuClock_DMA, false);

       ADC0->CMD=0x00000002;          //stop ADC0
       CMU_ClockEnable(cmuClock_ADC0, false);
       int i=0;
      uint32_t x=0;
            //x= ADC_DATA[0];

            while(i<ADCSAMPLES)
            {
             x= x+ADC_DATA[i];
             i++;
            }

            average_temp= x/ADCSAMPLES;


       Temp = convertToCelsius(average_temp);   //Converting temperature to Celsius with the help of a function

       UART_temp_ptr = (uint8_t *)&Temp;
       if(leave_over==1)
       {
       if(Accelerometer_alert==1)
       {

    	   float a = 100.0;
    	   uint8_t *uart_acc_ptr;
    	   uart_acc_ptr = (uint8_t *)&a;

           int e=0;
           while(e<4)
           {
        	   add_item(transmit_buffer, *uart_acc_ptr);

        	   tst[e+1]= *uart_acc_ptr;
        	   e++;
        	   uart_acc_ptr++;
           }

       }
       else
       {
       int f=0;
       while(f<4)
       {
    	   add_item(transmit_buffer, *UART_temp_ptr);

    	   tst[f+1]= *UART_temp_ptr;
    	   f++;
    	   UART_temp_ptr++;
       }
       }

       add_item(transmit_buffer, ambience);
       LEUART0_Configuration();
                LEUART_FreezeEnable(LEUART0, true);
                LEUART0->CMD |= LEUART_CMD_TXEN | LEUART_CMD_RXEN;
                LEUART0->ROUTE |= LEUART_ROUTE_TXPEN;
                LEUART0->IEN = LEUART_IEN_TXBL;
                LEUART_FreezeEnable(LEUART0, false);
       }
       else
       {
          uint8_t x=0;
          uint8_t *entry_ptr;
          entry_ptr = (uint8_t *)&x;
          int w=0;
                     while(w<4)
                     {
                  	   add_item(transmit_buffer, *entry_ptr);

                  	   tst[w+1]= *entry_ptr;
                  	   w++;
                  	 entry_ptr++;
                     }
                     add_item(transmit_buffer, ambience);
                     LEUART0_Configuration();
                              LEUART_FreezeEnable(LEUART0, true);
                              LEUART0->CMD |= LEUART_CMD_TXEN | LEUART_CMD_RXEN;
                              LEUART0->ROUTE |= LEUART_ROUTE_TXPEN;
                              LEUART0->IEN = LEUART_IEN_TXBL;
                              LEUART_FreezeEnable(LEUART0, false);
       }



       if(Temp>15&&Temp<35)
       {
     	  OFF_LED1();               //if temperature lies between 15 & 35 LED switches OFF
       }
       else
       {
     	  ON_LED1();                // if temperature crosses the range LED switches ON
       }

     }


     float convertToCelsius(int32_t average_temp)
       {
       	float temp;
       	/* factory calibration temperature from device information page*/
       	float cal_temp_0 =(float)((DEVINFO->CAL &_DEVINFO_CAL_TEMP_MASK)
       			        >>_DEVINFO_CAL_TEMP_SHIFT);
       	float cal_value_0 = (float)((DEVINFO->ADC0CAL2
       			          & _DEVINFO_ADC0CAL2_TEMP1V25_MASK)
       			          >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);
       	/*Tempearture gradient(from datasheet)*/
       	float t_grad = -6.27;
       	temp = (cal_temp_0 - ((cal_value_0 -average_temp) / t_grad));
       	return temp;

       }


     void DMA_Configuration(void)
     {
       /* Initializing the DMA */
    	 CMU_ClockEnable(cmuClock_DMA, true);
    	 DMA_Init_TypeDef        dma_Def;
       dma_Def.hprot        = 0;             //normally sets to 0 when protection is not an issue
       dma_Def.controlBlock = dmaControlBlock;// how many bytes are allocated for this process
       DMA_Init(&dma_Def);

       /* Setting up call-back function */
       cb.cbFunc  = DMA_Callback;     //call back function name
       cb.userPtr = NULL;                 //starting pointer

       /* Setting up channel */
       DMA_CfgChannel_TypeDef  CNLCFG;
       CNLCFG.highPri   = true; //priority of the channel
       CNLCFG.enableInt = true;//It should normally be enabled if using the callback feature for a channel, and disabled if not using the callback feature.
       CNLCFG.select    = DMAREQ_ADC0_SINGLE;//Channel control specifying the source of DMA signals.
       CNLCFG.cb        = &cb;  //The call back is invoked when the specified DMA cycle is complete (when dma_done signal asserted).
       DMA_CfgChannel(DMA_CHANNEL_ADC, &CNLCFG);

       /* Setting up channel descriptor */
       DMA_CfgDescr_TypeDef    DSCPTR;
       DSCPTR.arbRate = dmaArbitrate1; //Arbitration rate, i.e number of DMA transfers done before rearbitration takes place.
       DSCPTR.dstInc  = dmaDataInc2; // Destination increment size for each DMA transfer
       DSCPTR.hprot   = 0;
       DSCPTR.size    = dmaDataSize2; //DMA transfer unit size.
       DSCPTR.srcInc  = dmaDataIncNone;//Source increment size for each DMA transfer

       DMA_CfgDescr(DMA_CHANNEL_ADC, true, &DSCPTR);


       /* Starting transfer. Using Basic since every transfer must be initiated
        * by the ADC. */
       DMA_ActivateBasic(DMA_CHANNEL_ADC,  //DMA channel to activate DMA cycle for
       		                       true,   // activate using primary descriptor
       		                       false,  // burst feature is only used on peripherals supporting DMA bursts
       		                       (void *)&ADC_DATA, //Address to start location to transfer data to
       		                       (void *)&(ADC0->SINGLEDATA), //Address to start location to transfer data from
       		                       ADCSAMPLES - 1); //Number of DMA transfer elements (minus 1) to transfer (<= 1023).


     }
     void ADC_Configuration(void)
     {
     	//BlockSleepMode(EM1);
    	 CMU_ClockEnable(cmuClock_ADC0, true);

       ADC_Init_TypeDef        adc_Def;              //Initialize the ADC
       adc_Def.prescale= 12;                      // for the target of 10KSPS, with the help of the formula
                                              // (Ta+N)OVS=Time per sample
        //  (2+12)*(p/14000000)*400= 400*(1/75000)=>> p=13;
       adc_Def.lpfMode = adcLPFilterBypass;
       adc_Def.ovsRateSel= adcOvsRateSel2;
       adc_Def.warmUpMode=adcWarmupNormal;
       adc_Def.tailgate= false;
       //adcInit.timebase= ADC_TimebaseCalc();
       adc_Def.timebase=13;  // for timebase is 14, for this function N+1=14;
       ADC_Init(ADC0, &adc_Def);

          ADC_InitSingle_TypeDef  adc_single_Def;

          adc_single_Def.resolution= adcRes12Bit;   //12 bit resolution
          adc_single_Def.prsEnable = false;
          adc_single_Def.prsSel    = adcPRSSELCh0;
          adc_single_Def.acqTime   = adcAcqTime2;   // Acquisition time 2
          adc_single_Def.diff      = false;
          adc_single_Def.input     = adcSingleInputTemp;
          adc_single_Def.rep       = true;
          adc_single_Def.reference = adcRef1V25;
          adc_single_Def.leftAdjust= false;

       ADC_InitSingle(ADC0, &adc_single_Def);
      // ADC_Start(ADC0,adcStartSingle);
     }


   void CPU_transfer()      //This function is called when DMA-Transfer is disabled
   {
   	int count=0;
   	average_temp=0;
   	int sum=0;
   	ADC0->IEN |= 0x00000001;     //ADC0 interrupt is enabled when single conversion is complete
   	ADC_Configuration();
   	while(count<ADCSAMPLES)      // It runs till the count of 1000 is complete
   	{
   		while(((ADC0->IF) &&(0x00000001)==0))  // checks if interrupt is occurred by polling continuously so, that 1000 smaples are collected properly
   		{
   		}
   		ADC_DATA[count]=ADC0->SINGLEDATA;    // Transferring data from SINGLEDATA register to the ADC_DATA array
   		sum=sum+ADC_DATA[count];
   		count++;
   		ADC0->IFC=0x00000001;      // Interrupts are cleared in order to continue with the loop operation
      }
   	ADC0->CMD=0x00000002;        // After the 1000 samples i.e after 100 ms from the LETIMER0 started, stop the ADC conversion to save energy
   	average_temp=sum/ADCSAMPLES;
   	Temp = convertToCelsius(average_temp);  // Calling the Celsius converter function


   	    if(Temp>15&&Temp<35)
   	    {
   	  	  OFF_LED1();
   	    }
   	    else
   	    {
   	  	  ON_LED1();
   	    }
   	    UnBlockSleepMode(EM1);
   }

   void BME_Enable()
    {
   	 int i=0;
   	 	GPIO_PinModeSet(I2C1_port,3,gpioModePushPull,1);
   	 	GPIO_PinOutSet(I2C1_port, 3);
   	 	while(i<500)
   	 	{
   	 	   i++;
   	 	}
    }

    void BME_Disable()
    {
   	    GPIO_PinOutClear(I2C1_port, 3);
   	 	GPIO_PinModeSet(I2C1_intport, I2C1_intpin, gpioModeDisabled, 0);
   	 	GPIO_PinModeSet(BME_SCL_port, BME_SCL_pin, gpioModeDisabled , 0);
   	 	GPIO_PinModeSet(BME_SDA_port, BME_SDA_pin, gpioModeDisabled , 0);
   	    CMU_ClockEnable(cmuClock_I2C1,false);
    }



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

   	 Pressure_I2C_tx_buffer[0] = control_reg;
   	 Pressure_I2C_tx_buffer[1] = control_reg_data;
   	 Pressure_I2C_Write(2);
   	 Pressure_I2C_tx_buffer[0] = config_reg;
   	 Pressure_I2C_tx_buffer[1] = config_reg_data;
   	 Pressure_I2C_Write(2);


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
   	 data[i]= Pressure_I2C_Read(1);
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
   		 data[i]=Pressure_I2C_Read(1);
   		 i++;
   		 d++;
   	 }
   	 Raw_pressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
   	 Raw_temperature = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    }

    void sensor_calibration()
    {
   	 double Final_Temperature,Final_Pressure;
    signed long int Calibrated_Temperature;
   	 unsigned long int Calibrated_Pressure;
   	 Calibrated_Temperature = calibration_T(Raw_temperature);
   	 Calibrated_Pressure = calibration_P(Raw_pressure);
   	 Final_Temperature = (float)Calibrated_Temperature / 100.0;
   	 Final_Pressure = (float)Calibrated_Pressure / 1000.0;
   	pressure = (uint8_t) Final_Pressure;

   	 if(Final_Pressure>87)
   	 {
   		 ON_LED1();
   	 }
   	 else
   	 {
   		 OFF_LED1();
   	 }
   	 UART_press_ptr = (uint8_t *)&pressure;

   	 if(leave_over==1)
   	 {

   	 if(capacitive_alert == 1)
   	 {
   		add_item(transmit_buffer, 100);
   		tst[0]= 0;
   	 }
   	 else
   	 {

   	 add_item(transmit_buffer, *UART_press_ptr);
   	 tst[0]= *UART_press_ptr;
     }

   	 }
   	 else
   	 {
   		add_item(transmit_buffer, 0);
   		   		tst[0]= 0;
   	 }
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

    void Accelerometer_I2C_Configuration()
     {
     	int i;
     	CMU_ClockEnable(cmuClock_I2C1, true);     // enable clock for I2C1
     	I2C_Init_TypeDef I2C_Def;
     	I2C_Def.clhr=i2cClockHLRStandard;
     	I2C_Def.enable= true;
     	I2C_Def.freq= I2C_FREQ_STANDARD_MAX;
     	I2C_Def.master=true;
     	I2C_Def.refFreq=0;

     	/*

       true,                     Enable when init done
       true,                     Set to master mode
       0,                       Use currently configured reference clock
       I2C_FREQ_STANDARD_MAX,   Set to standard rate assuring being
                                 within I2C spec
       i2cClockHLRStandard      Set to use 4:4 low/high duty cycle

     	 */

     	GPIO_PinModeSet(Accelerometer_SDA_port,Accelerometer_SDA_pin,gpioModeWiredAndPullUpFilter, 1);
     	GPIO_PinModeSet(Accelerometer_SCL_port,Accelerometer_SCL_pin,gpioModeWiredAndPullUpFilter, 1);

     	for(i=0;i<9;i++)                          // to synchronise clocks
     	{
     		GPIO_PinOutClear(Accelerometer_SCL_port,Accelerometer_SCL_pin);
     		GPIO_PinOutSet(Accelerometer_SCL_port,Accelerometer_SCL_pin);
     	}
     	I2C1->ROUTE= I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN | I2C_ROUTE_LOCATION_LOC0;
     	I2C_Init(I2C1,&I2C_Def);
     	if (I2C1->STATE & I2C_STATE_BUSY)
     	{
     	        I2C1->CMD = I2C_CMD_ABORT;
     	}
     }

    void Pressure_I2C_Configuration()
         {
         	int i;
         	CMU_ClockEnable(cmuClock_I2C1, true);     // enable clock for I2C1
         	I2C_Init_TypeDef I2C_Def;
         	I2C_Def.clhr=i2cClockHLRStandard;
         	I2C_Def.enable= true;
         	I2C_Def.freq= I2C_FREQ_STANDARD_MAX;
         	I2C_Def.master=true;
         	I2C_Def.refFreq=0;

         	/*

           true,                     Enable when init done
           true,                     Set to master mode
           0,                       Use currently configured reference clock
           I2C_FREQ_STANDARD_MAX,   Set to standard rate assuring being
                                     within I2C spec
           i2cClockHLRStandard      Set to use 4:4 low/high duty cycle

         	 */

         	GPIO_PinModeSet(BME_SDA_port,BME_SDA_pin,gpioModeWiredAndPullUpFilter, 1);
         	GPIO_PinModeSet(BME_SCL_port,BME_SCL_pin,gpioModeWiredAndPullUpFilter, 1);

         	for(i=0;i<9;i++)                          // to synchronise clocks
         	{
         		GPIO_PinOutClear(BME_SCL_port,BME_SCL_pin);
         		GPIO_PinOutSet(BME_SCL_port,BME_SCL_pin);
         	}
         	I2C1->ROUTE= I2C_ROUTE_SCLPEN | I2C_ROUTE_SDAPEN | I2C_ROUTE_LOCATION_LOC2;
         	I2C_Init(I2C1,&I2C_Def);
         	if (I2C1->STATE & I2C_STATE_BUSY)
         	{
         	        I2C1->CMD = I2C_CMD_ABORT;
         	}
         }

     void Pressure_I2C_Write()
     {

     	I2C1->TXDATA=BME_slave_address<<1;
     	I2C1->CMD = I2C_CMD_START;                   //starts I2C1
     	I2C1->IFC=I2C_IFC_START;
     	while ((I2C1->IF & I2C_IF_ACK) ==  0)
     	{
     	}
     	I2C1->IFC = I2C_IFC_ACK;
     	I2C1->TXDATA=Pressure_I2C_tx_buffer[0];                // transmits buffer[0]
     	while ((I2C1->IF & I2C_IF_ACK) ==  0)
     	{

     	}
     	I2C1->IFC = I2C_IFC_ACK;
     	I2C1->TXDATA=Pressure_I2C_tx_buffer[1];                // transmits buffer[1]
     	while ((I2C1->IF & I2C_IF_ACK) ==  0)
     	{

     	}
         I2C1->IFC = I2C_IFC_ACK;
     	I2C1->CMD = I2C_CMD_STOP;                     // stops I2C1
         while ((I2C1->IF & I2C_IF_MSTOP) ==  0)
         {

         }
         I2C1->IFC=I2C_IFC_MSTOP;
     }

     int Pressure_I2C_Read()
     {
     	I2C1->TXDATA=(BME_slave_address)<<1|0x00;                      // write mode
     	I2C1->CMD = I2C_CMD_START;
     	I2C1->IFC=I2C_IFC_START;
     	while ((I2C1->IF & I2C_IF_ACK) ==  0)
     	{

     	}
     	I2C1->IFC = I2C_IFC_ACK;
     	I2C1->TXDATA= req_address;                                // command to get ADC0 value
     	while ((I2C1->IF & I2C_IF_ACK) ==  0)
     	{

     	}
     	I2C1->IFC = I2C_IFC_ACK;
     	I2C1->CMD = I2C_CMD_START;
     	I2C1->TXDATA=(BME_slave_address)<<1|0x01;                      // read mode
     	I2C1->IFC=I2C_IFC_START;
     	while ((I2C1->IF & I2C_IF_ACK) ==  0)
     	{

     	}
     	I2C1->IFC = I2C_IFC_ACK;
     	while ((I2C1->IF & I2C_IF_RXDATAV) ==  0)
     	{

     	}
     	return_data = I2C1->RXDATA;                  //  ADC0 LSB value

     	I2C1->CMD =I2C_CMD_NACK;
     	I2C1->CMD = I2C_CMD_STOP;
     	while ((I2C1->IF & I2C_IF_MSTOP) ==  0)
     	{

     	}
     	I2C1->IFC=I2C_IFC_MSTOP;
     	return(return_data);
     }

     void Accelerometer_I2C_Write()
     {

     	I2C1->TXDATA=slave_address<<1;
     	I2C1->CMD = I2C_CMD_START;                   //starts I2C1
     	I2C1->IFC=I2C_IFC_START;
     	while ((I2C1->IF & I2C_IF_ACK) ==  0)
     	{
     	}
     	I2C1->IFC = I2C_IFC_ACK;
     	I2C1->TXDATA=I2C_tx_buffer[0];                // transmits buffer[0]
     	while ((I2C1->IF & I2C_IF_ACK) ==  0)
     	{

     	}
     	I2C1->IFC = I2C_IFC_ACK;
     	I2C1->TXDATA=I2C_tx_buffer[1];                // transmits buffer[1]
     	while ((I2C1->IF & I2C_IF_ACK) ==  0)
     	{

     	}
         I2C1->IFC = I2C_IFC_ACK;
     	I2C1->CMD = I2C_CMD_STOP;                     // stops I2C1
         while ((I2C1->IF & I2C_IF_MSTOP) ==  0)
         {

         }
         I2C1->IFC=I2C_IFC_MSTOP;
     }

     void Accelerometer_I2C_Read()
     {
     	I2C1->TXDATA=(slave_address)<<1|0x00;                      // write mode
     	I2C1->CMD = I2C_CMD_START;
     	I2C1->IFC=I2C_IFC_START;
     	while ((I2C1->IF & I2C_IF_ACK) ==  0)
     	{

     	}
     	I2C1->IFC = I2C_IFC_ACK;
     	I2C1->TXDATA=0xAC;                                // command to get ADC0 value
     	while ((I2C1->IF & I2C_IF_ACK) ==  0)
     	{

     	}
     	I2C1->IFC = I2C_IFC_ACK;
     	I2C1->CMD = I2C_CMD_START;
     	I2C1->TXDATA=(slave_address)<<1|0x01;                      // read mode
     	I2C1->IFC=I2C_IFC_START;
     	while ((I2C1->IF & I2C_IF_ACK) ==  0)
     	{

     	}
     	I2C1->IFC = I2C_IFC_ACK;
     	while ((I2C1->IF & I2C_IF_RXDATAV) ==  0)
     	{

     	}
     	Sensor_light_LSB= I2C1->RXDATA;
     	I2C1->CMD =I2C_CMD_NACK;
     	I2C1->CMD = I2C_CMD_STOP;
     	while ((I2C1->IF & I2C_IF_MSTOP) ==  0)
     	{

     	}
     	I2C1->IFC=I2C_IFC_MSTOP;
     }

     void ACMP_Configuration()
        {
        	ACMP_Init_TypeDef ACMP0_Def;
        	int interrupt_Flag;
        	interrupt_Flag=ACMP0->IF;
        	ACMP0->IFC=interrupt_Flag;

        	ACMP0_Def.halfBias = true;                                   // keeping half bias to save energy even there is more reaction time
        	ACMP0_Def.hysteresisLevel = acmpHysteresisLevel7 ;           // keeping hysteresis level to be high to save energy even there is less accuracy
        	ACMP0_Def.warmTime = acmpWarmTime256 ;                       // warm up time for 10us is 140 clock cycles, upper bound is taken
        	ACMP0_Def.enable = true ;                                    // enabling after initializing
        	ACMP0_Def.vddLevel = vdd_level;                              // giving reference vdd level

        	ACMP_Init(ACMP0,&ACMP0_Def);

        	while(ACMP_STATUS_ACMPACT!=1)
        	{                                           // Till the warm up time, it checks for the active state
        	}
        	ACMP0->INPUTSEL |= 0x00010000;                               // Enable the  LPREF to save energy
        	ACMP_ChannelSet(ACMP0,acmpChannelVDD,acmpChannel6);          // Selecting Channel for light sensor in ACMP0

        }





      void LETIMER0_IRQHandler(void)
       {
     	int interrupt_Flag;
       interrupt_Flag = LETIMER0->IF;
       LETIMER_IntClear(LETIMER0, interrupt_Flag);

       if ((interrupt_Flag & LETIMER_IF_COMP1) !=0)          // if interrupt request is made through comp1, then excite pin is set to sense the light
       {

    	   //GPIO_PinOutSet(excite_port,excite_pin);    // configure the ACMP0
    	          	ACMP_Configuration();
                    ACMP_Enable(ACMP0);
                    GPIO_PinOutSet(excite_port,excite_pin);

       	       	       		 Pressure_I2C_Configuration();         // in period 1 configure I2C, Enable Sensor, then configure TSL2561
       	       		         BME_Enable();
       	       		         BME_Configuration();
       	       		         read_rawdata();
       	       		         BME_Disable();
       	       		         sensor_calibration();



       }
       else if ((interrupt_Flag & LETIMER_IF_UF) !=0)        // if the counter decrements from 65535 to 0. underflow flag interrupt request is made
       {
    	   int status = ACMP0->STATUS;                      // Checking the ACMP0 status
    	          		    status=status>>1;                        // shifting the status bit to LSB, to get the value
    	          		    if(Past_Ambience==0)                  // check for current ambience
    	          		    {
    	          		    	if(status==1)                        // if ACMP0 output is 1 with reference to high vdd level, and the ambience is 0 then turn off the LED
    	          		    	{                                    // Assign the low vdd level, then disable the ACMP0.
    	          		    		//OFF_LED0();
    	          		    		ambience=0;
    	          		            vdd_level=low_vdd_level;
    	                              Past_Ambience=1;             // change the checking ambience level to 1
    	          		    	}

    	          		    }

    	          		   else if(Past_Ambience==1)
    	          		    {

    	          		    	if(status==0)                       // if ACMP0 output is 0, with reference to low vdd level, and the ambience is 1 then turn on the LED
    	          		    	{                                   // assign the high vdd level, then disable the ACMP0
    	          		    	    //ON_LED0();
    	          		    		ambience=1;
    	          		    		vdd_level=high_vdd_level;
    	          		    		Past_Ambience=0;

    	          		    	}
    	          		    }

       		 OFF_LED1();
       		 OFF_LED0();

       		if(leave_over==1)
       		{
       		Accelerometer_I2C_Configuration();         // in period 1 configure I2C, Enable Sensor, then configure TSL2561
       		Accelerometer_Enable();
       		Accelerometer_Configuration();
            GPIO_IntEnable(GPIO_ODD_IRQn);
       		}
       		ACMP_Disable(ACMP0);
       		       		 GPIO_PinOutClear(excite_port,excite_pin);

       		      // stop exciting the light sensor, to save energy after 4ms
       		BlockSleepMode(EM1);
       		              		       	if(DMA_Initiate==1)              //check if DMA is Initiated
       		              		       	     {
       		              		       	     	DMA_Configuration();
       		              		       	     	ADC_Configuration();
       		              		       	     }
       		              		       	     else                             // else continue the task with CPU
       		              		       	     {
       		              		       	     	ADC_Configuration();
       		              		       	     	CPU_transfer();

       		              		       	     }
       		              		       	if(DMA_Initiate==1)          // DMA Initialized
       		              		       	{
       		              		         DMA_ActivateBasic(DMA_CHANNEL_ADC,  //DMA channel to activate DMA cycle for
       		              		       	  			      true,   // activate using primary descriptor
       		              		       	  			      false,  // burst feature is only used on peripherals supporting DMA bursts
       		              		       	  			   	  (void *)ADC_DATA, //Address to start location to transfer data to
       		              		       	  			   	  (void *)&(ADC0->SINGLEDATA), //Address to start location to transfer data from
       		              		       	  			      ADCSAMPLES - 1); //Number of DMA transfer elements (minus 1) to transfer (<= 1023).
       		              		         	 	 	 	 	 ADC_Start(ADC0,adcStartSingle);
       		              		         }
       		              		       	else
       		              		         {
       		              		       	ADC_Start(ADC0,adcStartSingle);   // with CPU
       		              		       	CPU_transfer();
       		              		        }



       		}



       }

     void Accelerometer_Configuration()
      {


      	//BlockSleepMode(EM3);
      	I2C_tx_buffer[0] = Control_Reg1_Address;   // command register
      	I2C_tx_buffer[1] = Control_Reg1_value;   // write register
      	Accelerometer_I2C_Write(2);
      	I2C_tx_buffer[0] = Control_Reg2_Address;   // command ON with control register
      	I2C_tx_buffer[1] = Control_Reg2_value;   // power the device
      	Accelerometer_I2C_Write(2);
          I2C_tx_buffer[0] = Control_Reg3_Address;   // Command with Timing
      	I2C_tx_buffer[1] = Control_Reg3_value;   // Nominal integration time of 101ms
      	Accelerometer_I2C_Write(2);
      	I2C_tx_buffer[0] = Control_Reg4_Address;   // command with Threshold low low
      	I2C_tx_buffer[1] = Control_Reg4_value;   // threshold low low
      	Accelerometer_I2C_Write(2);
      	I2C_tx_buffer[0] = Control_Reg5_Address;   // command with Threshold low high
      	I2C_tx_buffer[1] = Control_Reg5_value;   // threshold low high
      	Accelerometer_I2C_Write(2);
      	I2C_tx_buffer[0] = XYZ_Config_Address;   // command with Threshold high low
      	I2C_tx_buffer[1] = XYZ_Config_value;   // threshold high low
      	Accelerometer_I2C_Write(2);
          I2C_tx_buffer[0] = Motion_Config_Address;   // command with Threshold high high
      	I2C_tx_buffer[1] = Motion_Config_value;   // threshold high high
      	Accelerometer_I2C_Write(2);
      	I2C_tx_buffer[0] = Motion_Threshold_Address;   // command with interrupt control
      	I2C_tx_buffer[1] = Motion_Threshold_value;   // level interrupt with persistence value of 4
      	Accelerometer_I2C_Write(2);
      	I2C_tx_buffer[0] = Motion_Count_Address;   // command with interrupt control
      	I2C_tx_buffer[1] = AccMeter_MotionCount_Value;   // level interrupt with persistence value of 4
      	Accelerometer_I2C_Write(2);
      	I2C_tx_buffer[0] = Control_Reg1_Address;   // command with interrupt control
      	I2C_tx_buffer[1] = Control_Reg1_value | AccMeter_MASK_ACTIVE;   // level interrupt with persistence value of 4
      	Accelerometer_I2C_Write(2);


      	GPIO_IntDisable(GPIO_ODD_IRQn);
      	GPIO_PinModeSet(I2C1_intport, 1, gpioModeInput, 0);
      	GPIO_ExtIntConfig(I2C1_intport, 1, 1, true, false, true);
      	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
      	NVIC_EnableIRQ(GPIO_ODD_IRQn);
      	GPIO_PinModeSet(Accelerometer_SCL_port, Accelerometer_SCL_pin, gpioModeDisabled , 0);
      	GPIO_PinModeSet(Accelerometer_SDA_port, Accelerometer_SDA_pin, gpioModeDisabled , 0);
      	//UnBlockSleepMode(EM3);
      }

     void Accelerometer_Enable()                   // enable the sensor
     {
     	//BlockSleepMode(EM3);
     	int i=0;
     	GPIO_PinModeSet(I2C1_port,I2C1_pin,gpioModePushPull,0);
     	GPIO_PinOutSet(I2C1_port, I2C1_pin);
     	while(i<1000)
     	{
     	   i++;
     	}
     	//UnBlockSleepMode(EM3);
     }
     void Accelerometer_Disable()                     // disable the sensor
     {
     	//BlockSleepMode(EM3);
     	NVIC_DisableIRQ(GPIO_ODD_IRQn);
     	NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
     	GPIO_IntDisable(GPIO_ODD_IRQn);
     	GPIO_PinOutClear(I2C1_port, I2C1_pin);
     	GPIO_PinModeSet(I2C1_intport, I2C1_intpin, gpioModeDisabled, 0);
     	GPIO_PinModeSet(Accelerometer_SCL_port, Accelerometer_SCL_pin, gpioModeDisabled , 0);
     	GPIO_PinModeSet(Accelerometer_SDA_port, Accelerometer_SDA_pin, gpioModeDisabled , 0);
         CMU_ClockEnable(cmuClock_I2C1,false);
         //UnBlockSleepMode(EM3);
     }

     void GPIO_ODD_IRQHandler()   // ODD IRQ handler for interrupt PD1
     {


    	 ON_LED1();
     	GPIO_IntClear(GPIO_IntGet());
     	Accelerometer_alert=1;

     	//Accelerometer_Disable();

     }

//

     void leave_touch(void)
     {


    	     	 /* Get channels that are pressed, result is or'ed together */
    	     	     	          channels_touched = LETOUCH_GetChannelsTouched();



    	     	     	          /* Check if any channels are in the touched state. */

    	     	     	        if((channels_touched&2048))
    	     	     	        {
    	     	     	        	ON_LED1();

    	     	     	        	    	     	          	    	    		         	leave_over= 1;
    	     	     	        	    	     	          	    	    		         	    entry_over=0;
    	     	     	        }
    	     	     	      else if((channels_touched&512)| (channels_touched&1024))
    	     	     	               	     	     	        {
    	     	     	                                            capacitive_alert=1;

    	     	     	               	     	     	        }
    	     	     	               	     	     	        else
    	     	     	               	     	     	        {

    	     	     	               	     	     	        }
     }

     void entry_touch(void)
          {

         	     	     	          channels_touched = LETOUCH_GetChannelsTouched();



         	     	     	          /* Check if any channels are in the touched state. */

         	     	     	        if((channels_touched&256))
         	     	     	        {
         	     	     	        	ON_LED0();

         	     	     	        	    	     	          	    	    		         	leave_over=(!leave_over);
         	     	     	        	    	     	          	    	    		         	    entry_over=1;
         	     	     	        	    	     	          	    	    		         	GPIO_PinOutClear(gpioPortD,0);
         	     	     	        }
         	     	     	        else if((channels_touched&512)| (channels_touched&1024))
         	     	     	        {
                                      capacitive_alert=1;
         	     	     	        }
         	     	     	        else
         	     	     	        {

         	     	     	        }
          }



     void LEUART0_Configuration(void)
            {
         	   CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);                //Enabling LFXO as the oscillator to the LFB clock tree
         	   	CMU_ClockEnable(cmuClock_LEUART0, true);                         //Enabling clock to LEUART0

         	   	LEUART_Init_TypeDef LEUART0_InitStructure;

         	   	LEUART0_InitStructure.baudrate = 9600;                           //Setting Baud Rate = 9600
         	   	LEUART0_InitStructure.databits = leuartDatabits8;                //Setting the number of data bits in the frame
         	   	LEUART0_InitStructure.enable = leuartDisable;
         	   	LEUART0_InitStructure.parity = leuartNoParity;                   //Setting no parity bits
         	   	LEUART0_InitStructure.refFreq = 0;
         	   	LEUART0_InitStructure.stopbits = leuartStopbits1;                //Setting 1 stop bit in the UART frame

         	   	LEUART_Init(LEUART0, &LEUART0_InitStructure);                    //Initializing the LEUART0 with the required confiuration
         	   	LEUART0->ROUTE = LEUART_ROUTE_LOCATION_LOC0;                     //Routing LEUART0 pins to Location 0
         	    LEUART0->CTRL |= LEUART_CTRL_LOOPBK|LEUART_CTRL_AUTOTRI;
         	   	CMU_ClockEnable(cmuClock_GPIO, true);                            //Enabling clock to GPIO
         	   	GPIO_PinModeSet(LEUART0_Port, LEUART0_Pin, gpioModePushPull, 1); //Setting the pin mode of LEUART0 port and pin

         	   	NVIC_EnableIRQ(LEUART0_IRQn);

            }
            void LEUART0_IRQHandler(void)
                     {
         	   if(byte_num!=6)
         	                                            	{

         	                                            	uint8_t testnew = remove_item(transmit_buffer);
         	                                            	LEUART0->TXDATA = testnew;
         	                                            	while(!(LEUART0->STATUS & LEUART_STATUS_TXC));
         	                                            	while(!(LEUART0->STATUS & LEUART_STATUS_RXDATAV));
         	                                            	testnew = LEUART0->RXDATA;
         	                                            	LEUART0->IFC = LEUART_IEN_TXC;
         	                                            	byte_num++;
         	   //                                         	testpointer++;
         	                                            	}
         	                                            	else
         	                                            	{
         	                                            		byte_num=0;
         	                                            		LEUART_FreezeEnable(LEUART0, true);
         	                                            	    LEUART0->CMD &= ~LEUART_CMD_TXEN;
         	                                            	    LEUART0->ROUTE &= ~LEUART_ROUTE_TXPEN;
         	                                            	    LEUART0->IEN &= ~LEUART_IEN_TXBL;
         	                                            	    LEUART_FreezeEnable(LEUART0, false);

         	                                            	    //UnBlockSleepMode(EM2);
         	                                            	    CMU_ClockEnable(cmuClock_LEUART0, false);

         	                                            	}
                    }



     int main(void)
     {
     CHIP_Init();                                              // Chip_Initialization
     BlockSleepMode(EM3);
     //ULFRCO_Calibration();
     ClockManagementUnit_Configuration();

     GPIO_Configuration();
     transmit_buffer = &buffer1;
     circ_buff_initialize(transmit_buffer, 10);
     LET0_Configuration();
     LETIMER0->CMD=0X00000001;


            /* Init Capacitive touch for channels configured in sensitivity array */
            LETOUCH_Init(sensitivity);
            BlockSleepMode(EM2);
            /* If any channels are touched while starting, the calibration will not be correct. */
            /* Wait while channels are touched to be sure we continue in a calibrated state. */
          //  while(LETOUCH_GetChannelsTouched() != 0);

            /* Enter infinite loop. */

     while (1)
     {
    	 if(entry_over==0)
    	 {
    	 entry_touch();
    	 }
    	 else if(entry_over==1)
    	 {
    	 leave_touch();
    	 }
    	 else
    	 {

    	 }
          Sleep_routine();
     }

  }//main




