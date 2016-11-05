/*
 * main.c
 *
 *  Created on: Nov 5, 2016
 *      Author: Vishnu
 */


#include "MKL25Z4.h"
#include<stdint.h>
#include<string.h>
#include"log.h"
#include<math.h>
#include<stdlib.h>
#include "itoa.h"


uint32_t *src,*dst;
uint32_t s[1000];
uint32_t d[1000];
uint32_t memzero=0;
//uint32_t x[4]={10,100,1000,5000};
uint32_t length=5000;

uint32_t check;


void dma_init();
void dma_init1();
void dma_init2();
void memmove_dma();
void memzero_dma();
void start();
void TPM2_Setup();
int stop();


void TPM2_Setup()
{

	SIM_BASE_PTR->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	SIM_BASE_PTR->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	TPM2_BASE_PTR->SC = TPM_SC_CMOD(1) | TPM_SC_PS(0);
	TPM2_BASE_PTR->MOD = 32000;
	TPM2_BASE_PTR->CONTROLS[0].CnSC=TPM_CnSC_MSB_MASK|TPM_CnSC_ELSA_MASK;
	TPM2_BASE_PTR->CONTROLS[0].CnV=TPM2_BASE_PTR->MOD;
}

void start()
{
	TPM2_Setup();
}

int stop()
{
	TPM2_BASE_PTR->SC = TPM_SC_CMOD(0);
	int count=TPM2_CNT;
	int x=count/21;
	return x;
}

void main()
{
	int i=0,j=0;
	check=length;
	src = &s[0];
	dst = &d[0];
	memmove_dma();
    //memzero_dma();
}

void memmove_dma()
{
	if((length % 4)==0)
	{
		if((src+length-1)<dst)
			{
				dma_init();
				start();
				DMA_DCR0|=DMA_DCR_START_MASK;
			}
			else
			{
				src=src+length-1;
				dst=dst+length-1;
				dma_init1();
			    start();
				DMA_DCR0|=DMA_DCR_START_MASK;
			}
	}
	else
	{
		int mod,div;
		mod=length%4;
		div=length/4;
		length=div*4;
		if((src+length-1)<dst)
					{
						dma_init();
						start();
						DMA_DCR0|=DMA_DCR_START_MASK;
						length=mod;
						dma_init2();
						check=4;
						DMA_DCR0|=DMA_DCR_START_MASK;
					}
					else
					{
						src=src+length-1;
						dst=dst+length-1;
						dma_init1();
						length=mod;
						dma_init2();
					    start();
						DMA_DCR0|=DMA_DCR_START_MASK;
					}



	}

}

void memzero_dma()
{
	*src=0;
	if((length % 4)==0)
		{
			if((src+length-1)<dst)
				{
					dma_init();
					start();
					DMA_DCR0|=DMA_DCR_START_MASK;
				}
				else
				{
					src=src+length-1;
					dst=dst+length-1;
					dma_init1();
				    start();
					DMA_DCR0|=DMA_DCR_START_MASK;
				}
		}
		else
		{
			int mod,div;
			mod=length%4;
			div=length/4;
			length=div*4;
			if((src+length-1)<dst)
						{
							dma_init();
							start();
							DMA_DCR0|=DMA_DCR_START_MASK;
							length=mod;
							dma_init2();
							check=4;
							DMA_DCR0|=DMA_DCR_START_MASK;
						}
						else
						{
							src=src+length-1;
							dst=dst+length-1;
							dma_init1();
							length=mod;
							dma_init2();
						    start();
							DMA_DCR0|=DMA_DCR_START_MASK;
						}



		}
}


void dma_init()
{

	//ready = 0;

	// Enable clocks
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;

	// Config DMA Mux for ADC operation
	// Disable DMA Mux channel first
	DMAMUX0_CHCFG0 = 0x00;

	DMA0->DMA[0].DSR_BCR=DMA_DSR_BCR_DONE_MASK;// Writing a 1 to this bit clears all DMA status bits
	DMA0->DMA[0].DSR_BCR|=DMA_DSR_BCR_BCR_MASK&length;//no of bytes



		// Configure DMA
			//DMA_SAR0 = (uint32_t)&ADC0_RA;
			//DMA_SAR0 = (uint32_t) s;
			DMA_SAR0 = src;
			//DMA_DAR0 = (uint32_t)&value;
			//DMA_DAR0 = (uint32_t) d;
			DMA_DAR0 = dst;
			//DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(4); // 2 bytes (16 bits) per transfer
if(memzero==0)
{
			DMA_DCR0 |= (DMA_DCR_EINT_MASK|		// Enable interrupt
						 DMA_DCR_SSIZE(0) |		// Set source size to 16 bits
						 DMA_DCR_SINC_MASK|
						 DMA_DCR_DINC_MASK|		// Set increments to destination address
						 DMA_DCR_DSIZE(0));		// Set destination size of 16 bits
}
else
{
	DMA_DCR0 |= (DMA_DCR_EINT_MASK|		// Enable interrupt
							 DMA_DCR_SSIZE(0) |		// Set source size to 16 bits
							 DMA_DCR_DINC_MASK|		// Set increments to destination address
							 DMA_DCR_DSIZE(0));		// Set destination size of 16 bits

}

	// Enable DMA channel and source
	//DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(40); // Enable DMA channel and set ADC0 as source
     DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK|DMAMUX_CHCFG_SOURCE(60);
	// Enable interrupt
	NVIC_EnableIRQ(DMA0_IRQn);

}

/*
 * Handles DMA0 interrupt
 * Resets the BCR register and clears the DONE flag
 * */
void DMA0_IRQHandler(void)
{
	if( (check % 4)==0)
	{
	int time1=stop();
	char str[]=" DMA time for memzero 5000 bytes in us:                                        ";
	int l=strlen(str);
	LOG1(str,l,time1,8);
	__disable_irq();
	}

}

void dma_init1()
{

   int i=1;
   while(i<length)
   {
	   int k=1;
	   // Enable clocks
	   	SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	   	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;

	   	// Config DMA Mux for ADC operation
	   	// Disable DMA Mux channel first
	   	DMAMUX0_CHCFG0 = 0x00;

	   	DMA0->DMA[0].DSR_BCR=DMA_DSR_BCR_DONE_MASK;// Writing a 1 to this bit clears all DMA status bits
	   	DMA0->DMA[0].DSR_BCR|=DMA_DSR_BCR_BCR_MASK&k;//no of bytes

	   	// Configure DMA
	   	//DMA_SAR0 = (uint32_t)&ADC0_RA;
	   	//DMA_SAR0 = (uint32_t) s;
	   	DMA_SAR0 = src;
	   	//DMA_DAR0 = (uint32_t)&value;
	   	//DMA_DAR0 = (uint32_t) d;
	   	DMA_DAR0 = dst;
	   	//DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(4); // 2 bytes (16 bits) per transfer
	   	if(memzero==0)
	   	{
	   				DMA_DCR0 |= (DMA_DCR_EINT_MASK|		// Enable interrupt
	   							 DMA_DCR_SSIZE(0) |		// Set source size to 16 bits
	   							 DMA_DCR_SINC_MASK|
	   							 DMA_DCR_DINC_MASK|		// Set increments to destination address
	   							 DMA_DCR_DSIZE(0));		// Set destination size of 16 bits
	   	}
	   	else
	   	{
	   		DMA_DCR0 |= (DMA_DCR_EINT_MASK|		// Enable interrupt
	   								 DMA_DCR_SSIZE(0) |		// Set source size to 16 bits
	   								 DMA_DCR_DINC_MASK|		// Set increments to destination address
	   								 DMA_DCR_DSIZE(0));		// Set destination size of 16 bits

	   	}
	   	// Enable DMA channel and source
	   	//DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(40); // Enable DMA channel and set ADC0 as source
	        DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK|DMAMUX_CHCFG_SOURCE(60);
	   	// Enable interrupt
	   	NVIC_EnableIRQ(DMA0_IRQn);
	   	i++;
	   	src=src-2;
	   	dst=dst-2;
   }

}

void dma_init2()
{

	//ready = 0;

	// Enable clocks
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;

	// Config DMA Mux for ADC operation
	// Disable DMA Mux channel first
	DMAMUX0_CHCFG0 = 0x00;

	DMA0->DMA[0].DSR_BCR=DMA_DSR_BCR_DONE_MASK;// Writing a 1 to this bit clears all DMA status bits
	DMA0->DMA[0].DSR_BCR|=DMA_DSR_BCR_BCR_MASK&length;//no of bytes



		// Configure DMA
			//DMA_SAR0 = (uint32_t)&ADC0_RA;
			//DMA_SAR0 = (uint32_t) s;
			DMA_SAR0 = src;
			//DMA_DAR0 = (uint32_t)&value;
			//DMA_DAR0 = (uint32_t) d;
			DMA_DAR0 = dst;
			//DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(4); // 2 bytes (16 bits) per transfer
if(memzero==0)
{
			DMA_DCR0 |= (DMA_DCR_EINT_MASK|		// Enable interrupt
						 DMA_DCR_SSIZE(1) |		// Set source size to 16 bits
						 DMA_DCR_SINC_MASK|
						 DMA_DCR_DINC_MASK|		// Set increments to destination address
						 DMA_DCR_DSIZE(1));		// Set destination size of 16 bits
}
else
{
	DMA_DCR0 |= (DMA_DCR_EINT_MASK|		// Enable interrupt
							 DMA_DCR_SSIZE(1) |		// Set source size to 16 bits
							 DMA_DCR_DINC_MASK|		// Set increments to destination address
							 DMA_DCR_DSIZE(1));		// Set destination size of 16 bits

}

	// Enable DMA channel and source
	//DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(40); // Enable DMA channel and set ADC0 as source
     DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK|DMAMUX_CHCFG_SOURCE(60);
	// Enable interrupt
	NVIC_EnableIRQ(DMA0_IRQn);

}

