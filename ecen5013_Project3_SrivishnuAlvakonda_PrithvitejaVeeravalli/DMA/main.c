/*
 * main.c
 *
 *  Created on: Oct 31, 2016
 *      Author: Vishnu
 */
#include "MKL25Z4.h"
#include<stdint.h>
#include<string.h>
#include"log.h"
#include<math.h>
#include<stdlib.h>
#include "itoa.h"


void dma_init(int *src,int *dst);
void dma_init2(int *src,int *dst);
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
	int memzero=1;
	if(memzero==0)
	{
		int src_addr= 0x1234;
	     int *src;
		    int dst_addr= 0x2579;
		    int *dst;
		    src = &src_addr;
		    dst = &dst_addr;
    int byt_num = 10;
    int count;
    if((byt_num % 4)==0)
    {
    	if((byt_num % 4)==0)
    	    {
    	    count = (byt_num/4);
    	    }
    	    else
    	    {
    	    count = (byt_num/4)+1;
    	    }
    	    int current_state=0;

    		start();
    		while(current_state < count)
    		{
    			dma_init(src,dst);
    			src++;
    			dst++;
    			current_state++;
    		}



    	int time2=stop();
    		char str1[]="time for memmove_dma for 100bytes in us                                    :";
    		int l1=strlen(str1);
    		LOG1(str1,l1,time2,8);
    }
    else
    {
    	int current_state=0;
    	count = (byt_num/4);
    	    		start();
    	    		while(current_state < count)
    	    		{
    	    			dma_init(src,dst);
    	    			src++;
    	    			dst++;
    	    			current_state++;
    	    		}
    	int mod;
    	mod= (byt_num % 4);
    	int x=0;
    	while(x < mod)
    	{
    	dma_init2(src,dst);
    	x++;
    	}
    	int time2=stop();
    	    		    		char str1[]="time for memmove_dma for 10bytes in us                                     :";
    	    		    		int l1=strlen(str1);
    	    		    		LOG1(str1,l1,time2,8);

    }

	}

if(memzero==1)
{
	int  src_addr,dst_addr;
			    int *src, *temp;
			    temp= &src_addr;
			    src = &src_addr;
			    int *dst;
			    dst = &dst_addr;
	    uint32_t byt_num = 10;
	    //uint32_t byt_num = 100;
	    //uint32_t byt_num = 1000;
	    //uint32_t byt_num = 5000;
	    int zerocount=byt_num;
	    int count;

	    if(memzero==1)
	    {
	    	int k=1;
	    	while(k<zerocount+1)
	    	{
	    		*temp=0;
	    		temp++;
	    		k++;
	    	}
	    }



	        if((byt_num % 4)==0)
	    {
	    	if((byt_num % 4)==0)
	    	    {
	    	    count = (byt_num/4);
	    	    }
	    	    else
	    	    {
	    	    count = (byt_num/4)+1;
	    	    }
	    	    int current_state=0;

	    		start();

	    		while(current_state < count)
	    		{
	    			dma_init(src,dst);
	    			src++;
	    			dst++;
	    			current_state++;
	    		}



	    	int time2=stop();
	    	if(byt_num>1150 && byt_num<2300 )
	    	{
	    		time2=time2+2974;
	    	}
	    	if(byt_num>2300 && byt_num<3450 )
	    	    	{
	    	    		time2=time2+(2*2974);
	    	    	}
	    	if(byt_num>3450 && byt_num<4600 )
	    	    	    	{
	    	    	    		time2=time2+(3*2974);
	    	    	    	}
	    	if(byt_num>4600 && byt_num<5750 )
	    	    	    	{
	    	    	    		time2=time2+(3*2974);
	    	    	    	}
	    		char str1[]="time for memzero_dma for 100bytes in us                                    :";
	    		//char str1[]="time for memzero_dma for 1000bytes in us                                   :";
	    		//char str1[]="time for memzero_dma for 5000bytes in us                                   :";
	    		int l1=strlen(str1);
	    		LOG1(str1,l1,time2,8);
	    }
	    else
	    {
	    	int current_state=0;
	    	count = (byt_num/4);
	    	    		start();
	    	    		while(current_state < count)
	    	    		{
	    	    			dma_init(src,dst);
	    	    			src++;
	    	    			dst++;
	    	    			current_state++;
	    	    		}
	    	int mod;
	    	mod= (byt_num % 4);
	    	int x=0;
	    	while(x < mod)
	    	{
	    	dma_init2(src,dst);
	    	x++;
	    	}
	    	int time2=stop();
	    	if(byt_num>1150 && byt_num<2300 )
	    	    	{
	    	    		time2=time2+2974;
	    	    	}
	    	    	if(byt_num>2300 && byt_num<3450 )
	    	    	    	{
	    	    	    		time2=time2+(2*2974);
	    	    	    	}
	    	    	if(byt_num>3450 && byt_num<4600 )
	    	    	    	    	{
	    	    	    	    		time2=time2+(3*2974);
	    	    	    	    	}
	    	    	if(byt_num>4600 && byt_num<5750 )
	    	    	    	    	{
	    	    	    	    		time2=time2+(3*2974);
	    	    	    	    	}
	    	    		    		char str1[]=" time for memzero_dma for 10bytes in us                                       :";
	    	    		    		int l1=strlen(str1);
	    	    		    		LOG1(str1,l1,time2,8);

	    }

}
}


void dma_init(int *src,int *dst)
{

	//ready = 0;

	// Enable clocks
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;

	// Config DMA Mux for ADC operation
	// Disable DMA Mux channel first
	DMAMUX0_CHCFG0 = 0x00;

	// Configure DMA
	//DMA_SAR0 = (uint32_t)&ADC0_RA;
	DMA_SAR0 = src;
	//DMA_DAR0 = (uint32_t)&value;
	DMA_DAR0 = dst;
	DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(4); // 2 bytes (16 bits) per transfer

	DMA_DCR0 |= (DMA_DCR_EINT_MASK|		// Enable interrupt
				 DMA_DCR_ERQ_MASK |		// Enable peripheral request
				 DMA_DCR_CS_MASK  |
				 DMA_DCR_SSIZE(4) |		// Set source size to 16 bits
				 DMA_DCR_DINC_MASK|		// Set increments to destination address
				 DMA_DCR_DMOD(2)  |     // Destination address modulo of 16 bytes
				 DMA_DCR_DSIZE(4));		// Set destination size of 16 bits


	// Enable DMA channel and source
	//DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(40); // Enable DMA channel and set ADC0 as source
     DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK;
	// Enable interrupt
	NVIC_EnableIRQ(DMA0_IRQn);
	DMA_DCR_START_MASK;
}

/*
 * Handles DMA0 interrupt
 * Resets the BCR register and clears the DONE flag
 * */
void DMA0_IRQHandler(void)
{
	/* Enable DMA0*/
	DMA_DSR_BCR_DONE_MASK;
	DMA_DSR_BCR0 |= DMA_DSR_BCR_DONE_MASK;	// Clear Done Flag
	//DMA_DSR_BCR0 |= DMA_DSR_BCR_BCR(4);		// Set byte count register
	//ready += 1;

}

void dma_init2(int *src,int *dst)
{

	//ready = 0;

	// Enable clocks
	SIM_SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	SIM_SCGC7 |= SIM_SCGC7_DMA_MASK;

	// Config DMA Mux for ADC operation
	// Disable DMA Mux channel first
	DMAMUX0_CHCFG0 = 0x00;

	// Configure DMA
	//DMA_SAR0 = (uint32_t)&ADC0_RA;
	DMA_SAR0 = src;
	//DMA_DAR0 = (uint32_t)&value;
	DMA_DAR0 = dst;
	DMA_DSR_BCR0 = DMA_DSR_BCR_BCR(1); // 2 bytes (16 bits) per transfer

	DMA_DCR0 |= (DMA_DCR_EINT_MASK|		// Enable interrupt
				 DMA_DCR_ERQ_MASK |		// Enable peripheral request
				 DMA_DCR_CS_MASK  |
				 DMA_DCR_SSIZE(1) |		// Set source size to 16 bits
				 DMA_DCR_DINC_MASK|		// Set increments to destination address
				 DMA_DCR_DMOD(1)  |     // Destination address modulo of 16 bytes
				 DMA_DCR_DSIZE(1));		// Set destination size of 16 bits


	// Enable DMA channel and source
	//DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK | DMAMUX_CHCFG_SOURCE(40); // Enable DMA channel and set ADC0 as source
     DMAMUX0_CHCFG0 |= DMAMUX_CHCFG_ENBL_MASK;
	// Enable interrupt
	NVIC_EnableIRQ(DMA0_IRQn);
	DMA_DCR_START_MASK;
}
