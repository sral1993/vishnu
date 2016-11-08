/*
 * led.c
 *
 *  Created on: Nov 7, 2016
 *      Author: Prithvi
 */
/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "MKL25Z4.h"
#include "led.h"


void TPM2_Configure(int Mod)
{
    MCG_BASE_PTR->C1 = MCG_C1_IREFS_MASK | MCG_C1_IRCLKEN_MASK; //MCG Control 1 register...multipurpose clock generator
    MCG_BASE_PTR->C2 = MCG_C2_IRCS_MASK; //MCG Control 2 register
    SIM_BASE_PTR->SCGC6 |= SIM_SCGC6_TPM2_MASK; //system clock gating control
    SIM_BASE_PTR->SOPT2 |= SIM_SOPT2_TPMSRC(3); //system options register
    TPM2_BASE_PTR->SC = TPM_SC_CMOD(1) | TPM_SC_PS(7); //status and control register
    TPM2_BASE_PTR->MOD = Mod; // Modulo value
}
void TPM0_Configure(int Mod)
{
    MCG_BASE_PTR->C1 = MCG_C1_IREFS_MASK | MCG_C1_IRCLKEN_MASK;
    MCG_BASE_PTR->C2 = MCG_C2_IRCS_MASK; //Select fast internal clock
    SIM_BASE_PTR->SCGC6 |= SIM_SCGC6_TPM0_MASK; //Enable TPM2 clock
    SIM_BASE_PTR->SOPT2 |= SIM_SOPT2_TPMSRC(3);
    TPM0_BASE_PTR->SC = TPM_SC_CMOD(1) | TPM_SC_PS(7);
    TPM0_BASE_PTR->MOD = Mod;
}
void LED_RED(int Mod)
{
    TPM2_Configure(Mod);
    SIM_BASE_PTR->SCGC5 |=SIM_SCGC5_PORTB_MASK;
    PORTB_BASE_PTR->PCR[18] =PORT_PCR_MUX(3);
    GPIOB_PDDR= 1<<18;
    TPM2_BASE_PTR->CONTROLS[0].CnSC=TPM_CnSC_MSB_MASK|TPM_CnSC_ELSA_MASK;
    TPM2_BASE_PTR->CONTROLS[0].CnV=Mod;
}
void LED_GREEN(int Mod)
{
    TPM2_Configure(Mod);
    SIM_BASE_PTR->SCGC5 |=SIM_SCGC5_PORTB_MASK;
    PORTB_BASE_PTR->PCR[19] =PORT_PCR_MUX(3);
    GPIOB_PDDR= 1<<19;
    TPM2_BASE_PTR->CONTROLS[1].CnSC=TPM_CnSC_MSB_MASK|TPM_CnSC_ELSA_MASK;
    TPM2_BASE_PTR->CONTROLS[1].CnV=Mod;
}
void LED_BLUE(int Mod)
{
    TPM0_Configure(Mod);
    SIM_BASE_PTR->SCGC5 |=SIM_SCGC5_PORTD_MASK;
    PORTD_BASE_PTR->PCR[1] =PORT_PCR_MUX(4);
    GPIOD_PDDR= 1<<1;
    TPM0_BASE_PTR->CONTROLS[1].CnSC=TPM_CnSC_MSB_MASK|TPM_CnSC_ELSA_MASK;
    TPM0_BASE_PTR->CONTROLS[1].CnV=Mod;
}

void LED_YELLOW(int Mod)
{
    LED_RED(Mod);
    LED_GREEN(Mod);
}
void LED_CYAN(int Mod)
{
    LED_GREEN(Mod);
    LED_BLUE(Mod);
}
void LED_VIOLET(int Mod)
{
    LED_RED (Mod);
    LED_BLUE(Mod);
}
void LED_WHITE(int Mod)
{
    LED_RED(Mod);
    LED_GREEN(Mod);
    LED_BLUE(Mod);
}

void LED_OFF()
{

    SIM_BASE_PTR->SCGC5 |=SIM_SCGC5_PORTB_MASK;
    SIM_BASE_PTR->SCGC5 |=SIM_SCGC5_PORTD_MASK;
    PORTD_BASE_PTR->PCR[1] =PORT_PCR_MUX(1);
    PORTB_BASE_PTR->PCR[18] =PORT_PCR_MUX(1);
    PORTB_BASE_PTR->PCR[19] =PORT_PCR_MUX(1);
    GPIOB_PSOR=GPIO_PCOR_PTCO_MASK;
    GPIOD_PSOR=GPIO_PCOR_PTCO_MASK;
}



