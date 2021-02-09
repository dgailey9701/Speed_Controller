/*
 * Copyright 2017-2019 NXP
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
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
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

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Clocks v4.0
* BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/**
 * @file    clock_config.c
 * @brief   Board clocks initialization file.
 */
 
/* This is a template for board specific configuration created by MCUXpresso IDE Project Wizard.*/

#include "MKV31F25612.h"
#include "clock_config.h"

#define MCG_S_IRCST_VAL ((MCG->S & MCG_S_IRCST_MASK) >> MCG_S_IRCST_SHIFT)
#define MCG_S_CLKST_VAL (((uint32_t)MCG->S & (uint32_t)MCG_S_CLKST_MASK) >> (uint32_t)MCG_S_CLKST_SHIFT)
#define MCG_S_IREFST_VAL (((uint32_t)MCG->S & (uint32_t)MCG_S_IREFST_MASK) >> (uint32_t)MCG_S_IREFST_SHIFT)
#define MCG_S_PLLST_VAL ((MCG->S & MCG_S_PLLST_MASK) >> MCG_S_PLLST_SHIFT)
#define MCG_C1_FRDIV_VAL ((MCG->C1 & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT)
#define MCG_C2_LP_VAL (((uint32_t)MCG->C2 & (uint32_t)MCG_C2_LP_MASK) >> (uint32_t)MCG_C2_LP_SHIFT)
#define MCG_C2_RANGE_VAL ((MCG->C2 & MCG_C2_RANGE_MASK) >> MCG_C2_RANGE_SHIFT)
#define MCG_SC_FCRDIV_VAL ((MCG->SC & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT)
#define MCG_S2_PLLCST_VAL ((MCG->S2 & MCG_S2_PLLCST_MASK) >> MCG_S2_PLLCST_SHIFT)
#define MCG_C7_OSCSEL_VAL ((MCG->C7 & MCG_C7_OSCSEL_MASK) >> MCG_C7_OSCSEL_SHIFT)
#define MCG_C4_DMX32_VAL ((MCG->C4 & MCG_C4_DMX32_MASK) >> MCG_C4_DMX32_SHIFT)
#define MCG_C4_DRST_DRS_VAL ((MCG->C4 & MCG_C4_DRST_DRS_MASK) >> MCG_C4_DRST_DRS_SHIFT)
#define MCG_C7_PLL32KREFSEL_VAL ((MCG->C7 & MCG_C7_PLL32KREFSEL_MASK) >> MCG_C7_PLL32KREFSEL_SHIFT)
#define MCG_C5_PLLREFSEL0_VAL ((MCG->C5 & MCG_C5_PLLREFSEL0_MASK) >> MCG_C5_PLLREFSEL0_SHIFT)
#define MCG_C11_PLLREFSEL1_VAL ((MCG->C11 & MCG_C11_PLLREFSEL1_MASK) >> MCG_C11_PLLREFSEL1_SHIFT)
#define MCG_C11_PRDIV1_VAL ((MCG->C11 & MCG_C11_PRDIV1_MASK) >> MCG_C11_PRDIV1_SHIFT)
#define MCG_C12_VDIV1_VAL ((MCG->C12 & MCG_C12_VDIV1_MASK) >> MCG_C12_VDIV1_SHIFT)
#define MCG_C5_PRDIV0_VAL ((MCG->C5 & MCG_C5_PRDIV0_MASK) >> MCG_C5_PRDIV0_SHIFT)
#define MCG_C6_VDIV0_VAL ((MCG->C6 & MCG_C6_VDIV0_MASK) >> MCG_C6_VDIV0_SHIFT)


/**
 * @brief Set up and initialize all required blocks and functions related to the board hardware.
 */
void BOARD_InitBootClocks(void)
{
    uint8_t mcg_c4;
    uint8_t change_drs = 0;
    int i;

    uint32_t j = 30000U;
    // ++++++ Turn on all port clocks ++++++
    SIM->SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK |
                SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK |
                SIM_SCGC5_PORTE_MASK;


    MCG->C2 = (uint8_t)((MCG->C2 & ~(MCG_C2_EREFS0_MASK | MCG_C2_HGO0_MASK | MCG_C2_RANGE0_MASK)) | MCG_C2_RANGE(1U) | (uint8_t)0x4U);

    OSC->CR &= ~(OSC_CR_ERCLKEN_MASK | OSC_CR_EREFSTEN_MASK);
    OSC->CR |= 0x80U;

    OSC->DIV = 0;

	/* Wait for stable. */
	while (0U == (MCG->S & MCG_S_OSCINIT0_MASK))
	{
	}

    SIM->CLKDIV1 = 0x01230000U;

    MCG->C7 = (uint8_t)(MCG->C7 & ~MCG_C7_OSCSEL_MASK) | MCG_C7_OSCSEL(0);

	/* ERR009878 Delay at least 50 micro-seconds for external clock change valid. */
	i = 1500U;
	while (0U != (i--))
	{
		__NOP();
	}


    mcg_c4 = MCG->C4;

    if ((uint8_t)1 == MCG_S_IREFST_VAL)
    {
        change_drs = 1;
        /* Change the LSB of DRST_DRS. */
        MCG->C4 ^= (1U << MCG_C4_DRST_DRS_SHIFT);
    }

    /* Set CLKS and IREFS. */
    MCG->C1 = (uint8_t)((MCG->C1 & ~(MCG_C1_CLKS_MASK | MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK)) |
                        (MCG_C1_CLKS(0)         /* CLKS = 0 */
                         | MCG_C1_FRDIV(3)                  /* FRDIV */
                         | MCG_C1_IREFS(0))); /* IREFS = 0 */

    /* If use external crystal as clock source, wait for it stable. */
    if (MCG_C7_OSCSEL(0) == (MCG->C7 & MCG_C7_OSCSEL_MASK))
    {
        if (0U != (MCG->C2 & MCG_C2_EREFS_MASK))
        {
            while (0U == (MCG->S & MCG_S_OSCINIT0_MASK))
            {
            }
        }
    }

    /* Wait and check status. */
    while ((uint8_t)0 != MCG_S_IREFST_VAL)
    {
    }

    /* Errata: ERR007993 */
    if (change_drs)
    {
        MCG->C4 = mcg_c4;
    }

    /* Set DRS and DMX32. */
    mcg_c4  = (uint8_t)((mcg_c4 & ~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK)) |
                       (MCG_C4_DMX32(0) | MCG_C4_DRST_DRS(0)));
    MCG->C4 = mcg_c4;

    /* Wait for DRST_DRS update. */
    while (MCG->C4 != mcg_c4)
    {
    }

    /* Check MCG_S[CLKST] */
    while ((uint8_t)0 != MCG_S_CLKST_VAL)
    {
    }


    while (j--)
    {
        __NOP();
    }



//	MCG->C1 = 0x04;
//    MCG->C4 = 0xE0;

//    while (!(MCG->S & MCG_S_IREFST_MASK)){};


	//int gl_clock_delay = 0;

    //  ++++++ Set the clock speed to 120,000,000 Hz ++++++
    //  An external 8 Mhz crystal is used to generate the 120 Mhz system clock.
    //MCG->C4 = 0x00;
	MCG->C2 = 0x14;
    MCG->C1 = 0x98;

    while (!(MCG->S & MCG_S_OSCINIT0_MASK));

    // Wait for Reference clock Status bit to clear
    while (MCG->S & MCG_S_IREFST_MASK);

    // Wait for clock status bits to show clock source is ext ref clk
    while (((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x2);

    // Configure PLL0. OSC0 is selected as the reference clock
    // Select PLL0 as the source of the PLLS mux
    //MCG->C11 &= ~0x10;
    MCG->C5 = 0x01;

    while (!(MCG->S & MCG_S_OSCINIT0_MASK));

    MCG->C6 = 0x41;

    // Wait for the PLL status bit to be set
    while (!(MCG->S & MCG_S_PLLST_MASK)){};

    // Place a delay
   // while (gl_clock_delay < 1000)
   // {
   //     gl_clock_delay++;

   // }   // End of while (gl_clock_delay < 1000)

    //gl_clock_delay = 0;

    // Wait for LOCK bit to set
    while (!(MCG->S & MCG_S_LOCK0_MASK)){};

    // Place a delay
   // while (gl_clock_delay < 1000)
   // {
   //     gl_clock_delay++;

   // }   // End of while (gl_clock_delay < 1000)

   // gl_clock_delay = 0;

    MCG->C1 &= ~0xC0;

    // Place a delay
   // while (gl_clock_delay < 1000)
   // {
   //     gl_clock_delay++;

   // }   // End of while (gl_clock_delay < 1000)

   // gl_clock_delay = 0;

    // Wait for clock status bits to update
    while (((MCG->S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) != 0x3){};

    // Place a delay
//    while (gl_clock_delay < 1000)
//    {
//        gl_clock_delay++;

 //   }   // End of while (gl_clock_delay < 1000)

 //   gl_clock_delay = 0;

	/* Read core clock setting. */
//	SystemCoreClockUpdate();

    // ++++++ Turn on all port clocks ++++++
//    SIM->SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK |
//                SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK |
//                SIM_SCGC5_PORTE_MASK;
}
