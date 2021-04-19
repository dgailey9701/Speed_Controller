/*
 * Copyright 2016-2019 NXP
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
 
/**
 * @file    Speed_Controller.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKV31F25612.h"
#include "Amplifier.h"
#include "System_Speed_Controller.h"

int i = 0;

int main(void) {
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    // ++++++ Turn on all port clocks ++++++
    SIM->SCGC5 = SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK |
                SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK |
                SIM_SCGC5_PORTE_MASK;

    Amp_Init();

    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {


        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}


void System_FTM0_IRQ(void)
//*****************************************************************************
//  Description: This is the FTM0 interrupt, but it is used exclusively for
//               the timer overflow (TOF) interrupt. FTM0 is used for the
//               amplifiers PWM and is configured to operate at a rate of
//               20 Khz.
//
//               The FTM0 interrupt acts as the scheduler for the servo system.
//               At each iteration of the interrupt, a counter is incremented.
//               This counter drives a "time slice" state machine which
//               then calls one of the module foreground tasks. This task
//               should take no more than 50% of the time slice period to
//               complete in order to allow background tasks time to run. Once
//               the counter reaches 20, it wraps around and restarts the
//               task. In this way, each task runs at a rate of 1kHz, with the
//               Amplifier running at a rate of 20kHz.
//
//*****************************************************************************
{
    static uint8_t uc_time_slice = 0;

    // clear interrupt flag
    FTM0->SC &= ~FTM_SC_TOF_MASK;

    // Run the amplifier foreground update
    Amp_Foreground_Update();

    // Run the CPU time sharing state machine.
    switch(uc_time_slice)
    {
        case 0:
        {
            break;
        } // End of case 0

        case 1:
        {
            break;
        } // End of case 1

        case 2:
        {
            break;
        } // End of case 2

        case 3:
        {
            break;
        } // End of case 3

        case 4:
        {
            break;
        } // End of case 4

        case 5:
        {
            break;
        } // End of case 5

        case 6:
        {
            break;
        } // End of case 6

        case 7:
        {
            break;
        } // End of case 7

        case 8:
        {
            break;
        } // End of case 8

        case 9:
        {
            break;
        } // End of case 9

        case 10:
        {
            break;
        } // End of case 10

        case 11:
        {
            break;
        } // End of  case 11

        case 12:
        {
            break;
        } // End of case 12

        case 13:
        {
            break;
        } // End of case 13

        case 14:
        {
            break;
        } // End of case 14

        case 15:
        {
            break;
        } // End of case 15

        case 16:
        {
            break;
        } // End of case 16

        case 17:
        {
            break;
        } // End of case 17

        case 18:
        {
            break;
        } // End of case 18

        case 19:
        {
            break;
        } // End of case 19

        default:
        {
            break;

        }   // End of default:

    } // End of switch(uc_time_slice)

    if (++uc_time_slice > 19)
    {
        uc_time_slice = 0;

    } // End of if(uctime_slice > 19)

}   // End of void System_FTM0_IRQ(void)

