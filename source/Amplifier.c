//#############################################################################
//  File Name:  Amplifier.c
//
//  Function Prefix:  "Amp_"
//
//  Description:  This module implements the functions of a DC brushless servo
//                motor amplifier. It implements a 6-step commutation method
//                intended for use with a motor that has a trapezoidal BEMF
//                profile, although it will work with sinusoidal motors
//                (brushless AC) with increased torque ripple.
//
//                3 discrete Hall effect sensors are required for motor
//                commutation. These 3 discrete inputs will create 6 valid
//                "commutation states" that will determine which phases are
//                energized. Hence the 6-step commutation method.
//
//                The amplifier will control monitor motor current using 3
//                sensors placed in series with the phases. The commutation
//                state will determine which sensor is used to update the motor
//                current signal. A PI controller will be implemented to
//                control the behavior of the current loop. The input to the
//                module will be a fixed point current command and the
//                output of the module will be 6 PWM signals used to command
//                the output stage gate drivers.
//
//                This module will also contain the scheduler for the servo
//                system. An interrupt synchronized to the PWM output will be
//                divided into 20 sections. Each section will run a function
//                for a specific module that is time significant.
//
//                NOTE:  To reduce the impact of high frequency noise on the
//                       commutation sensors, the digital filtering is enabled
//                       for PortA and each pin used for the hall sensors
//                       are enabled. The input filter is set to 32.  With
//                       a 96 Mhz clock, this will pass a 1.5 Mhz signal.
//
//  Hardware Resources Required:
//
//                GPIO:
//                            PortA.28  Pin 79    R-S COMMUTATION INPUT
//                            PortA.27  Pin 78    S-T COMMUTATION INPUT
//                            PortA.26  Pin 77    T-R COMMUTATION INPUT
//                            PortB.3   Pin 84    BRIDGE OVER CURRENT INPUT
//
//                FTM0:       PortC.1   Pin 104   MOT_PH_R_HI_CON
//                            PortC.2   Pin 105   MOT_PH_R_LO_CON
//                            PortC.3   Pin 106   MOT_PH_S_HI_CON
//                            PortC.4   Pin 109   MOT_PH_S_LO_CON
//                            PortD.4   Pin 104   MOT_PH_T_HI_CON
//                            PortD.5   Pin 104   MOT_PH_T_LO_CON
//
//                ADC1:       SE8       Pin 81    I_PH_S_BUFFERED
//                            SE9       Pin 82    I_PH_T_BUFFERED
//                            SE10      Pin 85    I_PH_R_BUFFERED
//
//  Revisions -
//
//  Rev:   Date:        Name:   Description:
//         2/8/2020		DJG		New Code
//
//#############################################################################

#include "MKV31F25612.h"
#include "Amplifier.h"

void Amp_Init(void)
//*****************************************************************************
//  Description:    Initializes the hardware resources required for the
//                  amplifier module.  It also sets the default values of the
//                  variable used within this module.
//
//  NOTE:
//
//      See Module Description for information on hardware resources
//
//*****************************************************************************
{
    // Set the Digital Filter Width Register to 32, which at 96 MHZ translates
    // to a pass frequency of about 1.5 Mhz
    PORTB_DFWR = 31;

    // Enable the filter for Port Pin B3
    PORTB_DFER |= (1 << 3);

    // Initialize FTM0 as a system tick and PWM. Using FlexTimer Module in
    // Complementary Center-aligned mode See Application Note AN3729

    // Enable clock for FTM0
    SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;

    // Disable write protection
    FTM0_MODE |= FTM_MODE_WPDIS_MASK;

    // Output Polarity Setting
    FTM0_POL = 0x00;

    // Disable all channels outputs using the OUTPUT MASK feature.
    FTM0_OUTMASK =   FTM_OUTMASK_CH5OM_MASK | FTM_OUTMASK_CH4OM_MASK
                     | FTM_OUTMASK_CH3OM_MASK | FTM_OUTMASK_CH2OM_MASK
                     | FTM_OUTMASK_CH1OM_MASK | FTM_OUTMASK_CH0OM_MASK;

    // Set system clock as source for FTM0 (CLKS[1:0] = 01)
    FTM0_SC |= FTM_SC_CLKS(1) | FTM_SC_PS(0) ;

    // Set PWM frequency; MODULO = Fclk/Fpwm
    // for center aligned PWM using combine mode
    // MODULO = 96 MHz/ 20 kHz / 2 = 2399
    FTM0_MOD = (AMP_SYS_CLK/AMP_PWM_FREQ/2) - 1;

    //  Set the configuration to force the outputs to the safe values as
    //  specified in the POLn bits of the FTM0_POL when the controller
    //  is placed in BDM.
    FTM0_CONF = 0x40;

    // CNTIN = -(96 MHz / 20 kHz / 2) = -2400
    FTM0_CNTIN = -(AMP_SYS_CLK/AMP_PWM_FREQ/2);

    //  COMBINE = 1 - Set the combine mode
    //  COMP = 1 - Set to complementary mode
    //  DTEN = 1 - Enable dead-time
    //  SYNCEN = 1 - PWM update synchronization enabled,
    //  FAULTEN = 1 - fault control enabled
    FTM0_COMBINE = FTM_COMBINE_SYNCEN0_MASK | FTM_COMBINE_DTEN0_MASK
               | FTM_COMBINE_COMP0_MASK   | FTM_COMBINE_COMBINE0_MASK
               | FTM_COMBINE_FAULTEN0_MASK
               | FTM_COMBINE_SYNCEN1_MASK | FTM_COMBINE_DTEN1_MASK
               | FTM_COMBINE_COMP1_MASK   | FTM_COMBINE_COMBINE1_MASK
               | FTM_COMBINE_FAULTEN1_MASK
               | FTM_COMBINE_SYNCEN2_MASK | FTM_COMBINE_DTEN2_MASK
               | FTM_COMBINE_COMP2_MASK   | FTM_COMBINE_COMBINE2_MASK
               |FTM_COMBINE_FAULTEN2_MASK;

    //  FAULTM = 0x11 Enable Fault Control on all channels with automatic
    //                fault clearing.
    //  FTMEM = 1     Enable access to all FTM registers with no restrictions.
    FTM0_MODE |= FTM_MODE_FAULTM_MASK | FTM_MODE_FTMEN_MASK;

    //  CNTMAX = 1 - Enable synchronizing to the maximum value
    FTM0_SYNC |= FTM_SYNC_CNTMAX_MASK;

    // Set the Dead time to n counts.
    FTM0_DEADTIME = 0x6E;

    // Initial setting of value registers to 50 % of duty cycle
    FTM0_C0V = -gl_amp_null_dutycycle;
    FTM0_C1V = gl_amp_null_dutycycle;
    FTM0_C2V = -gl_amp_null_dutycycle;
    FTM0_C3V = gl_amp_null_dutycycle;
    FTM0_C4V = -gl_amp_null_dutycycle;
    FTM0_C5V = gl_amp_null_dutycycle;

    FTM0_PWMLOAD = FTM_PWMLOAD_LDOK_MASK;
    FTM0_EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;

    // SWSYNC = 1 - set PWM value update. This bit is cleared automatically
    FTM0_SYNC |= FTM_SYNC_SWSYNC_MASK;

    // ELSnB:ELSnA = 1:0 Set channel mode to generate positive PWM
    FTM0_C0SC |= FTM_CnSC_ELSB_MASK ;
    FTM0_C1SC |= FTM_CnSC_ELSB_MASK ;
    FTM0_C2SC |= FTM_CnSC_ELSB_MASK ;
    FTM0_C3SC |= FTM_CnSC_ELSB_MASK ;
    FTM0_C4SC |= FTM_CnSC_ELSB_MASK ;
    FTM0_C5SC |= FTM_CnSC_ELSB_MASK ;

    /* PORTs for FTM0 initialization */
    PORTC_PCR1 = PORT_PCR_MUX(4); /* FTM0 CH0 */
    PORTC_PCR2 = PORT_PCR_MUX(4); /* FTM0 CH1 */
    PORTC_PCR3 = PORT_PCR_MUX(4); /* FTM0 CH2 */
    PORTC_PCR4 = PORT_PCR_MUX(4); /* FTM0 CH3 */
    PORTD_PCR4 = PORT_PCR_MUX(4); /* FTM0 CH4 */
    PORTD_PCR5 = PORT_PCR_MUX(4); /* FTM0 CH5 */

    // Set FTMO ISR Priority and enable
    NVIC_SetPriority(FTM0_IRQn, 0x00);
    NVIC_EnableIRQ(FTM0_IRQn);
    NVIC_ClearPendingIRQ(FTM0_IRQn);

    //  Enable the interrupt
    FTM0_SC |= FTM_SC_TOIE_MASK;

    // Initialize controller value and parameter structures
    Amp_Init_Control_Values();

    // Ensure the disable amplifier flag is set
    Amp_Disable_Amplifier();

    // Set up the ADC for calibration
    Amp_ADC1_Init_Calibrate();

    // Turn off all fault indicators.
    gl_amp_cntrl_values[AMP_STATUS] &= ~(AMP_ALL_FAULTS_MASK);

    // Initialize motor current first order filter 3dB point approximately
    // 25 Hz - Used to determine motor torque.
    Filter_First_Order_Init(&g_torq_filter, 32511, 32767, 0);

}   // End of void Amp_Init(void)
