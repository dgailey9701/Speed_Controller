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
#include "System_Speed_Controller.h"
#include <stdint.h>
#include "Amplifier.h"

int32_t gl_amp_null_dutycycle = AMP_SYS_CLK/AMP_PWM_FREQ/4;

static int32_t gl_amp_cntrl[AMP_L_CNTRL_SIZE];

static float gf_amp_cntrl[AMP_F_CNTRL_SIZE];

static int32_t gl_current;

static amp_flags_t g_amp_flags = {0};

// Define the phase mask table. This table will be used along with the
// commutation state to determine which phase will be turned off.
static const uint16_t gun_amp_mask_table[8] = {0x3F, 0x0C, 0x03, 0x30,
                                        0x30, 0x03, 0x0C, 0x3F};

// The following phase invert table will drive the BLDC motor CCW with a
// positive duty cycle command.
static const uint16_t gun_amp_inv_table[8] = {0x00, 0x01, 0x04, 0x01, 0x02,
                                              0x02, 0x04, 0x00};

static float gf_amp_param[AMP_F_CNTRL_SIZE][4] = {
//  DEFAULT 	MIN			MAX			ATTRIBUTES
   {0.0f, 		0.0f, 		0.0f, 		RO_F},		// AMP_F_CUR_PHS
   {0.0f, 		0.0f, 		0.0f, 		RO_F},		// AMP_F_CUR_PHT
   {0.0f, 		0.0f, 		0.0f, 		RO_F},		// AMP_F_CUR_PHR

};

static int32_t gl_amp_param[AMP_L_CNTRL_SIZE][4] = {
//  DEFAULT 	MIN			MAX			ATTRIBUTES
   {0,			0,			0,			RO},		// AMP_STATUS
   {0,			0,			0,			RO},		// AMP_FRGD_STATE
   {0,			0,			0,			RO},		// AMP_FEEDBACK_PHS
   {0,			0,			0,			RO},		// AMP_FEEDBACK_PHT
   {0,			0,			0,			RO},		// AMP_FEEDBACK_PHR
   {0,			0,			0,			RO},		// AMP_COMMUTATION

};

// Declare local functions
void Amp_ADC0_Init_Calibrate (void);
void Amp_ADC0_Init_Sampling (void);
void Amp_ADC0_Calibrate(void);
void Amp_Init_Control_Values(void);
void Amp_Update_Current(void);
void Amp_Update_PWM_Dutycycle(int32_t);


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
    // Enable clock for FTM0
    SIM->SCGC6 |= SIM_SCGC6_FTM0_MASK;

    /* Port D pin alternate pin settings for FTM0 initialization */
    PORTD->PCR[0] = PORT_PCR_MUX(4); /* FTM0 CH0 */
    PORTD->PCR[1] = PORT_PCR_MUX(4); /* FTM0 CH1 */
    PORTD->PCR[2] = PORT_PCR_MUX(4); /* FTM0 CH2 */
    PORTD->PCR[3] = PORT_PCR_MUX(4); /* FTM0 CH3 */
    PORTD->PCR[4] = PORT_PCR_MUX(4); /* FTM0 CH4 */
    PORTD->PCR[5] = PORT_PCR_MUX(4); /* FTM0 CH5 */

    // Disable write protection
    FTM0->MODE |= (FTM_MODE_WPDIS_MASK | FTM_MODE_FTMEN_MASK);

    // Output Polarity Setting
    FTM0->POL = 0x2A;

    // Disable all channels outputs using the OUTPUT MASK feature.
    FTM0->OUTMASK = FTM_OUTMASK_CH5OM_MASK | FTM_OUTMASK_CH4OM_MASK
                   | FTM_OUTMASK_CH3OM_MASK | FTM_OUTMASK_CH2OM_MASK
                    | FTM_OUTMASK_CH1OM_MASK | FTM_OUTMASK_CH0OM_MASK;

    // Set system clock as source for FTM0 (CLKS[1:0] = 01)
    FTM0->SC |= FTM_SC_CLKS(1) | FTM_SC_PS(0);

    // Set PWM frequency; MODULO = Fclk/Fpwm
    // for center aligned PWM using combine mode
    // MODULO = 96 MHz/ 20 kHz / 2 = 2399
    FTM0->MOD = (AMP_SYS_CLK/AMP_PWM_FREQ/2) - 1;

    // CNTIN = -(96 MHz / 20 kHz / 2) = -2400
    FTM0->CNTIN = -(AMP_SYS_CLK/AMP_PWM_FREQ/2);

    //  Set the configuration to force the outputs to the safe values as
    //  specified in the POLn bits of the FTM0_POL when the controller
    //  is placed in BDM.
    FTM0->CONF = 0x00;

    FTM0->SYNCONF |= (FTM_SYNCONF_SYNCMODE_MASK | FTM_SYNCONF_SWOM_MASK |
    		FTM_SYNCONF_SWWRBUF_MASK | FTM_SYNCONF_SWINVC_MASK | FTM_SYNCONF_SWSOC_MASK);

    //  CNTMAX = 1 - Enable synchronizing to the maximum value
    FTM0->SYNC |= (FTM_SYNC_CNTMAX_MASK | FTM_CONF_BDMMODE(3));

    FTM0->SWOCTRL = 0;

    // Set the Dead time to n counts.
    FTM0->DEADTIME = 0x40;

    FTM0->PWMLOAD |= (FTM_PWMLOAD_LDOK_MASK);


    //  COMBINE = 1 - Set the combine mode
    //  COMP = 1 - Set to complementary mode
    //  DTEN = 1 - Enable dead-time
    //  SYNCEN = 1 - PWM update synchronization enabled,
    //  FAULTEN = 1 - fault control enabled
    FTM0->COMBINE = 0x00;

    // SWSYNC = 1 - set PWM value update. This bit is cleared automatically
    FTM0->SYNC |= FTM_SYNC_SWSYNC_MASK;

    // ELSnB:ELSnA = 1:0 Set channel mode to generate positive PWM
    FTM0->CONTROLS[0].CnSC = FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[1].CnSC = FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[2].CnSC = FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[3].CnSC = FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[4].CnSC = FTM_CnSC_ELSB_MASK;
    FTM0->CONTROLS[5].CnSC = FTM_CnSC_ELSB_MASK;

    // Initial setting of value registers to 50 % of duty cycle
    FTM0->CONTROLS[0].CnV = -gl_amp_null_dutycycle;
    FTM0->CONTROLS[1].CnV = gl_amp_null_dutycycle;
    FTM0->CONTROLS[2].CnV = -gl_amp_null_dutycycle;
    FTM0->CONTROLS[3].CnV = gl_amp_null_dutycycle;
    FTM0->CONTROLS[4].CnV = -gl_amp_null_dutycycle;
    FTM0->CONTROLS[5].CnV = gl_amp_null_dutycycle;

    FTM0->OUTMASK = 0x00;//gun_amp_mask_table[gl_amp_cntrl[AMP_COMMUTATION]];

    FTM0->INVCTRL = AMP_INVERT_NONE;
    FTM0->OUTMASK = AMP_DISABLE_ALL_PH;

    // SWSYNC = 1 - set PWM value update. This bit is cleared automatically
    FTM0->SYNC |= FTM_SYNC_SWSYNC_MASK;
    //  Enable the interrupt
    FTM0->SC |= FTM_SC_TOIE_MASK;

    FTM0->MODE |= FTM_MODE_INIT_MASK;


    // Set FTMO ISR Priority and enable
    NVIC_SetPriority(FTM0_IRQn, 0x00);
    NVIC_EnableIRQ(FTM0_IRQn);
    NVIC_ClearPendingIRQ(FTM0_IRQn);

    FTM0->EXTTRIG |= FTM_EXTTRIG_INITTRIGEN_MASK;

    Amp_Init_Control_Values();

    Amp_Disable_Amplifier();

    Amp_ADC0_Init_Calibrate();

}   // End of void Amp_Init(void)


void Amp_Init_Control_Values(void)
//*****************************************************************************
//  Description:
//
//*****************************************************************************
{
    for (uint16_t i = 0; i < AMP_L_CNTRL_SIZE; i++)
    {
        gl_amp_cntrl[i] = gl_amp_param[i][DEF];

    }

    for (uint16_t i = 0; i < AMP_F_CNTRL_SIZE; i++)
    {
        gf_amp_cntrl[i] = gf_amp_param[i][DEF];

    }

}


void Amp_ADC0_Init_Calibrate (void)
//*****************************************************************************
//  Description: This function initializes the resources required by ADC1
//               before calibration begins.
//
//*****************************************************************************
{
    // Configure ADC1 for calibration

    // Set the port pins to use the default ADC functionality
    PORTB->PCR[0] = (0|PORT_PCR_MUX(0));
    PORTB->PCR[1] = (0|PORT_PCR_MUX(0));
    PORTB->PCR[2] = (0|PORT_PCR_MUX(0));

    // Enable the ADC1 clock
    SIM->SCGC6 |= (SIM_SCGC6_ADC0_MASK);

    // CFG1: Total ADC clock = 120 MHz/16 = 7.5 MHz
    // ADICLK = 0b01 - Bus clock divided by 2
    // MODE = 0b11 - 16-bit conversion
    // ADLSMP = 1 - Long Sample Time
    // ADIV = 0b11 - Divide Ratio is 8
    ADC0->CFG1 = ADC_CFG1_ADICLK(0x00) | ADC_CFG1_MODE(0x3) |
    		ADC_CFG1_ADLSMP(0x1) | ADC_CFG1_ADIV(0x3);

    // CFG2:
    // ADLSTS = 0b10 - 6 extra ADCK cycles; 10 ADCK cycles total sample time
    // ADHSC = 1 - Hispeed conversion sequence
    // ADACKEN = 1 - Asynchronous clock and clock output enabled
    ADC0->CFG2 = ADC_CFG2_ADLSTS(0x3) | ADC_CFG2_ADHSC_MASK |
    													ADC_CFG2_ADACKEN_MASK;

    // SC3:
    // AVGS = 0b11 - 32 samples averaged
    // AVGE = 0b1 - Hardware average function enabled
    ADC0->SC3 = ADC_SC3_AVGS(0x3) | ADC_SC3_AVGE_MASK;

    // Start calibration on ADC

    // Clear any existing calibration failed flag
    ADC0->SC3 &= ~ADC_SC3_CALF_MASK;

    // Start calibration
    ADC0->SC3 |= ADC_SC3_CAL_MASK ;

}   // End of void Amp_ADC1_Init_Calibrate (void)


void Amp_ADC0_Init_Sampling (void)
//*****************************************************************************
//  Description: This function initializes the resources required by ADC 1 after
//               calibration to begin sampling
//
//*****************************************************************************
{
    //  ++++++ Initialize the PDB to trigger ADC1  ++++++
    //  Initialize the clock
    SIM->SCGC6 |= SIM_SCGC6_PDB_MASK;

    // Enable pre-trigger channel one
    PDB0->CH[0].C1 |= PDB_C1_EN(0x1) | PDB_C1_TOS(0x1);

    // Set the delay to the middle of the PWM period
    // TODO Need to figure out optimal location for PDB trigger. Use interrupt
    // to find best location
    PDB0->CH[0].DLY[0] = 1000;//AMP_SYS_CLK/AMP_PWM_FREQ/2;

    // Start up the unit, including initializing the interrupt
    PDB0->SC = (PDB_SC_PDBEN_MASK | PDB_SC_PRESCALER (0X0) |
    			PDB_SC_TRGSEL(0X8) | PDB_SC_MULT(0) | PDB_SC_LDOK_MASK);

    // CFG1: Total ADC clock = 120 MHz/8 = 15 MHz
    // ADICLK = 0b01 - Bus clock divided by 2
    // MODE = 0b11 - 16-bit conversion
    // ADLSMP = 1 - Long Sample Time
    // ADIV = 0b10 - Divide Ratio is 4
    ADC0->CFG1 = ADC_CFG1_ADICLK(0x0) | ADC_CFG1_MODE(0x3) |
    		ADC_CFG1_ADLSMP(0x1) | ADC_CFG1_ADIV(0x0);

    // CFG2:
    // ADLSTS = 0b10 - 6 extra ADCK cycles; 10 ADCK cycles total sample time
    // ADHSC = 1 - High Speed Conversion Sequence
    // ADACKEN = 1 - Asynchronous clock and clock output enabled
    // MUXSEL = 0 - ADxxa channels are selected
    ADC0->CFG2 = ADC_CFG2_MUXSEL(0x0) | ADC_CFG2_ADACKEN(0x1) |
    							ADC_CFG2_ADHSC(0x1) | ADC_CFG2_ADLSTS(0x2);

    // SC2:
    // ADTRG_HW = 1 - Hardware trigger selected
    // ACFE = 0 - Compare function disabled
    // ACFGT = 1 - Configures >= threshold
    // ACREN = 0 - Range function disabled
    // DMAEN = 0 - DMA is disabled
    // REFSEL = 0b00 - Default voltage reference
    ADC0->SC2 = ADC_SC2_ADTRG(0x1) | ADC_SC2_ACFE(0x0) | ADC_SC2_ACFGT(0x1) |
    			ADC_SC2_ACREN(0x0) | ADC_SC2_DMAEN(0x0) | ADC_SC2_REFSEL(0x0);

    // SC3:
    // CAL = 0 - Ensure the calibration flag is not set
    // ADCO = 0 - One set of conversions after initiating a conversion
    // AVGE = 1 - Hardware average functionality enabled
    // AVGS = 0b00 - 4 Samples averaged
    ADC0->SC3 = ADC_SC3_CALF(0x0) | ADC_SC3_ADCO(0x0) | ADC_SC3_AVGE(0x1) |
    											         ADC_SC3_AVGS(0x0);

    // SC1A:
    // AIEN = 0 - Conversion Complete interrupt is disabled
    // DIFF = 0 - Single-ended conversions and input channels are selected
    // ADCH = 0b01000 - Selects channel AD08 Initially (Will vary)
    ADC0->SC1[0] = ADC_SC1_AIEN(0x0) | ADC_SC1_DIFF(0x0) | ADC_SC1_ADCH(CHAN_PHASE_S);

}   // End of void Amp_ADC1_init_sampling (void)


void Amp_Foreground_Update(void)
//*****************************************************************************
//  Description:
//
//*****************************************************************************
{
	static uint32_t ul_count = 0;
	static uint32_t ul_foreground_counter = 0;

 //   FTM0->PWMLOAD = FTM_PWMLOAD_LDOK_MASK;
    switch (gl_amp_cntrl[AMP_FRGD_STATE])
    {
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        case AMP_STATE_CALIBRATE:
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        {
            FTM0->PWMLOAD = FTM_PWMLOAD_LDOK_MASK;
            // Initially disable amplifier
            Amp_Disable_Amplifier();

            if (gl_amp_cntrl[AMP_STATUS] & AMP_ADC_CALIBRATED)
            {
                // The motor current ADC has successfully calibrated,
                // initialize the ADC hardware to start sampling.
                Amp_ADC0_Init_Sampling();

                // Change the amplifier state to initialize the motor current
                // readings from the ADC.
                gl_amp_cntrl[AMP_FRGD_STATE] = AMP_STATE_INITIALIZE;

            }   // End of if (gl_amp_cntrl_values[AMP_STATUS] & AMP_ADC ...
            else if (gl_amp_cntrl[AMP_STATUS] & AMP_CALIBRATION_ERROR)
            {
                // Calibration failed, reinitialize ADC and attempt to recal
                Amp_ADC0_Init_Calibrate();

            }   // End of else if (gl_amp_cntrl_values[AMP_STATUS] & AMP ...
            else
            {
                // Check if the calibration has finished and if it was
                // successful.
                Amp_ADC0_Calibrate();

            }   // End of else

            break;

        }   // End of case AMP_STATE_CALIBRATE:
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        case AMP_STATE_INITIALIZE:
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        {
            // Initially disable amplifier
            Amp_Disable_Amplifier();

            // Increment the current read counter. Read the current several
            // times before
            ul_foreground_counter++;

            Amp_Update_Current();

            // Take several readings to allow the ADC voltages time to
            // initialize before normal runtime.
            if (ul_foreground_counter >= 500)
            {
                // Change the amplifier state to normal operation.
            	gl_amp_cntrl[AMP_FRGD_STATE] = AMP_STATE_RUNTIME;

                gl_amp_cntrl[AMP_STATUS] |= AMP_MODULE_INITD;

            }   // End of if (ul_current_count >= 500)

            break;

        }   // End of case AMP_STATE_INITIALIZE:
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        case AMP_STATE_RUNTIME:
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        {
        	ul_count++;

        	if (ul_count >= 20000)
        	{
            	gl_amp_cntrl[AMP_COMMUTATION]++;

            	if (gl_amp_cntrl[AMP_COMMUTATION] > 6)
            	{
            		gl_amp_cntrl[AMP_COMMUTATION] = 1;

            	}
            	ul_count = 0;
        	}


        	Amp_Update_Current();

        	gl_amp_cntrl[AMP_STATUS] |= AMP_MODULE_ENABLED;

            //FTM0->INVCTRL = AMP_INVERT_NONE;
            //FTM0->OUTMASK = AMP_DISABLE_ALL_PH;
        	Amp_Update_PWM_Dutycycle(0);

            break;

        }   // End of case AMP_STATE_RUNTIME:
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        default:
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        {
        	gl_amp_cntrl[AMP_FRGD_STATE] = AMP_STATE_RUNTIME;

        }   // End of default:

    }   // End of switch(ul_amp_state)

}


void Amp_ADC0_Calibrate (void)
//*****************************************************************************
//  Description: This function checks the calibration bits and runs
//               calculations for the ADC1 calibration. A flag is set if the
//               calibration is successful or if it has failed.
//
//*****************************************************************************
{
    uint16_t un_cal_temp = 0;

    if(ADC0->SC1[0] & ADC_SC1_COCO_MASK)
    {
        // The calibration has finished, check if it was successful.
        if (ADC0->SC3 & ADC_SC3_CALF_MASK)
        {
            // Set flag indicating a calibration error has occurred.
            gl_amp_cntrl[AMP_STATUS] |= (AMP_CALIBRATION_ERROR |
                                                AMP_CALIBRATION_ERROR_STICKY |
                                                     AMP_MODULE_FAULT_STICKY);

        }   // End of if (ADC1_SC3 & CALF_FAIL)
        else
        {
            un_cal_temp = 0;

            // The calibration was successful, calculate positive calibration
            un_cal_temp =  ADC0->CLP0;
            un_cal_temp += ADC0->CLP1;
            un_cal_temp += ADC0->CLP2;
            un_cal_temp += ADC0->CLP3;
            un_cal_temp += ADC0->CLP4;
            un_cal_temp += ADC0->CLPS;

            un_cal_temp = un_cal_temp >> 1;

            // Set MSb
            un_cal_temp |= 0x8000;

            // And save
            ADC0->PG = ADC_PG_PG(un_cal_temp);

            // Repeat for the negative calibration
            un_cal_temp = 0x00;

            un_cal_temp = ADC0->CLM0;
            un_cal_temp += ADC0->CLM1;
            un_cal_temp += ADC0->CLM2;
            un_cal_temp += ADC0->CLM3;
            un_cal_temp += ADC0->CLM4;
            un_cal_temp += ADC0->CLMS;

            un_cal_temp = un_cal_temp >> 1;

            un_cal_temp |= 0x8000;

            ADC0->MG = ADC_MG_MG(un_cal_temp);

            // Clear the calibration bit
            ADC0->SC3 &= ~ADC_SC3_CAL_MASK;

            // Clear calibration error flag.
            gl_amp_cntrl[AMP_STATUS] &= ~AMP_CALIBRATION_ERROR;

            // Set flag indicating ADC calibration is finished.
            gl_amp_cntrl[AMP_STATUS] |= AMP_ADC_CALIBRATED;

        }   // End of else

    }   // End of if(ADC1_SC1A & ADC_SC1_COCO_MASK)

}   // End of void Amp_ADC1_calibrate (void)


void Amp_Update_Current(void)
//*****************************************************************************
//  Description: This function updates the motor current variable and sets up
//               the next current reading by modifying the ADC channel based on
//               the current the latest commutation state. Status bits are
//               then updated based on motor current values.
//
//*****************************************************************************
{
    // Variable to used to read the raw ADC output. Set initially to the null
    // value.
    int32_t l_amp_feedback_raw = 32768;
    // Variable to dump unwanted ADC readings.
    static int32_t ul_prev_commutation = 0;

    if (ADC0->SC1[0] & ADC_SC1_COCO_MASK)
    {
        // Load conversion from buffer based on previous commutation state.
        switch (ul_prev_commutation)
        {
            //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            case 4:
            case 5:
            //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            {
            	gl_current = ADC0->R[0];
                gl_amp_cntrl[AMP_FEEDBACK_PHS] = l_amp_feedback_raw;

                break;

            }   // End of case 2: case 3:
            //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            case 1:
            case 3:
            //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            {
            	gl_current = ADC0->R[0];
                gl_amp_cntrl[AMP_FEEDBACK_PHR] = gl_current;

                break;

            }   // End of case 4: case 6:
            //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            case 2:
            case 6:
            //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            {
            	gl_current = ADC0->R[0];
                gl_amp_cntrl[AMP_FEEDBACK_PHT] = gl_current;

                break;

            }   // End of case 1: case 3:
            //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            default:
            //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            {
            	gl_current = ADC0->R[0];
                gl_amp_cntrl[AMP_FEEDBACK_PHS] = gl_current;

                break;

            }   // End of default:

        }   // End of switch (ul_prev_commutation)

        // The ADC buffer has been read, clear conversion flag
        ADC0->SC1[0] &= ~ADC_SC1_COCO_MASK;

    }   // End of if (ADC0_SC1A & ADC_SC1_COCO_MASK)


    // Change the ADC channel based on current commutation state. If invalid
    // commutation state detected, do not change channel.
    switch (gl_amp_cntrl[AMP_COMMUTATION])
    {
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        case 4:
        case 5:
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        {
            ADC0->SC1[0] = ADC_SC1_ADCH(CHAN_PHASE_S);
            break;

        }   // End of case 2: case 3:
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        case 1:
        case 3:
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        {
        	ADC0->SC1[0] = ADC_SC1_ADCH(CHAN_PHASE_R);
            break;

        }   // End of case 4: case 5:
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        case 2:
        case 6:
        //  +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        {
        	ADC0->SC1[0] = ADC_SC1_ADCH(CHAN_PHASE_T);
            break;

        }   // End of case 1: case 5:
        default:
        {

            break;

        }   // End of default:

    }   // End of switch (gl_amp_cntrl_values[AMP_COMMUTATION])

    // Load current commutation state into the previous for use in next run
    ul_prev_commutation = gl_amp_cntrl[AMP_COMMUTATION];

}   // End of void Amp_Update_Current(void)


void Amp_Update_PWM_Dutycycle(int32_t l_dutycycle_command)
//*****************************************************************************
//  Description: This function provides processing and logic to drive
//  the BLDC motor in a 6 step, complementary current mode fashion. Accepts
//  a duty cycle command and generates PWM signals based on this input as well
//  as the motor commutation state.
//
//  NOTE:
//     - The PWM output channels are in Combine mode. In this mode, the
//       n and n+1 channels are combined into one such that the n channel
//       determines when the main PWM output channel switches high and the
//       n+1 channel determines when the main PWM output switches low. The
//       secondary output will always be the compliment of the main PWM output.
//
//
//*****************************************************************************
{
    int16_t n_bot_dutycycle;
    int16_t n_top_dutycycle;

    // Check enable status of amplifier
    if (!(gl_amp_cntrl[AMP_STATUS] & AMP_MODULE_ENABLED))
    {
        // The amplifier is disabled, disable all PWM outputs.
        FTM0->INVCTRL = AMP_INVERT_NONE;
        FTM0->OUTMASK = AMP_DISABLE_ALL_PH;

    }   // End of if (!(gl_amp_cntrl_values[AMP_STATUS] & AMP_DRIVE_ENABLED))
    else
    {
        // The amplifier is enabled, process duty cycle request.

        // Limit input value to maximum signed 16-bit in the form of S1.15
        // Value multiplied by will be shifted by 15 bits, so essentially limit
        // duty cycle to maximum of 99.99% command
        if (l_dutycycle_command > 32767)
        {
            l_dutycycle_command = 32767;

        }
        else if (l_dutycycle_command < -32767)
        {
            l_dutycycle_command = -32767;

        }

        // Determine PWM channel value based on duty cycle command
        n_bot_dutycycle = -gl_amp_null_dutycycle +
                        ((gl_amp_null_dutycycle*l_dutycycle_command) >> 15);
        n_top_dutycycle = gl_amp_null_dutycycle -
                        ((gl_amp_null_dutycycle*l_dutycycle_command) >> 15);

        // Set all channel value registers with updated channel value
        FTM0->CONTROLS[0].CnV = n_bot_dutycycle;
        FTM0->CONTROLS[1].CnV = n_top_dutycycle;
        FTM0->CONTROLS[2].CnV = n_bot_dutycycle;
        FTM0->CONTROLS[3].CnV = n_top_dutycycle;
        FTM0->CONTROLS[4].CnV = n_bot_dutycycle;
        FTM0->CONTROLS[5].CnV = n_top_dutycycle;

        // Only use the commutation value to load FTM registers if it is valid.
        if ((gl_amp_cntrl[AMP_COMMUTATION] >= AMP_MIN_VALID_HALL) &&
            (gl_amp_cntrl[AMP_COMMUTATION] <= AMP_MAX_VALID_HALL))
        {
            // Set the inverted phase to the value in the invert channel array.
            FTM0->INVCTRL = gun_amp_inv_table[gl_amp_cntrl[AMP_COMMUTATION]];

            // Mask the channel indicated in the output mask array.
            FTM0->OUTMASK = gun_amp_mask_table[gl_amp_cntrl[AMP_COMMUTATION]];

        }   // End of if ((gl_amp_cntrl_values[AMP_COMMUTATION] >= ...

    }   // End of else

    // Synchronize loading of all register values from buffers
    FTM0->SYNC |= FTM_SYNC_SWSYNC_MASK;

}   // End of void Amp_Update_PWM_Dutycycle(int32_t l_dutycycle_command)


void Amp_Disable_Amplifier(void)
//*****************************************************************************
//  Description:
//
//*****************************************************************************
{
    g_amp_flags.disable_amp_request = TRUE;

}   // End of void Amp_Disable_Amplifier(void)



