/*
 * Amplifier.h
 *
 *  Created on: Dec 26, 2019
 *      Author: dgail
 */

#ifndef AMPLIFIER_H_
#define AMPLIFIER_H_

// Amplifier timing parameters
#define AMP_SYS_CLK 96000000
#define AMP_PWM_FREQ 60000

typedef enum
{
	AMP_F_CUR_PHS,

	AMP_F_CUR_PHT,

	AMP_F_CUR_PHR,

} amp_f_elements_t;

#define AMP_F_CNTRL_SIZE  3

typedef enum
{
	AMP_STATUS,

	AMP_FRGD_STATE,

	AMP_FEEDBACK_PHS,

	AMP_FEEDBACK_PHT,

	AMP_FEEDBACK_PHR,

	AMP_COMMUTATION,

} amp_l_cntrl_t;

#define AMP_L_CNTRL_SIZE  6

// Define amplifier foreground states
typedef enum
{
    AMP_STATE_CALIBRATE,
    AMP_STATE_INITIALIZE,
    AMP_STATE_RUNTIME

} amp_frgnd_state_t;

// Define ADC channels
#define CHAN_PHASE_S    8
#define CHAN_PHASE_T    9
#define CHAN_PHASE_R    12

// Define PWM no inversion for any phases
#define AMP_INVERT_NONE     0x00

// Define PWM output phase masking
#define AMP_DISABLE_ALL_PH  0x3F

// Define Maximum and Minimum Valid Hall States. Any hall state between or
// or equal to these values is considered valid.
#define AMP_MAX_VALID_HALL 6
#define AMP_MIN_VALID_HALL 1

// Define Amplifier internal flags. Each flag is represented by a native
// integer type. The flags are not bit packed into a single variable to avoid
// data corruption during read/modify/write operations. It may be possible to
// reduce the size of the flag variable to 8 bits if memory space becomes an
// issue.
typedef struct
{
    uint32_t od_update_request;
    uint32_t current_samp_initd;
    uint32_t enable_amp_request;
    uint32_t disable_amp_request;
    uint32_t clear_fault_request;

} amp_flags_t;

// Define Amplifier Fault bit masks
#define AMP_ALL_FAULTS_MASK                 0x00007FFE
#define AMP_TEMP_FAULTS_MASK                0x000000FE
#define AMP_STICKY_FAULTS_MASK              0x00007F00
#define AMP_TEMP_STICKY_SHIFT               7

// Define Amplifier Status Bits
#define AMP_MODULE_FAULT_STICKY             0x00000001
#define AMP_SW_OVER_CURRENT                 0x00000002
#define AMP_HW_OVER_CURRENT                 0x00000004
#define AMP_INVALID_COMMUTATION             0x00000008

#define AMP_CALIBRATION_ERROR               0x00000010
#define AMP_INVALID_DATA_ACCESS             0x00000020
#define AMP_CALCULATON_ERROR                0x00000040
#define AMP_ADC_CONVERSION_ERROR            0x00000080

#define AMP_SW_OVER_CURRENT_STICKY          0x00000100
#define AMP_HW_OVER_CURRENT_STICKY          0x00000200
#define AMP_INVALID_COMMUTATION_STICKY      0x00000400
#define AMP_CALIBRATION_ERROR_STICKY        0x00000800

#define AMP_INVALID_DATA_ACCESS_STICKY      0x00001000
#define AMP_CALCULATON_ERROR_STICKY         0x00002000
#define AMP_ADC_CONVERSION_ERROR_STICKY     0x00004000

#define AMP_MODULE_INITD                    0x01000000
#define AMP_ADC_CALIBRATED                  0x02000000
#define AMP_MODULE_ENABLED                  0x04000000





// Global function definitions
void Amp_Init(void);
void Amp_Background_Update(void);
void Amp_Foreground_Update(void);
void Amp_Disable_Amplifier(void);

#endif /* AMPLIFIER_H_ */
