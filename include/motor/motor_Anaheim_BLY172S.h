#ifndef INCLUDE_MOTOR_MOTOR_ANAHEIM_BLY172S_H_
#define INCLUDE_MOTOR_MOTOR_ANAHEIM_BLY172S_H_

#define USER_MOTOR_NAME "BLY172S"
#define USER_MOTOR_TYPE MOTOR_Type_Pm
#define USER_MOTOR_NUM_POLE_PAIRS (4)
#define USER_MOTOR_Rr (NULL)
#define USER_MOTOR_Rs (0.4110007)
#define USER_MOTOR_Ls_d (0.0007092811)
#define USER_MOTOR_Ls_q (0.0007092811)
#define USER_MOTOR_RATED_FLUX (0.03279636)
#define USER_MOTOR_MAGNETIZING_CURRENT (NULL)
#define USER_MOTOR_RES_EST_CURRENT (1.0)
#define USER_MOTOR_IND_EST_CURRENT (-1.0)
#define USER_MOTOR_MAX_CURRENT (5.0)
#define USER_MOTOR_FLUX_EST_FREQ_Hz (20.0)

#define USER_MOTOR_FREQ_LOW (10.0)    // Hz - suggested to set to 10% of rated motor frequency
#define USER_MOTOR_FREQ_HIGH (100.0)  // Hz - suggested to set to 100% of rated motor frequency
#define USER_MOTOR_FREQ_MAX (120.0)   // Hz - suggested to set to 120% of rated motor frequency
#define USER_MOTOR_VOLT_MIN (3.0)     // Volt - suggested to set to 15% of rated motor voltage
#define USER_MOTOR_VOLT_MAX (18.0)    // Volt - suggested to set to 100% of rated motor voltage

// IPD and AFSEL settings below are not necessarily valid for this motor
// Added so that proj_lab21 compiles without errors with default user.h settings
#define IPD_HFI_EXC_FREQ_HZ (750.0)       // excitation frequency, Hz
#define IPD_HFI_LP_SPD_FILT_HZ (35.0)     // lowpass filter cutoff frequency, Hz
#define IPD_HFI_HP_IQ_FILT_HZ (100.0)     // highpass filter cutoff frequency, Hz
#define IPD_HFI_KSPD (60.0)               // the speed gain value
#define IPD_HFI_EXC_MAG_COARSE_PU (0.25)  // coarse IPD excitation magnitude, pu
#define IPD_HFI_EXC_MAG_FINE_PU (0.2)     // fine IPD excitation magnitude, pu
#define IPD_HFI_EXC_TIME_COARSE_S (0.5)   // coarse wait time, sec max 0.64
#define IPD_HFI_EXC_TIME_FINE_S (0.5)     // fine wait time, sec max 0.4
#define AFSEL_FREQ_HIGH_PU (_IQ(20.0 / USER_IQ_FULL_SCALE_FREQ_Hz))
#define AFSEL_FREQ_LOW_PU (_IQ(10.0 / USER_IQ_FULL_SCALE_FREQ_Hz))
#define AFSEL_IQ_SLOPE_EST (_IQ((float)(1.0 / 0.1 / USER_ISR_FREQ_Hz)))
#define AFSEL_IQ_SLOPE_HFI (_IQ((float)(1.0 / 10.0 / USER_ISR_FREQ_Hz)))
#define AFSEL_IQ_SLOPE_THROTTLE_DWN (_IQ((float)(1.0 / 0.05 / USER_ISR_FREQ_Hz)))
#define AFSEL_MAX_IQ_REF_EST (_IQ(0.4))
#define AFSEL_MAX_IQ_REF_HFI (_IQ(0.4))

#define USER_MOTOR_KP _IQ(0.0)
#define USER_MOTOR_KI _IQ(0.0)

#define USER_MAX_SPEED (3.5)

#endif /* INCLUDE_MOTOR_MOTOR_ANAHEIM_BLY172S_H_ */
