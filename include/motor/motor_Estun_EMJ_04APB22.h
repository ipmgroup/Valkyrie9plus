#ifndef INCLUDE_MOTOR_MOTOR_ESTUN_EMJ_04APB22_H_
#define INCLUDE_MOTOR_MOTOR_ESTUN_EMJ_04APB22_H_

#define USER_MOTOR_NAME "04APB22"
#define USER_MOTOR_TYPE MOTOR_Type_Pm          // Motor_Type_Pm (All Synchronous: BLDC, PMSM, SMPM, IPM) or Motor_Type_Induction (Asynchronous ACI)
#define USER_MOTOR_NUM_POLE_PAIRS (4)          // PAIRS, not total poles. Used to calculate user RPM from rotor Hz only
#define USER_MOTOR_Rr (NULL)                   // Induction motors only, else NULL
#define USER_MOTOR_Rs (2.303403)               // Identified phase to neutral resistance in a Y equivalent circuit (Ohms, float)
#define USER_MOTOR_Ls_d (0.008464367)          // For PM, Identified average stator inductance  (Henry, float)
#define USER_MOTOR_Ls_q (0.008464367)          // For PM, Identified average stator inductance  (Henry, float)
#define USER_MOTOR_RATED_FLUX (0.38)           // Identified TOTAL flux linkage between the rotor and the stator (V/Hz)
#define USER_MOTOR_MAGNETIZING_CURRENT (NULL)  // Induction motors only, else NULL
#define USER_MOTOR_RES_EST_CURRENT (1.0)       // During Motor ID, maximum current (Amperes, float) used for Rs estimation, 10-20% rated current
#define USER_MOTOR_IND_EST_CURRENT (-1.0)      // During Motor ID, maximum current (negative Amperes, float) used for Ls estimation, use just enough to enable rotation
#define USER_MOTOR_MAX_CURRENT (3.82)          // CRITICAL: Used during ID and run-time, sets a limit on the maximum current command output of the provided Speed PI Controller to the Iq controller
#define USER_MOTOR_FLUX_EST_FREQ_Hz (20.0)     // During Motor ID, maximum commanded speed (Hz, float), ~10% rated

#define USER_MOTOR_KP _IQ(0.0)
#define USER_MOTOR_KI _IQ(0.0)

#define USER_MAX_SPEED (3.5)

#endif /* INCLUDE_MOTOR_MOTOR_ESTUN_EMJ_04APB22_H_ */
