#ifndef INCLUDE_MOTOR_MOTOR_MARATHON_5K33GN2A_H_
#define INCLUDE_MOTOR_MOTOR_MARATHON_5K33GN2A_H_

#define USER_MOTOR_NAME "5K33GN2A"
#define USER_MOTOR_TYPE MOTOR_Type_Induction           // Motor_Type_Pm (All Synchronous: BLDC, PMSM, SMPM, IPM) or Motor_Type_Induction (Asynchronous ACI)
#define USER_MOTOR_NUM_POLE_PAIRS (2)                  // PAIRS, not total poles. Used to calculate user RPM from rotor Hz only
#define USER_MOTOR_Rr (5.508003)                       // Identified phase to neutral in a Y equivalent circuit (Ohms, float)
#define USER_MOTOR_Rs (10.71121)                       // Identified phase to neutral in a Y equivalent circuit (Ohms, float)
#define USER_MOTOR_Ls_d (0.05296588)                   // For Induction, Identified average stator inductance  (Henry, float)
#define USER_MOTOR_Ls_q (0.05296588)                   // For Induction, Identified average stator inductance  (Henry, float)
#define USER_MOTOR_RATED_FLUX (0.8165 * 220.0 / 60.0)  // sqrt(2/3)* Rated V (line-line) / Rated Freq (Hz)
#define USER_MOTOR_MAGNETIZING_CURRENT (1.378)         // Identified magnetizing current for induction motors, else NULL
#define USER_MOTOR_RES_EST_CURRENT (0.5)               // During Motor ID, maximum current (Amperes, float) used for Rs estimation, 10-20% rated current
#define USER_MOTOR_IND_EST_CURRENT (NULL)              // not used for induction
#define USER_MOTOR_MAX_CURRENT (2.0)                   // CRITICAL: Used during ID and run-time, sets a limit on the maximum current command output of the provided Speed PI Controller to the Iq controller
#define USER_MOTOR_FLUX_EST_FREQ_Hz (5.0)              // During Motor ID, maximum commanded speed (Hz, float). Should always use 5 Hz for Induction.

#define USER_MOTOR_KP _IQ(0.0)
#define USER_MOTOR_KI _IQ(0.0)

#define USER_MAX_SPEED (3.5)

#endif /* INCLUDE_MOTOR_MOTOR_MARATHON_5K33GN2A_H_ */
