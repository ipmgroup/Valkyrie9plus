#ifndef INCLUDE_MOTOR_MOTOR_B2212_920_H_
#define INCLUDE_MOTOR_MOTOR_B2212_920_H_

#define USER_MOTOR_NAME                "B2212_920"
#define USER_MOTOR                     MOTOR_B2212_920
#define nUSER_MOTOR_TYPE                MOTOR_Type_Pm
#define nUSER_MOTOR_NUM_POLE_PAIRS      (7)
#define nUSER_MOTOR_Rr                  (NULL)
#define nUSER_MOTOR_Rs                  (0.02165625)
#define nUSER_MOTOR_Ls_d                (1.575498e-05)
#define nUSER_MOTOR_Ls_q                (1.575498e-05)
#define nUSER_MOTOR_RATED_FLUX          (0.005930144)
#define nUSER_MOTOR_MAGNETIZING_CURRENT (NULL)
#define nUSER_MOTOR_RES_EST_CURRENT     (1.0)
#define nUSER_MOTOR_IND_EST_CURRENT     (-1.0)
#define nUSER_MOTOR_MAX_CURRENT         (15.0)
#define nUSER_MOTOR_FLUX_EST_FREQ_Hz    (50.0)

#define nUSER_MOTOR_KP _IQ(3.5)
#define nUSER_MOTOR_KI _IQ(0.056)

#define nUSER_MAX_SPEED (15.0)

#endif
