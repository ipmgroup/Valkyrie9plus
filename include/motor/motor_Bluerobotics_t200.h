#ifndef INCLUDE_MOTOR_MOTOR_BLUEROBOTISC_T200_H_
#define INCLUDE_MOTOR_MOTOR_BLUEROBOTISC_T200_H_

#define USER_MOTOR_NAME                 "T200"
#define USER_MOTOR                      MOTOR_Bluerobotics_t200
#define nUSER_MOTOR_TYPE                MOTOR_Type_Pm
#define nUSER_MOTOR_NUM_POLE_PAIRS      (7)
#define nUSER_MOTOR_Rr                  (NULL)
#define nUSER_MOTOR_Rs                  (0.1091551)
#define nUSER_MOTOR_Ls_d                (2.341015e-05)
#define nUSER_MOTOR_Ls_q                (2.341015e-05)
#define nUSER_MOTOR_RATED_FLUX          (0.0116241)
#define nUSER_MOTOR_MAGNETIZING_CURRENT (NULL)
#define nUSER_MOTOR_RES_EST_CURRENT     (2.0)
#define nUSER_MOTOR_IND_EST_CURRENT     (-1.0)
#define nUSER_MOTOR_MAX_CURRENT         (20.0)
#define nUSER_MOTOR_FLUX_EST_FREQ_Hz    (30.0)

#define nUSER_MOTOR_KP _IQ(0.0)
#define nUSER_MOTOR_KI _IQ(0.0)

#define nUSER_MAX_SPEED (3.5)

#endif /* INCLUDE_MOTOR_MOTOR_BLUEROBOTISC_T200_H_ */