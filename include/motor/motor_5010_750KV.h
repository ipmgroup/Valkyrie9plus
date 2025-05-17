#ifndef INCLUDE_MOTOR_MOTOR_5010_750KV_H_
#define INCLUDE_MOTOR_MOTOR_5010_750KV_H_

#define USER_MOTOR_NAME                 "5010_750"
#define USER_MOTOR                      MOTOR_5010_750KV
#define nUSER_MOTOR_TYPE                MOTOR_Type_Pm
#define nUSER_MOTOR_NUM_POLE_PAIRS      (7)
#define nUSER_MOTOR_Rr                  (NULL)
#define nUSER_MOTOR_Rs                  (0.0453813002)
#define nUSER_MOTOR_Ls_d                (1.39672575e-05)
#define nUSER_MOTOR_Ls_q                (1.39672575e-05)
#define nUSER_MOTOR_RATED_FLUX          (0.00754115451)
#define nUSER_MOTOR_MAGNETIZING_CURRENT (NULL)
#define nUSER_MOTOR_RES_EST_CURRENT     (2.0)
#define nUSER_MOTOR_IND_EST_CURRENT     (-1.0)
#define nUSER_MOTOR_MAX_CURRENT         (20.0)
#define nUSER_MOTOR_FLUX_EST_FREQ_Hz    (60.0)

#define nUSER_MOTOR_KP _IQ(3.5)
#define nUSER_MOTOR_KI _IQ(0.056)

#define nUSER_MAX_SPEED (12.0)

#endif /* INCLUDE_MOTOR_MOTOR_5010_750KV_H_ */
