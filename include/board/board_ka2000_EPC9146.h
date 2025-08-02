#ifndef INCLUDE_BOARD_ka2000_EPC9146_H_
#define INCLUDE_BOARD_ka2000_EPC9146_H_

#include "sw/drivers/gpio/src/32b/f28x/f2806x/gpio.h"

// **************************************************************************
// the defines

#ifndef V_REG
#define V_REG 3.3f
#endif
#ifndef VIN_R1
#define VIN_R1 100000.0f
#endif
#ifndef VIN_R2
#define VIN_R2 4220.0f
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN 50
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES (0.0015f)
#endif

#define HW_REV_MAJOR 0
#define HW_REV_MINOR 1

#define DRV_NAME      "EPC9146"
#define APP_NODE_NAME "KA_EPC9146"

#define LED_BLINK_FREQ_Hz 4

// #define HAL_PWM_DBFED_CNT 1
// #define HAL_PWM_DBRED_CNT 1

#define HAL_PWM_DBFED_CNT (uint16_t)(0.02 * (float_t)nUSER_SYSTEM_FREQ_MHz)
#define HAL_PWM_DBRED_CNT (uint16_t)(0.02 * (float_t)nUSER_SYSTEM_FREQ_MHz)

#define nADC_DELAY_usec 10000L
// #define nADC_DELAY_usec 2000L

#define INVERTED_SHUNT_POLARITY

//! \brief J1 specific defines
// **************************************************************************
#define PWM_INTERRUPT PWM_Number_1

#define PWM_HANDLE_0_BASE_ADDR PWM_ePWM1_BASE_ADDR
#define PWM_HANDLE_1_BASE_ADDR PWM_ePWM2_BASE_ADDR
#define PWM_HANDLE_2_BASE_ADDR PWM_ePWM3_BASE_ADDR

#define EN_GATE_GPIO_NUMBER GPIO_Number_15

#define LOW_SPD_PRESCALER CLK_LowSpdPreScaler_SysClkOut_by_1

// #define nFAULT_INT GPIO_Number_22
#define ADC8_SUPPORT
// #define ADC9_SUPPORT
// #define TZ_DISABLE

#define ADC_SOC_TRIGGER_SOURCE   ADC_SocTrigSrc_EPWM1_ADCSOCA
#define ADC_SOC_0_CHANNEL_NUMBER ADC_SocChanNumber_B4    // ISEN_A
#define ADC_SOC_1_CHANNEL_NUMBER ADC_SocChanNumber_B4    // ISEN_A
#define ADC_SOC_2_CHANNEL_NUMBER ADC_SocChanNumber_B2    // ISEN_B
#define ADC_SOC_3_CHANNEL_NUMBER ADC_SocChanNumber_B1    // ISEN_C
#define ADC_SOC_4_CHANNEL_NUMBER ADC_SocChanNumber_A1    // VSEN_A
#define ADC_SOC_5_CHANNEL_NUMBER ADC_SocChanNumber_A2    // VSEN_B
#define ADC_SOC_6_CHANNEL_NUMBER ADC_SocChanNumber_A4    // VSEN_C
#define ADC_SOC_7_CHANNEL_NUMBER ADC_SocChanNumber_A6    // VDCBUS

// Tsens
#ifdef ADC8_SUPPORT
#define ADC_SOC_8_CHANNEL_NUMBER ADC_SocChanNumber_A0
#endif

// Speed
#ifdef ADC9_SUPPORT
#define ADC_SOC_9_CHANNEL_NUMBER ADC_SocChanNumber_B6
#endif

// AD590
#define R_tem    7870.0
#define Kelvin_A 0.000001
#define Kelvin_V (float)(R_tem * Kelvin_A)

#define ADC8_FUNCTIONALITY                                                                                                                         \
    do {                                                                                                                                           \
        gMotorVars.TempSenDegCelsius = ((V_REG / 4095.0) * (float_t)ADC_readResult(halHandle->adcHandle, ADC_ResultNumber_8) / Kelvin_V) - 273.15; \
    } while (0)

#define DRV_TEMPERATURE
#define OVERHEAT 120.0

#define ADC9_FUNCTIONALITY                                                                                          \
    do {                                                                                                            \
        gMotorVars.Throttle = (V_REG / 4095.0) * (float_t)ADC_readResult(halHandle->adcHandle, ADC_ResultNumber_9); \
    } while (0)

// #define HALL_GPIO_A GPIO_Number_19
// #define HALL_GPIO_B GPIO_Number_44
// #define HALL_GPIO_C GPIO_Number_50
#define gLED2 GPIO_Number_25
#define gLED3 GPIO_Number_39

#define TX_DELAY 2

//! \brief GPIO configuration
// **************************************************************************

// PWM1
#undef GPIO_0_CONFIG
#define GPIO_0_CONFIG(h)                                    \
    do {                                                    \
        GPIO_setMode(h, GPIO_Number_0, GPIO_0_Mode_EPWM1A); \
    } while (0)

// PWM2
#undef GPIO_1_CONFIG
#define GPIO_1_CONFIG(h)                                    \
    do {                                                    \
        GPIO_setMode(h, GPIO_Number_1, GPIO_1_Mode_EPWM1B); \
    } while (0)

// PWM3
#undef GPIO_2_CONFIG
#define GPIO_2_CONFIG(h)                                    \
    do {                                                    \
        GPIO_setMode(h, GPIO_Number_2, GPIO_2_Mode_EPWM2A); \
    } while (0)

// PWM4
#undef GPIO_3_CONFIG
#define GPIO_3_CONFIG(h)                                    \
    do {                                                    \
        GPIO_setMode(h, GPIO_Number_3, GPIO_3_Mode_EPWM2B); \
    } while (0)

// PWM5
#undef GPIO_4_CONFIG
#define GPIO_4_CONFIG(h)                                    \
    do {                                                    \
        GPIO_setMode(h, GPIO_Number_4, GPIO_4_Mode_EPWM3A); \
    } while (0)

// PWM6
#undef GPIO_5_CONFIG
#define GPIO_5_CONFIG(h)                                    \
    do {                                                    \
        GPIO_setMode(h, GPIO_Number_5, GPIO_5_Mode_EPWM3B); \
    } while (0)

// nFAULT
#undef GPIO_12_CONFIG
#define GPIO_12_CONFIG(h)                                      \
    do {                                                       \
        GPIO_setMode(h, GPIO_Number_12, GPIO_12_Mode_TZ1_NOT); \
    } while (0)

// Enable Gate
#undef GPIO_15_CONFIG
#define GPIO_15_CONFIG(h)                                             \
    do {                                                              \
        GPIO_setQualificationPeriod(h, GPIO_Number_15, 5);            \
        GPIO_setMode(h, GPIO_Number_15, GPIO_15_Mode_GeneralPurpose); \
        GPIO_setLow(h, GPIO_Number_15);                               \
        GPIO_setDirection(h, GPIO_Number_15, GPIO_Direction_Output);  \
    } while (0)

// SPIA SIMO
#undef GPIO_16_CONFIG
#define GPIO_16_CONFIG(h)                                       \
    do {                                                        \
        GPIO_setQualificationPeriod(h, GPIO_Number_16, 5);      \
        GPIO_setMode(h, GPIO_Number_16, GPIO_16_Mode_SPISIMOA); \
    } while (0)

// SPIA SOMI
#undef GPIO_17_CONFIG
#define GPIO_17_CONFIG(h)                                       \
    do {                                                        \
        GPIO_setMode(h, GPIO_Number_17, GPIO_17_Mode_SPISOMIA); \
    } while (0)

// SPIA CLK
#undef GPIO_18_CONFIG
#define GPIO_18_CONFIG(h)                                      \
    do {                                                       \
        GPIO_setMode(h, GPIO_Number_18, GPIO_18_Mode_SPICLKA); \
    } while (0)

// SPIA CS
#undef GPIO_19_CONFIG
#define GPIO_19_CONFIG(h)                                          \
    do {                                                           \
        GPIO_setMode(h, GPIO_Number_19, GPIO_19_Mode_SPISTEA_NOT); \
    } while (0)

// CAL
#undef GPIO_20_CONFIG
#define GPIO_20_CONFIG(h)                                             \
    do {                                                              \
        GPIO_setQualificationPeriod(h, GPIO_Number_20, 5);            \
        GPIO_setMode(h, GPIO_Number_20, GPIO_20_Mode_GeneralPurpose); \
        GPIO_setLow(h, GPIO_Number_20);                               \
        GPIO_setDirection(h, GPIO_Number_20, GPIO_Direction_Output);  \
    } while (0)

// CAN RX
#undef GPIO_30_CONFIG
#define GPIO_30_CONFIG(h)                                     \
    do {                                                      \
        GPIO_setMode(h, GPIO_Number_30, GPIO_30_Mode_CANRXA); \
    } while (0)

// CAN TX
#undef GPIO_31_CONFIG
#define GPIO_31_CONFIG(h)                                     \
    do {                                                      \
        GPIO_setMode(h, GPIO_Number_31, GPIO_31_Mode_CANTXA); \
    } while (0)

// LED D9
#undef GPIO_25_CONFIG
#define GPIO_25_CONFIG(h)                                             \
    do {                                                              \
        GPIO_setMode(h, GPIO_Number_25, GPIO_25_Mode_GeneralPurpose); \
        GPIO_setLow(h, GPIO_Number_25);                               \
        GPIO_setDirection(h, GPIO_Number_25, GPIO_Direction_Output);  \
    } while (0)

// FAST define

#define nUSER_IQ_FULL_SCALE_FREQ_Hz (513.3)    // 800 Example with buffer for 8-pole 6 KRPM motor to be run to 10 KRPM with field weakening; Hz =(RPM * Poles) / 120

#define nUSER_ADC_FULL_SCALE_VOLTAGE_V ((V_REG) * ((VIN_R1) + (VIN_R2)) / (VIN_R2))
#define nUSER_ADC_FULL_SCALE_CURRENT_A (V_REG / (CURRENT_SHUNT_RES * (float)CURRENT_AMP_GAIN))
#define nUSER_IQ_FULL_SCALE_VOLTAGE_V  (48.0)                                    // 109.63 Set to Vbus
#define nUSER_IQ_FULL_SCALE_CURRENT_A  (nUSER_ADC_FULL_SCALE_CURRENT_A * 0.5)    // EPC9186 = 240.16A

#define nUSER_NUM_CURRENT_SENSORS (3)
#define nUSER_NUM_VOLTAGE_SENSORS (3)

#define I_A_offset (0.59)
#define I_B_offset (0.59)
#define I_C_offset (0.59)

#define V_A_offset (0.38)
#define V_B_offset (0.38)
#define V_C_offset (0.38)

#define USER_MAX_PWM_FREQ_kHz (35.0)

#define nUSER_PWM_FREQ_kHz                    (30.0)    // 30.0 Example, 8.0 - 30.0 KHz typical; 45-80 KHz may be required for very low inductance, high speed motors
#define nUSER_NUM_PWM_TICKS_PER_ISR_TICK      (3)
#define nUSER_MAX_VS_MAG_PU                   (2.0 / 3.0)    // (1.0)  // Perfect Sinusoidal. (2.0/3.0) Set to 0.5 if a current reconstruction technique is not used.  Look at the module svgen_current in lab10a-x for more info.
#define nUSER_NUM_ISR_TICKS_PER_CTRL_TICK     (1)            // 2 Example, controller clock rate (CTRL) runs at PWM / 2; ex 30 KHz PWM, 15 KHz control
#define nUSER_NUM_CTRL_TICKS_PER_CURRENT_TICK (1)            // 1 Typical, Forward FOC current controller (Iq/Id/IPARK/SVPWM) runs at same rate as CTRL.
#define nUSER_NUM_CTRL_TICKS_PER_EST_TICK     (1)            // 1 Typical, FAST estimator runs at same rate as CTRL;
#define nUSER_NUM_CTRL_TICKS_PER_SPEED_TICK   (15)           // 15 Typical to match PWM, ex: 15KHz PWM, controller, and current loop, 1KHz speed loop
#define nUSER_NUM_CTRL_TICKS_PER_TRAJ_TICK    (15)           // 15 Typical to match PWM, ex: 10KHz controller & current loop, 1KHz speed loop, 1 KHz Trajectory
#define nUSER_R_OVER_L_EST_FREQ_Hz            (300)          // 300 Default
// #define nUSER_VOLTAGE_FILTER_POLE_Hz          (344.62)    // BOOSTXL-DRV8305 = 344.62 Hz
#define nUSER_VOLTAGE_FILTER_POLE_Hz (1194.0)

#define nUSER_VOLTAGE_SF ((float_t)((nUSER_ADC_FULL_SCALE_VOLTAGE_V) / (nUSER_IQ_FULL_SCALE_VOLTAGE_V)))
#define nUSER_CURRENT_SF ((float_t)((nUSER_ADC_FULL_SCALE_CURRENT_A) / (nUSER_IQ_FULL_SCALE_CURRENT_A)))

#define nUSER_SYSTEM_FREQ_MHz (90.0)

#define USER_CTRL_HANDLE_ADDRESS   (0x13C40)
#define USER_CTRL_HANDLE_ADDRESS_1 (0x13D36)

#define USER_EST_HANDLE_ADDRESS   (0x13840)
#define USER_EST_HANDLE_ADDRESS_1 (0x13A3E)

#define USER_VD_SF                            (0.95)
#define nUSER_MAX_ACCEL_Hzps                  (20.0)    // 20.0 Default
#define nUSER_MAX_ACCEL_EST_Hzps              (5.0)     // 5.0 Default, don't change
#define nUSER_IDRATED_FRACTION_FOR_RATED_FLUX (1.0)     // 1.0 Default, don't change
#define nUSER_IDRATED_FRACTION_FOR_L_IDENT    (1.0)     // 1.0 Default, don't change
#define nUSER_IDRATED_DELTA                   (0.00002)
#define nUSER_SPEEDMAX_FRACTION_FOR_L_IDENT   (1.0)      // 1.0 Default, don't change
#define nUSER_FLUX_FRACTION                   (1.0)      // 1.0 Default, don't change
#define nUSER_POWERWARP_GAIN                  (1.0)      // 1.0 Default, don't change
#define nUSER_OFFSET_POLE_rps                 (20.0)     // 20.0 Default, do not change
#define nUSER_FLUX_POLE_rps                   (100.0)    // 100.0 Default, do not change
#define nUSER_DIRECTION_POLE_rps              (6.0)      // 6.0 Default, do not change
#define nUSER_SPEED_POLE_rps                  (100.0)    // 100.0 Default, do not change
#define nUSER_DCBUS_POLE_rps                  (100.0)    // 100.0 Default, do not change
#define nUSER_EST_KAPPAQ                      (1.5)      // 1.5 Default, do not change

#endif /* INCLUDE_BOARD_ka2000_EPC9146_H_ */
