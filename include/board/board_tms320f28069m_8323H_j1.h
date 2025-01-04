// Note: to use DRV8323 you need to add C9, C10 and C11 to the board.
// Note: to use DRV8323H you need to add R23 to the board.

// Development for BOOSTXL-DRV8323RH has been halted
// because the BOOSTXL-DRV8323RH board has failed.
// The cause of the failure is unknown.
//
// The MODE signal (GPIO13) is at a high level ("1").
// GPIO12 has been deliberately set to a low level ("0")
// to prevent potential damage to the board in the future.
//
// TODO:
// - Implement the HAL_enableDrv function.
// - Add handling for the nFAULT signal.

#ifndef INCLUDE_BOARD_BOARD_TMS320F28069M_8323H_J1_H_
#define INCLUDE_BOARD_BOARD_TMS320F28069M_8323H_J1_H_

#include "sw/drivers/gpio/src/32b/f28x/f2806x/gpio.h"

// **************************************************************************
// the defines

#ifndef V_REG
#define V_REG 3.3f
#endif
#ifndef VIN_R1
#define VIN_R1 82000.0f
#endif
#ifndef VIN_R2
#define VIN_R2 4990.0f
#endif
#ifndef CURRENT_AMP_GAIN
// #define CURRENT_AMP_GAIN 5
#define CURRENT_AMP_GAIN 10
// #define CURRENT_AMP_GAIN 40
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES 0.007f
#endif

#define HW_REV_MAJOR 0
#define HW_REV_MINOR 1

#define DRV_NAME      "DRV8323H"
#define APP_NODE_NAME "BOOSTXL-DRV8323RH"

#define LED_BLINK_FREQ_Hz 5

#define HAL_PWM_DBFED_CNT (uint16_t)(0.2 * (float_t)nUSER_SYSTEM_FREQ_MHz)
#define HAL_PWM_DBRED_CNT (uint16_t)(0.2 * (float_t)nUSER_SYSTEM_FREQ_MHz)

#define nADC_DELAY_usec 10000L

#define PWM_INTERRUPT PWM_Number_1

#define PWM_HANDLE_0_BASE_ADDR PWM_ePWM1_BASE_ADDR
#define PWM_HANDLE_1_BASE_ADDR PWM_ePWM2_BASE_ADDR
#define PWM_HANDLE_2_BASE_ADDR PWM_ePWM3_BASE_ADDR

#define EN_GATE_GPIO_NUMBER GPIO_Number_12

#define LOW_SPD_PRESCALER CLK_LowSpdPreScaler_SysClkOut_by_2

#define nFAULT_INT GPIO_Number_22
// #define ADC8_SUPPORT
// #define ADC9_SUPPORT
#define TZ_DISABLE

#define ADC_SOC_TRIGGER_SOURCE   ADC_SocTrigSrc_EPWM1_ADCSOCA
#define ADC_SOC_0_CHANNEL_NUMBER ADC_SocChanNumber_B0    // ISEN_A
#define ADC_SOC_1_CHANNEL_NUMBER ADC_SocChanNumber_B0    // ISEN_A
#define ADC_SOC_2_CHANNEL_NUMBER ADC_SocChanNumber_A0    // ISEN_B
#define ADC_SOC_3_CHANNEL_NUMBER ADC_SocChanNumber_B2    // ISEN_C
#define ADC_SOC_4_CHANNEL_NUMBER ADC_SocChanNumber_A7    // VSEN_A
#define ADC_SOC_5_CHANNEL_NUMBER ADC_SocChanNumber_B1    // VSEN_B
#define ADC_SOC_6_CHANNEL_NUMBER ADC_SocChanNumber_A2    // VSEN_C
#define ADC_SOC_7_CHANNEL_NUMBER ADC_SocChanNumber_A6    // VDCBUS

#ifdef ADC8_SUPPORT
#define ADC_SOC_8_CHANNEL_NUMBER ADC_SocChanNumber_B6
#endif

#ifdef ADC9_SUPPORT
#define ADC_SOC_9_CHANNEL_NUMBER ADC_SocChanNumber_A1
#endif

#define ADC8_FUNCTIONALITY                                                                                                   \
    do {                                                                                                                     \
        gMotorVars.TempSenDegCelsius = (V_REG / 4095.0) * (float_t)ADC_readResult(halHandle->adcHandle, ADC_ResultNumber_8); \
    } while (0)

#define ADC9_FUNCTIONALITY                                                                                          \
    do {                                                                                                            \
        gMotorVars.Throttle = (V_REG / 4095.0) * (float_t)ADC_readResult(halHandle->adcHandle, ADC_ResultNumber_9); \
    } while (0)

#define FAULT_RESET                              \
    do {                                         \
        gDrvSpi83xxVars.Ctrl_Reg_02.CLR_FLT = 1; \
    } while (0)

#define HALL_GPIO_A GPIO_Number_19
#define HALL_GPIO_B GPIO_Number_44
#define HALL_GPIO_C GPIO_Number_50
#define gLED2       GPIO_Number_34
#define gLED3       GPIO_Number_39

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

// PWM4A
#undef GPIO_6_CONFIG
#define GPIO_6_CONFIG(h)                                    \
    do {                                                    \
        GPIO_setMode(h, GPIO_Number_6, GPIO_6_Mode_EPWM4A); \
    } while (0)

// PWM4B
#undef GPIO_7_CONFIG
#define GPIO_7_CONFIG(h)                                    \
    do {                                                    \
        GPIO_setMode(h, GPIO_Number_7, GPIO_7_Mode_EPWM4B); \
    } while (0)

// PWM5A
#undef GPIO_8_CONFIG
#define GPIO_8_CONFIG(h)                                    \
    do {                                                    \
        GPIO_setMode(h, GPIO_Number_8, GPIO_8_Mode_EPWM5A); \
    } while (0)

// PWM5B
#undef GPIO_9_CONFIG
#define GPIO_9_CONFIG(h)                                    \
    do {                                                    \
        GPIO_setMode(h, GPIO_Number_9, GPIO_9_Mode_EPWM5B); \
    } while (0)

// PWM6A
#undef GPIO_10_CONFIG
#define GPIO_10_CONFIG(h)                                     \
    do {                                                      \
        GPIO_setMode(h, GPIO_Number_10, GPIO_10_Mode_EPWM6A); \
    } while (0)

// PWM6B
#undef GPIO_11_CONFIG
#define GPIO_11_CONFIG(h)                                     \
    do {                                                      \
        GPIO_setMode(h, GPIO_Number_11, GPIO_11_Mode_EPWM6B); \
    } while (0)

// DRV8383H Enable Gate
#undef GPIO_12_CONFIG
#define GPIO_12_CONFIG(h)                                             \
    do {                                                              \
        GPIO_setMode(h, GPIO_Number_12, GPIO_12_Mode_GeneralPurpose); \
        GPIO_setLow(h, GPIO_Number_12);                               \
        GPIO_setDirection(h, GPIO_Number_12, GPIO_Direction_Output);  \
    } while (0)

// DRV8323H MODE
// #undef GPIO_13_CONFIG    // R23 must be installed.
#define GPIO_13_CONFIG(h)                                             \
    do {                                                              \
        GPIO_setMode(h, GPIO_Number_13, GPIO_13_Mode_GeneralPurpose); \
        GPIO_setDirection(h, GPIO_Number_13, GPIO_Direction_Input);   \
    } while (0)

// UARTB RX
#undef GPIO_15_CONFIG
#define GPIO_15_CONFIG(h)                                      \
    do {                                                       \
        GPIO_setMode(h, GPIO_Number_15, GPIO_15_Mode_SCIRXDB); \
    } while (0)

// DRV8323H nFAULT
#undef GPIO_22_CONFIG
#define GPIO_22_CONFIG(h)                                             \
    do {                                                              \
        GPIO_setMode(h, GPIO_Number_22, GPIO_22_Mode_GeneralPurpose); \
        GPIO_setDirection(h, GPIO_Number_13, GPIO_Direction_Input);   \
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

// I2C Data
#undef GPIO_32_CONFIG
#define GPIO_32_CONFIG(h)                                   \
    do {                                                    \
        GPIO_setMode(h, GPIO_Number_32, GPIO_32_Mode_SDAA); \
    } while (0)

// I2C Clock
#undef GPIO_33_CONFIG
#define GPIO_33_CONFIG(h)                                   \
    do {                                                    \
        GPIO_setMode(h, GPIO_Number_33, GPIO_33_Mode_SCLA); \
    } while (0)

// LED D9
#undef GPIO_34_CONFIG
#define GPIO_34_CONFIG(h)                                             \
    do {                                                              \
        GPIO_setMode(h, GPIO_Number_34, GPIO_34_Mode_GeneralPurpose); \
        GPIO_setLow(h, GPIO_Number_34);                               \
        GPIO_setDirection(h, GPIO_Number_34, GPIO_Direction_Output);  \
    } while (0)

// LED D10
#undef GPIO_39_CONFIG
#define GPIO_39_CONFIG(h)                                             \
    do {                                                              \
        GPIO_setMode(h, GPIO_Number_39, GPIO_39_Mode_GeneralPurpose); \
        GPIO_setLow(h, GPIO_Number_39);                               \
        GPIO_setDirection(h, GPIO_Number_39, GPIO_Direction_Output);  \
    } while (0)

// DAC1
#undef GPIO_40_CONFIG
#define GPIO_40_CONFIG(h)                                     \
    do {                                                      \
        GPIO_setMode(h, GPIO_Number_40, GPIO_40_Mode_EPWM7A); \
    } while (0)

// DAC2
#undef GPIO_41_CONFIG
#define GPIO_41_CONFIG(h)                                     \
    do {                                                      \
        GPIO_setMode(h, GPIO_Number_41, GPIO_41_Mode_EPWM7B); \
    } while (0)

// DAC3
#undef GPIO_42_CONFIG
#define GPIO_42_CONFIG(h)                                     \
    do {                                                      \
        GPIO_setMode(h, GPIO_Number_42, GPIO_42_Mode_EPWM8A); \
    } while (0)

// DAC4
#undef GPIO_43_CONFIG
#define GPIO_43_CONFIG(h)                                     \
    do {                                                      \
        GPIO_setMode(h, GPIO_Number_43, GPIO_43_Mode_EPWM8B); \
    } while (0)

// DRV8323RH GAIN configuration
#undef GPIO_51_CONFIG
#if CURRENT_AMP_GAIN == 10
#define GPIO_51_CONFIG(h)                                             \
    do {                                                              \
        GPIO_setMode(h, GPIO_Number_51, GPIO_51_Mode_GeneralPurpose); \
        GPIO_setDirection(h, GPIO_Number_51, GPIO_Direction_Input);   \
    } while (0)
#elif CURRENT_AMP_GAIN == 5
#define GPIO_51_CONFIG(h)                                             \
    do {                                                              \
        GPIO_setMode(h, GPIO_Number_51, GPIO_51_Mode_GeneralPurpose); \
        GPIO_setDirection(h, GPIO_Number_51, GPIO_Direction_Output);  \
        GPIO_setLow(h, GPIO_Number_51);                               \
    } while (0)
#elif CURRENT_AMP_GAIN == 40
#define GPIO_51_CONFIG(h)                                             \
    do {                                                              \
        GPIO_setMode(h, GPIO_Number_51, GPIO_51_Mode_GeneralPurpose); \
        GPIO_setDirection(h, GPIO_Number_51, GPIO_Direction_Output);  \
        GPIO_setHigh(h, GPIO_Number_51);                              \
    } while (0)
#else
#error "Unsupported CURRENT_AMP_GAIN value"
#endif

// DRV83XX CAL
#undef GPIO_55_CONFIG
#define GPIO_55_CONFIG(h)                                             \
    do {                                                              \
        GPIO_setMode(h, GPIO_Number_55, GPIO_55_Mode_GeneralPurpose); \
        GPIO_setLow(h, GPIO_Number_55);                               \
        GPIO_setDirection(h, GPIO_Number_55, GPIO_Direction_Output);  \
    } while (0)

// UARTB TX
#undef GPIO_58_CONFIG
#define GPIO_58_CONFIG(h)                                      \
    do {                                                       \
        GPIO_setMode(h, GPIO_Number_58, GPIO_58_Mode_SCITXDB); \
    } while (0)

#define nUSER_ADC_FULL_SCALE_VOLTAGE_V ((V_REG) * ((VIN_R1) + (VIN_R2)) / (VIN_R2))
#define nUSER_ADC_FULL_SCALE_CURRENT_A (V_REG / (CURRENT_SHUNT_RES * (float)CURRENT_AMP_GAIN))

#define nUSER_IQ_FULL_SCALE_FREQ_Hz (800.0)    // 800 Example with buffer for 8-pole 6 KRPM motor to be run to 10 KRPM with field weakening; Hz =(RPM * Poles) / 120

#define nUSER_IQ_FULL_SCALE_VOLTAGE_V (24.0)    // 24.0 Set to Vbus
#define nUSER_IQ_FULL_SCALE_CURRENT_A (nUSER_ADC_FULL_SCALE_CURRENT_A * 0.5)

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
#define nUSER_MAX_VS_MAG_PU                   (2.0 / 3.0)    // Set to 0.5 if a current reconstruction technique is not used.  Look at the module svgen_current in lab10a-x for more info.
#define nUSER_NUM_ISR_TICKS_PER_CTRL_TICK     (1)            // 2 Example, controller clock rate (CTRL) runs at PWM / 2; ex 30 KHz PWM, 15 KHz control
#define nUSER_NUM_CTRL_TICKS_PER_CURRENT_TICK (1)            // 1 Typical, Forward FOC current controller (Iq/Id/IPARK/SVPWM) runs at same rate as CTRL.
#define nUSER_NUM_CTRL_TICKS_PER_EST_TICK     (1)            // 1 Typical, FAST estimator runs at same rate as CTRL;
#define nUSER_NUM_CTRL_TICKS_PER_SPEED_TICK   (15)           // 15 Typical to match PWM, ex: 15KHz PWM, controller, and current loop, 1KHz speed loop
#define nUSER_NUM_CTRL_TICKS_PER_TRAJ_TICK    (15)           // 15 Typical to match PWM, ex: 10KHz controller & current loop, 1KHz speed loop, 1 KHz Trajectory
#define nUSER_R_OVER_L_EST_FREQ_Hz            (300)          // 300 Default
#define nUSER_VOLTAGE_FILTER_POLE_Hz          (344.62)       // BOOSTXL-DRV8305 = 344.62 Hz

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

#endif /* INCLUDE_BOARD_BOARD_TMS320F28069M_8323H_J1_H_ */
