// Note: to use DRV8323 you need to add C9, C10 and C11 to the board.

#ifndef INCLUDE_BOARD_BOARD_TMS320F28069M_8305_J5_H_
#define INCLUDE_BOARD_BOARD_TMS320F28069M_8305_J5_H_

#include "sw/drivers/gpio/src/32b/f28x/f2806x/gpio.h"

// **************************************************************************
// the defines

#define HW_REV_MAJOR 0
#define HW_REV_MINOR 1

#define DRV_NAME      "DRV8323RS"
#define APP_NODE_NAME "BOOSTXL-DRV8323RS"
#define DRV83XX

#ifndef AMP_GAIN
#define CURRENT_AMP_GAIN 20    // 5 10 20 40
#endif

#if CURRENT_AMP_GAIN == 5
#define dCSA_GAIN Gain_5VpV
#elif CURRENT_AMP_GAIN == 10
#define dCSA_GAIN Gain_10VpV
#elif CURRENT_AMP_GAIN == 20
#define dCSA_GAIN Gain_20VpV
#elif CURRENT_AMP_GAIN == 40
#define dCSA_GAIN Gain_40VpV
#else
#error "Unsupported CURRENT_AMP_GAIN value"
#endif

#ifndef CSA_GAIN
#undef CSA_GAIN Gain_20VpV
#endif

#ifndef V_REG
#define V_REG 3.3f
#endif
#ifndef VIN_R1
#define VIN_R1 82000.0f
#endif
#ifndef VIN_R2
#define VIN_R2 4990.0f
#endif

#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES 0.007f
#endif

#define USER_ADC_FULL_SCALE_VOLTAGE_V ((V_REG) * ((VIN_R1) + (VIN_R2)) / (VIN_R2))
#define USER_ADC_FULL_SCALE_CURRENT_A (V_REG / (CURRENT_SHUNT_RES * (float)CURRENT_AMP_GAIN))

#define LED_BLINK_FREQ_Hz 5

#define HAL_PWM_DBFED_CNT (uint16_t)(0.2 * (float_t)nUSER_SYSTEM_FREQ_MHz)
#define HAL_PWM_DBRED_CNT (uint16_t)(0.2 * (float_t)nUSER_SYSTEM_FREQ_MHz)

#define nADC_DELAY_usec 10000L

#define NUM_DRV_ERRORS 2

#define DEFINE_DRV_ERROR_STRINGS(num) \
    const char *str_DrvError[num] = { \
        "DrvError0:",                 \
        "DrvError1:"};

#define nFAULT_INT  GPIO_Number_22
#define ENABLE_GATE GPIO_Number_12
// #define ADC8_SUPPORT
// #define ADC9_SUPPORT
// #define TZ_DISABLE

#define TX_DELAY 0

//! \brief J5 specific defines
// **************************************************************************
#define PWM_INTERRUPT PWM_Number_4

#define PWM_HANDLE_0_BASE_ADDR PWM_ePWM4_BASE_ADDR
#define PWM_HANDLE_1_BASE_ADDR PWM_ePWM5_BASE_ADDR
#define PWM_HANDLE_2_BASE_ADDR PWM_ePWM6_BASE_ADDR

#define EN_GATE_GPIO_NUMBER GPIO_Number_20
#define DRV_SPI_HANDLE      spiBHandle

#define LOW_SPD_PRESCALER CLK_LowSpdPreScaler_SysClkOut_by_2

// #define PWMDACS
#define SPIB
#define SPI_CS GPIO_Number_53

#define ADC_SOC_TRIGGER_SOURCE   ADC_SocTrigSrc_EPWM4_ADCSOCA
#define ADC_SOC_0_CHANNEL_NUMBER ADC_SocChanNumber_B3
#define ADC_SOC_1_CHANNEL_NUMBER ADC_SocChanNumber_B3
#define ADC_SOC_2_CHANNEL_NUMBER ADC_SocChanNumber_A3
#define ADC_SOC_3_CHANNEL_NUMBER ADC_SocChanNumber_B5
#define ADC_SOC_4_CHANNEL_NUMBER ADC_SocChanNumber_B7
#define ADC_SOC_5_CHANNEL_NUMBER ADC_SocChanNumber_B4
#define ADC_SOC_6_CHANNEL_NUMBER ADC_SocChanNumber_A5
#define ADC_SOC_7_CHANNEL_NUMBER ADC_SocChanNumber_B5

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

#define HALL_GPIO_A GPIO_Number_54
#define HALL_GPIO_B GPIO_Number_55
#define HALL_GPIO_C GPIO_Number_56
#define gLED2       GPIO_Number_34
#define gLED3       GPIO_Number_39

#define DRV8323_SPI

#define DRV83XX_CONFIG(h, v)                       \
    v->Ctrl_Reg_05.OCP_MODE   = Latched_Shutdown;  \
    v->Ctrl_Reg_05.OCP_DEG    = OCPDeg_4_us;       \
    v->Ctrl_Reg_05.VDS_LVL    = VDS_Level_0p200_V; \
    v->Ctrl_Reg_03.IDRIVEP_HS = ISour_HS_0p440_A;  \
    v->Ctrl_Reg_03.IDRIVEN_HS = ISink_HS_2p000_A;  \
    v->Ctrl_Reg_04.IDRIVEP_LS = ISour_LS_0p440_A;  \
    v->Ctrl_Reg_04.IDRIVEN_LS = ISink_LS_2p000_A;  \
    v->Ctrl_Reg_06.CSA_GAIN   = Gain_20VpV;        \
    v->Ctrl_Reg_06.VREF_DIV   = 1;                 \
    v->Ctrl_Reg_06.CSA_CAL_A  = true;              \
    v->Ctrl_Reg_06.CSA_CAL_B  = true;              \
    v->Ctrl_Reg_06.CSA_CAL_C  = true;              \
    v->WriteCmd               = true;              \
    HAL_writeDrvData(h, v);                        \
    usDelay(100);                                  \
    v->Ctrl_Reg_06.CSA_CAL_A = false;              \
    v->Ctrl_Reg_06.CSA_CAL_B = false;              \
    v->Ctrl_Reg_06.CSA_CAL_C = false;              \
    v->WriteCmd              = true;               \
    HAL_writeDrvData(h, v);                        \
    v->ReadCmd = true;                             \
    HAL_readDrvData(h, v)

//! \brief GPIO configuration
// **************************************************************************

// PWM1
#undef GPIO_0_CONFIG
#define GPIO_0_CONFIG(h) GPIO_setMode(h, GPIO_Number_0, GPIO_0_Mode_EPWM1A)

// PWM2
#undef GPIO_1_CONFIG
#define GPIO_1_CONFIG(h) GPIO_setMode(h, GPIO_Number_1, GPIO_1_Mode_EPWM1B)

// PWM3
#undef GPIO_2_CONFIG
#define GPIO_2_CONFIG(h) GPIO_setMode(h, GPIO_Number_2, GPIO_2_Mode_EPWM2A)

// PWM4
#undef GPIO_3_CONFIG
#define GPIO_3_CONFIG(h) GPIO_setMode(h, GPIO_Number_3, GPIO_3_Mode_EPWM2B)

// PWM5
#undef GPIO_4_CONFIG
#define GPIO_4_CONFIG(h) GPIO_setMode(h, GPIO_Number_4, GPIO_4_Mode_EPWM3A)

// PWM6
#undef GPIO_5_CONFIG
#define GPIO_5_CONFIG(h) GPIO_setMode(h, GPIO_Number_5, GPIO_5_Mode_EPWM3B)

// PWM4A
#undef GPIO_6_CONFIG
#define GPIO_6_CONFIG(h) GPIO_setMode(h, GPIO_Number_6, GPIO_6_Mode_EPWM4A)

// PWM4B
#undef GPIO_7_CONFIG
#define GPIO_7_CONFIG(h) GPIO_setMode(h, GPIO_Number_7, GPIO_7_Mode_EPWM4B)

// PWM5A
#undef GPIO_8_CONFIG
#define GPIO_8_CONFIG(h) GPIO_setMode(h, GPIO_Number_8, GPIO_8_Mode_EPWM5A)

// PWM5B
#undef GPIO_9_CONFIG
#define GPIO_9_CONFIG(h) GPIO_setMode(h, GPIO_Number_9, GPIO_9_Mode_EPWM5B)

// PWM6A
#undef GPIO_10_CONFIG
#define GPIO_10_CONFIG(h) GPIO_setMode(h, GPIO_Number_10, GPIO_10_Mode_EPWM6A)

// PWM6B
#undef GPIO_11_CONFIG
#define GPIO_11_CONFIG(h) GPIO_setMode(h, GPIO_Number_11, GPIO_11_Mode_EPWM6B)

// SPIB CLK
#undef GPIO_14_CONFIG
#define GPIO_14_CONFIG(h) GPIO_setMode(h, GPIO_Number_14, GPIO_14_Mode_SPICLKB)

// UARTB RX
#undef GPIO_15_CONFIG
#define GPIO_15_CONFIG(h) GPIO_setMode(h, GPIO_Number_15, GPIO_15_Mode_SCIRXDB)

// SPIA SIMO
#undef GPIO_16_CONFIG
#define GPIO_16_CONFIG(h)                              \
    GPIO_setQualificationPeriod(h, GPIO_Number_16, 5); \
    GPIO_setMode(h, GPIO_Number_16, GPIO_16_Mode_SPISIMOA)

// SPIA SOMI
#undef GPIO_17_CONFIG
#define GPIO_17_CONFIG(h) GPIO_setMode(h, GPIO_Number_17, GPIO_17_Mode_SPISOMIA)

// SPIA CLK
#undef GPIO_18_CONFIG
#define GPIO_18_CONFIG(h) GPIO_setMode(h, GPIO_Number_18, GPIO_18_Mode_SPICLKA)

// DRV83XX Enable Gate
#undef GPIO_20_CONFIG
#define GPIO_20_CONFIG(h)                                         \
    GPIO_setMode(h, GPIO_Number_20, GPIO_20_Mode_GeneralPurpose); \
    GPIO_setLow(h, GPIO_Number_20);                               \
    GPIO_setDirection(h, GPIO_Number_20, GPIO_Direction_Output)

// SPIB SIMO
#undef GPIO_24_CONFIG
#define GPIO_24_CONFIG(h) GPIO_setMode(h, GPIO_Number_24, GPIO_24_Mode_SPISIMOB)

// SPIB SOMI
#undef GPIO_25_CONFIG
#define GPIO_25_CONFIG(h) GPIO_setMode(h, GPIO_Number_25, GPIO_25_Mode_SPISOMIB)

// OCTWn
#undef GPIO_28_CONFIG
#define GPIO_28_CONFIG(h) GPIO_setMode(h, GPIO_Number_28, GPIO_28_Mode_TZ2_NOT)

// FAULTn
#undef GPIO_29_CONFIG
#define GPIO_29_CONFIG(h) GPIO_setMode(h, GPIO_Number_29, GPIO_29_Mode_TZ3_NOT)

// CAN RX
#undef GPIO_30_CONFIG
#define GPIO_30_CONFIG(h) GPIO_setMode(h, GPIO_Number_30, GPIO_30_Mode_CANRXA)

// CAN TX
#undef GPIO_31_CONFIG
#define GPIO_31_CONFIG(h) GPIO_setMode(h, GPIO_Number_31, GPIO_31_Mode_CANTXA)

// I2C Data
#undef GPIO_32_CONFIG
#define GPIO_32_CONFIG(h) GPIO_setMode(h, GPIO_Number_32, GPIO_32_Mode_SDAA)

// I2C Clock
#undef GPIO_33_CONFIG
#define GPIO_33_CONFIG(h) GPIO_setMode(h, GPIO_Number_33, GPIO_33_Mode_SCLA)

// LED D9
#undef GPIO_34_CONFIG
#define GPIO_34_CONFIG(h)                                         \
    GPIO_setMode(h, GPIO_Number_34, GPIO_34_Mode_GeneralPurpose); \
    GPIO_setLow(h, GPIO_Number_34);                               \
    GPIO_setDirection(h, GPIO_Number_34, GPIO_Direction_Output)

// LED D10
#undef GPIO_39_CONFIG
#define GPIO_39_CONFIG(h)                                         \
    GPIO_setMode(h, GPIO_Number_39, GPIO_39_Mode_GeneralPurpose); \
    GPIO_setLow(h, GPIO_Number_39);                               \
    GPIO_setDirection(h, GPIO_Number_39, GPIO_Direction_Output)

// DAC1
#undef GPIO_40_CONFIG
#define GPIO_40_CONFIG(h) GPIO_setMode(h, GPIO_Number_40, GPIO_40_Mode_EPWM7A)

// DAC2
#undef GPIO_41_CONFIG
#define GPIO_41_CONFIG(h) GPIO_setMode(h, GPIO_Number_41, GPIO_41_Mode_EPWM7B)

// DAC3
#undef GPIO_42_CONFIG
#define GPIO_42_CONFIG(h) GPIO_setMode(h, GPIO_Number_42, GPIO_42_Mode_EPWM8A)

// DAC4
#undef GPIO_43_CONFIG
#define GPIO_43_CONFIG(h) GPIO_setMode(h, GPIO_Number_43, GPIO_43_Mode_EPWM8B)

// DRV83XX SPI CS
#undef GPIO_51_CONFIG
#define GPIO_51_CONFIG(h)                                         \
    GPIO_setMode(h, GPIO_Number_51, GPIO_51_Mode_GeneralPurpose); \
    GPIO_setHigh(h, GPIO_Number_51);                              \
    GPIO_setDirection(h, GPIO_Number_51, GPIO_Direction_Output)

// DRV83XX SPI CS
#undef GPIO_53_CONFIG
#define GPIO_53_CONFIG(h)                                         \
    GPIO_setMode(h, GPIO_Number_53, GPIO_53_Mode_GeneralPurpose); \
    GPIO_setHigh(h, GPIO_Number_53);                              \
    GPIO_setDirection(h, GPIO_Number_53, GPIO_Direction_Output)

// DRV83XX CAL
#undef GPIO_55_CONFIG
#define GPIO_55_CONFIG(h)                                         \
    GPIO_setMode(h, GPIO_Number_55, GPIO_55_Mode_GeneralPurpose); \
    GPIO_setLow(h, GPIO_Number_55);                               \
    GPIO_setDirection(h, GPIO_Number_55, GPIO_Direction_Output)

// GPIO
#undef GPIO_56_CONFIG
#define GPIO_56_CONFIG(h)                                         \
    GPIO_setQualificationPeriod(h, GPIO_Number_56, 5);            \
    GPIO_setMode(h, GPIO_Number_56, GPIO_56_Mode_GeneralPurpose); \
    GPIO_setDirection(h, GPIO_Number_56, GPIO_Direction_Output);  \
    GPIO_setLow(h, GPIO_Number_56)

// UARTB TX
#undef GPIO_58_CONFIG
#define GPIO_58_CONFIG(h) GPIO_setMode(h, GPIO_Number_58, GPIO_58_Mode_SCITXDB)

// FAST define

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

#endif /* INCLUDE_BOARD_BOARD_TMS320F28069M_8305_J5_H_ */
