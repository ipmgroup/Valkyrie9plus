#ifndef _USER_H_
#define _USER_H_
/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 * --/COPYRIGHT--*/

// modules
#include "sw/modules/ctrl/src/32b/ctrl_obj.h"
#include "sw/modules/est/src/32b/est.h"
#include "sw/modules/est/src/est_Flux_states.h"
#include "sw/modules/est/src/est_Ls_states.h"
#include "sw/modules/est/src/est_Rs_states.h"
#include "sw/modules/est/src/est_states.h"
#include "sw/modules/motor/src/32b/motor.h"
#include "sw/modules/types/src/types.h"

#include "userParams.h"
#include "user_config.h"
#include "user_hw_select.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USER_FLASH_INIT                                                      \
    {                                                                        \
        .iqFullScaleCurrent_A       = nUSER_IQ_FULL_SCALE_CURRENT_A,         \
        .iqFullScaleVoltage_V       = nUSER_IQ_FULL_SCALE_VOLTAGE_V,         \
        .iqFullScaleFreq_Hz         = nUSER_IQ_FULL_SCALE_FREQ_Hz,           \
        .numIsrTicksPerCtrlTick     = nUSER_NUM_ISR_TICKS_PER_CTRL_TICK,     \
        .numCtrlTicksPerCurrentTick = nUSER_NUM_CTRL_TICKS_PER_CURRENT_TICK, \
        .numCtrlTicksPerEstTick     = nUSER_NUM_CTRL_TICKS_PER_EST_TICK,     \
        .numCtrlTicksPerSpeedTick   = nUSER_NUM_CTRL_TICKS_PER_SPEED_TICK,   \
        .numCtrlTicksPerTrajTick    = nUSER_NUM_CTRL_TICKS_PER_TRAJ_TICK,    \
        .numCurrentSensors          = nUSER_NUM_CURRENT_SENSORS,             \
        .numVoltageSensors          = nUSER_NUM_VOLTAGE_SENSORS,             \
        .offsetPole_rps             = nUSER_OFFSET_POLE_rps,                 \
        .fluxPole_rps               = nUSER_FLUX_POLE_rps,                   \
        .maxAccel_Hzps              = nUSER_MAX_ACCEL_Hzps,                  \
        .maxAccel_est_Hzps          = nUSER_MAX_ACCEL_EST_Hzps,              \
        .directionPole_rps          = nUSER_DIRECTION_POLE_rps,              \
        .speedPole_rps              = nUSER_SPEED_POLE_rps,                  \
        .dcBusPole_rps              = nUSER_DCBUS_POLE_rps,                  \
        .fluxFraction               = nUSER_FLUX_FRACTION,                   \
        .indEst_speedMaxFraction    = nUSER_SPEEDMAX_FRACTION_FOR_L_IDENT,   \
        .powerWarpGain              = nUSER_POWERWARP_GAIN,                  \
        .systemFreq_MHz             = nUSER_SYSTEM_FREQ_MHz,                 \
        .voltage_sf                 = nUSER_VOLTAGE_SF,                      \
        .current_sf                 = nUSER_CURRENT_SF,                      \
        .voltageFilterPole_Hz       = nUSER_VOLTAGE_FILTER_POLE_Hz,          \
        .maxVsMag_pu                = nUSER_MAX_VS_MAG_PU,                   \
        .estKappa                   = nUSER_EST_KAPPAQ,                      \
        .motor_type                 = nUSER_MOTOR_TYPE,                      \
        .motor_numPolePairs         = nUSER_MOTOR_NUM_POLE_PAIRS,            \
        .motor_ratedFlux            = nUSER_MOTOR_RATED_FLUX,                \
        .motor_Rr                   = nUSER_MOTOR_Rr,                        \
        .motor_Rs                   = nUSER_MOTOR_Rs,                        \
        .motor_Ls_d                 = nUSER_MOTOR_Ls_d,                      \
        .motor_Ls_q                 = nUSER_MOTOR_Ls_q,                      \
        .maxCurrent                 = nUSER_MOTOR_MAX_CURRENT,               \
        .maxCurrent_resEst          = nUSER_MOTOR_RES_EST_CURRENT,           \
        .maxCurrent_indEst          = nUSER_MOTOR_IND_EST_CURRENT,           \
        .IdRated                    = nUSER_MOTOR_MAGNETIZING_CURRENT,       \
        .IdRatedFraction_indEst     = nUSER_IDRATED_FRACTION_FOR_L_IDENT,    \
        .IdRatedFraction_ratedFlux  = nUSER_IDRATED_FRACTION_FOR_RATED_FLUX, \
        .IdRated_delta              = nUSER_IDRATED_DELTA,                   \
        .fluxEstFreq_Hz             = nUSER_MOTOR_FLUX_EST_FREQ_Hz,          \
        .RoverL_estFreq_Hz          = nUSER_R_OVER_L_EST_FREQ_Hz,            \
        .PwmFreq_kHz                = nUSER_PWM_FREQ_kHz,                    \
        .numPwmTicksPerIsrTick      = nUSER_NUM_PWM_TICKS_PER_ISR_TICK,      \
        .kp_spd                     = nUSER_MOTOR_KP,                        \
        .ki_spd                     = nUSER_MOTOR_KI,                        \
        .UserMaxAccel_krpmps        = _IQ(5.0),                              \
        .ledBlinkFreq_Hz            = LED_BLINK_FREQ_Hz,                     \
        .ecan_node_id               = USER_NODE_ID,                          \
        .telem_rate                 = USER_STATUS_BROADCAST_PERIOD_US,       \
        .esc_index                  = 0,                                     \
        .midpoint                   = 0,                                     \
        .min_esc                    = 819,                                   \
        .max_esc                    = 8191,                                  \
        .can_speed                  = 1000000,                               \
        .max_speed                  = nUSER_MAX_SPEED,                       \
        .control_word               = 105,                                   \
        .FlagDebugInfo              = 1,                                     \
    }

#ifndef USER_MOTOR
#error Motor is not defined in user.h
#endif

#ifndef nUSER_MOTOR_TYPE
#error The motor type is not defined in user.h
#endif

#ifndef nUSER_MOTOR_NUM_POLE_PAIRS
#error Number of motor pole pairs is not defined in user.h
#endif

#ifndef nUSER_MOTOR_Rr
#error The rotor resistance is not defined in user.h
#endif

#ifndef nUSER_MOTOR_Rs
#error The stator resistance is not defined in user.h
#endif

#ifndef nUSER_MOTOR_Ls_d
#error The direct stator inductance is not defined in user.h
#endif

#ifndef nUSER_MOTOR_Ls_q
#error The quadrature stator inductance is not defined in user.h
#endif

#ifndef nUSER_MOTOR_RATED_FLUX
#error The rated flux of motor is not defined in user.h
#endif

#ifndef nUSER_MOTOR_MAGNETIZING_CURRENT
#error The magnetizing current is not defined in user.h
#endif

#ifndef nUSER_MOTOR_RES_EST_CURRENT
#error The resistance estimation current is not defined in user.h
#endif

#ifndef nUSER_MOTOR_IND_EST_CURRENT
#error The inductance estimation current is not defined in user.h
#endif

#ifndef nUSER_MOTOR_MAX_CURRENT
#error The maximum current is not defined in user.h
#endif

#ifndef nUSER_MOTOR_FLUX_EST_FREQ_Hz
#error The flux estimation frequency is not defined in user.h
#endif

// **************************************************************************
// the functions

//! \brief      Takes base params to set the user parameter values
//! \param[in]  pUserParams      The pointer to the user param structure
//! \param[in]  pUserBaseParams  The pointer to the user base param structure
// extern void USER_setParamsFromBase(USER_Base_Params *pUserBaseParams, USER_Params *pUserParams);

//! \brief      Sets the user parameter values
//! \param[in]  pUserParams  The pointer to the user param structure
extern void USER_setParams(USER_Params *pUserParams);

//! \brief      Checks for errors in the user parameter values
//! \param[in]  pUserParams  The pointer to the user param structure
extern void USER_checkForErrors(USER_Params *pUserParams);

//! \brief      Gets the error code in the user parameters
//! \param[in]  pUserParams  The pointer to the user param structure
//! \return     The error code
extern USER_ErrorCode_e USER_getErrorCode(USER_Params *pUserParams);

//! \brief      Sets the error code in the user parameters
//! \param[in]  pUserParams  The pointer to the user param structure
//! \param[in]  errorCode    The error code
extern void USER_setErrorCode(USER_Params *pUserParams, const USER_ErrorCode_e errorCode);

//! \brief      Recalculates Inductances with the correct Q Format
//! \param[in]  handle       The controller (CTRL) handle
extern void USER_softwareUpdate1p6(CTRL_Handle handle, USER_Params *pUserParams);

//! \brief      Updates Id and Iq PI gains
//! \param[in]  handle       The controller (CTRL) handle
extern void USER_calcPIgains(CTRL_Handle handle, USER_Params *pUserParams);

//! \brief      Computes the scale factor needed to convert from torque created by Ld, Lq, Id and Iq, from per unit to Nm
//! \return     The scale factor to convert torque from (Ld - Lq) * Id * Iq from per unit to Nm, in IQ24 format
extern _iq USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(USER_Params *pUserParams);

//! \brief      Computes the scale factor needed to convert from torque created by flux and Iq, from per unit to Nm
//! \return     The scale factor to convert torque from Flux * Iq from per unit to Nm, in IQ24 format
extern _iq USER_computeTorque_Flux_Iq_pu_to_Nm_sf(USER_Params *pUserParams);

//! \brief      Computes the scale factor needed to convert from per unit to Wb
//! \return     The scale factor to convert from flux per unit to flux in Wb, in IQ24 format
extern _iq USER_computeFlux_pu_to_Wb_sf(USER_Params *pUserParams);

//! \brief      Computes the scale factor needed to convert from per unit to V/Hz
//! \return     The scale factor to convert from flux per unit to flux in V/Hz, in IQ24 format
extern _iq USER_computeFlux_pu_to_VpHz_sf(USER_Params *pUserParams);

//! \brief      Computes Flux in Wb or V/Hz depending on the scale factor sent as parameter
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  sf           The scale factor to convert flux from per unit to Wb or V/Hz
//! \return     The flux in Wb or V/Hz depending on the scale factor sent as parameter, in IQ24 format
extern _iq USER_computeFlux(CTRL_Handle handle, const _iq sf);

//! \brief      Computes Torque in Nm
//! \param[in]  handle          The controller (CTRL) handle
//! \param[in]  torque_Flux_sf  The scale factor to convert torque from (Ld - Lq) * Id * Iq from per unit to Nm
//! \param[in]  torque_Ls_sf    The scale factor to convert torque from Flux * Iq from per unit to Nm
//! \return     The torque in Nm, in IQ24 format
extern _iq USER_computeTorque_Nm(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf);

extern _iq USER_computeTorque_Nm_S(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf, USER_Torque *pTorque);

//! \brief      Computes Torque in lbin
//! \param[in]  handle          The controller (CTRL) handle
//! \param[in]  torque_Flux_sf  The scale factor to convert torque from (Ld - Lq) * Id * Iq from per unit to lbin
//! \param[in]  torque_Ls_sf    The scale factor to convert torque from Flux * Iq from per unit to lbin
//! \return     The torque in lbin, in IQ24 format
extern _iq USER_computeTorque_lbin(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf);

extern void copyUserParams_FLASH_to_Params(USER_Params_FLASH *src, USER_Params *dst);
extern void USER_setOtherParams(USER_Params *pUserParams);

#ifdef __cplusplus
}
#endif    // extern "C"

//@} // ingroup
#endif    // end of _USER_H_ definition
