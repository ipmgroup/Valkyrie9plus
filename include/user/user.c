/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 * --/COPYRIGHT--*/

#include "user.h"
#include "ctrl.h"
#include <math.h>

void copyUserParams_FLASH_to_Params(USER_Params_FLASH *src, USER_Params *dst) {
    dst->iqFullScaleCurrent_A = src->iqFullScaleCurrent_A;
    dst->iqFullScaleVoltage_V = src->iqFullScaleVoltage_V;
    dst->iqFullScaleFreq_Hz   = src->iqFullScaleFreq_Hz;
    dst->PwmFreq_kHz          = src->PwmFreq_kHz;

    dst->numIsrTicksPerCtrlTick     = src->numIsrTicksPerCtrlTick;
    dst->numCtrlTicksPerCurrentTick = src->numCtrlTicksPerCurrentTick;
    dst->numCtrlTicksPerEstTick     = src->numCtrlTicksPerEstTick;
    dst->numCtrlTicksPerSpeedTick   = src->numCtrlTicksPerSpeedTick;
    dst->numCtrlTicksPerTrajTick    = src->numCtrlTicksPerTrajTick;
    dst->numPwmTicksPerIsrTick      = src->numPwmTicksPerIsrTick;

    dst->numCurrentSensors       = src->numCurrentSensors;
    dst->numVoltageSensors       = src->numVoltageSensors;
    dst->voltageFilterPole_Hz    = src->voltageFilterPole_Hz;
    dst->offsetPole_rps          = src->offsetPole_rps;
    dst->fluxPole_rps            = src->fluxPole_rps;
    dst->maxAccel_Hzps           = src->maxAccel_Hzps;
    dst->maxAccel_est_Hzps       = src->maxAccel_est_Hzps;
    dst->directionPole_rps       = src->directionPole_rps;
    dst->speedPole_rps           = src->speedPole_rps;
    dst->dcBusPole_rps           = src->dcBusPole_rps;
    dst->fluxFraction            = src->fluxFraction;
    dst->indEst_speedMaxFraction = src->indEst_speedMaxFraction;
    dst->systemFreq_MHz          = src->systemFreq_MHz;

    dst->voltage_sf         = src->voltage_sf;
    dst->current_sf         = src->current_sf;
    dst->maxVsMag_pu        = src->maxVsMag_pu;
    dst->estKappa           = src->estKappa;
    dst->motor_type         = src->motor_type;
    dst->motor_numPolePairs = src->motor_numPolePairs;
    dst->motor_ratedFlux    = src->motor_ratedFlux;
    dst->motor_Rr           = src->motor_Rr;
    dst->motor_Rs           = src->motor_Rs;
    dst->motor_Ls_d         = src->motor_Ls_d;
    dst->motor_Ls_q         = src->motor_Ls_q;

    dst->powerWarpGain     = src->powerWarpGain;
    dst->maxCurrent_resEst = src->maxCurrent_resEst;
    dst->maxCurrent_indEst = src->maxCurrent_indEst;
    dst->maxCurrent        = src->maxCurrent;

    dst->IdRated                   = src->IdRated;
    dst->IdRatedFraction_ratedFlux = src->IdRatedFraction_ratedFlux;
    dst->IdRatedFraction_indEst    = src->IdRatedFraction_indEst;
    dst->IdRated_delta             = src->IdRated_delta;

    dst->fluxEstFreq_Hz    = src->fluxEstFreq_Hz;
    dst->RoverL_estFreq_Hz = src->RoverL_estFreq_Hz;
    dst->kp_spd            = src->kp_spd;
    dst->ki_spd            = src->ki_spd;
    dst->can_speed         = src->can_speed;
    dst->FW_Speed_Krpm     = src->FW_Speed_Krpm;
}

void USER_setOtherParams(USER_Params *pUserParams) {

    pUserParams->voltageFilterPole_rps = (2.0 * MATH_PI * pUserParams->voltageFilterPole_Hz);

    pUserParams->zeroSpeedLimit    = (0.5 / pUserParams->iqFullScaleFreq_Hz);
    pUserParams->forceAngleFreq_Hz = (2.0 * pUserParams->zeroSpeedLimit * pUserParams->iqFullScaleFreq_Hz);
    pUserParams->pwmPeriod_usec    = (1000.0 / pUserParams->PwmFreq_kHz);
    pUserParams->isrPeriod_usec    = (pUserParams->pwmPeriod_usec * (float_t)pUserParams->numPwmTicksPerIsrTick);
    pUserParams->isrFreq_Hz        = pUserParams->PwmFreq_kHz * 1000.0 / (float_t)pUserParams->numPwmTicksPerIsrTick;

    pUserParams->ctrlFreq_Hz            = ((uint_least32_t)pUserParams->isrFreq_Hz / (uint_least32_t)pUserParams->numIsrTicksPerCtrlTick);
    pUserParams->estFreq_Hz             = pUserParams->ctrlFreq_Hz / (uint_least32_t)pUserParams->numCtrlTicksPerEstTick;
    pUserParams->trajFreq_Hz            = pUserParams->ctrlFreq_Hz / (uint_least32_t)pUserParams->numCtrlTicksPerTrajTick;
    pUserParams->ctrlPeriod_usec        = pUserParams->isrPeriod_usec * (float_t)pUserParams->numIsrTicksPerCtrlTick;
    pUserParams->ctrlPeriod_sec         = pUserParams->ctrlPeriod_usec / 1000000.0;
    pUserParams->maxNegativeIdCurrent_a = (-0.5 * pUserParams->maxCurrent);

    pUserParams->maxAccel_krpmps_Sf        = _IQ((float_t)pUserParams->motor_numPolePairs * 1000.0 / (float_t)pUserParams->trajFreq_Hz / pUserParams->iqFullScaleFreq_Hz / 60.0);
    pUserParams->maxCurrentSlope           = pUserParams->maxCurrent_resEst / pUserParams->iqFullScaleCurrent_A / pUserParams->trajFreq_Hz;
    pUserParams->maxCurrentSlope_powerWarp = 0.3 * pUserParams->maxCurrent_resEst / pUserParams->iqFullScaleCurrent_A / pUserParams->trajFreq_Hz;

    pUserParams->ctrlWaitTime[CTRL_State_Error]   = 0;
    pUserParams->ctrlWaitTime[CTRL_State_Idle]    = 0;
    pUserParams->ctrlWaitTime[CTRL_State_OffLine] = (uint_least32_t)(5.0 * pUserParams->ctrlFreq_Hz);
    pUserParams->ctrlWaitTime[CTRL_State_OnLine]  = 0;

    pUserParams->estWaitTime[EST_State_Error]           = 0;
    pUserParams->estWaitTime[EST_State_Idle]            = 0;
    pUserParams->estWaitTime[EST_State_RoverL]          = (uint_least32_t)(8.0 * pUserParams->estFreq_Hz);
    pUserParams->estWaitTime[EST_State_Rs]              = 0;
    pUserParams->estWaitTime[EST_State_RampUp]          = (uint_least32_t)((5.0 + pUserParams->fluxEstFreq_Hz / pUserParams->maxAccel_est_Hzps) * pUserParams->estFreq_Hz);
    pUserParams->estWaitTime[EST_State_IdRated]         = (uint_least32_t)(30.0 * pUserParams->estFreq_Hz);
    pUserParams->estWaitTime[EST_State_RatedFlux_OL]    = (uint_least32_t)(0.2 * pUserParams->estFreq_Hz);
    pUserParams->estWaitTime[EST_State_RatedFlux]       = 0;
    pUserParams->estWaitTime[EST_State_RampDown]        = (uint_least32_t)(2.0 * pUserParams->estFreq_Hz);
    pUserParams->estWaitTime[EST_State_LockRotor]       = 0;
    pUserParams->estWaitTime[EST_State_Ls]              = 0;
    pUserParams->estWaitTime[EST_State_Rr]              = (uint_least32_t)(20.0 * pUserParams->estFreq_Hz);
    pUserParams->estWaitTime[EST_State_MotorIdentified] = 0;
    pUserParams->estWaitTime[EST_State_OnLine]          = 0;

    pUserParams->FluxWaitTime[EST_Flux_State_Error] = 0;
    pUserParams->FluxWaitTime[EST_Flux_State_Idle]  = 0;
    pUserParams->FluxWaitTime[EST_Flux_State_CL1]   = (uint_least32_t)(10.0 * pUserParams->estFreq_Hz);
    pUserParams->FluxWaitTime[EST_Flux_State_CL2]   = (uint_least32_t)(0.2 * pUserParams->estFreq_Hz);
    pUserParams->FluxWaitTime[EST_Flux_State_Fine]  = (uint_least32_t)(4.0 * pUserParams->estFreq_Hz);
    pUserParams->FluxWaitTime[EST_Flux_State_Done]  = 0;

    pUserParams->LsWaitTime[EST_Ls_State_Error]  = 0;
    pUserParams->LsWaitTime[EST_Ls_State_Idle]   = 0;
    pUserParams->LsWaitTime[EST_Ls_State_RampUp] = (uint_least32_t)(3.0 * pUserParams->estFreq_Hz);
    pUserParams->LsWaitTime[EST_Ls_State_Init]   = (uint_least32_t)(3.0 * pUserParams->estFreq_Hz);
    pUserParams->LsWaitTime[EST_Ls_State_Coarse] = (uint_least32_t)(0.2 * pUserParams->estFreq_Hz);
    pUserParams->LsWaitTime[EST_Ls_State_Fine]   = (uint_least32_t)(30.0 * pUserParams->estFreq_Hz);
    pUserParams->LsWaitTime[EST_Ls_State_Done]   = 0;

    pUserParams->RsWaitTime[EST_Rs_State_Error]  = 0;
    pUserParams->RsWaitTime[EST_Rs_State_Idle]   = 0;
    pUserParams->RsWaitTime[EST_Rs_State_RampUp] = (uint_least32_t)(1.0 * pUserParams->estFreq_Hz);
    pUserParams->RsWaitTime[EST_Rs_State_Coarse] = (uint_least32_t)(2.0 * pUserParams->estFreq_Hz);
    pUserParams->RsWaitTime[EST_Rs_State_Fine]   = (uint_least32_t)(7.0 * pUserParams->estFreq_Hz);
    pUserParams->RsWaitTime[EST_Rs_State_Done]   = 0;
    return;
}

void USER_checkForErrors(USER_Params *pUserParams) {
    USER_setErrorCode(pUserParams, USER_ErrorCode_NoError);

    if ((pUserParams->iqFullScaleCurrent_A <= 0.0) ||
        (pUserParams->iqFullScaleCurrent_A <= (0.02 * pUserParams->maxCurrent * pUserParams->iqFullScaleFreq_Hz / 128.0)) ||
        (pUserParams->iqFullScaleCurrent_A <= (2.0 * pUserParams->maxCurrent * pUserParams->iqFullScaleFreq_Hz * pUserParams->ctrlPeriod_sec / 128.0))) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleCurrent_A_Low);
    }

    if ((pUserParams->iqFullScaleCurrent_A < pUserParams->IdRated) ||
        (pUserParams->iqFullScaleCurrent_A < pUserParams->maxCurrent_resEst) ||
        (pUserParams->iqFullScaleCurrent_A < pUserParams->maxCurrent_indEst) ||
        (pUserParams->iqFullScaleCurrent_A < pUserParams->maxCurrent)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleCurrent_A_Low);
    }

    if ((pUserParams->motor_ratedFlux > 0.0) && (pUserParams->motor_type == MOTOR_Type_Pm)) {
        if (pUserParams->iqFullScaleVoltage_V >= ((float_t)pUserParams->estFreq_Hz * pUserParams->motor_ratedFlux * 0.7)) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_High);
        }
    }

    if ((pUserParams->motor_ratedFlux > 0.0) && (pUserParams->motor_type == MOTOR_Type_Induction)) {
        if (pUserParams->iqFullScaleVoltage_V >= ((float_t)pUserParams->estFreq_Hz * pUserParams->motor_ratedFlux * 0.05)) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_High);
        }
    }

    if ((pUserParams->iqFullScaleVoltage_V <= 0.0) ||
        (pUserParams->iqFullScaleVoltage_V <= (0.5 * pUserParams->maxCurrent * pUserParams->motor_Ls_d * pUserParams->voltageFilterPole_rps)) ||
        (pUserParams->iqFullScaleVoltage_V <= (0.5 * pUserParams->maxCurrent * pUserParams->motor_Ls_q * pUserParams->voltageFilterPole_rps))) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_Low);
    }

    if ((pUserParams->iqFullScaleFreq_Hz > (4.0 * pUserParams->voltageFilterPole_Hz)) ||
        (pUserParams->iqFullScaleFreq_Hz >= ((128.0 * pUserParams->iqFullScaleCurrent_A) / (0.02 * pUserParams->maxCurrent))) ||
        (pUserParams->iqFullScaleFreq_Hz >= ((128.0 * pUserParams->iqFullScaleCurrent_A) / (2.0 * pUserParams->maxCurrent * pUserParams->ctrlPeriod_sec))) ||
        (pUserParams->iqFullScaleFreq_Hz >= (128.0 * (float_t)pUserParams->motor_numPolePairs * 1000.0 / 60.0))) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleFreq_Hz_High);
    }

    if ((pUserParams->iqFullScaleFreq_Hz < 50.0) ||
        (pUserParams->iqFullScaleFreq_Hz < pUserParams->fluxEstFreq_Hz) ||
        (pUserParams->iqFullScaleFreq_Hz < pUserParams->speedPole_rps) ||
        (pUserParams->iqFullScaleFreq_Hz <= ((float_t)pUserParams->motor_numPolePairs * 1000.0 / (60.0 * 128.0))) ||
        (pUserParams->iqFullScaleFreq_Hz < (pUserParams->maxAccel_Hzps / ((float_t)pUserParams->trajFreq_Hz))) ||
        (pUserParams->iqFullScaleFreq_Hz < (pUserParams->maxAccel_est_Hzps / ((float_t)pUserParams->trajFreq_Hz))) ||
        (pUserParams->iqFullScaleFreq_Hz < ((float_t)pUserParams->RoverL_estFreq_Hz))) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleFreq_Hz_Low);
    }

    if (pUserParams->numPwmTicksPerIsrTick > 3) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_numPwmTicksPerIsrTick_High);
    }

    if (pUserParams->numPwmTicksPerIsrTick < 1) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_numPwmTicksPerIsrTick_Low);
    }

    if (pUserParams->numIsrTicksPerCtrlTick < 1) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_numIsrTicksPerCtrlTick_Low);
    }

    if (pUserParams->numCtrlTicksPerCurrentTick < 1) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerCurrentTick_Low);
    }

    if (pUserParams->numCtrlTicksPerEstTick < 1) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerEstTick_Low);
    }

    if ((pUserParams->numCtrlTicksPerSpeedTick < 1) ||
        (pUserParams->numCtrlTicksPerSpeedTick < pUserParams->numCtrlTicksPerCurrentTick)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerSpeedTick_Low);
    }

    if (pUserParams->numCtrlTicksPerTrajTick < 1) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerTrajTick_Low);
    }

    if (nUSER_NUM_CURRENT_SENSORS > 3) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_numCurrentSensors_High);
    }

    if (nUSER_NUM_CURRENT_SENSORS < 2) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_numCurrentSensors_Low);
    }

    if (nUSER_NUM_VOLTAGE_SENSORS > 3) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_numVoltageSensors_High);
    }

    if (nUSER_NUM_VOLTAGE_SENSORS < 3) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_numVoltageSensors_Low);
    }

    if (pUserParams->offsetPole_rps > ((float_t)pUserParams->ctrlFreq_Hz)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_offsetPole_rps_High);
    }

    if (pUserParams->offsetPole_rps <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_offsetPole_rps_Low);
    }

    if (pUserParams->fluxPole_rps > ((float_t)pUserParams->estFreq_Hz)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_fluxPole_rps_High);
    }

    if (pUserParams->fluxPole_rps <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_fluxPole_rps_Low);
    }

    if (pUserParams->zeroSpeedLimit > 1.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_zeroSpeedLimit_High);
    }

    if (pUserParams->zeroSpeedLimit <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_zeroSpeedLimit_Low);
    }

    if (pUserParams->forceAngleFreq_Hz > ((float_t)pUserParams->estFreq_Hz)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_forceAngleFreq_Hz_High);
    }

    if (pUserParams->forceAngleFreq_Hz <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_forceAngleFreq_Hz_Low);
    }

    if (pUserParams->maxAccel_Hzps > ((float_t)pUserParams->trajFreq_Hz * pUserParams->iqFullScaleFreq_Hz)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_Hzps_High);
    }

    if (pUserParams->maxAccel_Hzps <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_Hzps_Low);
    }

    if (pUserParams->maxAccel_est_Hzps > ((float_t)pUserParams->trajFreq_Hz * pUserParams->iqFullScaleFreq_Hz)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_est_Hzps_High);
    }

    if (pUserParams->maxAccel_est_Hzps <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_est_Hzps_Low);
    }

    if (pUserParams->directionPole_rps > ((float_t)pUserParams->estFreq_Hz)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_directionPole_rps_High);
    }

    if (pUserParams->directionPole_rps <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_directionPole_rps_Low);
    }

    if ((pUserParams->speedPole_rps > pUserParams->iqFullScaleFreq_Hz) ||
        (pUserParams->speedPole_rps > ((float_t)pUserParams->estFreq_Hz))) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_speedPole_rps_High);
    }

    if (pUserParams->speedPole_rps <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_speedPole_rps_Low);
    }

    if (pUserParams->dcBusPole_rps > ((float_t)pUserParams->estFreq_Hz)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_dcBusPole_rps_High);
    }

    if (pUserParams->dcBusPole_rps <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_dcBusPole_rps_Low);
    }

    if (pUserParams->fluxFraction > 1.2) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_fluxFraction_High);
    }

    if (pUserParams->fluxFraction < 0.05) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_fluxFraction_Low);
    }

    if (pUserParams->indEst_speedMaxFraction > (pUserParams->iqFullScaleCurrent_A / pUserParams->maxCurrent)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_indEst_speedMaxFraction_High);
    }

    if (pUserParams->indEst_speedMaxFraction <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_indEst_speedMaxFraction_Low);
    }

    if (pUserParams->powerWarpGain > 2.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_powerWarpGain_High);
    }

    if (pUserParams->powerWarpGain < 1.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_powerWarpGain_Low);
    }

    if (pUserParams->systemFreq_MHz > 90.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_systemFreq_MHz_High);
    }

    if (pUserParams->systemFreq_MHz <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_systemFreq_MHz_Low);
    }

    if (pUserParams->PwmFreq_kHz > (1000.0 * pUserParams->systemFreq_MHz / 100.0)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_pwmFreq_kHz_High);
    }

    if (pUserParams->PwmFreq_kHz < (1000.0 * pUserParams->systemFreq_MHz / 65536.0)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_pwmFreq_kHz_Low);
    }

    if (nUSER_VOLTAGE_SF >= 128.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_voltage_sf_High);
    }

    if (nUSER_VOLTAGE_SF < 0.1) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_voltage_sf_Low);
    }

    if (nUSER_CURRENT_SF >= 128.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_current_sf_High);
    }

    if (nUSER_CURRENT_SF < 0.1) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_current_sf_Low);
    }

    if (pUserParams->voltageFilterPole_Hz > ((float_t)pUserParams->estFreq_Hz / MATH_PI)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_voltageFilterPole_Hz_High);
    }

    if (pUserParams->voltageFilterPole_Hz < (pUserParams->iqFullScaleFreq_Hz / 4.0)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_voltageFilterPole_Hz_Low);
    }

    if (pUserParams->maxVsMag_pu > (4.0 / 3.0)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxVsMag_pu_High);
    }

    if (pUserParams->maxVsMag_pu <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxVsMag_pu_Low);
    }

    if (pUserParams->estKappa > 1.5) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_estKappa_High);
    }

    if (pUserParams->estKappa < 1.5) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_estKappa_Low);
    }

    if ((pUserParams->motor_type != MOTOR_Type_Induction) && (pUserParams->motor_type != MOTOR_Type_Pm)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_motor_type_Unknown);
    }

    if (pUserParams->motor_numPolePairs < 1) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_motor_numPolePairs_Low);
    }

    if ((pUserParams->motor_ratedFlux != 0.0) && (pUserParams->motor_type == MOTOR_Type_Pm)) {
        if (pUserParams->motor_ratedFlux > (pUserParams->iqFullScaleFreq_Hz * 65536.0 / (float_t)pUserParams->estFreq_Hz / 0.7)) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_High);
        }

        if (pUserParams->motor_ratedFlux < (pUserParams->iqFullScaleVoltage_V / (float_t)pUserParams->estFreq_Hz / 0.7)) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_Low);
        }
    }

    if ((pUserParams->motor_ratedFlux != 0.0) && (pUserParams->motor_type == MOTOR_Type_Induction)) {
        if (pUserParams->motor_ratedFlux > (pUserParams->iqFullScaleFreq_Hz * 65536.0 / (float_t)pUserParams->estFreq_Hz / 0.05)) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_High);
        }

        if (pUserParams->motor_ratedFlux < (pUserParams->iqFullScaleVoltage_V / (float_t)pUserParams->estFreq_Hz / 0.05)) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_Low);
        }
    }

    if (pUserParams->motor_type == MOTOR_Type_Pm) {
        if (pUserParams->motor_Rr > 0.0) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_High);
        }

        if (pUserParams->motor_Rr < 0.0) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_Low);
        }
    }

    if ((pUserParams->motor_Rr != 0.0) && (pUserParams->motor_type == MOTOR_Type_Induction)) {
        if (pUserParams->motor_Rr > (0.7 * 65536.0 * pUserParams->iqFullScaleVoltage_V / pUserParams->iqFullScaleCurrent_A)) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_High);
        }

        if (pUserParams->motor_Rr < (0.7 * pUserParams->iqFullScaleVoltage_V / (pUserParams->iqFullScaleCurrent_A * 65536.0))) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_Low);
        }
    }

    if (pUserParams->motor_Rs != 0.0) {
        if (pUserParams->motor_Rs > (0.7 * 65536.0 * pUserParams->iqFullScaleVoltage_V / pUserParams->iqFullScaleCurrent_A)) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rs_High);
        }

        if (pUserParams->motor_Rs < (0.7 * pUserParams->iqFullScaleVoltage_V / (pUserParams->iqFullScaleCurrent_A * 65536.0))) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rs_Low);
        }
    }

    if (pUserParams->motor_Ls_d != 0.0) {
        if (pUserParams->motor_Ls_d > (0.7 * 65536.0 * pUserParams->iqFullScaleVoltage_V / (pUserParams->iqFullScaleCurrent_A * pUserParams->voltageFilterPole_rps))) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_d_High);
        }

        if (pUserParams->motor_Ls_d < (0.7 * pUserParams->iqFullScaleVoltage_V / (pUserParams->iqFullScaleCurrent_A * pUserParams->voltageFilterPole_rps * 65536.0))) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_d_Low);
        }
    }

    if (pUserParams->motor_Ls_q != 0.0) {
        if (pUserParams->motor_Ls_q > (0.7 * 65536.0 * pUserParams->iqFullScaleVoltage_V / (pUserParams->iqFullScaleCurrent_A * pUserParams->voltageFilterPole_rps))) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_q_High);
        }

        if (pUserParams->motor_Ls_q < (0.7 * pUserParams->iqFullScaleVoltage_V / (pUserParams->iqFullScaleCurrent_A * pUserParams->voltageFilterPole_rps * 65536.0))) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_q_Low);
        }
    }

    if (pUserParams->maxCurrent_resEst > pUserParams->maxCurrent) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_resEst_High);
    }

    if (pUserParams->maxCurrent_resEst < 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_resEst_Low);
    }

    if (pUserParams->motor_type == MOTOR_Type_Pm) {
        if (pUserParams->maxCurrent_indEst > 0.0) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_High);
        }

        if (pUserParams->maxCurrent_indEst < (-pUserParams->maxCurrent)) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_Low);
        }
    }

    if (pUserParams->motor_type == MOTOR_Type_Induction) {
        if (pUserParams->maxCurrent_indEst > 0.0) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_High);
        }

        if (pUserParams->maxCurrent_indEst < 0.0) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_Low);
        }
    }

    if (pUserParams->maxCurrent > pUserParams->iqFullScaleCurrent_A) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_High);
    }

    if (pUserParams->maxCurrent <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_Low);
    }

    if (pUserParams->maxCurrentSlope > 1.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_High);
    }

    if (pUserParams->maxCurrentSlope <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_Low);
    }

    if (pUserParams->maxCurrentSlope_powerWarp > 1.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_powerWarp_High);
    }

    if (pUserParams->maxCurrentSlope_powerWarp <= 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_powerWarp_Low);
    }

    if (pUserParams->motor_type == MOTOR_Type_Pm) {
        if (pUserParams->IdRated > 0.0) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_High);
        }

        if (pUserParams->IdRated < 0.0) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_Low);
        }
    }

    if (pUserParams->motor_type == MOTOR_Type_Induction) {
        if (pUserParams->IdRated > pUserParams->maxCurrent) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_High);
        }

        if (pUserParams->IdRated < 0.0) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_Low);
        }
    }

    if (pUserParams->motor_type == MOTOR_Type_Induction) {
        if (pUserParams->IdRatedFraction_ratedFlux > (pUserParams->iqFullScaleCurrent_A / (1.2 * pUserParams->maxCurrent))) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_ratedFlux_High);
        }

        if (pUserParams->IdRatedFraction_ratedFlux < 0.1) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_ratedFlux_Low);
        }
    }

    if (pUserParams->motor_type == MOTOR_Type_Induction) {
        if (pUserParams->IdRatedFraction_indEst > (pUserParams->iqFullScaleCurrent_A / pUserParams->maxCurrent)) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_indEst_High);
        }

        if (pUserParams->IdRatedFraction_indEst < 0.1) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_indEst_Low);
        }
    }

    if (pUserParams->motor_type == MOTOR_Type_Induction) {
        if (pUserParams->IdRated_delta > (pUserParams->iqFullScaleCurrent_A / ((float_t)pUserParams->numIsrTicksPerCtrlTick * pUserParams->maxCurrent))) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_delta_High);
        }

        if (pUserParams->IdRated_delta < 0.0) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_delta_Low);
        }
    }

    if (pUserParams->fluxEstFreq_Hz > pUserParams->iqFullScaleFreq_Hz) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_fluxEstFreq_Hz_High);
    }

    if ((pUserParams->fluxEstFreq_Hz < 0.0) ||
        (pUserParams->fluxEstFreq_Hz < (pUserParams->zeroSpeedLimit * pUserParams->iqFullScaleFreq_Hz))) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_fluxEstFreq_Hz_Low);
    }

    if (pUserParams->motor_Ls_d != 0.0) {
        if (((float_t)pUserParams->ctrlFreq_Hz >= (128.0 * pUserParams->iqFullScaleVoltage_V / (0.5 * (pUserParams->motor_Ls_d + 1e-9) * pUserParams->iqFullScaleCurrent_A)))) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_High);
        }
    }

    if (pUserParams->motor_Ls_q != 0.0) {
        if (((float_t)pUserParams->ctrlFreq_Hz >= (128.0 * pUserParams->iqFullScaleVoltage_V / (0.5 * (pUserParams->motor_Ls_q + 1e-9) * pUserParams->iqFullScaleCurrent_A)))) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_High);
        }
    }

    if (((float_t)pUserParams->ctrlFreq_Hz < pUserParams->iqFullScaleFreq_Hz) ||
        ((float_t)pUserParams->ctrlFreq_Hz < pUserParams->offsetPole_rps) ||
        ((float_t)pUserParams->ctrlFreq_Hz < 250.0) ||
        ((float_t)pUserParams->ctrlFreq_Hz <= (2.0 * pUserParams->iqFullScaleFreq_Hz * pUserParams->maxCurrent / (128.0 * pUserParams->iqFullScaleCurrent_A)))) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_Low);
    }

    if ((pUserParams->motor_Rs != 0.0) && (pUserParams->motor_Ls_d != 0.0) && (pUserParams->motor_Ls_q != 0.0)) {
        if (((float_t)pUserParams->ctrlFreq_Hz <= (pUserParams->motor_Rs / (pUserParams->motor_Ls_d + 1e-9))) ||
            ((float_t)pUserParams->ctrlFreq_Hz <= (pUserParams->motor_Rs / (pUserParams->motor_Ls_q + 1e-9)))) {
            USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_Low);
        }
    }

    if (((float_t)pUserParams->estFreq_Hz < pUserParams->forceAngleFreq_Hz) ||
        ((float_t)pUserParams->estFreq_Hz < pUserParams->voltageFilterPole_rps) ||
        ((float_t)pUserParams->estFreq_Hz < pUserParams->dcBusPole_rps) ||
        ((float_t)pUserParams->estFreq_Hz < pUserParams->fluxPole_rps) ||
        ((float_t)pUserParams->estFreq_Hz < pUserParams->directionPole_rps) ||
        ((float_t)pUserParams->estFreq_Hz < pUserParams->speedPole_rps) ||
        ((float_t)pUserParams->estFreq_Hz < 0.2)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_estFreq_Hz_Low);
    }

    if (pUserParams->RoverL_estFreq_Hz > pUserParams->iqFullScaleFreq_Hz) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_RoverL_estFreq_Hz_High);
    }

    if (((float_t)pUserParams->trajFreq_Hz < 1.0) ||
        ((float_t)pUserParams->trajFreq_Hz < pUserParams->maxAccel_Hzps / pUserParams->iqFullScaleFreq_Hz) ||
        ((float_t)pUserParams->trajFreq_Hz < pUserParams->maxAccel_est_Hzps / pUserParams->iqFullScaleFreq_Hz)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_trajFreq_Hz_Low);
    }

    if (pUserParams->maxNegativeIdCurrent_a > 0.0) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxNegativeIdCurrent_a_High);
    }

    if (pUserParams->maxNegativeIdCurrent_a < (-pUserParams->maxCurrent)) {
        USER_setErrorCode(pUserParams, USER_ErrorCode_maxNegativeIdCurrent_a_Low);
    }

    return;
}    // end of USER_checkForErrors() function

USER_ErrorCode_e USER_getErrorCode(USER_Params *pUserParams) {
    return (pUserParams->errorCode);
}    // end of USER_getErrorCode() function

void USER_setErrorCode(USER_Params *pUserParams, const USER_ErrorCode_e errorCode) {
    pUserParams->errorCode = errorCode;

    return;
}    // end of USER_setErrorCode() function

void USER_softwareUpdate1p6(CTRL_Handle handle, USER_Params *pUserParams) {
    CTRL_Obj     *obj                 = (CTRL_Obj *)handle;
    float_t       fullScaleInductance = pUserParams->iqFullScaleVoltage_V / (pUserParams->iqFullScaleCurrent_A * pUserParams->voltageFilterPole_rps);
    float_t       Ls_coarse_max       = _IQ30toF(EST_getLs_coarse_max_pu(obj->estHandle));
    int_least8_t  lShift              = ceil(log(obj->motorParams.Ls_d_H / (Ls_coarse_max * fullScaleInductance)) / log(2.0));
    uint_least8_t Ls_qFmt             = 30 - lShift;
    float_t       L_max               = fullScaleInductance * pow(2.0, lShift);
    _iq           Ls_d_pu             = _IQ30(obj->motorParams.Ls_d_H / L_max);
    _iq           Ls_q_pu             = _IQ30(obj->motorParams.Ls_q_H / L_max);

    // store the results
    EST_setLs_d_pu(obj->estHandle, Ls_d_pu);
    EST_setLs_q_pu(obj->estHandle, Ls_q_pu);
    EST_setLs_qFmt(obj->estHandle, Ls_qFmt);

    return;
}    // end of softwareUpdate1p6() function

#ifndef NO_CTRL
void USER_calcPIgains(CTRL_Handle handle, USER_Params *pUserParams) {
    CTRL_Obj *obj              = (CTRL_Obj *)handle;
    float_t   fullScaleCurrent = pUserParams->iqFullScaleCurrent_A;
    float_t   fullScaleVoltage = pUserParams->iqFullScaleVoltage_V;
    float_t   ctrlPeriod_sec   = CTRL_getCtrlPeriod_sec(handle);
    float_t   Ls_d;
    float_t   Ls_q;
    float_t   Rs;
    float_t   RoverLs_d;
    float_t   RoverLs_q;
    _iq       Kp_Id;
    _iq       Ki_Id;
    _iq       Kp_Iq;
    _iq       Ki_Iq;
    _iq       Kd;

#ifdef __TMS320C28XX_FPU32__
    int32_t tmp;

    // when calling EST_ functions that return a float, and fpu32 is enabled, an integer is needed as a return
    // so that the compiler reads the returned value from the accumulator instead of fpu32 registers
    tmp  = EST_getLs_d_H(obj->estHandle);
    Ls_d = *((float_t *)&tmp);

    tmp  = EST_getLs_q_H(obj->estHandle);
    Ls_q = *((float_t *)&tmp);

    tmp = EST_getRs_Ohm(obj->estHandle);
    Rs  = *((float_t *)&tmp);
#else
    Ls_d = EST_getLs_d_H(obj->estHandle);

    Ls_q = EST_getLs_q_H(obj->estHandle);

    Rs = EST_getRs_Ohm(obj->estHandle);
#endif

    RoverLs_d = Rs / Ls_d;
    Kp_Id     = _IQ((0.25 * Ls_d * fullScaleCurrent) / (ctrlPeriod_sec * fullScaleVoltage));
    Ki_Id     = _IQ(RoverLs_d * ctrlPeriod_sec);

    RoverLs_q = Rs / Ls_q;
    Kp_Iq     = _IQ((0.25 * Ls_q * fullScaleCurrent) / (ctrlPeriod_sec * fullScaleVoltage));
    Ki_Iq     = _IQ(RoverLs_q * ctrlPeriod_sec);

    Kd = _IQ(0.0);

    // set the Id controller gains
    PID_setKi(obj->pidHandle_Id, Ki_Id);
    CTRL_setGains(handle, CTRL_Type_PID_Id, Kp_Id, Ki_Id, Kd);

    // set the Iq controller gains
    PID_setKi(obj->pidHandle_Iq, Ki_Iq);
    CTRL_setGains(handle, CTRL_Type_PID_Iq, Kp_Iq, Ki_Iq, Kd);

    return;
}    // end of calcPIgains() function
#endif

//! \brief     Computes the scale factor needed to convert from torque created by Ld, Lq, Id and Iq, from per unit to Nm
//!
_iq USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(USER_Params *pUserParams) {
    float_t FullScaleInductance = (pUserParams->iqFullScaleVoltage_V / (pUserParams->iqFullScaleCurrent_A * pUserParams->voltageFilterPole_rps));
    float_t FullScaleCurrent    = (pUserParams->iqFullScaleCurrent_A);
    float_t lShift              = ceil(log(pUserParams->motor_Ls_d / (0.7 * FullScaleInductance)) / log(2.0));

    return (_IQ(FullScaleInductance * FullScaleCurrent * FullScaleCurrent * pUserParams->motor_numPolePairs * 1.5 * pow(2.0, lShift)));
}    // end of USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf() function

//! \brief     Computes the scale factor needed to convert from torque created by flux and Iq, from per unit to Nm
//!
_iq USER_computeTorque_Flux_Iq_pu_to_Nm_sf(USER_Params *pUserParams) {
    float_t FullScaleFlux    = (pUserParams->iqFullScaleVoltage_V / (float_t)pUserParams->estFreq_Hz);
    float_t FullScaleCurrent = (pUserParams->iqFullScaleCurrent_A);
    float_t maxFlux          = (pUserParams->motor_ratedFlux * ((pUserParams->motor_type == MOTOR_Type_Induction) ? 0.05 : 0.7));
    float_t lShift           = -ceil(log(FullScaleFlux / maxFlux) / log(2.0));

    return (_IQ(FullScaleFlux / (2.0 * MATH_PI) * FullScaleCurrent * pUserParams->motor_numPolePairs * 1.5 * pow(2.0, lShift)));
}    // end of USER_computeTorque_Flux_Iq_pu_to_Nm_sf() function

//! \brief     Computes the scale factor needed to convert from per unit to Wb
//!
_iq USER_computeFlux_pu_to_Wb_sf(USER_Params *pUserParams) {
    float_t FullScaleFlux = (pUserParams->iqFullScaleVoltage_V / (float_t)pUserParams->estFreq_Hz);
    float_t maxFlux       = (pUserParams->motor_ratedFlux * ((pUserParams->motor_type == MOTOR_Type_Induction) ? 0.05 : 0.7));
    float_t lShift        = -ceil(log(FullScaleFlux / maxFlux) / log(2.0));

    return (_IQ(FullScaleFlux / (2.0 * MATH_PI) * pow(2.0, lShift)));
}    // end of USER_computeFlux_pu_to_Wb_sf() function

//! \brief     Computes the scale factor needed to convert from per unit to V/Hz
//!
_iq USER_computeFlux_pu_to_VpHz_sf(USER_Params *pUserParams) {
    float_t FullScaleFlux = (pUserParams->iqFullScaleVoltage_V / (float_t)pUserParams->estFreq_Hz);
    float_t maxFlux       = (pUserParams->motor_ratedFlux * ((pUserParams->motor_type == MOTOR_Type_Induction) ? 0.05 : 0.7));
    float_t lShift        = -ceil(log(FullScaleFlux / maxFlux) / log(2.0));

    return (_IQ(FullScaleFlux * pow(2.0, lShift)));
}    // end of USER_computeFlux_pu_to_VpHz_sf() function

//! \brief     Computes Flux in Wb or V/Hz depending on the scale factor sent as parameter
//!
_iq USER_computeFlux(CTRL_Handle handle, const _iq sf) {
    CTRL_Obj *obj = (CTRL_Obj *)handle;

    return (_IQmpy(EST_getFlux_pu(obj->estHandle), sf));
}    // end of USER_computeFlux() function

//! \brief     Computes Torque in Nm
//!
_iq USER_computeTorque_Nm(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf) {
    CTRL_Obj *obj = (CTRL_Obj *)handle;

    _iq Flux_pu            = EST_getFlux_pu(obj->estHandle);
    _iq Id_pu              = PID_getFbackValue(obj->pidHandle_Id);
    _iq Iq_pu              = PID_getFbackValue(obj->pidHandle_Iq);
    _iq Ld_minus_Lq_pu     = _IQ30toIQ(EST_getLs_d_pu(obj->estHandle) - EST_getLs_q_pu(obj->estHandle));
    _iq Torque_Flux_Iq_Nm  = _IQmpy(_IQmpy(Flux_pu, Iq_pu), torque_Flux_sf);
    _iq Torque_Ls_Id_Iq_Nm = _IQmpy(_IQmpy(_IQmpy(Ld_minus_Lq_pu, Id_pu), Iq_pu), torque_Ls_sf);
    _iq Torque_Nm          = Torque_Flux_Iq_Nm + Torque_Ls_Id_Iq_Nm;

    return (Torque_Nm);
}    // end of USER_computeTorque_Nm() function

_iq USER_computeTorque_Nm_S(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf, USER_Torque *pTorque) {
    CTRL_Obj *obj = (CTRL_Obj *)handle;

    pTorque->Flux_pu = EST_getFlux_pu(obj->estHandle);
    pTorque->Id_pu = PID_getFbackValue(obj->pidHandle_Id);
    pTorque->Iq_pu = PID_getFbackValue(obj->pidHandle_Iq);
    pTorque->Ld_minus_Lq_pu = _IQ30toIQ(EST_getLs_d_pu(obj->estHandle) - EST_getLs_q_pu(obj->estHandle));
    pTorque->Torque_Flux_Iq_Nm = _IQmpy(_IQmpy(pTorque->Flux_pu, pTorque->Iq_pu), torque_Flux_sf);
    pTorque->Torque_Ls_Id_Iq_Nm = _IQmpy(_IQmpy(_IQmpy(pTorque->Ld_minus_Lq_pu, pTorque->Id_pu), pTorque->Iq_pu), torque_Ls_sf);
    pTorque->Torque_Nm = pTorque->Torque_Flux_Iq_Nm + pTorque->Torque_Ls_Id_Iq_Nm;

    return pTorque->Torque_Nm;
}    // end of USER_computeTorque_Nm_S() function

//! \brief     Computes Torque in Nm
//!
_iq USER_computeTorque_lbin(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf) {
    CTRL_Obj *obj = (CTRL_Obj *)handle;

    _iq Flux_pu            = EST_getFlux_pu(obj->estHandle);
    _iq Id_pu              = PID_getFbackValue(obj->pidHandle_Id);
    _iq Iq_pu              = PID_getFbackValue(obj->pidHandle_Iq);
    _iq Ld_minus_Lq_pu     = _IQ30toIQ(EST_getLs_d_pu(obj->estHandle) - EST_getLs_q_pu(obj->estHandle));
    _iq Torque_Flux_Iq_Nm  = _IQmpy(_IQmpy(Flux_pu, Iq_pu), torque_Flux_sf);
    _iq Torque_Ls_Id_Iq_Nm = _IQmpy(_IQmpy(_IQmpy(Ld_minus_Lq_pu, Id_pu), Iq_pu), torque_Ls_sf);
    _iq Torque_Nm          = Torque_Flux_Iq_Nm + Torque_Ls_Id_Iq_Nm;

    return (_IQmpy(Torque_Nm, _IQ(MATH_Nm_TO_lbin_SF)));
}    // end of USER_computeTorque_lbin() function

// end of file
