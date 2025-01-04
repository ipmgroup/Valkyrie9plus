#ifndef _MAIN_H_
#define _MAIN_H_
/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

//! \file   solutions/instaspin_foc/src/main.h
//! \brief Defines the structures, global initialization, and functions used in MAIN
//!
//! (C) Copyright 2011, Texas Instruments, Inc.

// **************************************************************************
// the includes

// modules
#include "sw/modules/est/src/32b/est.h"
#include "sw/modules/fem/src/32b/fem.h"
#include "sw/modules/fw/src/32b/fw.h"
#include "sw/modules/math/src/32b/math.h"
#include "sw/modules/memCopy/src/memCopy.h"
#include "sw/modules/svgen/src/32b/svgen_current.h"

#include "flyingStart.h"
#include "sw/modules/cpu_time/src/32b/cpu_time.h"
#include "sw/modules/hallbldc/src/32b/hallbldc.h"

#ifdef EEPROM_EMULATION
#include "libraries/flash_api/f2806x/include/Flash2806x_API_Library.h"
#endif

// drivers

// platforms
#include "ctrl.h"
#include "hal.h"
#include "user.h"

// **************************************************************************
// the defines

//! \brief Defines the number of main iterations before global variables are updated
//!
#define NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE 1

#define MOTOR_Vars_INIT                                                  \
    {                                                                    \
        .Flag_enableSys            = false,                              \
        .Flag_Run_Identify         = false,                              \
        .Flag_MotorIdentified      = false,                              \
        .Flag_enableForceAngle     = true,                               \
        .Flag_enableFieldWeakening = false,                              \
        .Flag_enableRsRecalc       = false,                              \
        .Flag_enableUserParams     = true,                               \
        .Flag_enableOffsetcalc     = true,                               \
        .Flag_enablePowerWarp      = false,                              \
        .Flag_enableSpeedCtrl      = false,                              \
        .Flag_enableRun            = false,                              \
        .Flag_RunState             = false,                              \
        .Flag_enableFlyingStart    = false,                              \
        .CtrlState                 = CTRL_State_Idle,                    \
        .EstState                  = EST_State_Idle,                     \
        .UserErrorCode             = USER_ErrorCode_NoError,             \
        .CtrlVersion               = {0, CTRL_TargetProc_Unknown, 0, 0}, \
        .IdRef_A                   = _IQ(0.0),                           \
        .IqRef_A                   = _IQ(0.0),                           \
        .SpeedRef_pu               = _IQ(0.0),                           \
        .SpeedRef_krpm             = _IQ(0.1),                           \
        .SpeedTraj_krpm            = _IQ(0.0),                           \
        .MaxAccel_krpmps           = _IQ(0.2),                           \
        .Speed_krpm                = _IQ(0.0),                           \
        .SvgenMaxModulation_ticks  = 400,                                \
        .Flux_Wb                   = _IQ(0.0),                           \
        .Torque_Nm                 = _IQ(0.0),                           \
        .MagnCurr_A                = 0.0,                                \
        .Rr_Ohm                    = 0.0,                                \
        .Rs_Ohm                    = 0.0,                                \
        .RsOnLine_Ohm              = 0.0,                                \
        .Lsd_H                     = 0.0,                                \
        .Lsq_H                     = 0.0,                                \
        .Flux_VpHz                 = 0.0,                                \
        .ipd_excFreq_Hz            = 0.0,                                \
        .ipd_Kspd                  = _IQ(0.0),                           \
        .ipd_excMag_coarse_pu      = _IQ(0.0),                           \
        .ipd_excMag_fine_pu        = _IQ(0.0),                           \
        .ipd_waitTime_coarse_sec   = 0.0,                                \
        .ipd_waitTime_fine_sec     = 0.0,                                \
        .Kp_spd                    = _IQ(0.0),                           \
        .Ki_spd                    = _IQ(0.0),                           \
        .Kp_Idq                    = _IQ(0.0),                           \
        .Ki_Idq                    = _IQ(0.0),                           \
        .Vd                        = _IQ(0.0),                           \
        .Vq                        = _IQ(0.0),                           \
        .Vs                        = _IQ(0.0),                           \
        .VdcBus_kV                 = _IQ(0.0),                           \
        .Id_A                      = _IQ(0.0),                           \
        .Iq_A                      = _IQ(0.0),                           \
        .Is_A                      = _IQ(0.0),                           \
        .I_bias                    = {0, 0, 0},                          \
        .V_bias                    = {0, 0, 0},                          \
        .SpeedSet_krpm             = _IQ(0.6),                           \
        .angle_sen_pu              = _IQ(0.0),                           \
        .angle_est_pu              = _IQ(0.0),                           \
        .speed_sen_pu              = _IQ(0.0),                           \
        .speed_est_pu              = _IQ(0.0),                           \
        .speedHigh_hall2fast_pu    = _IQ(0.0),                           \
        .speedLow_hall2fast_pu     = _IQ(0.0),                           \
        .IdSet_A                   = _IQ(0.0),                           \
        .IqSet_A                   = _IQ(0.0),                           \
        .IdRef_pu                  = _IQ(0.0),                           \
        .IqRef_pu                  = _IQ(0.0),                           \
        .Flag_nFault               = 0,                                  \
        .Flag_nFault_OST           = 0,                                  \
        .Flag_nFaultDroneCan       = 0,                                  \
        .Flag_Overheat             = 0,                                  \
        .FaultReg                  = {0, 0, 0, 0},                       \
        .CpuUsagePercentageMin     = 0,                                  \
        .CpuUsagePercentageAvg     = 0,                                  \
        .CpuUsagePercentageMax     = 0                                   \
    }

// .OverModulation            = _IQ(nUSER_MAX_VS_MAG_PU),           \
// .VdcBus_kV                 = _IQ(0.8 * nUSER_MAX_VS_MAG_PU),     \
// .RsOnLineCurrent_A         = _IQ(0.1 * nUSER_MOTOR_MAX_CURRENT)  \

typedef struct _MOTOR_Vars_t_ {
    bool Flag_enableSys;
    bool Flag_Run_Identify;
    bool Flag_MotorIdentified;
    bool Flag_enableForceAngle;
    bool Flag_enableFieldWeakening;
    bool Flag_enableRsRecalc;
    bool Flag_enableUserParams;
    bool Flag_enableOffsetcalc;
    bool Flag_enablePowerWarp;
    bool Flag_enableSpeedCtrl;

    bool Flag_enableRun;
    bool Flag_RunState;
    bool Flag_enableFlyingStart;

    CTRL_State_e CtrlState;
    EST_State_e  EstState;

    USER_ErrorCode_e UserErrorCode;

    CTRL_Version CtrlVersion;

    _iq IdRef_A;
    _iq IqRef_A;
    _iq SpeedRef_pu;
    _iq SpeedRef_krpm;
    _iq SpeedTraj_krpm;
    _iq MaxAccel_krpmps;
    _iq Speed_krpm;
    _iq OverModulation;
    _iq RsOnLineCurrent_A;
    _iq SvgenMaxModulation_ticks;
    _iq Flux_Wb;
    _iq Torque_Nm;

    float_t MagnCurr_A;
    float_t Rr_Ohm;
    float_t Rs_Ohm;
    float_t RsOnLine_Ohm;
    float_t Lsd_H;
    float_t Lsq_H;
    float_t Flux_VpHz;

    float_t ipd_excFreq_Hz;
    _iq     ipd_Kspd;
    _iq     ipd_excMag_coarse_pu;
    _iq     ipd_excMag_fine_pu;
    float   ipd_waitTime_coarse_sec;
    float   ipd_waitTime_fine_sec;

    _iq Kp_spd;
    _iq Ki_spd;

    _iq Kp_Idq;
    _iq Ki_Idq;

    _iq Vd;
    _iq Vq;
    _iq Vs;
    _iq VsRef;
    _iq VdcBus_kV;

    _iq Id_A;
    _iq Iq_A;
    _iq Is_A;

    MATH_vec3 I_bias;
    MATH_vec3 V_bias;

    _iq SpeedSet_krpm;

    _iq angle_sen_pu;
    _iq angle_est_pu;
    _iq speed_sen_pu;
    _iq speed_est_pu;

    _iq speedHigh_hall2fast_pu;
    _iq speedLow_hall2fast_pu;
    _iq IdSet_A;
    _iq IqSet_A;
    _iq IdRef_pu;
    _iq IqRef_pu;

    float_t TempSenDegCelsius;
    float_t Throttle;

    bool     Flag_nFault;
    bool     Flag_nFault_OST;
    bool     Flag_nFaultDroneCan;
    bool     Flag_Overheat;
    uint16_t FaultReg[4];
    float_t  CpuUsagePercentageMin;
    float_t  CpuUsagePercentageAvg;
    float_t  CpuUsagePercentageMax;
} MOTOR_Vars_t;

static inline void updateMotorVars(USER_Params *pUserParams, volatile MOTOR_Vars_t *pMotorVars) {
    pMotorVars->OverModulation    = _IQ(pUserParams->maxVsMag_pu);
    pMotorVars->RsOnLineCurrent_A = _IQ(0.1 * pUserParams->maxCurrent);
    pMotorVars->VsRef             = _IQ(0.8 * pUserParams->maxVsMag_pu);
    pMotorVars->Kp_spd            = pUserParams->kp_spd;
    pMotorVars->Ki_spd            = pUserParams->ki_spd;
}

typedef struct {
    uint32_t CBC;
    uint32_t OST;
} TzCnt_t;

typedef enum
{
    PHASE_READING,
    PHASE_RESETTING,
    PHASE_GO
} FaultPhase_e;

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

//! \brief The main interrupt service (ISR) routine
//!
interrupt void mainISR(void);

void runCurrentReconstruction(void);

void runFieldWeakening(void);

void runOffsetsCalculation(void);

void runSetTrigger(void);

//! \brief     Updates the global motor variables
//!
void updateGlobalVariables_motor(CTRL_Handle handle);

//! \brief     Updates the global variables
//!
void updateGlobalVariables(EST_Handle handle);

//! \brief     Updates version 1p6 of library
//!
void softwareUpdate1p6(EST_Handle handle);

//! \brief     Reset Ls Q format to a higher value when Ls identification starts
//!
void CTRL_resetLs_qFmt(CTRL_Handle handle, const uint_least8_t qFmt);

//! \brief     Recalculate Kp and Ki gains to fix the R/L limitation of 2000.0 and Kp limitation of 0.11
//! \brief     as well as recalculates gains based on estimator state to allow low inductance pmsm to id
//!
void recalcKpKiPmsm(CTRL_Handle handle);

//! \brief     Recalculate Kp and Ki gains to fix the R/L limitation of 2000.0 and Kp limitation of 0.11
//!
void recalcKpKi(CTRL_Handle handle);

//! \brief     Calculates the maximum qFmt value for Ls identification, to get a more accurate Ls per unit
//!
void CTRL_calcMax_Ls_qFmt(CTRL_Handle handle, uint_least8_t *p_qFmt);

//! \brief     Updates Iq reference and also sets the right sign to the speed reference for correct force angle
//!
void updateIqRef(CTRL_Handle handle);

//! \brief     Updates Kp and Ki gains in the controller object
//!
void updateKpKiGains(CTRL_Handle handle);

//! \brief     Runs Rs online
//!
void runRsOnLine(CTRL_Handle handle);

//! \brief      Runs Rs Online for lab11a; function prototype changed to expect EST_Handle object
//!             instead of CTRL_Handle object (lab11a features no CTRL object)
//!
void updateRsOnLine(EST_Handle handle);

//! \brief     Updates CPU usage
//!
void updateCPUusage(void);

//! \brief     Set electrical frequency limit to zero while identifying an induction motor
//!
void setFeLimitZero(CTRL_Handle handle);

//! \brief     Calculates Dir_qFmt for ACIM
//!
void acim_Dir_qFmtCalc(CTRL_Handle handle);

//! \brief     Sets up the Clarke transform for current
//!
void setupClarke_I(CLARKE_Handle, const uint_least8_t);

//! \brief     Sets up the Clarke transform for voltage
//!
void setupClarke_V(CLARKE_Handle, const uint_least8_t);

_iq angleDelayComp(const _iq fm_pu, const _iq angleUncomp_pu);

CTRL_Handle CTRL_init(void *pMemory, const size_t numBytes);

void EST_getIdq_pu(EST_Handle handle, MATH_vec2 *pIdq_pu);

#if !defined(FAST_ROM_V1p6)
void EST_setEstParams(EST_Handle handle, USER_Params *pUserParams);
void EST_setupEstIdleState(EST_Handle handle);
#endif

EST_Handle EST_init(void *pMemory, const size_t numBytes);

//! \brief     Runs PowerWarp
//! \param[in] handle     The estimator (EST) handle
//! \param[in] Id_int_pu  The intermediate value along the Id trajectory in pu
//! \param[in] Iq_pu      The measured Iq value in pu
//! \return    The target value for the Id trajectory in pu
_iq EST_runPowerWarp(EST_Handle handle, const _iq Id_int_pu, const _iq Iq_pu);

_iq getAbsElecAngle(const _iq angle_pu);

_iq getAbsMechAngle(_iq *pAngle_mech_poles, _iq *pAngle_z1_pu, const _iq angle_pu);

//! \brief     motor run control for flying start
//!
void motor_RunCtrl(CTRL_Handle handle);

uint64_t micros64(void);

int measureTemperatureC();

//@} //defgroup
#endif    // end of _MAIN_H_ definition
