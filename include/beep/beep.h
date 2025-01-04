#ifndef BEEP_H
#define BEEP_H

#include "hal.h"
#include "sw/modules/math/src/32b/math.h"
#include <stdbool.h>
#include <stdint.h>
#include "main.h"

typedef struct _Beep_Obj_ {
    bool           beepStart;
    bool           beepActive;
    uint16_t       tempo;
    uint16_t       length;
    uint16_t       toneCounter;
    uint16_t       beepCnt;
    uint16_t       beepAlt;
    uint16_t       durationCnt;
    uint16_t       toneFreqHz;
    uint_least32_t toneDurationMs;

    _iq MaxDuration;
    _iq BeepOffset;
    _iq BeepCurrent;
} Beep_Obj;

typedef struct _Beep_Obj_ *Beep_Handle;

// void Beep_init(Beep_Handle handle);
Beep_Handle Beep_init(void *pMemory, const size_t numBytes);
void        BeepOffset(Beep_Handle handle, USER_Params gUserParams, MOTOR_Vars_t gMotorVars);
void        BeepStop(Beep_Handle handle, HAL_Handle halHandle, USER_Params gUserParams);
void        BeepSet(Beep_Handle handle, USER_Params gUserParams, HAL_PwmData_t *gPwmData);
void        BeepStart(Beep_Handle handle, const uint16_t *gFreq, const uint16_t *gBeats);

#endif    // BEEP_H
