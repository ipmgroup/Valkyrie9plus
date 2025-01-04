#include "beep.h"
#include "note.h"


Beep_Handle Beep_init(void *pMemory, const size_t numBytes) {

    Beep_Handle bpHandle;

    if (numBytes < sizeof(Beep_Obj))
        return (Beep_Handle)NULL;

    // assign the handle
    bpHandle = (Beep_Handle)pMemory;

    bpHandle->beepStart   = false;
    bpHandle->beepActive  = false;
    bpHandle->tempo       = 1;
    bpHandle->length      = BEEP_LENGTH;
    bpHandle->toneCounter = 0;
    bpHandle->BeepCurrent = BEEP_CURRENT;
    bpHandle->MaxDuration = _IQ(0.0);
    bpHandle->BeepOffset  = _IQ(0.0);
    bpHandle->beepCnt     = 0;
    bpHandle->beepAlt     = 0;
    bpHandle->durationCnt = 0;

    return (bpHandle);
}

void BeepOffset(Beep_Handle handle, USER_Params gUserParams, MOTOR_Vars_t gMotorVars) {
    Beep_Obj *bp = (Beep_Obj *)handle;
    if (bp->beepStart) {
        bp->MaxDuration = 1000 * _IQdiv((gUserParams.motor_Ls_d * bp->BeepCurrent), (gMotorVars.VdcBus_kV));
        bp->BeepOffset  = _IQdiv(bp->MaxDuration, _IQ(gUserParams.pwmPeriod_usec));
    }
}

void BeepStop(Beep_Handle handle, HAL_Handle halHandle, USER_Params gUserParams) {
    Beep_Obj *bp = (Beep_Obj *)handle;
    if (bp->beepActive && ++bp->durationCnt >= (uint_least32_t)((gUserParams.isrFreq_Hz * bp->toneDurationMs)/1000.0f)) {
        HAL_disablePwm(halHandle);
        bp->durationCnt = 0;
        bp->beepActive  = 0;
    }
    else {
        HAL_enablePwm(halHandle);
    }
}

void BeepSet(Beep_Handle handle, USER_Params gUserParams, HAL_PwmData_t *gPwmData) {
    Beep_Obj *bp = (Beep_Obj *)handle;
    if (bp->beepActive && ++bp->beepCnt >= (uint_least32_t)((gUserParams.isrFreq_Hz / (float_t )bp->toneFreqHz) / 2.0f)) {
        if (bp->beepAlt) {
            gPwmData->Tabc.value[0] = bp->BeepOffset;
            gPwmData->Tabc.value[1] = -bp->BeepOffset;
            gPwmData->Tabc.value[2] = -bp->BeepOffset;
        }
        else {
            gPwmData->Tabc.value[0] = -bp->BeepOffset;
            gPwmData->Tabc.value[1] = bp->BeepOffset;
            gPwmData->Tabc.value[2] = bp->BeepOffset;
        }

        bp->beepAlt = !bp->beepAlt;
        bp->beepCnt = 0;
    }
}

void BeepStart(Beep_Handle handle, const uint16_t *gFreq, const uint16_t *gBeats) {
    Beep_Obj *bp = (Beep_Obj *)handle;
    if (bp->beepStart && !bp->beepActive) {
        bp->toneFreqHz     = gFreq[bp->toneCounter];
        // bp->toneDurationMs = (1000 / gBeats[bp->toneCounter]) * bp->tempo;
        bp->toneDurationMs = gBeats[bp->toneCounter] / bp->tempo;

        bp->beepActive = 1;
        bp->toneCounter++;

        if (bp->toneCounter == bp->length) {
            bp->toneCounter = 0;
            bp->beepStart   = 0;
        }
    }
}
