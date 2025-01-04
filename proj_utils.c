#include <proj_utils.h>

#ifdef ECAN
#include <can.h>
#include <fifo.h>
#endif
#include "canard.h"
#include "sw/drivers/wdog/src/32b/f28x/f2806x/wdog.h"

uint64_t uptime_s = 0;

#ifdef DRONECAN
uint16_t transfersReceived = 0;
#endif  // DRONECAN

void initializeFIFO(FIFO_ID_Obj *fifo) {
    memset(fifo, 0, sizeof(FIFO_ID_Obj));
}

// void MOTOR_updateUserParams(volatile MOTOR_Vars_t *pMotorVars, USER_Params *pUserParams) {
//     // Set values that depend on user params.
//     pMotorVars->OverModulation    = _IQ(pUserParams->maxVsMag_pu);
//     pMotorVars->RsOnLineCurrent_A = _IQ(0.1 * pUserParams->maxCurrent);
//     pMotorVars->VsRef             = _IQ(0.8 * pUserParams->maxVsMag_pu);
//     pMotorVars->Kp_spd            = pUserParams->kp_spd;
//     pMotorVars->Ki_spd            = pUserParams->ki_spd;
// }

void MOTOR_setUserBaseFromActive(USER_Params_FLASH *pUserBaseParams, volatile MOTOR_Vars_t *pMotorVars) {
    pUserBaseParams->motor_Rr        = pMotorVars->Rr_Ohm;
    pUserBaseParams->motor_Rs        = pMotorVars->Rs_Ohm;
    pUserBaseParams->motor_Ls_d      = pMotorVars->Lsd_H;
    pUserBaseParams->motor_Ls_q      = pMotorVars->Lsq_H;
    pUserBaseParams->motor_ratedFlux = pMotorVars->Flux_VpHz;
    pUserBaseParams->IdRated         = pMotorVars->MagnCurr_A;
}

void MOTOR_setActiveFromUserBase(volatile MOTOR_Vars_t *pMotorVars, const USER_Params_FLASH *pUserBaseParams) {
    pMotorVars->Rr_Ohm     = pUserBaseParams->motor_Rr;         // Copy rotor resistance
    pMotorVars->Rs_Ohm     = pUserBaseParams->motor_Rs;         // Copy stator resistance
    pMotorVars->Lsd_H      = pUserBaseParams->motor_Ls_d;       // Copy d-axis inductance
    pMotorVars->Lsq_H      = pUserBaseParams->motor_Ls_q;       // Copy q-axis inductance
    pMotorVars->Flux_VpHz  = pUserBaseParams->motor_ratedFlux;  // Copy rated flux in VpHz
    pMotorVars->MagnCurr_A = pUserBaseParams->IdRated;          // Copy rated magnetizing current
}

// void resetDevice(HAL_Handle halHandle) {
//     WDOG_disable(halHandle->wdogHandle);
//     EALLOW;
//     halHandle->wdogHandle->SCSR = 0x0;  // Set wdog overflow for device reset.
//     halHandle->wdogHandle->WDCR = 0x0;  // Set WDCHK 000 to trigger reset.
//     EDIS;
//     WDOG_enable(halHandle->wdogHandle);
//     for (;;);
// }

void resetDevice(HAL_Handle halHandle) {
    if (halHandle == NULL || halHandle->wdogHandle == NULL) {
        return;
    }
    WDOG_disable(halHandle->wdogHandle);
    EALLOW;

    if ((halHandle->wdogHandle->WDCR & 0x0080) == 0) {
        halHandle->wdogHandle->SCSR = 0x0;
        halHandle->wdogHandle->WDCR = (0x00 << 3) | 0x005A;
        EDIS;
        WDOG_enable(halHandle->wdogHandle);
        for (;;);
    } else {
        // Если Watchdog не отключен, выйдем из функции и зафиксируем ошибку
        // Можно добавить логирование или другую обработку ошибок
        EDIS;
        return;
    }
}

#ifdef ECAN
void ECAN_setup(HAL_Handle halHandle, ECAN_Mailbox *mailbox, FIFO_ID_Obj ECAN_rxFIFO_ID, FIFO_ID_Obj ECAN_txFIFO_ID, uint16_t nodeID) {
    // for (int i = 0; i <= 15; i++) {
    for (ECAN_MailBox_e mailbox = MailBox0; mailbox <= MailBox15; mailbox++) {
        ECAN_configMailbox(
            halHandle->ecanaHandle, mailbox, nodeID, Enable_Mbox, Tx_Dir, Extended_ID, DLC_8,
            Overwrite_on,
            LAMI_0, Mask_is_used, 0x1FFFFFFF);

        // ECAN_setTx_Priority(halHandle->ecanaHandle, i, 15 - i);
    }

    // for (int i = 16; i <= 31; i++) {
    for (ECAN_MailBox_e mailbox = MailBox16; mailbox <= MailBox31; mailbox++) {
        ECAN_configMailbox(
            halHandle->ecanaHandle, mailbox, nodeID, Enable_Mbox, Rx_Dir, Extended_ID, DLC_8,
            (mailbox == MailBox16) ? Overwrite_on : Overwrite_off,
            // Overwrite_off,
            LAMI_0, Mask_is_used, 0x1FFFFFFF);
    }
    //                             TX_max    TX_min    RX_max     RX_min
    ECAN_initMailboxUse(mailbox, MailBox15, MailBox0, MailBox31, MailBox16);

    //  ECAN_SelfTest(halHandle->ecanaHandle, Self_test_mode);
    ECAN_SelfTest(halHandle->ecanaHandle, Normal_mode);

#ifdef ECAN_ADVANCED_BUFFERING
    // RX. Interrupt when the last mailbox receives something (i.e. when all mailboxes are full).
    // ECAN_configMailbox_Int(halHandle->ecanaHandle, MailBox16, Int_enable, Line1);
    ECAN_configMailbox_Int(halHandle->ecanaHandle, MailBox31, Int_enable, Line1);
    ECAN_GlobalInt_Mask(halHandle->ecanaHandle, INT0_ENABLE | INT1_ENABLE);
#endif
    FIFO_FLUSH(ECAN_rxFIFO_ID);
    FIFO_FLUSH(ECAN_txFIFO_ID);
}

// ECAN_rxFIFO_status ECAN_getRxMsg(HAL_Handle halHandle, FIFO_ID_Obj *pECAN_rxFIFO_ID, ECAN_Mailbox *pECAN_mailbox, MSG_t *output_frame) {
//     bool newMessagesAvailable = ECAN_checkMail(halHandle->ecanaHandle);

//     if (!newMessagesAvailable && FIFO_IS_EMPTY(*pECAN_rxFIFO_ID)) {
//         return no_new_messages;
//     }

//     if (newMessagesAvailable) {
//         ECAN_getMsgFIFO_ID_N(halHandle->ecanaHandle, pECAN_mailbox, pECAN_rxFIFO_ID);
//     }

//     if (!(FIFO_IS_EMPTY(*pECAN_rxFIFO_ID))) {
//         *output_frame = FIFO_FRONT(*pECAN_rxFIFO_ID);
//         FIFO_POP(*pECAN_rxFIFO_ID);
//     }

//     if (!(FIFO_IS_EMPTY(*pECAN_rxFIFO_ID))) {
//         return fifo_not_empty;
//     }

//     return all_messages_read;
// }


#pragma CODE_SECTION(ECAN_getRxMsg, "ramfuncs");

ECAN_rxFIFO_status ECAN_getRxMsg(HAL_Handle halHandle, FIFO_ID_Obj *pECAN_rxFIFO_ID, ECAN_Mailbox *pECAN_mailbox, MSG_t *output_frame) {
    bool newMessagesAvailable = ECAN_checkMail(halHandle->ecanaHandle);
    if (!newMessagesAvailable && FIFO_IS_EMPTY(*pECAN_rxFIFO_ID)) {
        return no_new_messages;
    }

    if (newMessagesAvailable) {
        if (!ECAN_getMsgFIFO_ID_N(halHandle->ecanaHandle, pECAN_mailbox, pECAN_rxFIFO_ID)) {
        }
    }

    if (!FIFO_IS_EMPTY(*pECAN_rxFIFO_ID)) {
        *output_frame = FIFO_FRONT(*pECAN_rxFIFO_ID);
        FIFO_POP(*pECAN_rxFIFO_ID);
    }

    return FIFO_IS_EMPTY(*pECAN_rxFIFO_ID) ? all_messages_read : fifo_not_empty;
}

#endif  // ECAN

uint32_t usSinceLastTimerInterrupt(TIMER_Handle timerHandle, USER_Params *pUserParams) {
    // If this function is called when TIM == 0, the interrupt should happen before the calculation here,
    // meaning it should return 0 and not PRD+1/systemFreq_MHz.
    return timerHandle->TIM != 0 ? (timerHandle->PRD + 1 - timerHandle->TIM) / pUserParams->systemFreq_MHz : 0;
}

void pauseTimer(TIMER_Handle timerHandle) {
    timerHandle->TCR |= TIMER_TCR_TSS_BITS;
}

void continueTimer(TIMER_Handle timerHandle) {
    timerHandle->TCR &= ~TIMER_TCR_TSS_BITS;
}

uint64_t getUptime_s() {
    return uptime_s;
}

uint64_t getUptime_us(HAL_Handle halHandle, USER_Params *pUserParams) {
    TIMER_Handle timerHandle = halHandle->timerHandle[0];

    // Pause timer to prevent a race condition.
    pauseTimer(timerHandle);
    uint64_t usUptime = uptime_s * 1000000 + usSinceLastTimerInterrupt(timerHandle, pUserParams);
    continueTimer(timerHandle);

    return usUptime;
}

uint64_t getUptimeEstimate_us(HAL_Handle halHandle, USER_Params *pUserParams) {
    TIMER_Handle timerHandle = halHandle->timerHandle[0];

    uint64_t usUptime = uptime_s * 1000000 + usSinceLastTimerInterrupt(timerHandle, pUserParams);

    return usUptime;
}

#pragma CODE_SECTION(convertCanardFrameToC2000, "ramfuncs");

void convertCanardFrameToC2000(const CanardCANFrame *canardFrame, MSG_t *c2000Frame) {
    c2000Frame->msgID      = canardFrame->id;
    c2000Frame->dataLength = canardFrame->data_len > 8 ? 8 : canardFrame->data_len;
    c2000Frame->dataL      = 0;
    c2000Frame->dataH      = 0;

    for (int i = 0; i < 4 && i < canardFrame->data_len; i++) {
        c2000Frame->dataL |= (uint32_t)(canardFrame->data[i] & 0xFF) << ((3 - i) * 8);
    }

    for (int i = 4; i < 8 && i < canardFrame->data_len; i++) {
        c2000Frame->dataH |= (uint32_t)(canardFrame->data[i] & 0xFF) << ((7 - i) * 8);
    }
}

#pragma CODE_SECTION(convertC2000FrameToCanard, "ramfuncs");
void convertC2000FrameToCanard(const MSG_t *c2000Frame, CanardCANFrame *canardFrame) {
    canardFrame->id = c2000Frame->msgID;
    canardFrame->id |= CANARD_CAN_FRAME_EFF;
    canardFrame->data_len = (uint8_t)(c2000Frame->dataLength > 8 ? 8 : c2000Frame->dataLength);

    for (int i = 0; i < 8; i++) {
        canardFrame->data[i] = 0;
    }

    for (int i = 0; i < 4 && i < canardFrame->data_len; i++) {
        canardFrame->data[i] = (uint8_t)((c2000Frame->dataL >> ((3 - i) * 8)) & 0xFF);
    }

    for (int i = 4; i < 8 && i < canardFrame->data_len; i++) {
        canardFrame->data[i] = (uint8_t)((c2000Frame->dataH >> ((7 - i) * 8)) & 0xFF);
    }
}

uint32_t packFlagsIntoControlWord(MOTOR_Vars_t *gMotorVars) {
    uint32_t control_word = 0;

    control_word |= (gMotorVars->Flag_enableSys & 0x01) << 0;
    control_word |= (gMotorVars->Flag_Run_Identify & 0x01) << 1;
    control_word |= (gMotorVars->Flag_enableFieldWeakening & 0x01) << 2;
    control_word |= (gMotorVars->Flag_enableForceAngle & 0x01) << 3;
    control_word |= (gMotorVars->Flag_enableRsRecalc & 0x01) << 4;
    control_word |= (gMotorVars->Flag_enablePowerWarp & 0x01) << 5;
    control_word |= (gMotorVars->Flag_enableUserParams & 0x01) << 6;

    return control_word;
}

void extractFlagsFromControlWord(USER_Params_FLASH *settings, volatile MOTOR_Vars_t *gMotorVars) {
    gMotorVars->Flag_enableSys            = (settings->control_word & 0x01);
    gMotorVars->Flag_Run_Identify         = (settings->control_word >> 1) & 0x01;
    gMotorVars->Flag_enableFieldWeakening = (settings->control_word >> 2) & 0x01;
    gMotorVars->Flag_enableForceAngle     = (settings->control_word >> 3) & 0x01;
    gMotorVars->Flag_enableRsRecalc       = (settings->control_word >> 4) & 0x01;
    gMotorVars->Flag_enablePowerWarp      = (settings->control_word >> 5) & 0x01;
    gMotorVars->Flag_enableUserParams     = (settings->control_word >> 6) & 0x01;
}
