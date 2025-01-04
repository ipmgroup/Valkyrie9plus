#ifndef PROJ_UTILS_H_
#define PROJ_UTILS_H_

#include <main.h>
#include "canard.h"
#include "dronecan_msgs.h"

extern uint64_t uptime_s;

#ifdef ECAN
typedef enum ECAN_rxFIFO_status {
    no_new_messages   = 0,
    all_messages_read = 1,
    fifo_not_empty    = 2
} ECAN_rxFIFO_status;
#endif

// void MOTOR_updateUserParams(volatile MOTOR_Vars_t *pMotorVars, USER_Params *pUserParams);
void MOTOR_setUserBaseFromActive(USER_Params_FLASH *pUserBaseParams, volatile MOTOR_Vars_t *pMotorVars);
void MOTOR_setActiveFromUserBase(volatile MOTOR_Vars_t *pMotorVars, const USER_Params_FLASH *pUserBaseParams);
#ifdef ECAN
void ECAN_setup(HAL_Handle halHandle, ECAN_Mailbox *ECAN_Mailbox, FIFO_ID_Obj ECAN_rxFIFO_ID, FIFO_ID_Obj ECAN_txFIFO_ID, uint16_t nodeID);
// Reads all frames from the registers to the FIFO and writes the first frame received to the given pointer.
ECAN_rxFIFO_status ECAN_getRxMsg(HAL_Handle halHandle, FIFO_ID_Obj *ECAN_rxFIFO_ID, ECAN_Mailbox *ECAN_mailbox, MSG_t *frame);
#endif

// Gets the uptime of the device in s.
uint64_t getUptime_s();
// Gets the uptime of the device in µs. Assumes timer0 is used for timekeeping.
// While executed, the timer is stopped, so frequently calling this will cause the time measurement to drift.
uint64_t getUptime_us(HAL_Handle halHandle, USER_Params *pUserParams);
// Same as getUptime_us(), but does not stop the timer when called.
// This means it has no impact on timer drift, but the value will sometimes be off by up to 1 second, depending on race conditions.
uint64_t getUptimeEstimate_us(HAL_Handle halHandle, USER_Params *pUserParams);

void initializeFIFO(FIFO_ID_Obj *fifo);
void resetDevice(HAL_Handle halHandle);
void convertCanardFrameToC2000(const CanardCANFrame *canardFrame, MSG_t *c2000Frame);
void convertC2000FrameToCanard(const MSG_t *c2000Frame, CanardCANFrame *canardFrame);

uint32_t packFlagsIntoControlWord(MOTOR_Vars_t *gMotorVars);
void     extractFlagsFromControlWord(USER_Params_FLASH *settings, volatile MOTOR_Vars_t *gMotorVars);

#endif /* PROJ_UTILS_H_ */
