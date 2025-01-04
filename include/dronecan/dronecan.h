#ifndef _DRONECAN_H_
#define _DRONECAN_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <main.h>

#include "canard.h"
#include "dronecan_msgs.h"
#include <fifo.h>
#include <proj_utils.h>

#define MICRO64() getUptimeEstimate_us(dc.pHalHandle, &gUserParams)

#ifndef SW_REV_MAJOR
#define SW_REV_MAJOR 1
#endif

#ifndef SW_REV_MINOR
#define SW_REV_MINOR 2
#endif

#ifndef HW_REV_MAJOR
#define HW_REV_MAJOR 3
#endif

#ifndef HW_REV_MINOR
#define HW_REV_MINOR 4
#endif

#ifndef GIT_HASH
#define GIT_HASH 0x11111111
#endif

#ifndef DRV_NAME
#define DRV_NAME "NONE"
#endif

#ifndef USER_MAX_PWM_FREQ_kHz
#define USER_MAX_PWM_FREQ_kHz (35.0)
#endif

#ifndef TX_DELAY
#define TX_DELAY 100
#endif

#define ESC_MID 4095

// #ifdef BOARD_HEADER
// #include BOARD_HEADER
// #else
// #error "BOARD_HEADER is not defined"
// #endif

#ifndef APP_NODE_NAME
#define APP_NODE_NAME "ExampleWM18"
#endif

#define UNIQUE_ID_LENGTH_BYTES 16

typedef enum
{
    DC_OK                           = 0,
    DC_ERROR_INVALID_ARGUMENT       = -2,
    DC_ERROR_OUT_OF_MEMORY          = -3,
    DC_ERROR_NODE_ID_NOT_SET        = -4,
    DC_ERROR_INTERNAL               = -9,
    DC_ERROR_RX_INCOMPATIBLE_PACKET = -10,
    DC_ERROR_RX_WRONG_ADDRESS       = -11,
    DC_ERROR_RX_NOT_WANTED          = -12,
    DC_ERROR_RX_MISSED_START        = -13,
    DC_ERROR_RX_WRONG_TOGGLE        = -14,
    DC_ERROR_RX_UNEXPECTED_TID      = -15,
    DC_ERROR_RX_SHORT_FRAME         = -16,
    DC_ERROR_RX_BAD_CRC             = -17
} DC_ErrorCode_e;

typedef enum
{
    T_NONE = 0,
    T_INT8,
    T_UINT8,
    T_INT16,
    T_UINT16,
    T_INT32,
    T_UINT32,
    T_FLOAT,
    T_IQ,
    T_BOOL,
    T_STRING
} var_type;

typedef union {
    int8_t   i8_val;
    uint8_t  u8_val;
    int16_t  i16_val;
    uint16_t u16_val;
    int32_t  i32_val;
    uint32_t u32_val;
    float    f_val;
    _iq      iq_val;
    bool     b_val;
    char    *str_val;
} param_value_t;

typedef enum
{
    PHASE_IDLE,
    PHASE_AKTIV,
    PHASE_COPYING,
    PHASE_CHECKING,
    PHASE_SAVING,
    PHASE_REBOOTING
} ProfileState;

typedef enum
{
    STATE_ENABLE_SYS,
    STATE_ENABLE_USER_PARAMS,
    STATE_RUN_IDENTIFY
} Start_t;

typedef struct {
    HAL_Handle         pHalHandle;
    FIFO_ID_Obj       *pECAN_rxFIFO_ID;
    ECAN_Mailbox      *pECAN_mailbox;
    ECAN_rxFIFO_status rx_fifo_status;

    uint64_t       next_1hz_service_at;
    uint64_t       next_50hz_service_at;
    uint64_t       next_DC_clear_service_at;
    uint16_t       peak_percent;
    MSG_t          tx_frame;
    MSG_t          rx_frame;
    CanardCANFrame CanardRx_frame;
    DC_ErrorCode_e CanardRxError;
    int16_t        RAWcmd;
    bool           Flag_enableSave;
    bool           Flag_setting;
    uint64_t       last_tx_delay_time;
    uint64_t       tx_delay_duration_us;
    bool           manual_control;
    bool           motor_start;
} DroneCAN_t;

void dronecan_init(HAL_Handle HalHandle,
                   FIFO_ID_Obj  *pECAN_rxFIFO_ID,
                   ECAN_Mailbox *pECAN_mailbox);
void dronecan_update();

#endif
