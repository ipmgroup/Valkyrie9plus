#include "main.h"
#include <canard.h>

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stdbool.h>

#include "version.h"

#include "dronecan.h"
#include "user_config.h"
#include "user_hw_select.h"
#include <dronecan_msgs.h>

#include "user.h"
#include <can.h>
#include <fifo.h>

#include "to_string.h"

#ifdef BEEP
#include "beep.h"
#endif

extern USER_Params       gUserParams;
extern USER_Params_FLASH gUserBaseParams;
extern MOTOR_Vars_t      gMotorVars;
#ifdef DRV83XX
extern DRV_SPI_83XX_Vars_t gDrvSpi83xxVars;
#endif

#ifdef BEEP
extern Beep_Obj bp;
#endif

static CanardInstance canard;
static uint8_t        memory_pool[1024];

#ifdef DEFINE_DRV_ERROR_STRINGS
DEFINE_DRV_ERROR_STRINGS(NUM_DRV_ERRORS);
#endif

char buffer_str[50];
char str_UserErrorCode[] = "UserErrorCode:";
char str_CtrlState[]     = "CtrlState:";
char str_EstState[]      = "EstState:";

static DroneCAN_t dc;
static struct esc_state {
    float    throttle;
    uint64_t last_update_us;
} esc;

static USER_ErrorCode_e previousUserErrorCode = USER_ErrorCode_NoError;
static CTRL_State_e     previousCtrlState     = CTRL_State_Idle;
static EST_State_e      previousEstState      = EST_State_Idle;

ProfileState currentState = PHASE_IDLE;

static char drv_name[10]   = DRV_NAME;
static char motor_name[10] = USER_MOTOR_NAME;
static struct parameter {
    const char    *name;
    var_type       vtype;
    param_value_t *value;
    param_value_t  min_value;
    param_value_t  max_value;
    param_value_t  def_value;
} parameters[] = {
    {"CAN_NODE", T_UINT16, (param_value_t *)&gUserBaseParams.ecan_node_id, {.u16_val = 1}, {.u16_val = 127}, {.u16_val = 6}},                                // CAN node ID
    {"CAN_SPEED", T_UINT32, (param_value_t *)&gUserBaseParams.can_speed, {.u32_val = 250000}, {.u32_val = 100000}, {.u32_val = 1000000}},                    // CAN bus speed
    {"ESC_INDEX", T_UINT8, (param_value_t *)&gUserBaseParams.esc_index, {.u8_val = 0}, {.u8_val = 32}, {.u8_val = 0}},                                       // ESC index in RawCommand
    {"PWM_FREQ", T_FLOAT, (param_value_t *)&gUserBaseParams.PwmFreq_kHz, {.f_val = 15}, {.f_val = USER_MAX_PWM_FREQ_kHz}, {.f_val = 30}},                    // PWM frequency
    {"PWM_TICKS", T_UINT8, (param_value_t *)&gUserBaseParams.numPwmTicksPerIsrTick, {.u8_val = 1}, {.u8_val = 3}, {.u8_val = 3}},                            // PWM ticks per ISR tick
    {"LED_FREQ", T_UINT8, (param_value_t *)&gUserBaseParams.ledBlinkFreq_Hz, {.u8_val = 1}, {.u8_val = 5}, {.u8_val = 5}},                                   // LED blink frequency
    {"MANUAL_CONTROL", T_BOOL, (param_value_t *)&dc.manual_control, {.b_val = 0}, {.b_val = 1}, {.b_val = 0}},                                               // Manual control
    {"MOTOR_AUTO_ID", T_BOOL, (param_value_t *)&gUserParams.Motor_Auto_ID, {.b_val = 0}, {.b_val = 1}, {.b_val = 0}},                                        // Start Motor Auto ID
    {"Debug_Info", T_BOOL, (param_value_t *)&gUserBaseParams.FlagDebugInfo, {.b_val = 0}, {.b_val = 1}, {.b_val = 1}},                                       // Debug Info
    {"TELEM_RATE", T_UINT16, (param_value_t *)&gUserBaseParams.telem_rate, {.u16_val = 1}, {.u16_val = 100}, {.u16_val = 20}},                               // Telemetry rate in Hz
    {"ACCELERATION", T_IQ, (param_value_t *)&gUserBaseParams.UserMaxAccel_krpmps, {.iq_val = _IQ(0.5f)}, {.iq_val = _IQ(100.0f)}, {.iq_val = _IQ(5.0f)}},    // Acceleration in RPM/s^2
    {"USER_KP", T_IQ, (param_value_t *)&gUserBaseParams.kp_spd, {.iq_val = _IQ(0.0f)}, {.iq_val = _IQ(100.0f)}, {.iq_val = _IQ(3.5f)}},                      // Proportional gain for PID
    {"USER_KI", T_IQ, (param_value_t *)&gUserBaseParams.ki_spd, {.iq_val = _IQ(0.0f)}, {.iq_val = _IQ(100.0f)}, {.iq_val = _IQ(0.056f)}},                    // Integral gain for PID
    {"AUTO_KP", T_IQ, (param_value_t *)&gMotorVars.Kp_spd, {.iq_val = _IQ(0.0f)}, {.iq_val = _IQ(100.0f)}, {.iq_val = _IQ(3.5f)}},                           // Proportional gain for PID
    {"AUTO_KI", T_IQ, (param_value_t *)&gMotorVars.Ki_spd, {.iq_val = _IQ(0.0f)}, {.iq_val = _IQ(100.0f)}, {.iq_val = _IQ(0.056f)}},                         // Integral gain for PID
    {"RES_EST_CURRENT", T_FLOAT, (param_value_t *)&gUserBaseParams.maxCurrent_resEst, {.f_val = 0.0f}, {.f_val = 50.0f}, {.f_val = 2.0f}},                   // Max current for resistance estimation
    {"IND_EST_CURRENT", T_FLOAT, (param_value_t *)&gUserBaseParams.maxCurrent_indEst, {.f_val = -50.0f}, {.f_val = 0.0f}, {.f_val = -1.0f}},                 // Max current for inductance estimation
    {"FLUX_EST_FREQ", T_FLOAT, (param_value_t *)&gUserBaseParams.fluxEstFreq_Hz, {.f_val = 1.0f}, {.f_val = 250.0f}, {.f_val = 60.0f}},                      // Flux estimation frequency

    {"MOTOR_POLES", T_UINT16, (param_value_t *)&gUserBaseParams.motor_numPolePairs, {.u8_val = 1}, {.u8_val = 50}, {.u8_val = 7}},         // Number of motor poles
    {"MOTOR_Rr", T_FLOAT, (param_value_t *)&gUserBaseParams.motor_Rr, {.f_val = 0.0f}, {.f_val = 100.0f}, {.f_val = 0.0001f}},             // Motor winding resistance
    {"MOTOR_Rs", T_FLOAT, (param_value_t *)&gUserBaseParams.motor_Rs, {.f_val = 0.0f}, {.f_val = 100.0f}, {.f_val = 0.0001f}},             // Motor winding resistance
    {"MOTOR_Ls_d", T_FLOAT, (param_value_t *)&gUserBaseParams.motor_Ls_d, {.f_val = 0.0f}, {.f_val = 10.0f}, {.f_val = 0.0001f}},          // Motor d-axis inductance
    {"MOTOR_Ls_q", T_FLOAT, (param_value_t *)&gUserBaseParams.motor_Ls_q, {.f_val = 0.0f}, {.f_val = 10.0f}, {.f_val = 0.0001f}},          // Motor d-axis inductance
    {"MOTOR_FLUX", T_FLOAT, (param_value_t *)&gUserBaseParams.motor_ratedFlux, {.f_val = 0.0f}, {.f_val = 10.0f}, {.f_val = 0.002f}},      // Motor rated flux
    {"MOTOR_MAX_CURRENT", T_FLOAT, (param_value_t *)&gUserBaseParams.maxCurrent, {.f_val = 0.0f}, {.f_val = 500.0f}, {.f_val = 20.0f}},    // Max allowable motor current

    {"MAX_SPEED_KRPMS", T_FLOAT, (param_value_t *)&gUserBaseParams.max_speed, {.f_val = 0.0f}, {.f_val = 100000.0f}, {.f_val = 5.0f}},    // Max motor speed in RPM
    {"MIDPOINT", T_BOOL, (param_value_t *)&gUserBaseParams.midpoint, {.b_val = 0}, {.b_val = 1}, {.b_val = 0}},                           // Midpoint for ESC
    {"MIN_ESC", T_UINT16, (param_value_t *)&gUserBaseParams.min_esc, {.u16_val = 0}, {.u16_val = 8191}, {.u16_val = 819}},                // Minimum ESC value
    {"MAX_ESC", T_UINT16, (param_value_t *)&gUserBaseParams.max_esc, {.u16_val = 0}, {.u16_val = 8191}, {.u16_val = 8191}},               // Maximum ESC value
    {"DRV_NAME", T_STRING, (param_value_t *)&drv_name, {.str_val = ""}, {.str_val = ""}, {.str_val = ""}},                                // DRV name (string)
    {"MOTOR_NAME", T_STRING, (param_value_t *)&motor_name, {.str_val = ""}, {.str_val = ""}, {.str_val = ""}},                            // MOTOR name (string)

    {"ENABLE_SYS", T_BOOL, (param_value_t *)&gMotorVars.Flag_enableSys, {.b_val = 0}, {.b_val = 1}, {.b_val = 1}},                    // Enable system flag
    {"RUN_IDENTIFY", T_BOOL, (param_value_t *)&gMotorVars.Flag_Run_Identify, {.b_val = 0}, {.b_val = 1}, {.b_val = 0}},               // Run motor identification
    {"FIELD_WEAKENING", T_BOOL, (param_value_t *)&gMotorVars.Flag_enableFieldWeakening, {.b_val = 0}, {.b_val = 1}, {.b_val = 1}},    // Enable field weakening
    {"FORCE_ANGLE", T_BOOL, (param_value_t *)&gMotorVars.Flag_enableForceAngle, {.b_val = 0}, {.b_val = 1}, {.b_val = 0}},            // Enable force angle
    {"Rs_RECALC", T_BOOL, (param_value_t *)&gMotorVars.Flag_enableRsRecalc, {.b_val = 0}, {.b_val = 1}, {.b_val = 0}},                // Enable Rs recalculation
    {"POWER_WARP", T_BOOL, (param_value_t *)&gMotorVars.Flag_enablePowerWarp, {.b_val = 0}, {.b_val = 1}, {.b_val = 1}},              // Enable power warp
    {"USER_PARAMS", T_BOOL, (param_value_t *)&gMotorVars.Flag_enableUserParams, {.b_val = 0}, {.b_val = 1}, {.b_val = 1}},            // Enable user parameters
#ifdef BEEP
    {"BEEP", T_BOOL, (param_value_t *)&bp.beepStart, {.b_val = 0}, {.b_val = 1}, {.b_val = 0}},    // Enable beep
#endif
};

// some convenience macros
#define MIN(a, b)                                                    ((a) < (b) ? (a) : (b))
#define C_TO_KELVIN(temp)                                            (temp + 273.15f)
#define ARRAY_SIZE(x)                                                (sizeof(x) / sizeof(x[0]))
#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_MOTOR_ID  111
#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ARMING    112
#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_MOTOR_OFF 113

static struct uavcan_protocol_NodeStatus node_status;

size_t strnlen(const char *s, size_t maxlen) {
    size_t len = 0;
    while (len < maxlen && s[len] != '\0') {
        len++;
    }
    return len;
}

void getUniqueID(uint8_t *out_uid) {
    uint8_t i;
    for (i = 0; i < UNIQUE_ID_LENGTH_BYTES; i++) {
        out_uid[i] = i;
    }
}

float convertThrottle(int throttle, USER_Params_FLASH *pUserBaseParams) {
    float throttle_range;
    float normalized_throttle;

    float dir = (throttle < 0) ? -1.0f : 1.0f;
    throttle  = abs(throttle);

    if (throttle < pUserBaseParams->min_esc) {
        return 0.0f;
    }

    if (pUserBaseParams->midpoint == 1) {
        if (throttle > ESC_MID) {
            throttle_range      = (float)(pUserBaseParams->max_esc - ESC_MID);
            normalized_throttle = (float)(throttle - ESC_MID) / throttle_range;
            if (dir == -1.0f) {
                dir = 1.0f;
            }
        }
        else {
            throttle_range      = (float)(ESC_MID - pUserBaseParams->min_esc);
            normalized_throttle = (float)(ESC_MID - throttle) / throttle_range;
            if (dir == 1.0f) {
                dir = -1.0f;
            }
            // else {
            //     dir = 1.0f;
            // }
        }
    }
    else {
        throttle_range      = (float)(pUserBaseParams->max_esc - pUserBaseParams->min_esc);
        normalized_throttle = (float)(throttle - pUserBaseParams->min_esc) / throttle_range;
    }

    return dir * (normalized_throttle * pUserBaseParams->max_speed);
}

static void handle_ArmingStatus(CanardInstance *ins, CanardRxTransfer *transfer) {
    struct uavcan_equipment_safety_ArmingStatus cmd;
    if (uavcan_equipment_safety_ArmingStatus_decode(transfer, &cmd)) {
        return;
    }
    if (!dc.manual_control) {
        // dc.motor_start = 1;
        gMotorVars.Flag_enableSys        = 1;
        gMotorVars.Flag_enableUserParams = 1;
        gMotorVars.Flag_Run_Identify     = 1;
    }
    dc.manual_control = 0;
}

static void can_printf(const char *message, uint8_t level) {
    uint8_t                                 buffer[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];
    struct uavcan_protocol_debug_LogMessage pkt = {0};

    uint32_t message_length = strlen(message);
    uint32_t copy_length    = MIN(message_length, sizeof(pkt.text.data) - 1);
    pkt.level.value         = level;
    strncpy((char *)pkt.text.data, message, copy_length);
    pkt.text.data[copy_length]        = '\0';
    pkt.text.len                      = copy_length;
    uint16_t       len                = uavcan_protocol_debug_LogMessage_encode(&pkt, buffer);
    static uint8_t logmsg_transfer_id = 0;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
                    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
                    &logmsg_transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

static void handle_param_GetSet(CanardInstance *ins, CanardRxTransfer *transfer) {
    struct uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req)) {
        return;
    }

    struct parameter *p = NULL;
    if (req.name.len != 0) {
        for (uint16_t i = 0; i < ARRAY_SIZE(parameters); i++) {
            if (req.name.len == strlen(parameters[i].name) && strncmp((const char *)req.name.data, parameters[i].name, req.name.len) == 0) {
                p = &parameters[i];
                break;
            }
        }
    }
    else if (req.index < ARRAY_SIZE(parameters)) {
        p = &parameters[req.index];
    }
    if (p != NULL && req.name.len != 0 && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
        switch (p->vtype) {
            case T_INT8:
                *(int8_t *)p->value = (int8_t)req.value.integer_value;
                break;
            case T_UINT8:
                *(uint8_t *)p->value = (uint8_t)req.value.integer_value;
                break;
            case T_INT16:
                *(int16_t *)p->value = (int16_t)req.value.integer_value;
                break;
            case T_UINT16:
                *(uint16_t *)p->value = (uint16_t)req.value.integer_value;
                break;
            case T_INT32:
                *(int32_t *)p->value = (int32_t)req.value.integer_value;
                break;
            case T_UINT32:
                *(uint32_t *)p->value = (uint32_t)req.value.integer_value;
                break;
            case T_FLOAT:
                *(float *)p->value = req.value.real_value;
                break;
            case T_IQ:
                *(_iq *)p->value = _IQ(req.value.real_value);
                break;
            case T_BOOL:
                // *(uint8_t *)p->value  = req.value.boolean_value ? 1 : 0;
                *(bool *)p->value = req.value.boolean_value;
                break;
            // case T_STRING:
            //     strncpy((char *)p->value, (const char *)req.value.string_value.data, MIN(req.value.string_value.len, 9));
            //     break;
            default:
                return;
        }
        dc.Flag_setting = 1;
    }

    struct uavcan_protocol_param_GetSetResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    if (p != NULL) {
        // pkt.value.union_tag = p->var_type;
        switch (p->vtype) {
            case T_UINT8:
                pkt.value.union_tag         = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                pkt.min_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
                pkt.max_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;

                pkt.value.integer_value         = p->value->u8_val;
                pkt.default_value.integer_value = p->def_value.u8_val;
                pkt.max_value.integer_value     = p->max_value.u8_val;
                pkt.min_value.integer_value     = p->min_value.u8_val;
                break;

            case T_INT8:
                pkt.value.union_tag         = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                pkt.min_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
                pkt.max_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;

                pkt.value.integer_value         = p->value->i8_val;
                pkt.default_value.integer_value = p->def_value.i8_val;
                pkt.max_value.integer_value     = p->max_value.i8_val;
                pkt.min_value.integer_value     = p->min_value.i8_val;
                break;

            case T_INT16:
                pkt.value.union_tag         = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                pkt.min_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
                pkt.max_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;

                pkt.value.integer_value         = p->value->i16_val;
                pkt.default_value.integer_value = p->def_value.i16_val;
                pkt.max_value.integer_value     = p->max_value.i16_val;
                pkt.min_value.integer_value     = p->min_value.i16_val;
                break;

            case T_UINT16:
                pkt.value.union_tag         = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                pkt.min_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
                pkt.max_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;

                pkt.value.integer_value         = p->value->u16_val;
                pkt.default_value.integer_value = p->def_value.u16_val;
                pkt.max_value.integer_value     = p->max_value.u16_val;
                pkt.min_value.integer_value     = p->min_value.u16_val;
                break;

            case T_INT32:
                pkt.value.union_tag         = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                pkt.min_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
                pkt.max_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;

                pkt.value.integer_value         = p->value->i32_val;
                pkt.default_value.integer_value = p->def_value.i32_val;
                pkt.max_value.integer_value     = p->max_value.i32_val;
                pkt.min_value.integer_value     = p->min_value.i32_val;
                break;

            case T_UINT32:
                pkt.value.union_tag         = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
                pkt.min_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
                pkt.max_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;

                pkt.value.integer_value         = p->value->u32_val;
                pkt.default_value.integer_value = p->def_value.u32_val;
                pkt.max_value.integer_value     = p->max_value.u32_val;
                pkt.min_value.integer_value     = p->min_value.u32_val;
                break;

            case T_FLOAT:
                pkt.value.union_tag         = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
                pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
                pkt.min_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
                pkt.max_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;

                pkt.value.real_value         = p->value->f_val;
                pkt.default_value.real_value = p->def_value.f_val;
                pkt.max_value.real_value     = p->max_value.f_val;
                pkt.min_value.real_value     = p->min_value.f_val;
                break;

            case T_IQ:
                pkt.value.union_tag         = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
                pkt.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
                pkt.min_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
                pkt.max_value.union_tag     = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;

                pkt.value.real_value         = _IQtoF(p->value->iq_val);
                pkt.default_value.real_value = _IQtoF(p->def_value.iq_val);
                pkt.max_value.real_value     = _IQtoF(p->max_value.iq_val);
                pkt.min_value.real_value     = _IQtoF(p->min_value.iq_val);
                break;

            case T_BOOL:
                pkt.value.union_tag     = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
                pkt.value.boolean_value = p->value->b_val;
                // pkt.value.boolean_value = (*(uint8_t *)p->value->b_val) ? true : false;
                break;
            case T_STRING:
                pkt.value.union_tag        = UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE;
                pkt.value.string_value.len = strlen((char *)p->value);
                strncpy((char *)pkt.value.string_value.data, (char *)p->value, pkt.value.string_value.len);
                break;
            default:
                return;
        }
        pkt.name.len = strlen(p->name);
        strcpy((char *)pkt.name.data, p->name);
    }

    uint8_t  buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_GetSetResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

static void handle_param_ExecuteOpcode(CanardInstance *ins, CanardRxTransfer *transfer) {
    struct uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req)) {
        return;
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        gUserBaseParams = (USER_Params_FLASH)USER_FLASH_INIT;
        // here is where you would reset all parameters to defaults
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
        dc.Flag_enableSave = 1;
        // here is where you would save all the changed parameters to permanent storage
    }

    struct uavcan_protocol_param_ExecuteOpcodeResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.ok = true;

    uint8_t  buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

static void handle_RestartNode(CanardInstance *ins, CanardRxTransfer *transfer) {
    resetDevice(dc.pHalHandle);
}

static void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer) {
    uint8_t                                    buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt;

    memset(&pkt, 0, sizeof(pkt));

    node_status.uptime_sec = micros64() / 1000000ULL;
    pkt.status             = node_status;

    pkt.software_version.major                = SW_REV_MAJOR;
    pkt.software_version.minor                = SW_REV_MINOR;
    pkt.software_version.optional_field_flags = 0;
    pkt.software_version.vcs_commit           = GIT_HASH;

    pkt.hardware_version.major = HW_REV_MAJOR;
    pkt.hardware_version.minor = HW_REV_MINOR;

    getUniqueID(pkt.hardware_version.unique_id);

    strncpy((char *)pkt.name.data, APP_NODE_NAME, sizeof(pkt.name.data));
    pkt.name.len = strnlen((char *)pkt.name.data, sizeof(pkt.name.data));

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

static void handle_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer) {
    struct uavcan_equipment_esc_RawCommand cmd;
    if (uavcan_equipment_esc_RawCommand_decode(transfer, &cmd)) {
        return;
    }
    if (cmd.cmd.len <= gUserBaseParams.esc_index) {
        return;
    }
    dc.RAWcmd                = cmd.cmd.data[(unsigned)gUserBaseParams.esc_index];
    esc.throttle             = convertThrottle(dc.RAWcmd, &gUserBaseParams);
    gMotorVars.SpeedRef_krpm = _IQ(esc.throttle);
    esc.last_update_us       = micros64();
}

static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer) {
    // switch on data type ID to pass to the right handler function
    if (transfer->transfer_type == CanardTransferTypeRequest) {
        // check if we want to handle a specific service request
        switch (transfer->data_type_id) {
            case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
                handle_GetNodeInfo(ins, transfer);
                break;
            }
            case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
                handle_param_GetSet(ins, transfer);
                break;
            }
            case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
                handle_param_ExecuteOpcode(ins, transfer);
                break;
            }
            case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
                handle_RestartNode(ins, transfer);
                break;
            }
        }
    }

    if (transfer->transfer_type == CanardTransferTypeBroadcast) {
        // check if we want to handle a specific broadcast message
        switch (transfer->data_type_id) {
            case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID: {
                handle_RawCommand(ins, transfer);
                break;
            }
            case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID: {
                handle_ArmingStatus(ins, transfer);
                break;
            }
        }
    }
}

static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t             *out_data_type_signature,
                                 uint16_t              data_type_id,
                                 CanardTransferType    transfer_type,
                                 uint8_t               source_node_id) {
    if (transfer_type == CanardTransferTypeRequest) {
        // check if we want to handle a specific service request
        switch (data_type_id) {
            case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
                *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
                return true;
            }
            case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
                *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
                return true;
            }
            case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
                *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
                return true;
            }
            case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
                *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
                return true;
            }
        }
    }

    if (transfer_type == CanardTransferTypeBroadcast) {
        // see if we want to handle a specific broadcast packet
        switch (data_type_id) {
            case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID: {
                *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
                return true;
            }
            case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID: {
                *out_data_type_signature = UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_SIGNATURE;
                return true;
            }
        }
    }
    // we don't want any other messages
    return false;
}

static void send_NodeStatus(void) {
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    node_status.uptime_sec = uptime_s;
    node_status.health     = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode       = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode   = 0;

    // put whatever you like in here for display in GUI
    node_status.vendor_specific_status_code = 4243;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

static void process1HzTasks(uint64_t timestamp_usec) {
    canardCleanupStaleTransfers(&canard, timestamp_usec);
    const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&canard);

    dc.peak_percent = 100U * stats.peak_usage_blocks / stats.capacity_blocks;

    send_NodeStatus();

    if (!gMotorVars.Flag_Run_Identify) {
        return;
    }
    uint64_t current_time = micros64();
    if (current_time - esc.last_update_us > 1000000 && !dc.manual_control) {
        gMotorVars.Flag_Run_Identify = false;
        gMotorVars.IdRef_A           = _IQ(0.0);
    }
}
// static void processClearTasks(uint64_t timestamp_usec) {
//     canardCleanupStaleTransfers(&canard, timestamp_usec);
//     const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&canard);
//     dc.peak_percent                           = 100U * stats.peak_usage_blocks / stats.capacity_blocks;
// }

static void send_ESCStatus(void) {
    struct uavcan_equipment_esc_Status pkt;
    memset(&pkt, 0, sizeof(pkt));
    uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];

    pkt.error_count = 0;
    pkt.voltage     = _IQtoF(gMotorVars.VdcBus_kV) * 1000;
    pkt.current     = _IQtoF(gMotorVars.Is_A);
#ifdef DRV_TEMPERATURE
    pkt.temperature = C_TO_KELVIN(gMotorVars.TempSenDegCelsius);
#else
    pkt.temperature = measureTemperatureC();
#endif
    pkt.rpm = _IQtoF(gMotorVars.Speed_krpm) * 1000;
    // pkt.power_rating_pct = (uint8_t)gMotorVars.CpuUsagePercentageMax;
    pkt.power_rating_pct = (uint8_t)gMotorVars.CpuUsagePercentageAvg;
    pkt.esc_index        = gUserBaseParams.esc_index;

    uint32_t       len = uavcan_equipment_esc_Status_encode(&pkt, buffer);
    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
                    UAVCAN_EQUIPMENT_ESC_STATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

void sendDEBUG(uint8_t currentValue, const char *prefix, char *buffer_str, size_t buffer_size) {
    strncpy(buffer_str, prefix, buffer_size - 1);
    buffer_str[buffer_size - 1] = '\0';
    size_t prefix_len           = strlen(buffer_str);
    uintToStringInBuffer(currentValue, buffer_str + prefix_len, buffer_size - prefix_len);
    can_printf(buffer_str, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG);
}

void sendDEBUGfloat(float currentValue, const char *prefix, char *buffer_str, size_t buffer_size) {
    strncpy(buffer_str, prefix, buffer_size - 1);
    buffer_str[buffer_size - 1] = '\0';
    size_t prefix_len           = strlen(buffer_str);
    floatToStringInBuffer(currentValue, buffer_str + prefix_len, buffer_size - prefix_len, 9);
    can_printf(buffer_str, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG);
}

void saveParameters(void) {
    if (dc.Flag_enableSave) {
        switch (currentState) {
            case PHASE_IDLE:
                gMotorVars.Flag_enableSys    = true;
                gMotorVars.Flag_Run_Identify = true;
                currentState                 = PHASE_AKTIV;
                break;
            case PHASE_AKTIV:
                if (gMotorVars.Flag_Run_Identify) {
                    gMotorVars.Flag_enableSys = false;
                    currentState              = PHASE_COPYING;
                }
                break;
            case PHASE_COPYING:
                if (!gMotorVars.Flag_enableSys) {
                    gUserParams.copyActiveParamsToUserBase = true;
                    currentState                           = PHASE_CHECKING;
                }
                break;
            case PHASE_CHECKING:
                if (!gUserParams.copyActiveParamsToUserBase) {
                    gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);
                    sendDEBUG(gMotorVars.UserErrorCode, str_UserErrorCode, buffer_str, sizeof(buffer_str));
                    if (gMotorVars.UserErrorCode == USER_ErrorCode_NoError) {
                        currentState = PHASE_SAVING;
                    }
                    else {
                        currentState       = PHASE_IDLE;
                        dc.Flag_enableSave = false;
                    }
                }
                break;
            case PHASE_SAVING:
                gUserParams.saveUserParams = true;
                currentState               = PHASE_REBOOTING;
                break;
            case PHASE_REBOOTING:
                if (!gUserParams.saveUserParams) {
                    if (gUserParams.writtenSuccessfully) {
                        can_printf("Saved parameters", UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO);
                        gUserParams.writtenSuccessfully = false;
                        // resetDevice(dc.pHalHandle);
                    }
                    else {
                        can_printf("Failed to save parameters", UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR);
                    }
                    currentState       = PHASE_IDLE;
                    dc.Flag_enableSave = false;
                }
                break;
            default:
                currentState = PHASE_IDLE;
                break;
        }
    }
}

void MotorAutoIdHandler(void) {
    static int phase = 0;
    if (gUserParams.Motor_Auto_ID) {
        switch (phase) {
            case 0:
                dc.manual_control                    = true;
                gMotorVars.Flag_enableUserParams     = false;
                gMotorVars.Flag_Run_Identify         = false;
                gMotorVars.Flag_enableSys            = false;
                gMotorVars.Flag_enableFieldWeakening = false;
                phase                                = 1;
                can_printf("Start motor identification", UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO);
                break;
            case 1:
                gMotorVars.Flag_enableSys    = true;
                gMotorVars.Flag_Run_Identify = true;
                phase                        = 2;
                break;
            case 2:
                if (!gMotorVars.Flag_Run_Identify) {
                    MOTOR_setUserBaseFromActive(&gUserBaseParams, &gMotorVars);
                    gUserParams.Motor_Auto_ID = false;
                    phase                     = 0;
                    can_printf("Motor identified", UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO);
                    // gMotorVars.Flag_enableUserParams = true;
                }
                break;
            default:
                phase = 0;
                break;
        }
    }
    else {
        phase = 0;
    }
}

#pragma CODE_SECTION(processTxRxOnce, "ramfuncs");

static void processTxRxOnce(int32_t timeout_msec) {
    uint64_t start_time             = micros64();
    uint64_t timeout_us             = timeout_msec * 1000ULL;
    uint16_t current_priority_level = 0x1F;

    // Transmitting
    for (const CanardCANFrame *txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        if ((micros64() - start_time) >= timeout_us) {
            return;
        }

        if ((micros64() - dc.last_tx_delay_time) >= dc.tx_delay_duration_us) {
            dc.last_tx_delay_time = micros64();
            convertCanardFrameToC2000(txf, &dc.tx_frame);
            const int16_t tx_res = ECAN_sendFrameToAvailableTxMbox(dc.pHalHandle->ecanaHandle, &dc.tx_frame, 0, current_priority_level);
            if (tx_res > 0) {
                canardPopTxQueue(&canard);
                if (current_priority_level > 0) {
                    current_priority_level--;
                }
            }
            else {
                break;
            }
        }
        else {
            break;
        }
    }

    // Receiving
    dc.rx_fifo_status = ECAN_getRxMsg(dc.pHalHandle, dc.pECAN_rxFIFO_ID, dc.pECAN_mailbox, &dc.rx_frame);

    if (dc.rx_fifo_status != no_new_messages) {
        convertC2000FrameToCanard(&dc.rx_frame, &dc.CanardRx_frame);
        const uint64_t timestamp = micros64();

        if ((micros64() - start_time) >= timeout_us) {
            return;
        }

        dc.CanardRxError = (DC_ErrorCode_e)canardHandleRxFrame(&canard, &dc.CanardRx_frame, timestamp);
    }
}

void sendFaultMessage(uint16_t currentValue, const char *prefix, char *buffer_str, size_t buffer_size) {
    strncpy(buffer_str, prefix, buffer_size - 1);
    buffer_str[buffer_size - 1] = '\0';
    size_t prefix_len           = strlen(buffer_str);
    uint16ToStringInBuffer(currentValue, buffer_str + prefix_len, buffer_size - prefix_len);
    can_printf(buffer_str, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR);
}

void sendUpdateIfChanged(uint8_t currentValue, uint8_t *previousValue, const char *prefix, char *buffer_str, size_t buffer_size) {
    if (currentValue != *previousValue) {
        strncpy(buffer_str, prefix, buffer_size - 1);
        buffer_str[buffer_size - 1] = '\0';

        size_t prefix_len = strlen(buffer_str);
        uintToStringInBuffer(currentValue, buffer_str + prefix_len, buffer_size - prefix_len);
        can_printf(buffer_str, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG);
        *previousValue = currentValue;
    }
}

void checkAndSendUpdates(char *buffer_str, size_t buffer_size) {
    sendUpdateIfChanged(gMotorVars.UserErrorCode, &previousUserErrorCode, str_UserErrorCode, buffer_str, buffer_size);
    sendUpdateIfChanged(gMotorVars.CtrlState, &previousCtrlState, str_CtrlState, buffer_str, buffer_size);
    sendUpdateIfChanged(gMotorVars.EstState, &previousEstState, str_EstState, buffer_str, buffer_size);
}

void Motor_Start() {
    static Start_t state = STATE_ENABLE_SYS;

    if (!dc.motor_start) {
        return;
    }

    switch (state) {
        case STATE_ENABLE_SYS:
            gMotorVars.Flag_enableSys = 1;
            if (gMotorVars.Flag_enableSys == 1) {
                state = STATE_ENABLE_USER_PARAMS;
            }
            break;

        case STATE_ENABLE_USER_PARAMS:
            gMotorVars.Flag_enableUserParams = 1;
            if (gMotorVars.Flag_enableUserParams == 1) {
                state = STATE_RUN_IDENTIFY;
            }
            break;

        case STATE_RUN_IDENTIFY:
            gMotorVars.Flag_Run_Identify = 1;
            if (gMotorVars.Flag_Run_Identify == 1) {
                dc.motor_start = 0;
                state          = STATE_ENABLE_SYS;
            }
            break;

        default:

            break;
    }
}

void dronecan_init(HAL_Handle    HalHandle,
                   FIFO_ID_Obj  *pECAN_rxFIFO_ID,
                   ECAN_Mailbox *pECAN_mailbox) {
    dc.pHalHandle           = HalHandle;
    dc.pECAN_rxFIFO_ID      = pECAN_rxFIFO_ID;
    dc.pECAN_mailbox        = pECAN_mailbox;
    dc.RAWcmd               = false;
    dc.Flag_enableSave      = false;
    dc.Flag_enableSave      = false;
    dc.last_tx_delay_time   = false,
    dc.tx_delay_duration_us = TX_DELAY;
    dc.manual_control       = 1;
    dc.motor_start          = 0;

    dc.tx_frame = (MSG_t){0};
    dc.rx_frame = (MSG_t){0};

    canardInit(&canard,
               memory_pool,
               sizeof(memory_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               NULL);

    canardSetLocalNodeID(&canard, (uint8_t)gUserBaseParams.ecan_node_id);

    dc.next_1hz_service_at  = micros64();
    dc.next_50hz_service_at = micros64();
    // dc.next_DC_clear_service_at = micros64();
}

#pragma CODE_SECTION(dronecan_update, "ramfuncs");

void dronecan_update() {
    processTxRxOnce(5);
    const uint64_t ts = micros64();

    // if (ts >= dc.next_DC_clear_service_at) {
    //     dc.next_DC_clear_service_at += 1000000 / 4;
    //     processClearTasks(ts);
    // }

    if (dc.Flag_setting == 1) {
        gUserBaseParams.control_word = packFlagsIntoControlWord(&gMotorVars);

        copyUserParams_FLASH_to_Params(&gUserBaseParams, &gUserParams);
        USER_setOtherParams(&gUserParams);
        updateMotorVars(&gUserParams, &gMotorVars);
        gMotorVars.MaxAccel_krpmps = gUserBaseParams.UserMaxAccel_krpmps;
        dc.Flag_setting            = 0;
    }

#ifdef Fault_OST
    if (gMotorVars.Flag_nFault_OST && gMotorVars.Flag_Run_Identify) {
        can_printf("OST Fault detected", UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR);
        gMotorVars.Flag_nFault_OST = 0;
    }
#endif
#ifdef DRV_TEMPERATURE
    if (gMotorVars.Flag_Overheat) {
        can_printf("Overheat", UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR);
        gMotorVars.Flag_Overheat = 0;
    }
#endif

    if (gMotorVars.Flag_nFaultDroneCan) {
        can_printf("Fault detected", UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR);
#ifdef DRV83XX
        uint16_t fault[NUM_DRV_ERRORS] = {0};
        PackStatusRegisters(&gDrvSpi83xxVars, fault);

        for (int i = 0; i < NUM_DRV_ERRORS; i++) {
            if (fault[i] != 0) {
                sendFaultMessage(fault[i], str_DrvError[i], buffer_str, sizeof(buffer_str));
            }
        }
#endif
        gMotorVars.Flag_nFaultDroneCan = 0;
    }

    saveParameters();
    MotorAutoIdHandler();
    // Motor_Start();

    if (ts >= dc.next_1hz_service_at) {
        dc.next_1hz_service_at += 1000000ULL;
        process1HzTasks(ts);
    }
    if (ts >= dc.next_50hz_service_at) {
        dc.next_50hz_service_at += 1000000ULL / gUserBaseParams.telem_rate;
        if (gUserBaseParams.FlagDebugInfo) {
            checkAndSendUpdates(buffer_str, sizeof(buffer_str));
        }
        send_ESCStatus();
    }
    return;
}
