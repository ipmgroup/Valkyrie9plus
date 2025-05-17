#ifndef INCLUDE_USER_USER_HW_SELECT_H_
#define INCLUDE_USER_USER_HW_SELECT_H_

#define SW_REV_MAJOR 0
#define SW_REV_MINOR 1

#define BOARD_TMS320F28069M_8305_j1  1
#define BOARD_TMS320F28069M_8305_j5  2
#define BOARD_TMS320F28069M_8301_j1  3
#define BOARD_TMS320F28069M_8301_j5  4
#define BOARD_TMS320F28069M_8323_j1  5
#define BOARD_TMS320F28069M_8323_j5  6
#define BOARD_TMS320F28069M_8323H_j1 7
#define BOARD_CAN_FOC                8
#define BOARD_EPC9186_j1             9
#define BOARD_EPC9173_j1             10
#define BOARD_EPC9167_j1             11

#ifndef BOARD
#define BOARD BOARD_TMS320F28069M_8305_j1
#endif

// Defines for GPIO configuration
#define GPIO_35_CONFIG(h) GPIO_setMode(h, GPIO_Number_35, GPIO_35_Mode_JTAG_TDI)
#define GPIO_36_CONFIG(h) GPIO_setMode(h, GPIO_Number_36, GPIO_36_Mode_JTAG_TMS)
#define GPIO_37_CONFIG(h) GPIO_setMode(h, GPIO_Number_37, GPIO_37_Mode_JTAG_TDO)
#define GPIO_38_CONFIG(h) GPIO_setMode(h, GPIO_Number_38, GPIO_38_Mode_JTAG_TCK)

#if BOARD == BOARD_TMS320F28069M_8305_j1
#define BOARD_HEADER "board_tms320f28069m_8305_j1.h"
#elif BOARD == BOARD_TMS320F28069M_8305_j5
#define BOARD_HEADER "board_tms320f28069m_8305_j5.h"
#elif BOARD == BOARD_TMS320F28069M_8301_j1
#define BOARD_HEADER "board_tms320f28069m_8301_j1.h"
#elif BOARD == BOARD_TMS320F28069M_8301_j5
#define BOARD_HEADER "board_tms320f28069m_8301_j5.h"
#elif BOARD == BOARD_TMS320F28069M_8323_j1
#define BOARD_HEADER "board_tms320f28069m_8323_j1.h"
#elif BOARD == BOARD_TMS320F28069M_8323_j5
#define BOARD_HEADER "board_tms320f28069m_8323_j5.h"
#elif BOARD == BOARD_TMS320F28069M_8323H_j1
#define BOARD_HEADER "board_tms320f28069m_8323H_j1.h"
#elif BOARD == BOARD_CAN_FOC
#define BOARD_HEADER "board_can_foc.h"
#elif BOARD == BOARD_EPC9186_j1
#define BOARD_HEADER "board_EPC9186_j1.h"
#elif BOARD == BOARD_EPC9173_j1
#define BOARD_HEADER "board_EPC9173_j1.h"
#elif BOARD == BOARD_EPC9167_j1
#define BOARD_HEADER "board_EPC9167_j1.h"
#else
#error "Unsupported BOARD selected"
#endif

#include BOARD_HEADER

#define MOTOR_Estun_EMJ_04APB22     1
#define MOTOR_Anaheim_BLY172S       2
#define MOTOR_Teknic_M2310PLN04K    3
#define MOTOR_HACKER_A40_14L        4
#define MOTOR_Eflite_power10        5
#define MOTOR_Anaheim_Salient       6
#define MOTOR_Belt_Drive_Washer_IPM 7
#define MOTOR_Marathon_5K33GN2A     8
#define MOTOR_Bluerobotics_t200     9
#define MOTOR_B2212_920             10
#define MOTOR_5010_750KV            11

#ifndef MOTOR
#define MOTOR MOTOR_Bluerobotics_t200
#endif

#if MOTOR == MOTOR_Estun_EMJ_04APB22
#include "motor_Estun_EMJ_04APB22.h"
#elif MOTOR == MOTOR_Anaheim_BLY172S
#include "motor_Anaheim_BLY172S.h"
#elif MOTOR == MOTOR_Teknic_M2310PLN04K
#include "motor_Teknic_M2310PLN04K.h"
#elif MOTOR == MOTOR_HACKER_A40_14L
#include "motor_HACKER_A40_14L.h"
#elif MOTOR == MOTOR_Eflite_power10
#include "motor_Eflite_power10.h"
#elif MOTOR == MOTOR_Bluerobotics_t200
#include "motor_Bluerobotics_t200.h"
#elif MOTOR == MOTOR_B2212_920
#include "motor_B2212_920.h"
#elif MOTOR == MOTOR
#include "motor_5010_750KV.h"

// IPM motors
#elif MOTOR == MOTOR_Anaheim_Salient
#include "motor_Anaheim_Salient.h"
#elif MOTOR == MOTOR_Belt_Drive_Washer_IPM
#include "motor_Belt_Drive_Washer_IPM.h"

// ACIM motors
#elif MOTOR == MOTOR_Marathon_5K33GN2A
#include "motor_Marathon_5K33GN2A.h"

#else
#error "Motor type is not defined or unsupported"
#endif

#endif /* INCLUDE_USER_USER_HW_SELECT_H_ */
