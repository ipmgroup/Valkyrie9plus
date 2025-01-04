/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
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
#ifndef _DRV8323_H_
#define _DRV8323_H_

//! \file   drivers/drvic/drv8323/src/32b/f28x/f2806x/drv8323.h
//! \brief  Contains public interface to various functions related
//!         to the DRV8305 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

// **************************************************************************
// the includes

// drivers
#include "spi.h"
#include "sw/drivers/gpio/src/32b/f28x/f2806x/gpio.h"

// **************************************************************************
// modules

// **************************************************************************
// solutions

//!
//! \defgroup DRV8323

//!
//! \ingroup DRV8323
//@{

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines

//! \brief Defines the address mask
//!
#define DRV8323_ADDR_MASK (0x7800)

//! \brief Defines the data mask
//!
#define DRV8323_DATA_MASK (0x07FF)

//! \brief Defines the R/W mask
//!
#define DRV8323_RW_MASK (0x8000)

//! \brief Defines the R/W mask
//!
#define DRV8323_FAULT_TYPE_MASK (0x07FF)

#define DRV8323_STATUS00_VDS_LC_BITS (1 << 0)
#define DRV8323_STATUS00_VDS_HC_BITS (1 << 1)
#define DRV8323_STATUS00_VDS_LB_BITS (1 << 2)
#define DRV8323_STATUS00_VDS_HB_BITS (1 << 3)
#define DRV8323_STATUS00_VDS_LA_BITS (1 << 4)
#define DRV8323_STATUS00_VDS_HA_BITS (1 << 5)

//! \brief Defines the location of the OTSD (Over temperature shutdown) bits in the Status 1 register
//!
#define DRV8323_STATUS00_OTSD_BITS (1 << 6)
#define DRV8323_STATUS00_UVLO_BITS (1 << 7)
#define DRV8323_STATUS00_GDF_BITS (1 << 8)
#define DRV8323_STATUS00_VDS_OCP_BITS (1 << 9)
#define DRV8323_STATUS00_FAULT_BITS (1 << 10)

//! \brief Defines the location of the VGS_LC bits in the Status 2 register
//!
#define DRV8323_STATUS01_VGS_LC_BITS (1 << 0)

//! \brief Defines the location of the VGS_HC bits in the Status 2 register
//!
#define DRV8323_STATUS01_VGS_HC_BITS (1 << 1)

//! \brief Defines the location of the VGS_LB bits in the Status 2 register
//!
#define DRV8323_STATUS01_VGS_LB_BITS (1 << 2)

//! \brief Defines the location of the VGS_HB bits in the Status 2 register
//!
#define DRV8323_STATUS01_VGS_HB_BITS (1 << 3)

//! \brief Defines the location of the VGS_LA bits in the Status 2 register
//!
#define DRV8323_STATUS01_VGS_LA_BITS (1 << 4)

//! \brief Defines the location of the VGS_HA bits in the Status 2 register
//!
#define DRV8323_STATUS01_VGS_HA_BITS (1 << 5)

//! \brief Defines the location of the CPUV (charge pump undervoltage) bits in the Status 2 register
//!
#define DRV8323_STATUS01_CPUV_BITS (1 << 6)

//! \brief Defines the location of the OTW bits in the Status 2 register
//!
#define DRV8323_STATUS01_OTW_BITS (1 << 7)

//! \brief Defines the location of the SC_OC bits in the Status 2 register
//!
#define DRV8323_STATUS01_SC_OC_BITS (1 << 8)

//! \brief Defines the location of the SB_OC bits in the Status 2 register
//!
#define DRV8323_STATUS01_SB_OC_BITS (1 << 9)

//! \brief Defines the location of the SA_OC bits in the Status 2 register
//!
#define DRV8323_STATUS01_SA_OC_BITS (1 << 10)

//! \brief Defines the location of the CLR_FLT bits in the Control 2 register
//!
#define DRV8323_CTRL02_CLR_FLT_BITS (1 << 0)

//! \brief Defines the location of the BRAKE bits in the Control 2 register
//!
#define DRV8323_CTRL02_BRAKE_BITS (1 << 1)

//! \brief Defines the location of the COAST bits in the Control 2 register
//!
#define DRV8323_CTRL02_COAST_BITS (1 << 2)

//! \brief Defines the location of the 1PWM_DIR bits in the Control 2 register
//!
#define DRV8323_CTRL02_PWM1_DIR_BITS (1 << 3)

//! \brief Defines the location of the 1PWM_COM bits in the Control 2 register
//!
#define DRV8323_CTRL02_PWM1_COM_BITS (1 << 4)

//! \brief Defines the location of the PWM_MODE bits in the Control 2 register
//!
#define DRV8323_CTRL02_PWM_MODE_BITS (3 << 5)

//! \brief Defines the location of the OTW_REP bits in the Control 2 register
//!
#define DRV8323_CTRL02_OTW_REP_BITS (1 << 7)

//! \brief Defines the location of the DIS_GDF bits in the Control 2 register
//!
#define DRV8323_CTRL02_DIS_GDF_BITS (1 << 8)

//! \brief Defines the location of the DIS_CPUV bits in the Control 2 register
//!
#define DRV8323_CTRL02_DIS_CPUV_BITS (1 << 9)

//! \brief Defines the location of the RESERVED1 bits in the Control 2 register
//!
#define DRV8323_CTRL02_RESERVED1_BITS (1 << 10)

//! \brief Defines the location of the IDRIVEN_HS bits in the Control 3 register
//!
#define DRV8323_CTRL03_IDRIVEN_HS_BITS (15 << 0)

//! \brief Defines the location of the IDRIVEP_HS bits in the Control 3 register
//!
#define DRV8323_CTRL03_IDRIVEP_HS_BITS (15 << 4)

//! \brief Defines the location of the LOCK bits in the Control 3 register
//!
#define DRV8323_CTRL03_LOCK_BITS (7 << 8)

//! \brief Defines the location of the IDRIVEN_LS bits in the Control 4 register
//!
#define DRV8323_CTRL04_IDRIVEN_LS_BITS (15 << 0)

//! \brief Defines the location of the IDRIVEP_LS bits in the Control 4 register
//!
#define DRV8323_CTRL04_IDRIVEP_LS_BITS (15 << 4)

//! \brief Defines the location of the TDRIVE bits in the Control 4 register
//!
#define DRV8323_CTRL04_TDRIVE_BITS (3 << 8)

//! \brief Defines the location of the CBC bits in the Control 4 register
//!
#define DRV8323_CTRL04_CBC_BITS (1 << 10)

//! \brief Defines the location of the VDS_LVL bits in the Control 5 register
//!
#define DRV8323_CTRL05_VDS_LVL_BITS (15 << 0)

//! \brief Defines the location of the OCP_DEG bits in the Control 5 register
//!
#define DRV8323_CTRL05_OCP_DEG_BITS (3 << 4)

//! \brief Defines the location of the OCP_MODE bits in the Control 5 register
//!
#define DRV8323_CTRL05_OCP_MODE_BITS (3 << 6)

//! \brief Defines the location of the DEAD_TIME bits in the Control 5 register
//!
#define DRV8323_CTRL05_DEAD_TIME_BITS (3 << 8)

//! \brief Defines the location of the TRETRY bits in the Control 5 register
//!
#define DRV8323_CTRL05_TRETRY_BITS (1 << 10)

//! \brief Defines the location of the SEN_LVL bits in the Control 6 register
//!
#define DRV8323_CTRL06_SEN_LVL_BITS (3 << 0)

//! \brief Defines the location of the CSA_CAL_C bits in the Control 6 register
//!
#define DRV8323_CTRL06_CSA_CAL_C_BITS (1 << 2)

//! \brief Defines the location of the CSA_CAL_B bits in the Control 6 register
//!
#define DRV8323_CTRL06_CSA_CAL_B_BITS (1 << 3)

//! \brief Defines the location of the CSA_CAL_A bits in the Control 6 register
//!
#define DRV8323_CTRL06_CSA_CAL_A_BITS (1 << 4)

//! \brief Defines the location of the DIS_SEN bits in the Control 6 register
//!
#define DRV8323_CTRL06_DIS_SEN_BITS (1 << 5)

//! \brief Defines the location of the CSA_GAIN bits in the Control 6 register
//!
#define DRV8323_CTRL06_CSA_GAIN_BITS (3 << 6)

//! \brief Defines the location of the LS_REF bits in the Control 6 register
//!
#define DRV8323_CTRL06_LS_REF_BITS (1 << 8)

//! \brief Defines the location of the VREF_DIV bits in the Control 6 register
//!
#define DRV8323_CTRL06_VREF_DIV_BITS (1 << 9)

//! \brief Defines the location of the CSA_FET bits in the Control 6 register
//!
#define DRV8323_CTRL06_CSA_FET_BITS (1 << 10)

// **************************************************************************
// the typedefs

//! \brief Enumeration for the R/W modes
//!
typedef enum {
    CtrlMode_Read  = 1 << 15,  //!< Read Mode
    CtrlMode_Write = 0 << 15   //!< Write Mode
} DRV8323_CtrlMode_e;

//! \brief Enumeration for the Status 0 register, faults
//!
typedef enum {
    VDS_LC  = (1 << 0),  //!< VDS overcurrent fault on C low-side MOSFET
    VDS_HC  = (1 << 1),  //!< VDS overcurrent fault on C high-side MOSFET
    VDS_LB  = (1 << 2),  //!< VDS overcurrent fault on B low-side MOSFET
    VDS_HB  = (1 << 3),  //!< VDS overcurrent fault on B high-side MOSFET
    VDS_LA  = (1 << 4),  //!< VDS overcurrent fault on A low-side MOSFET
    VDS_HA  = (1 << 5),  //!< VDS overcurrent fault on A high-side MOSFET
    OTSD    = (1 << 6),  //!< Overtemperature shutdown
    UVLO    = (1 << 7),  //!< Undervoltage lockout fault condition
    GDF     = (1 << 8),  //!< Gate driver fault condition
    VDS_OCP = (1 << 9),  //!< VDS monitor overcurrent fault condition
    FAULT   = (1 << 10)  //!< FAULT type, 0-Warning, 1-Latched
} DRV8323_STATUS00_WarningWatchdog_e;

//! \brief Enumeration for the Status 1 register, OV/VDS faults
//!
typedef enum {
    VGS_LC = (1 << 0),  //!< VGS gate drive fault on C low-side MOSFET
    VGS_HC = (1 << 1),  //!< VGS gate drive fault on C high-side MOSFET
    VGS_LB = (1 << 2),  //!< VGS gate drive fault on B low-side MOSFET
    VGS_HB = (1 << 3),  //!< VGS gate drive fault on B high-side MOSFET
    VGS_LA = (1 << 4),  //!< VGS gate drive fault on A low-side MOSFET
    VGS_HA = (1 << 5),  //!< VGS gate drive fault on A high-side MOSFET
    CPUV   = (1 << 6),  //!< charge pump undervoltage fault
    OTW    = (1 << 7),  //!< overtemperature warning
    SC_OC  = (1 << 8),  //!< overcurrent on phase C
    SB_OC  = (1 << 9),  //!< overcurrent on phase B
    SA_OC  = (1 << 10)  //!< overcurrent on phase A
} DRV8323_STATUS01_OvVdsFaults_e;

//! \brief Enumeration for the driver PWM mode
//!
typedef enum {
    PwmMode_6 = (0 << 5),  //!< PWM_MODE = 6 inputs
    PwmMode_3 = (1 << 5),  //!< PWM_MODE = 3 inputs
    PwmMode_1 = (2 << 5)   //!< PWM_MODE = 1 input
} DRV8323_CTRL02_PwmMode_e;

//! \brief Enumeration for the high side gate drive peak source current; TODO gate currents not consistent with DS
//!
typedef enum {
    ISour_HS_0p010_A = (0 << 4),   //!< IDRIVEP_HS = 0.010A
    ISour_HS_0p030_A = (1 << 4),   //!< IDRIVEP_HS = 0.030A
    ISour_HS_0p060_A = (2 << 4),   //!< IDRIVEP_HS = 0.060A
    ISour_HS_0p080_A = (3 << 4),   //!< IDRIVEP_HS = 0.080A
    ISour_HS_0p120_A = (4 << 4),   //!< IDRIVEP_HS = 0.120A
    ISour_HS_0p140_A = (5 << 4),   //!< IDRIVEP_HS = 0.140A
    ISour_HS_0p170_A = (6 << 4),   //!< IDRIVEP_HS = 0.170A
    ISour_HS_0p190_A = (7 << 4),   //!< IDRIVEP_HS = 0.190A
    ISour_HS_0p260_A = (8 << 4),   //!< IDRIVEP_HS = 0.260A
    ISour_HS_0p330_A = (9 << 4),   //!< IDRIVEP_HS = 0.330A
    ISour_HS_0p370_A = (10 << 4),  //!< IDRIVEP_HS = 0.370A
    ISour_HS_0p440_A = (11 << 4),  //!< IDRIVEP_HS = 0.440A
    ISour_HS_0p570_A = (12 << 4),  //!< IDRIVEP_HS = 0.570A
    ISour_HS_0p680_A = (13 << 4),  //!< IDRIVEP_HS = 0.680A
    ISour_HS_0p820_A = (14 << 4),  //!< IDRIVEP_HS = 0.820A
    ISour_HS_1p000_A = (15 << 4)   //!< IDRIVEP_HS = 1.000A
} DRV8323_CTRL03_PeakSourCurHS_e;

//! \brief Enumeration for the high side gate drive peak sink current; TODO gate currents not consistent with DS
//!
typedef enum {
    ISink_HS_0p020_A = (0 << 0),   //!< IDRIVEN_HS = 0.020A
    ISink_HS_0p060_A = (1 << 0),   //!< IDRIVEN_HS = 0.060A
    ISink_HS_0p120_A = (2 << 0),   //!< IDRIVEN_HS = 0.120A
    ISink_HS_0p160_A = (3 << 0),   //!< IDRIVEN_HS = 0.160A
    ISink_HS_0p240_A = (4 << 0),   //!< IDRIVEN_HS = 0.240A
    ISink_HS_0p280_A = (5 << 0),   //!< IDRIVEN_HS = 0.280A
    ISink_HS_0p340_A = (6 << 0),   //!< IDRIVEN_HS = 0.340A
    ISink_HS_0p380_A = (7 << 0),   //!< IDRIVEN_HS = 0.380A
    ISink_HS_0p520_A = (8 << 0),   //!< IDRIVEN_HS = 0.520A
    ISink_HS_0p660_A = (9 << 0),   //!< IDRIVEN_HS = 0.660A
    ISink_HS_0p740_A = (10 << 0),  //!< IDRIVEN_HS = 0.740A
    ISink_HS_0p880_A = (11 << 0),  //!< IDRIVEN_HS = 0.880A
    ISink_HS_1p140_A = (12 << 0),  //!< IDRIVEN_HS = 1.140A
    ISink_HS_1p360_A = (13 << 0),  //!< IDRIVEN_HS = 1.360A
    ISink_HS_1p640_A = (14 << 0),  //!< IDRIVEN_HS = 1.640A
    ISink_HS_2p000_A = (15 << 0)   //!< IDRIVEN_HS = 2.000A
} DRV8323_CTRL03_PeakSinkCurHS_e;

//! \brief Enumeration for the high side and low side gate drive peak source time; TODO adapt timings to DRV8323
//!
typedef enum {
    Lock_lock   = (6 << 8),  //!< Lock settings
    Lock_unlock = (3 << 8)   //!< Unlock settings
} DRV8323_CTRL03_Lock_e;

//! \brief Enumeration for the high side and low side gate drive peak source time; TODO adapt timings to DRV8323
//!
typedef enum {
    TSour_250_ns  = (0 << 8),  //!< TDRIVEN = 250ns
    TSour_500_ns  = (1 << 8),  //!< TDRIVEN = 500ns
    TSour_1000_ns = (2 << 8),  //!< TDRIVEN = 1000ns
    TSour_2000_ns = (3 << 8)   //!< TDRIVEN = 2000ns
} DRV8323_CTRL04_PeakTime_e;

//! \brief Enumeration for the low side gate drive peak source current; TODO adapt current ratings
//!
typedef enum {
    ISour_LS_0p010_A = (0 << 4),   //!< IDRIVEP_LS = 0.010A
    ISour_LS_0p030_A = (1 << 4),   //!< IDRIVEP_LS = 0.030A
    ISour_LS_0p060_A = (2 << 4),   //!< IDRIVEP_LS = 0.060A
    ISour_LS_0p080_A = (3 << 4),   //!< IDRIVEP_LS = 0.080A
    ISour_LS_0p120_A = (4 << 4),   //!< IDRIVEP_LS = 0.120A
    ISour_LS_0p140_A = (5 << 4),   //!< IDRIVEP_LS = 0.140A
    ISour_LS_0p170_A = (6 << 4),   //!< IDRIVEP_LS = 0.170A
    ISour_LS_0p190_A = (7 << 4),   //!< IDRIVEP_LS = 0.190A
    ISour_LS_0p260_A = (8 << 4),   //!< IDRIVEP_LS = 0.260A
    ISour_LS_0p330_A = (9 << 4),   //!< IDRIVEP_LS = 0.330A
    ISour_LS_0p370_A = (10 << 4),  //!< IDRIVEP_LS = 0.370A
    ISour_LS_0p440_A = (11 << 4),  //!< IDRIVEP_LS = 0.440A
    ISour_LS_0p570_A = (12 << 4),  //!< IDRIVEP_LS = 0.570A
    ISour_LS_0p680_A = (13 << 4),  //!< IDRIVEP_LS = 0.680A
    ISour_LS_0p820_A = (14 << 4),  //!< IDRIVEP_LS = 0.820A
    ISour_LS_1p000_A = (15 << 4)   //!< IDRIVEP_LS = 1.000A
} DRV8323_CTRL04_PeakSourCurLS_e;

//! \brief Enumeration for the low side gate drive peak sink current; TODO adapt current ratings
//!
typedef enum {
    ISink_LS_0p020_A = (0 << 0),   //!< IDRIVEN_LS = 0.020A
    ISink_LS_0p060_A = (1 << 0),   //!< IDRIVEN_LS = 0.060A
    ISink_LS_0p120_A = (2 << 0),   //!< IDRIVEN_LS = 0.120A
    ISink_LS_0p160_A = (3 << 0),   //!< IDRIVEN_LS = 0.160A
    ISink_LS_0p240_A = (4 << 0),   //!< IDRIVEN_LS = 0.240A
    ISink_LS_0p280_A = (5 << 0),   //!< IDRIVEN_LS = 0.280A
    ISink_LS_0p340_A = (6 << 0),   //!< IDRIVEN_LS = 0.340A
    ISink_LS_0p380_A = (7 << 0),   //!< IDRIVEN_LS = 0.380A
    ISink_LS_0p520_A = (8 << 0),   //!< IDRIVEN_LS = 0.520A
    ISink_LS_0p660_A = (9 << 0),   //!< IDRIVEN_LS = 0.660A
    ISink_LS_0p740_A = (10 << 0),  //!< IDRIVEN_LS = 0.740A
    ISink_LS_0p880_A = (11 << 0),  //!< IDRIVEN_LS = 0.880A
    ISink_LS_1p140_A = (12 << 0),  //!< IDRIVEN_LS = 1.140A
    ISink_LS_1p360_A = (13 << 0),  //!< IDRIVEN_LS = 1.360A
    ISink_LS_1p640_A = (14 << 0),  //!< IDRIVEN_LS = 1.640A
    ISink_LS_2p000_A = (15 << 0)   //!< IDRIVEN_LS = 2.000A
} DRV8323_CTRL04_PeakSinkCurLS_e;

//! \brief Enumeration for the VDS comparator threshold
//!
typedef enum {
    VDS_Level_0p060_V = (0 << 0),   //!< VDS_LEVEL = 0.060V
    VDS_Level_0p130_V = (1 << 0),   //!< VDS_LEVEL = 0.130V
    VDS_Level_0p200_V = (2 << 0),   //!< VDS_LEVEL = 0.200V
    VDS_Level_0p260_V = (3 << 0),   //!< VDS_LEVEL = 0.260V
    VDS_Level_0p310_V = (4 << 0),   //!< VDS_LEVEL = 0.310V
    VDS_Level_0p450_V = (5 << 0),   //!< VDS_LEVEL = 0.450V
    VDS_Level_0p530_V = (6 << 0),   //!< VDS_LEVEL = 0.530V
    VDS_Level_0p600_V = (7 << 0),   //!< VDS_LEVEL = 0.600V
    VDS_Level_0p680_V = (8 << 0),   //!< VDS_LEVEL = 0.680V
    VDS_Level_0p750_V = (9 << 0),   //!< VDS_LEVEL = 0.750V
    VDS_Level_0p940_V = (10 << 0),  //!< VDS_LEVEL = 0.940V
    VDS_Level_1p130_V = (11 << 0),  //!< VDS_LEVEL = 1.130V
    VDS_Level_1p300_V = (12 << 0),  //!< VDS_LEVEL = 1.300V
    VDS_Level_1p500_V = (13 << 0),  //!< VDS_LEVEL = 1.500V
    VDS_Level_1p700_V = (14 << 0),  //!< VDS_LEVEL = 1.700V
    VDS_Level_1p880_V = (15 << 0)   //!< VDS_LEVEL = 1.880V
} DRV8323_CTRL05_VDSLVL_e;

//! \brief Enumeration for the OCP/VDS sense deglitch time; TODO adapt deglitch time comments
//!
typedef enum {
    OCPDeg_2_us = (0 << 4),  //!< TVDS = 2us
    OCPDeg_4_us = (1 << 4),  //!< TVDS = 3us
    OCPDeg_6_us = (2 << 4),  //!< TVDS = 6us
    OCPDeg_8_us = (3 << 4)   //!< TVDS = 8us
} DRV8323_CTRL05_OcpDeg_e;

//! \brief Enumeration for the OCP report mode
//!
typedef enum {
    Latched_Shutdown = (0 << 6),  //!< OCP_MODE = Latched fault
    Automatic_Retry  = (1 << 6),  //!< OCP_MODE = Automatic Retry
    Report_Only      = (2 << 6),  //!< OCP_MODE = Report only
    Disable_OCP      = (3 << 6)   //!< OCP_MODE = Disabled
} DRV8323_CTRL05_OcpMode_e;

//! \brief Enumeration for the driver dead time
//!
typedef enum {
    DeadTime_50_ns  = (0 << 8),  //!< DEAD_TIME = 50ns
    DeadTime_100_ns = (1 << 8),  //!< DEAD_TIME = 100ns
    DeadTime_200_ns = (2 << 8),  //!< DEAD_TIME = 200ns
    DeadTime_400_ns = (3 << 8)   //!< DEAD_TIME = 400ns
} DRV8323_CTRL05_DeadTime_e;

//! \brief Enumeration for the Sense OCP level
//!
typedef enum {
    SEN_Lvl_Ocp_0p25 = (0 << 0),  //!< SEN_LVL = 0.25V
    SEN_Lvl_Ocp_0p50 = (1 << 0),  //!< SEN_LVL = 0.50V
    SEN_Lvl_Ocp_0p75 = (2 << 0),  //!< SEN_LVL = 0.75V
    SEN_Lvl_Ocp_1p00 = (3 << 0)   //!< SEN_LVL = 1.00V
} DRV8323_CTRL06_SENLevel_e;

//! \brief Enumeration for the gain of shunt amplifier
//!
typedef enum {
    Gain_5VpV  = (0 << 6),  //!< GAIN_CSA = 5V/V
    Gain_10VpV = (1 << 6),  //!< GAIN_CSA = 10V/V
    Gain_20VpV = (2 << 6),  //!< GAIN_CSA = 20V/V
    Gain_40VpV = (3 << 6)   //!< GAIN_CSA = 40V/V
} DRV8323_CTRL06_CSAGain_e;

//! \brief Enumeration for the register addresses
//!
typedef enum {
    Address_Status_0  = 0 << 11,  //!< Status Register 0
    Address_Status_1  = 1 << 11,  //!< Status Register 1
    Address_Control_2 = 2 << 11,  //!< Control Register 2
    Address_Control_3 = 3 << 11,  //!< Control Register 3
    Address_Control_4 = 4 << 11,  //!< Control Register 4
    Address_Control_5 = 5 << 11,  //!< Control Register 5
    Address_Control_6 = 6 << 11   //!< Control Register 6
} DRV8323_Address_e;

//! \brief Object for the DRV8323 STATUS00 register
//!
typedef struct _DRV_SPI_8323_Stat00_t_ {
    bool VDS_LC;   // Bits 0
    bool VDS_HC;   // Bits 1
    bool VDS_LB;   // Bits 2
    bool VDS_HB;   // Bits 3
    bool VDS_LA;   // Bits 4
    bool VDS_HA;   // Bits 5
    bool OTSD;     // Bits 6
    bool UVLO;     // Bits 7
    bool GDF;      // Bits 8
    bool VDS_OCP;  // Bits 9
    bool FAULT;    // Bits 10
} DRV_SPI_8323_Stat00_t_;

//! \brief Object for the DRV8323 STATUS01 register
//!
typedef struct _DRV_SPI_8323_Stat01_t_ {
    bool VGS_LC;  // Bits 0
    bool VGS_HC;  // Bits 1
    bool VGS_LB;  // Bits 2
    bool VGS_HB;  // Bits 3
    bool VGS_LA;  // Bits 4
    bool VGS_HA;  // Bits 5
    bool CPUV;    // Bits 6
    bool OTW;     // Bits 7
    bool SC_OC;   // Bits 8
    bool SB_OC;   // Bits 9
    bool SA_OC;   // Bits 10
} DRV_SPI_8323_Stat01_t_;

//! \brief Object for the DRV8323 CTRL02 register
//!
typedef struct _DRV_SPI_8323_Ctrl02_t_ {
    bool                     CLR_FLT;      // Bits 0
    bool                     BRAKE;        // Bits 1
    bool                     COAST;        // Bits 2
    bool                     PWM1_DIR;     // Bits 3
    bool                     PWM1_COM;     // Bits 4
    DRV8323_CTRL02_PwmMode_e PWM_MODE;     // Bits 6-5
    bool                     OTW_REP;      // Bits 7
    bool                     DIS_GDF;      // Bits 8
    bool                     DIS_CPUV;     // Bits 9
    bool                     CTRL02_RSV1;  // Bits 10
} DRV_SPI_8323_Ctrl02_t_;

//! \brief Object for the DRV8323 CTRL03 register
//!
typedef struct _DRV_SPI_8323_Ctrl03_t_ {
    DRV8323_CTRL03_PeakSinkCurHS_e IDRIVEN_HS;  // Bits 3-0
    DRV8323_CTRL03_PeakSourCurHS_e IDRIVEP_HS;  // Bits 7-4
    DRV8323_CTRL03_Lock_e          LOCK;        // Bits 10-8
} DRV_SPI_8323_Ctrl03_t_;

//! \brief Object for the DRV8323 CTRL04 register
//!
typedef struct _DRV_SPI_8323_Ctrl04_t_ {
    DRV8323_CTRL04_PeakSinkCurLS_e IDRIVEN_LS;  // Bits 3-0
    DRV8323_CTRL04_PeakSourCurLS_e IDRIVEP_LS;  // Bits 7-4
    DRV8323_CTRL04_PeakTime_e      TDRIVE;      // Bits 9-8
    bool                           CBC;         // Bits 10
} DRV_SPI_8323_Ctrl04_t_;

//! \brief Object for the DRV8323 CTRL05 register
//!
typedef struct _DRV_SPI_8323_Ctrl05_t_ {
    DRV8323_CTRL05_VDSLVL_e   VDS_LVL;    // Bits 3-0
    DRV8323_CTRL05_OcpDeg_e   OCP_DEG;    // Bits 5-4
    DRV8323_CTRL05_OcpMode_e  OCP_MODE;   // Bits 7-5
    DRV8323_CTRL05_DeadTime_e DEAD_TIME;  // Bits 9-8
    bool                      TRETRY;     // Bits 10
} DRV_SPI_8323_Ctrl05_t_;

//! \brief Object for the DRV8323 CTRL06 register
//!
typedef struct _DRV_SPI_8323_Ctrl06_t_ {
    DRV8323_CTRL06_SENLevel_e SEN_LVL;    // Bits 1-0
    bool                      CSA_CAL_C;  // Bits 2
    bool                      CSA_CAL_B;  // Bits 3
    bool                      CSA_CAL_A;  // Bits 4
    bool                      DIS_SEN;    // Bits 5
    DRV8323_CTRL06_CSAGain_e  CSA_GAIN;   // Bits 7-6
    bool                      LS_REF;     // Bits 8
    bool                      VREF_DIV;   // Bits 9
    bool                      CSA_FET;    // Bits 10
} DRV_SPI_8323_Ctrl06_t_;

//! \brief Object for the DRV8323 registers and commands
//!
typedef struct _DRV_SPI_8323_Vars_t_ {
    DRV_SPI_8323_Stat00_t_ Stat_Reg_00;
    DRV_SPI_8323_Stat01_t_ Stat_Reg_01;

    DRV_SPI_8323_Ctrl02_t_ Ctrl_Reg_02;
    DRV_SPI_8323_Ctrl03_t_ Ctrl_Reg_03;
    DRV_SPI_8323_Ctrl04_t_ Ctrl_Reg_04;
    DRV_SPI_8323_Ctrl05_t_ Ctrl_Reg_05;
    DRV_SPI_8323_Ctrl06_t_ Ctrl_Reg_06;
    bool                   WriteCmd;
    bool                   ReadCmd;

    uint16_t ManWriteAddr;
    uint16_t ManReadAddr;
    uint16_t ManWriteData;
    uint16_t ManReadData;
    bool     ManWriteCmd;
    bool     ManReadCmd;
} DRV_SPI_8323_Vars_t;

//! \brief Defines the DRV8323 object
//!
typedef struct _DRV8323_Obj_ {
    SPI_Handle    spiHandle;         //!< the handle for the serial peripheral interface
    GPIO_Handle   gpioHandle;        //!< the gpio handle that is connected to the drv8323 enable pin
    GPIO_Number_e enGateGpioNumber;  //!< the gpio number that is connected to the drv8323 enable pin
    bool          RxTimeOut;         //!< the timeout flag for the RX fifo
    bool          enableTimeOut;     //!< the timeout flag for drv8323 enable
} DRV8323_Obj;

//! \brief Defines the DRV8323 handle
//!
typedef struct _DRV8323_Obj_ *DRV8323_Handle;

//! \brief Defines the DRV8323 Word type
//!
typedef uint16_t DRV8323_Word_t;

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

//! \brief     Initializes the DRV8323 object
//! \param[in] pMemory   A pointer to the memory for the DRV8323 object
//! \param[in] numBytes  The number of bytes allocated for the DRV8323 object, bytes
//! \return    The DRV8305 object handle
extern DRV8323_Handle DRV8323_init(void *pMemory, const size_t numBytes);

//! \brief     Builds the control word
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \param[in] data      The data
//! \return    The control word
static inline DRV8323_Word_t DRV8323_buildCtrlWord(const DRV8323_CtrlMode_e ctrlMode,
                                                   const DRV8323_Address_e  regAddr,
                                                   const uint16_t           data) {
    DRV8323_Word_t ctrlWord = ctrlMode | regAddr | (data & DRV8323_DATA_MASK);

    return (ctrlWord);
}  // end of DRV8323_buildCtrlWord() function

//! \brief     Enables the DRV8305
//! \param[in] handle     The DRV8305 handle
extern void DRV8323_enable(DRV8323_Handle handle);

//! \brief     Gets the high side gate drive peak source current level
//! \param[in] handle     The DRV8305 handle
//! \return    The peak source current level, A
// extern DRV8305_CTRL05_PeakSourCurHS_e DRV8305_getPeakSourCurHS(DRV8305_Handle handle);

//! \brief     Gets the high side gate drive peak sink current level
//! \param[in] handle     The DRV8305 handle
//! \return    The peak sink current level, A
// extern DRV8305_CTRL05_PeakSinkCurHS_e DRV8305_getPeakSinkCurHS(DRV8305_Handle handle);

//! \brief     Gets the high side and low side gate drive peak source time
//! \param[in] handle     The DRV8305 handle
//! \return    The peak source time, ns
// extern DRV8305_CTRL05_PeakSourTime_e DRV8305_getPeakSourTime(DRV8305_Handle handle);

//! \brief     Gets the low side gate drive peak source current level
//! \param[in] handle     The DRV8305 handle
//! \return    The peak source current level, A
// extern DRV8305_CTRL06_PeakSourCurLS_e DRV8305_getPeakSourCurLS(DRV8305_Handle handle);

//! \brief     Gets the low side side gate drive peak sink current level
//! \param[in] handle     The DRV8305 handle
//! \return    The peak sink current level, A
// extern DRV8305_CTRL06_PeakSinkCurLS_e DRV8305_getPeakSinkCurLS(DRV8305_Handle handle);

//! \brief     Gets the high side and low side gate drive peak sink time
//! \param[in] handle     The DRV8305 handle
//! \return    The peak sink time, ns
// extern DRV8305_CTRL06_PeakSinkTime_e DRV8305_getPeakSinkTime(DRV8305_Handle handle);

//! \brief     Gets the VDS sense deglitch time
//! \param[in] handle     The DRV8305 handle
//! \return    The deglitch time time, us
// extern DRV8305_CTRL07_VDSDeglitch_e DRV8305_getVDSDeglitch(DRV8305_Handle handle);

//! \brief     Gets the VDS sense blanking time
//! \param[in] handle     The DRV8305 handle
//! \return    The blanking time time, us
// extern DRV8305_CTRL07_VDSBlanking_e DRV8305_getVDSBlanking(DRV8305_Handle handle);

//! \brief     Gets the driver dead time
//! \param[in] handle     The DRV8305 handle
//! \return    The dead time, ns
// extern DRV8305_CTRL07_DeadTime_e DRV8305_getDeadTime(DRV8305_Handle handle);

//! \brief     Gets the driver PWM mode
//! \param[in] handle     The DRV8305 handle
//! \return    The PWM mode
// extern DRV8305_CTRL07_PwmMode_e DRV8305_getPwmMode(DRV8305_Handle handle);

//! \brief     Gets the driver commutation mode for 1PWM mode
//! \param[in] handle     The DRV8305 handle
//! \return    The commutation mode
// extern DRV8305_CTRL07_CommOption_e DRV8305_getCommOption(DRV8305_Handle handle);

//! \brief     Gets the watchdog timer setting
//! \param[in] handle     The DRV8305 handle
//! \return    The watchdog timer setting, ms
// extern DRV8305_CTRL09_WatchDelay_e DRV8305_getWatchDelay(DRV8305_Handle handle);

//! \brief     Gets the gain of shunt amplifier 1
//! \param[in] handle     The DRV8305 handle
//! \return    The amplifier gain, V/V
// extern DRV8305_CTRL0A_CSGain1_e DRV8305_getCSGain1(DRV8305_Handle handle);

//! \brief     Gets the gain of shunt amplifier 2
//! \param[in] handle     The DRV8305 handle
//! \return    The amplifier gain, V/V
// extern DRV8305_CTRL0A_CSGain2_e DRV8305_getCSGain2(DRV8305_Handle handle);

//! \brief     Gets the gain of shunt amplifier 3
//! \param[in] handle     The DRV8305 handle
//! \return    The amplifier gain, V/V
// extern DRV8305_CTRL0A_CSGain3_e DRV8305_getCSGain3(DRV8305_Handle handle);

//! \brief     Gets the blanking time of the shunt amplifiers
//! \param[in] handle     The DRV8305 handle
//! \return    The amplifier blanking time, us
// extern DRV8305_CTRL0A_CSBlank_e DRV8305_getCSBlank(DRV8305_Handle handle);

//! \brief     Gets the DC calibration mode
//! \param[in] handle     The DRV8305 handle
//! \param[in] ampNumber  The shunt amplifier number
//! \return    The DC calibration mode
// extern DRV8305_CTRL0A_DcCalMode_e DRV8305_getDcCalMode(DRV8305_Handle handle,
//                                                 const DRV8305_ShuntAmpNumber_e ampNumber);

//! \brief     Gets the under voltage level of VREF
//! \param[in] handle     The DRV8305 handle
//! \return    The UV level, %
// extern DRV8305_CTRL0B_VregUvLevel_e DRV8305_getVregUvLevel(DRV8305_Handle handle);

//! \brief     Gets the delay time to power down VREF after SLEEP
//! \param[in] handle     The DRV8305 handle
//! \return    The delay time, us
// extern DRV8305_CTRL0B_SleepDelay_e DRV8305_getSleepDelay(DRV8305_Handle handle);

//! \brief     Gets the VREF scaling factor
//! \param[in] handle     The DRV8305 handle
//! \return    The VREF scaling factor
// extern DRV8305_CTRL0B_VrefScaling_e DRV8305_getVrefScaling(DRV8305_Handle handle);

//! \brief     Gets the VDS protection mode
//! \param[in] handle     The DRV8305 handle
//! \return    The VDS protection mode
// extern DRV8305_CTRL0C_VDSMode_e DRV8305_getVDSMode(DRV8305_Handle handle);

//! \brief     Gets the VDS OC level
//! \param[in] handle     The DRV8305 handle
//! \return    The VDS OC level, V
// extern DRV8305_CTRL0C_VDSLevel_e DRV8305_getVDSLevel(DRV8305_Handle handle);

//! \brief     Sets the SPI handle in the DRV8323
//! \param[in] handle     The DRV8323 handle
//! \param[in] spiHandle  The SPI handle to use
void DRV8323_setSpiHandle(DRV8323_Handle handle, SPI_Handle spiHandle);

//! \brief     Sets the GPIO handle in the DRV8323
//! \param[in] handle       The DRV8323 handle
//! \param[in] gpioHandle   The GPIO handle to use
void DRV8323_setGpioHandle(DRV8323_Handle handle, GPIO_Handle gpioHandle);

//! \brief     Sets the GPIO number in the DRV8323
//! \param[in] handle       The DRV8323 handle
//! \param[in] enGateGpioNumber   The GPIO number to use
void DRV8323_setEnGateGpioNumber(DRV8323_Handle handle, GPIO_Number_e enGateGpioNumber);

//! \brief     Sets the high side gate drive peak source current level
//! \param[in] handle   The DRV8305 handle
//! \param[in] level    The peak source current level, A
// extern void DRV8305_setPeakSourCurHS(DRV8305_Handle handle,const DRV8305_CTRL05_PeakSourCurHS_e level);

//! \brief     Sets the high side gate drive peak sink current level
//! \param[in] handle   The DRV8305 handle
//! \param[in] level    The peak sink current level, A
// extern void DRV8305_setPeakSinkCurHS(DRV8305_Handle handle,const DRV8305_CTRL05_PeakSinkCurHS_e level);

//! \brief     Sets the high side and low side gate drive peak source time
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The peak source time, ns
// extern void DRV8305_setPeakSourTime(DRV8305_Handle handle,const DRV8305_CTRL05_PeakSourTime_e time);

//! \brief     Sets the the low side gate drive peak source current level
//! \param[in] handle   The DRV8305 handle
//! \param[in] level    The peak source current level, A
// extern void DRV8305_setPeakSourCurLS(DRV8305_Handle handle,const DRV8305_CTRL06_PeakSourCurLS_e level);

//! \brief     Sets the low side side gate drive peak sink current level
//! \param[in] handle   The DRV8305 handle
//! \param[in] level    The peak sink current level, A
// extern void DRV8305_setPeakSinkCurLS(DRV8305_Handle handle,const DRV8305_CTRL06_PeakSinkCurLS_e level);

//! \brief     Sets the high side and low side gate drive peak sink time
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The peak sink time, ns
// extern void DRV8305_setPeakSinkTime(DRV8305_Handle handle,const DRV8305_CTRL06_PeakSinkTime_e time);

//! \brief     Sets the VDS sense deglitch time
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The deglitch time time, us
// extern void DRV8305_setVDSDeglitch(DRV8305_Handle handle,const DRV8305_CTRL07_VDSDeglitch_e time);

//! \brief     Sets the VDS sense blanking time
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The blanking time time, us
// extern void DRV8305_setVDSBlanking(DRV8305_Handle handle,const DRV8305_CTRL07_VDSBlanking_e time);

//! \brief     Sets the driver dead time
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The dead time, ns
// extern void DRV8305_setDeadTime(DRV8305_Handle handle,const DRV8305_CTRL07_DeadTime_e time);

//! \brief     Sets the driver PWM mode
//! \param[in] handle   The DRV8305 handle
//! \param[in] mode     The PWM mode
// extern void DRV8305_setPwmMode(DRV8305_Handle handle,const DRV8305_CTRL07_PwmMode_e mode);

//! \brief     Sets the driver commutation mode for 1PWM mode
//! \param[in] handle   The DRV8305 handle
//! \param[in] mode     The commutation mode
// extern void DRV8305_setCommOption(DRV8305_Handle handle,const DRV8305_CTRL07_CommOption_e mode);

//! \brief     Sets the watchdog timer setting
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The watchdog timer setting, ms
// extern void DRV8305_setWatchDelay(DRV8305_Handle handle,const DRV8305_CTRL09_WatchDelay_e time);

//! \brief     Sets the gain of shunt amplifier 1
//! \param[in] handle   The DRV8305 handle
//! \param[in] gain     The amplifier gain, V/V
// extern void DRV8305_setCSGain1(DRV8305_Handle handle,const DRV8305_CTRL0A_CSGain1_e gain);

//! \brief     Sets the gain of shunt amplifier 2
//! \param[in] handle   The DRV8305 handle
//! \param[in] gain     The amplifier gain, V/V
// extern void DRV8305_setCSGain2(DRV8305_Handle handle,const DRV8305_CTRL0A_CSGain2_e gain);

//! \brief     Sets the gain of shunt amplifier 3
//! \param[in] handle   The DRV8305 handle
//! \param[in] gain     The amplifier gain, V/V
// extern void DRV8305_setCSGain3(DRV8305_Handle handle,const DRV8305_CTRL0A_CSGain3_e gain);

//! \brief     Sets the blanking time of the shunt amplifiers
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The amplifier blanking time, us
// extern void DRV8305_setCSBlank(DRV8305_Handle handle,const DRV8305_CTRL0A_CSBlank_e time);

//! \brief     Sets the DC calibration mode
//! \param[in] handle     The DRV8305 handle
//! \param[in] ampNumber  The shunt amplifier number
//! \param[in] mode       The DC calibration mode
// extern void DRV8305_setDcCalMode(DRV8305_Handle handle,
//                                  const DRV8305_ShuntAmpNumber_e ampNumber,
//                                  const DRV8305_CTRL0A_DcCalMode_e mode);

//! \brief     Sets the under voltage level of VREF
//! \param[in] handle   The DRV8305 handle
//! \param[in] level    The UV level, %
// extern void DRV8305_setVregUvLevel(DRV8305_Handle handle,const DRV8305_CTRL0B_VregUvLevel_e level);

//! \brief     Sets the delay time to power down VREF after SLEEP
//! \param[in] handle   The DRV8305 handle
//! \param[in] time     The delay time, us
// extern void DRV8305_setSleepDelay(DRV8305_Handle handle,const DRV8305_CTRL0B_SleepDelay_e time);

//! \brief     Sets the VREF scaling factor
//! \param[in] handle   The DRV8305 handle
//! \param[in] mode     The VREF scaling factor
// extern void DRV8305_setVrefScaling(DRV8305_Handle handle,const DRV8305_CTRL0B_VrefScaling_e mode);

//! \brief     Sets the VDS protection mode
//! \param[in] handle   The DRV8305 handle
//! \param[in] mode     The VDS protection mode
// extern void DRV8305_setVDSMode(DRV8305_Handle handle,const DRV8305_CTRL0C_VDSMode_e mode);

//! \brief     Sets the VDS OC level
//! \param[in] handle   The DRV8305 handle
//! \param[in] level    The VDS OC level, V
// extern void DRV8305_setVDSLevel(DRV8305_Handle handle,const DRV8305_CTRL0C_VDSLevel_e level);

//! \brief     Determines if DRV8305 fault has occurred
//! \param[in] handle     The DRV8305 handle
//! \return    A boolean value denoting if a fault has occurred (true) or not (false)
// extern bool DRV8305_isFault(DRV8305_Handle handle);

//! \brief     Resets the DRV8305
//! \param[in] handle   The DRV8305 handle
// extern void DRV8305_reset(DRV8305_Handle handle);

//! \brief     Resets the enable timeout flag
//! \param[in] handle   The DRV8323 handle
static inline void DRV8323_resetEnableTimeout(DRV8323_Handle handle) {
    DRV8323_Obj *obj = (DRV8323_Obj *)handle;

    obj->enableTimeOut = false;

    return;
}  // end of DRV8323_resetEnableTimeout() function

//! \brief     Resets the RX fifo timeout flag
//! \param[in] handle   The DRV8323 handle
static inline void DRV8323_resetRxTimeout(DRV8323_Handle handle) {
    DRV8323_Obj *obj = (DRV8323_Obj *)handle;

    obj->RxTimeOut = false;

    return;
}  // end of DRV8323_resetRxTimeout() function

//! \brief     Initialize the interface to all 8323 SPI variables
//! \param[in] handle  The DRV8323 handle
extern void DRV8323_setupSpi(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars);

//! \brief     Reads data from the DRV8323 register
//! \param[in] handle   The DRV8323 handle
//! \param[in] regAddr  The register address
//! \return    The data value
extern uint16_t DRV8323_readSpi(DRV8323_Handle handle, const DRV8323_Address_e regAddr);

//! \brief     Writes data to the DRV8323 register
//! \param[in] handle   The DRV8323 handle
//! \param[in] regAddr  The register name
//! \param[in] data     The data value
extern void DRV8323_writeSpi(DRV8323_Handle handle, const DRV8323_Address_e regAddr, const uint16_t data);

//! \brief     Write to the DRV8323 SPI registers
//! \param[in] handle  The DRV8323 handle
//! \param[in] Spi_8323_Vars  The (DRV_SPI_8323_Vars_t) structure that contains all DRV8323 Status/Control register options
extern void DRV8323_writeData(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars);

//! \brief     Read from the DRV8323 SPI registers
//! \param[in] handle  The DRV8323 handle
//! \param[in] Spi_8323_Vars  The (DRV_SPI_8323_Vars_t) structure that contains all DRV8323 Status/Control register options
extern void DRV8323_readData(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars);

static inline void PackStatusRegisters(const DRV_SPI_8323_Vars_t *drvVars, uint16_t fault[]) {
    fault[0] = (drvVars->Stat_Reg_00.VDS_LC << 0) |
               (drvVars->Stat_Reg_00.VDS_HC << 1) |
               (drvVars->Stat_Reg_00.VDS_LB << 2) |
               (drvVars->Stat_Reg_00.VDS_HB << 3) |
               (drvVars->Stat_Reg_00.VDS_LA << 4) |
               (drvVars->Stat_Reg_00.VDS_HA << 5) |
               (drvVars->Stat_Reg_00.OTSD << 6) |
               (drvVars->Stat_Reg_00.UVLO << 7) |
               (drvVars->Stat_Reg_00.GDF << 8) |
               (drvVars->Stat_Reg_00.VDS_OCP << 9) |
               (drvVars->Stat_Reg_00.FAULT << 10);

    fault[1] = (drvVars->Stat_Reg_01.VGS_LC << 0) |
               (drvVars->Stat_Reg_01.VGS_HC << 1) |
               (drvVars->Stat_Reg_01.VGS_LB << 2) |
               (drvVars->Stat_Reg_01.VGS_HB << 3) |
               (drvVars->Stat_Reg_01.VGS_LA << 4) |
               (drvVars->Stat_Reg_01.VGS_HA << 5) |
               (drvVars->Stat_Reg_01.CPUV << 6) |
               (drvVars->Stat_Reg_01.OTW << 7) |
               (drvVars->Stat_Reg_01.SC_OC << 8) |
               (drvVars->Stat_Reg_01.SB_OC << 9) |
               (drvVars->Stat_Reg_01.SA_OC << 10);
}

#ifdef __cplusplus
}
#endif  // extern "C"

//@}  // ingroup

#endif  // end of _DRV8305_H_ definition
