/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 */

// system includes
#include "main.h"
#include <math.h>

#ifdef BEEP
#include "beep.h"
#include <note.h>
#endif

#ifdef EEPROM_EMULATION
#include <flash_utils.h>
#endif

#include "ADC_temp.h"
#include <proj_utils.h>

#ifdef ECAN
#include "can.h"
#include "fifo.h"
#endif

#include "dronecan.h"

#ifdef FLASH
#pragma CODE_SECTION(mainISR, "ramfuncs");
#endif

#ifdef ECAN
ECAN_Mailbox gECAN_Mailbox;
FIFO_ID_Obj  gECAN_rxFIFO_ID;
FIFO_ID_Obj  gECAN_txFIFO_ID;

uint32_t ecan_int0count = 0;    // Counter to track the # of level 0 interrupts
uint32_t ecan_int1count = 0;    // Counter to track the # of level 1 interrupts
#endif

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

CTRL_Handle ctrlHandle;

#ifdef CSM_ENABLE
#pragma DATA_SECTION(halHandle, "rom_accessed_data");
#endif

HAL_Handle halHandle;

#ifdef CSM_ENABLE
#pragma DATA_SECTION(gUserParams, "rom_accessed_data");
#endif

#pragma DATA_SECTION(gUserBaseParams_flash, "persistent_memory");

const USER_Params_FLASH gUserBaseParams_flash = USER_FLASH_INIT;

USER_Params_FLASH gUserBaseParams;
USER_Params       gUserParams;

USER_Torque gTorque = {_IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_PwmData_t gPwmData = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

HAL_AdcData_t gAdcData;

_iq gMaxCurrentSlope = _IQ(0.0);

#ifdef FAST_ROM_V1p6
CTRL_Obj *controller_obj;
#else

#ifdef CSM_ENABLE
#pragma DATA_SECTION(ctrl, "rom_accessed_data");
#endif

CTRL_Obj ctrl;    // v1p7 format
#endif

uint16_t gLEDcnt = 0;

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

#ifdef CSM_ENABLE
extern uint16_t *econst_start, *econst_end, *econst_ram_load;
extern uint16_t *switch_start, *switch_end, *switch_ram_load;
#endif
#endif

#ifdef OVERMOD
SVGENCURRENT_Obj    svgencurrent;
SVGENCURRENT_Handle svgencurrentHandle;

// set the offset, default value of 1 microsecond
int16_t gCmpOffset = (int16_t)(1.0 * nUSER_SYSTEM_FREQ_MHz);

MATH_vec3 gIavg         = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};
uint16_t  gIavg_shift   = 1;
MATH_vec3 gPwmData_prev = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};
#endif

#ifdef BEEP
Beep_Obj       bp;
Beep_Handle    beepHandle;
const uint16_t gFreq[]  = INIT_FREQ;
const uint16_t gBeats[] = INIT_BEATS;
#endif

FaultPhase_e FaultState = PHASE_READING;

FW_Obj    fw;
FW_Handle fwHandle;

// #define FW_INC_DELTA _IQ(0.001)
// #define FW_DEC_DELTA _IQ(0.001)

_iq Iq_Max_pu;

#ifdef DRV83XX
DRV_SPI_83XX_Vars_t gDrvSpi83xxVars;
#endif

#ifdef CPU_USAGE
void            updateCPUusage(void);
CPU_TIME_Handle cpu_timeHandle;
CPU_TIME_Obj    cpu_time;
float_t         gCpu_usage_den = 0.0;
uint32_t        gTimer1Cnt     = 0;
#endif

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;

// **************************************************************************
// the functions

void FaultHandler(void);

int measureTemperatureC() {
    uint16_t chanBak = halHandle->adcHandle->ADCSOCxCTL[ADC_SocNumber_0];

    // connect channel A5 internally to the temperature sensor
    ADC_setTempSensorSrc(halHandle->adcHandle, ADC_TempSensorSrc_Int);
    // set SOC0 channel select to ADCINA5
    ADC_setSocChanNumber(halHandle->adcHandle, ADC_SocNumber_0, ADC_SocChanNumber_A5);

    uint16_t temp = ADC_readResult(halHandle->adcHandle, ADC_ResultNumber_0);

    halHandle->adcHandle->ADCSOCxCTL[ADC_SocNumber_0] = chanBak;

    return (int)ADC_getTemperatureK(halHandle->adcHandle, temp);
}

void main(void) {
    uint_least8_t estNumber = 0;

#ifdef FAST_ROM_V1p6
    uint_least8_t ctrlNumber = 0;
#endif

// Only used if running from FLASH
// Note that the variable FLASH is defined by the project
#ifdef FLASH
    // Copy time critical code and Flash setup code to RAM
    // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the linker files.
    memCopy((uint16_t *)&RamfuncsLoadStart, (uint16_t *)&RamfuncsLoadEnd, (uint16_t *)&RamfuncsRunStart);

#ifdef CSM_ENABLE
    // copy .econst to unsecure RAM
    if (*econst_end - *econst_start) {
        memCopy((uint16_t *)&econst_start, (uint16_t *)&econst_end, (uint16_t *)&econst_ram_load);
    }

    // copy .switch to unsecure RAM
    if (*switch_end - *switch_start) {
        memCopy((uint16_t *)&switch_start, (uint16_t *)&switch_end, (uint16_t *)&switch_ram_load);
    }
#endif
#endif

    Flash_Init();

    // initialize the user parameters
    gUserBaseParams = gUserBaseParams_flash;
    copyUserParams_FLASH_to_Params(&gUserBaseParams, &gUserParams);
    USER_setOtherParams(&gUserParams);

    // initialize the hardware abstraction layer
    halHandle = HAL_init(&hal, sizeof(hal));

    // check for errors in user parameters
    USER_checkForErrors(&gUserParams);

    updateMotorVars(&gUserParams, &gMotorVars);
    gMotorVars.SpeedRef_krpm   = _IQ(0.0);
    gMotorVars.MaxAccel_krpmps = gUserBaseParams.UserMaxAccel_krpmps;

    gUserParams.copyActiveParamsToUserBase = false;
    gUserParams.saveUserParams             = false;
    gUserParams.writtenSuccessfully        = false;
    gUserParams.Motor_Auto_ID              = false;

    // store user parameter error in global variable
    gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);

    // set the hardware abstraction layer parameters
    HAL_setParams(halHandle, &gUserParams);

    // initialize the controller
#ifdef FAST_ROM_V1p6
    ctrlHandle     = CTRL_initCtrl(ctrlNumber, estNumber);    // v1p6 format (06xF and 06xM devices)
    controller_obj = (CTRL_Obj *)ctrlHandle;
#else
    ctrlHandle = CTRL_initCtrl(estNumber, &ctrl, sizeof(ctrl));    // v1p7 format default
#endif

    {
        CTRL_Version version;

        // get the version number
        CTRL_getVersion(ctrlHandle, &version);

        gMotorVars.CtrlVersion = version;
    }

    // set the default controller parameters
    CTRL_setParams(ctrlHandle, &gUserParams);

    // Initialize field weakening
    fwHandle = FW_init(&fw, sizeof(fw));

    // Disable field weakening
    FW_setFlag_enableFw(fwHandle, false);

    // Clear field weakening counter
    FW_clearCounter(fwHandle);

    // Set the number of ISR per field weakening ticks
    FW_setNumIsrTicksPerFwTick(fwHandle, FW_NUM_ISR_TICKS_PER_CTRL_TICK);

    // Set the deltas of field weakening
    FW_setDeltas(fwHandle, FW_INC_DELTA, FW_DEC_DELTA);

    // Set initial output of field weakening to zero
    FW_setOutput(fwHandle, _IQ(0.0));

    // Set the field weakening controller limits
    FW_setMinMax(fwHandle, _IQ(gUserParams.maxNegativeIdCurrent_a / gUserParams.iqFullScaleCurrent_A), _IQ(0.0));

#ifdef OVERMOD
    // Initialize and setup the 100% SVM generator
    svgencurrentHandle = SVGENCURRENT_init(&svgencurrent, sizeof(svgencurrent));

    // setup svgen current
    {
        float_t  minWidth_microseconds = 2.0;
        uint16_t minWidth_counts       = (uint16_t)(minWidth_microseconds * nUSER_SYSTEM_FREQ_MHz);
        float_t  fdutyLimit            = 0.5 - (2.0 * minWidth_microseconds * gUserParams.PwmFreq_kHz * 0.001);
        _iq      dutyLimit             = _IQ(fdutyLimit);

        SVGENCURRENT_setMinWidth(svgencurrentHandle, minWidth_counts);
        SVGENCURRENT_setIgnoreShunt(svgencurrentHandle, use_all);
        SVGENCURRENT_setMode(svgencurrentHandle, all_phase_measurable);
        SVGENCURRENT_setVlimit(svgencurrentHandle, dutyLimit);
    }

    // set overmodulation to maximum value
    gMotorVars.OverModulation = _IQ(MATH_TWO_OVER_THREE);
#endif

    // setup faults
    HAL_setupFaults(halHandle);

    // initialize the interrupt vector table
    HAL_initIntVectorTable(halHandle);

    // enable the ADC interrupts
    HAL_enableAdcInts(halHandle);

#ifdef ECAN
    initializeFIFO(&gECAN_rxFIFO_ID);
    initializeFIFO(&gECAN_txFIFO_ID);
    ECAN_setup(halHandle, &gECAN_Mailbox, gECAN_rxFIFO_ID, gECAN_txFIFO_ID, gUserBaseParams.ecan_node_id);

    // enable ECAN interrupts
    HAL_enableEcanInt(halHandle);
#endif    // ECAN

#ifdef nFAULT_INT
    HAL_enableFaultInt(halHandle);
#endif

    // enable timer 0 interrupts
    HAL_enableTimer0Int(halHandle);

    // enable global interrupts
    HAL_enableGlobalInts(halHandle);

    // enable debug interrupts
    HAL_enableDebugInt(halHandle);

    // disable the PWM
    HAL_disablePwm(halHandle);

    HAL_enableDrv(halHandle);

    // turn on the DRV83XX if present
#ifdef DRV83XX
    // initialize the DRV83XX interface
    HAL_setupDrvSpi(halHandle, &gDrvSpi83xxVars);
    // configure the DRV83XX
    HAL_configDrv(halHandle, &gDrvSpi83xxVars);
#endif
    // enable DC bus compensation
    CTRL_setFlag_enableDcBusComp(ctrlHandle, true);

    // compute scaling factors for flux and torque calculations
    gFlux_pu_to_Wb_sf            = USER_computeFlux_pu_to_Wb_sf(&gUserParams);
    gFlux_pu_to_VpHz_sf          = USER_computeFlux_pu_to_VpHz_sf(&gUserParams);
    gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(&gUserParams);
    gTorque_Flux_Iq_pu_to_Nm_sf  = USER_computeTorque_Flux_Iq_pu_to_Nm_sf(&gUserParams);

    // Make sure the motor isn't running when turned on

    dronecan_init(halHandle, &gECAN_rxFIFO_ID, &gECAN_Mailbox);

    extractFlagsFromControlWord(&gUserBaseParams, &gMotorVars);

    gMotorVars.Flag_enableSys        = true;
    gMotorVars.Flag_enableUserParams = true;
    gMotorVars.Flag_nFault           = false;
    gMotorVars.Flag_nFaultDroneCan   = false;

#ifdef CPU_USAGE
    cpu_timeHandle = CPU_TIME_init(&cpu_time, sizeof(cpu_time));
    CPU_TIME_setParams(cpu_timeHandle, HAL_readPwmPeriod(halHandle, PWM_Number_1));
    gCpu_usage_den = (float_t)cpu_timeHandle->pwm_period * (float_t)gUserParams.numPwmTicksPerIsrTick * 2.0;
#endif

#ifdef BEEP
    //    Beep_init(beepHandle);
    beepHandle = Beep_init(&bp, sizeof(bp));
#endif

    for (;;) {
        // Waiting for enable system flag to be set
        while (!(gMotorVars.Flag_enableSys)) {
#ifdef EEPROM_EMULATION
            gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);
            if (gUserParams.copyActiveParamsToUserBase == true) {
                gUserParams.copyActiveParamsToUserBase = false;
                USER_checkForErrors(&gUserParams);
            }
            if (gUserParams.saveUserParams == true) {
                gUserParams.saveUserParams = false;
                HAL_disableGlobalInts(halHandle);
                gUserParams.writtenSuccessfully = Flash_SaveUserBaseParams(&gUserBaseParams);
                HAL_enableGlobalInts(halHandle);
            }
#endif
#ifdef CPU_USAGE
            // update CPU usage
            updateCPUusage();
#endif
            dronecan_update();
        }

        Flag_Latch_softwareUpdate = true;
        // Enable the Library internal PI.  Iq is referenced by the speed PI now
        CTRL_setFlag_enableSpeedCtrl(ctrlHandle, true);
        // loop while the enable system flag is true
        while (gMotorVars.Flag_enableSys) {

#ifdef ADC8_SUPPORT
            ADC8_FUNCTIONALITY;
#endif

#ifdef ADC9_SUPPORT
            ADC9_FUNCTIONALITY;
#endif

#ifdef DRV_TEMPERATURE
            if (gMotorVars.TempSenDegCelsius > OVERHEAT) {
                gMotorVars.Flag_Overheat = true;
            }
#endif

#ifdef CPU_USAGE
            // update CPU usage
            updateCPUusage();
#endif

#ifdef BEEP
            BeepOffset(beepHandle, gUserParams, gMotorVars);
#endif
            dronecan_update();

            CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

            // increment counters
            gCounter_updateGlobals++;

            // enable/disable the use of motor parameters being loaded from user.h
            CTRL_setFlag_enableUserMotorParams(ctrlHandle, gMotorVars.Flag_enableUserParams);

            // enable/disable Rs recalibration during motor startup
            EST_setFlag_enableRsRecalc(obj->estHandle, gMotorVars.Flag_enableRsRecalc);

            // enable/disable automatic calculation of bias values
            CTRL_setFlag_enableOffset(ctrlHandle, gMotorVars.Flag_enableOffsetcalc);

            if (CTRL_isError(ctrlHandle)) {
                // set the enable controller flag to false
                CTRL_setFlag_enableCtrl(ctrlHandle, false);

                // set the enable system flag to false
                gMotorVars.Flag_enableSys = false;

                // disable the PWM
                HAL_disablePwm(halHandle);
            }
            else {
                // update the controller state
                bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

                // enable or disable the control
                CTRL_setFlag_enableCtrl(ctrlHandle, gMotorVars.Flag_Run_Identify);

                if (flag_ctrlStateChanged) {
                    CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

                    if (ctrlState == CTRL_State_OffLine) {
                        // enable the PWM
                        HAL_enablePwm(halHandle);
                    }
                    else if (ctrlState == CTRL_State_OnLine) {
                        if (gMotorVars.Flag_enableOffsetcalc == true) {
                            // update the ADC bias values
                            HAL_updateAdcBias(halHandle);
                        }
                        else {
                            // set the current bias
                            HAL_setBias(halHandle, HAL_SensorType_Current, 0, _IQ(I_A_offset));
                            HAL_setBias(halHandle, HAL_SensorType_Current, 1, _IQ(I_B_offset));
                            HAL_setBias(halHandle, HAL_SensorType_Current, 2, _IQ(I_C_offset));

                            // set the voltage bias
                            HAL_setBias(halHandle, HAL_SensorType_Voltage, 0, _IQ(V_A_offset));
                            HAL_setBias(halHandle, HAL_SensorType_Voltage, 1, _IQ(V_B_offset));
                            HAL_setBias(halHandle, HAL_SensorType_Voltage, 2, _IQ(V_C_offset));
                        }

                        // Return the bias value for currents
                        gMotorVars.I_bias.value[0] = HAL_getBias(halHandle, HAL_SensorType_Current, 0);
                        gMotorVars.I_bias.value[1] = HAL_getBias(halHandle, HAL_SensorType_Current, 1);
                        gMotorVars.I_bias.value[2] = HAL_getBias(halHandle, HAL_SensorType_Current, 2);

                        // Return the bias value for voltages
                        gMotorVars.V_bias.value[0] = HAL_getBias(halHandle, HAL_SensorType_Voltage, 0);
                        gMotorVars.V_bias.value[1] = HAL_getBias(halHandle, HAL_SensorType_Voltage, 1);
                        gMotorVars.V_bias.value[2] = HAL_getBias(halHandle, HAL_SensorType_Voltage, 2);

                        // enable the PWM
                        HAL_enablePwm(halHandle);
                    }
                    else if (ctrlState == CTRL_State_Idle) {
                        // disable the PWM
                        HAL_disablePwm(halHandle);
                        gMotorVars.Flag_Run_Identify = false;
                    }

                    if ((CTRL_getFlag_enableUserMotorParams(ctrlHandle) == true) &&
                        (ctrlState > CTRL_State_Idle) &&
                        (gMotorVars.CtrlVersion.minor == 6)) {
                        // call this function to fix 1p6
                        USER_softwareUpdate1p6(ctrlHandle, &gUserParams);
                    }
                }
            }

            if (EST_isMotorIdentified(obj->estHandle)) {
                _iq Id_squared_pu = _IQmpy(CTRL_getId_ref_pu(ctrlHandle),
                                           CTRL_getId_ref_pu(ctrlHandle));
#ifndef OVERMOD
                _iq Is_Max_squared_pu = _IQ((gUserParams.maxCurrent * gUserParams.maxCurrent) /
                                            (gUserParams.iqFullScaleCurrent_A * gUserParams.iqFullScaleCurrent_A));

                // Take into consideration that Iq^2+Id^2 = Is^2
                Iq_Max_pu = _IQsqrt(Is_Max_squared_pu - Id_squared_pu);

                // Set new max trajectory
                CTRL_setSpdMax(ctrlHandle, Iq_Max_pu);
#else
                // Set the maximum current controller output for the Iq and Id current
                // controllers to enable over-modulation. An input into the SVM above
                // 1/SQRT(3) = 0.5774 is in the over-modulation region.  An input of
                // 0.5774 is where the crest of the sinewave touches the 100% duty
                // cycle.  At an input of 2/3, the SVM generator produces a trapezoidal
                // waveform touching every corner of the hexagon
                CTRL_setMaxVsMag_pu(ctrlHandle, gMotorVars.OverModulation);
#endif
                // set the current ramp
                EST_setMaxCurrentSlope_pu(obj->estHandle, gMaxCurrentSlope);
                gMotorVars.Flag_MotorIdentified = true;

                // set the speed reference
                CTRL_setSpd_ref_krpm(ctrlHandle, gMotorVars.SpeedRef_krpm);

                // set the speed acceleration
                CTRL_setMaxAccel_pu(ctrlHandle, _IQmpy(gUserParams.maxAccel_krpmps_Sf, gMotorVars.MaxAccel_krpmps));
                
                // set the Id reference
                CTRL_setId_ref_pu(ctrlHandle, _IQmpy(gMotorVars.IdRef_A, _IQ(1.0 / gUserParams.iqFullScaleCurrent_A)));

                if (Flag_Latch_softwareUpdate) {
                    Flag_Latch_softwareUpdate = false;

                    USER_calcPIgains(ctrlHandle, &gUserParams);

                    // initialize the watch window kp and ki current values with pre-calculated values
                    gMotorVars.Kp_Idq = CTRL_getKp(ctrlHandle, CTRL_Type_PID_Id);
                    gMotorVars.Ki_Idq = CTRL_getKi(ctrlHandle, CTRL_Type_PID_Id);
                }
            }
            else {
                Flag_Latch_softwareUpdate = true;

                // initialize the watch window kp and ki values with pre-calculated values

                if (gUserBaseParams.kp_spd == _IQ(0.0f) && gUserBaseParams.ki_spd == _IQ(0.0f)) {
                    gMotorVars.Kp_spd = CTRL_getKp(ctrlHandle, CTRL_Type_PID_spd);
                    gMotorVars.Ki_spd = CTRL_getKi(ctrlHandle, CTRL_Type_PID_spd);
                }

                // the estimator sets the maximum current slope during identification
                gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
            }

            // when appropriate, update the global variables
            if (gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE) {
                // reset the counter
                gCounter_updateGlobals = 0;

                updateGlobalVariables_motor(ctrlHandle);
            }

            // update Kp and Ki gains
            updateKpKiGains(ctrlHandle);

            // set field weakening enable flag depending on user's input
            FW_setFlag_enableFw(fwHandle, gMotorVars.Flag_enableFieldWeakening);

            // enable/disable the forced angle
            EST_setFlag_enableForceAngle(obj->estHandle, gMotorVars.Flag_enableForceAngle);

            // enable or disable power warp
            CTRL_setFlag_enablePowerWarp(ctrlHandle, gMotorVars.Flag_enablePowerWarp);

            FaultHandler();

#ifdef DRV83XX
            HAL_writeDrvData(halHandle, &gDrvSpi83xxVars);

            HAL_readDrvData(halHandle, &gDrvSpi83xxVars);
#endif

        }    // end of while(gFlag_enableSys) loop

        // disable the PWM
        HAL_disablePwm(halHandle);

        // set the default controller parameters (Reset the control to re-identify the motor)
        CTRL_setParams(ctrlHandle, &gUserParams);
        gMotorVars.Flag_Run_Identify = false;
    }    // end of for(;;) loop

}    // end of main() function

interrupt void mainISR(void) {
#ifdef OVERMOD
    SVGENCURRENT_IgnoreShunt_e ignoreShuntThisCycle = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle);
#endif
#ifdef CPU_USAGE
    gTimer1Cnt = HAL_readTimerCnt(halHandle, 2);
    CPU_TIME_updateCnts(cpu_timeHandle, gTimer1Cnt);
#endif

    // toggle status LED
#ifdef gLED2
    if (++gLEDcnt >= gUserParams.isrFreq_Hz / gUserBaseParams.ledBlinkFreq_Hz) {
        HAL_toggleLed(halHandle, (GPIO_Number_e)HAL_Gpio_LED2);
        gLEDcnt = 0;
    }
#endif

    // acknowledge the ADC interrupt
    HAL_acqAdcInt(halHandle, ADC_IntNumber_1);

    // convert the ADC data
    HAL_readAdcData(halHandle, &gAdcData);

#ifdef OVERMOD
    // run the current reconstruction algorithm
    SVGENCURRENT_RunRegenCurrent(svgencurrentHandle,
                                 (MATH_vec3 *)(gAdcData.I.value));

    gIavg.value[0] += (gAdcData.I.value[0] - gIavg.value[0]) >> gIavg_shift;
    gIavg.value[1] += (gAdcData.I.value[1] - gIavg.value[1]) >> gIavg_shift;
    gIavg.value[2] += (gAdcData.I.value[2] - gIavg.value[2]) >> gIavg_shift;

    if (ignoreShuntThisCycle > use_all) {
        gAdcData.I.value[0] = gIavg.value[0];
        gAdcData.I.value[1] = gIavg.value[1];
        gAdcData.I.value[2] = gIavg.value[2];
    }
#endif

    // run the controller
    CTRL_run(ctrlHandle, halHandle, &gAdcData, &gPwmData, &gUserParams);

#ifdef OVERMOD
    // run the PWM compensation and current ignore algorithm
    SVGENCURRENT_compPwmData(svgencurrentHandle, &(gPwmData.Tabc), &gPwmData_prev);
#endif

#ifdef BEEP
    BeepStop(beepHandle, halHandle, gUserParams);
    BeepSet(beepHandle, gUserParams, &gPwmData);
    BeepStart(beepHandle, gFreq, gBeats);
#endif

    // write the PWM compare values
    HAL_writePwmData(halHandle, &gPwmData);

#ifdef OVERMOD
    {
        SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle);
        SVGENCURRENT_VmidShunt_e   midVolShunt          = SVGENCURRENT_getVmid(svgencurrentHandle);

        // Set trigger point in the middle of the low side pulse
        HAL_setTrigger(halHandle, ignoreShuntNextCycle, midVolShunt);
    }
#endif

    if (FW_getFlag_enableFw(fwHandle) == true) {
        // enable field weakening && gMotorVars.Speed_krpm > gUserParams.FW_Speed_Krpm) {
        FW_incCounter(fwHandle);

        if (FW_getCounter(fwHandle) > FW_getNumIsrTicksPerFwTick(fwHandle)) {
            _iq refValue;
            _iq fbackValue;
            _iq output;

            FW_clearCounter(fwHandle);

            refValue = gMotorVars.VsRef;

            fbackValue = gMotorVars.Vs;

            FW_run(fwHandle, refValue, fbackValue, &output);

            CTRL_setId_ref_pu(ctrlHandle, output);

            float Id_ref_pu = _IQtoF(CTRL_getId_ref_pu(ctrlHandle));

            // gMotorVars.IdRef_A = _IQmpy(CTRL_getId_ref_pu(ctrlHandle), _IQ(gUserParams.iqFullScaleCurrent_A));
            gMotorVars.IdRef_A = _IQ(Id_ref_pu * gUserParams.iqFullScaleCurrent_A);
        }
    }
    else {
        CTRL_setId_ref_pu(ctrlHandle, _IQmpy(gMotorVars.IdRef_A, _IQ(1.0 / gUserParams.iqFullScaleCurrent_A)));
    }

    // setup the controller
    CTRL_setup(ctrlHandle);
#ifdef CPU_USAGE
    gTimer1Cnt = HAL_readTimerCnt(halHandle, 2);
    CPU_TIME_run(cpu_timeHandle, gTimer1Cnt);
#endif
    return;
}    // end of mainISR() function

#pragma CODE_SECTION(updateGlobalVariables_motor, "ramfuncs");

void updateGlobalVariables_motor(CTRL_Handle handle) {
    CTRL_Obj *obj = (CTRL_Obj *)handle;
    int32_t   tmp;

    // get the speed estimate
    gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

    // get the real time speed reference coming out of the speed trajectory generator
    gMotorVars.SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle), EST_get_pu_to_krpm_sf(obj->estHandle));

    // get the torque estimate
    // gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);
    gMotorVars.Torque_Nm = USER_computeTorque_Nm_S(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf, &gTorque);

    // when calling EST_ functions that return a float, and fpu32 is enabled, an integer is needed as a return
    // so that the compiler reads the returned value from the accumulator instead of fpu32 registers
    // get the magnetizing current
    tmp                   = EST_getIdRated(obj->estHandle);
    gMotorVars.MagnCurr_A = *((float_t *)&tmp);

    // get the rotor resistance
    tmp               = EST_getRr_Ohm(obj->estHandle);
    gMotorVars.Rr_Ohm = *((float_t *)&tmp);

    // get the stator resistance
    tmp               = EST_getRs_Ohm(obj->estHandle);
    gMotorVars.Rs_Ohm = *((float_t *)&tmp);

    // get the stator inductance in the direct coordinate direction
    tmp              = EST_getLs_d_H(obj->estHandle);
    gMotorVars.Lsd_H = *((float_t *)&tmp);

    // get the stator inductance in the quadrature coordinate direction
    tmp              = EST_getLs_q_H(obj->estHandle);
    gMotorVars.Lsq_H = *((float_t *)&tmp);

    // get the flux in V/Hz in floating point
    tmp                  = EST_getFlux_VpHz(obj->estHandle);
    gMotorVars.Flux_VpHz = *((float_t *)&tmp);

    // get the flux in Wb in fixed point
    gMotorVars.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

    // get the controller state
    gMotorVars.CtrlState = CTRL_getState(handle);

    // get the estimator state
    gMotorVars.EstState = EST_getState(obj->estHandle);

    // read Vd and Vq vectors per units
    // gMotorVars.Vd = CTRL_getVd_out_pu(ctrlHandle);
    // gMotorVars.Vq = CTRL_getVq_out_pu(ctrlHandle);

    // calculate vector Vs in per units
    // gMotorVars.Vs = _IQsqrt(_IQmpy(gMotorVars.Vd, gMotorVars.Vd) + _IQmpy(gMotorVars.Vq, gMotorVars.Vq));

    float_t Vd_out_pu = _IQtoF(CTRL_getVd_out_pu(ctrlHandle));
    float_t Vq_out_pu = _IQtoF(CTRL_getVq_out_pu(ctrlHandle));
    float_t Vs_V      = sqrt((Vd_out_pu * Vd_out_pu) + (Vq_out_pu * Vq_out_pu));

    gMotorVars.Vd = _IQ(Vd_out_pu);
    gMotorVars.Vq = _IQ(Vq_out_pu);
    gMotorVars.Vs = _IQ(Vs_V);

    // read Id and Iq vectors in amps
    float_t Id_in_pu = _IQtoF(CTRL_getId_in_pu(ctrlHandle));
    float_t Iq_in_pu = _IQtoF(CTRL_getIq_in_pu(ctrlHandle));

    float_t Id_A = Id_in_pu * gUserParams.iqFullScaleCurrent_A;
    float_t Iq_A = Iq_in_pu * gUserParams.iqFullScaleCurrent_A;
    float_t Is_A = sqrt((Id_A * Id_A) + (Iq_A * Iq_A));

    gMotorVars.Id_A = _IQ(Id_A);
    gMotorVars.Iq_A = _IQ(Iq_A);
    gMotorVars.Is_A = _IQ(Is_A);

    // gMotorVars.Id_A = _IQmpy(CTRL_getId_in_pu(ctrlHandle), _IQ(gUserParams.iqFullScaleCurrent_A));
    // gMotorVars.Iq_A = _IQmpy(CTRL_getIq_in_pu(ctrlHandle), _IQ(gUserParams.iqFullScaleCurrent_A));

    // calculate vector Is in amps
    // NOTE TI's original code contains a BUG. Default I8Q24 is not big enough to keep (I*_A ^ 2) value!
    // gMotorVars.Is_A = _IQsqrt(_IQmpy(gMotorVars.Id_A, gMotorVars.Id_A) + _IQmpy(gMotorVars.Iq_A, gMotorVars.Iq_A));
    // float Id_A      = _IQtoF(gMotorVars.Id_A);
    // float Iq_A      = _IQtoF(gMotorVars.Iq_A);

    // Get the DC buss voltage
    gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus, _IQ(gUserParams.iqFullScaleVoltage_V / 1000.0));

    // #ifdef ADC8_SUPPORT
    //     ADC8_FUNCTIONALITY;
    // #endif

    // #ifdef ADC9_SUPPORT
    //     ADC9_FUNCTIONALITY;
    // #endif

    return;
}    // end of updateGlobalVariables_motor() function

void updateKpKiGains(CTRL_Handle handle) {
    if ((gMotorVars.CtrlState == CTRL_State_OnLine) &&
        (gMotorVars.Flag_MotorIdentified == true) &&
        (Flag_Latch_softwareUpdate == false)) {
        // set the kp and ki speed values from the watch window
        CTRL_setKp(handle, CTRL_Type_PID_spd, gMotorVars.Kp_spd);
        CTRL_setKi(handle, CTRL_Type_PID_spd, gMotorVars.Ki_spd);

        // set the kp and ki current values for Id and Iq from the watch window
        CTRL_setKp(handle, CTRL_Type_PID_Id, gMotorVars.Kp_Idq);
        CTRL_setKi(handle, CTRL_Type_PID_Id, gMotorVars.Ki_Idq);
        CTRL_setKp(handle, CTRL_Type_PID_Iq, gMotorVars.Kp_Idq);
        CTRL_setKi(handle, CTRL_Type_PID_Iq, gMotorVars.Ki_Idq);
    }

    return;
}    // end of updateKpKiGains() function

#ifdef ECAN
interrupt void ecan0ISR(void) {
    ecan_int0count++;
    HAL_pieAckInt(halHandle, PIE_GroupNumber_9);
    return;
}

interrupt void ecan1ISR(void) {
    ecan_int1count++;
    ECAN_getMsgFIFO_ID_N(halHandle->ecanaHandle, &gECAN_Mailbox, &gECAN_rxFIFO_ID);
    HAL_pieAckInt(halHandle, PIE_GroupNumber_9);
    return;
}
#endif

interrupt void timer0ISR(void) {
    uptime_s++;
    HAL_pieAckInt(halHandle, PIE_GroupNumber_1);
    return;
}

uint32_t last_timer_count = 0;
uint64_t overflow_count   = 0;

uint64_t micros64(void) {
    uint32_t current_timer_count = TIMER_getCount(halHandle->timerHandle[0]);
    if (current_timer_count > last_timer_count) {
        overflow_count++;
    }
    last_timer_count      = current_timer_count;
    uint64_t timer_period = (uint64_t)(gUserParams.systemFreq_MHz * 1000000);
    uint64_t micros       = (overflow_count * timer_period) + (timer_period - current_timer_count);

    return micros / gUserParams.systemFreq_MHz;
}

#ifdef nFAULT_INT
__interrupt void nFAULT_ISR(void) {
    HAL_Obj *obj           = (HAL_Obj *)halHandle;
    gMotorVars.Flag_nFault = true;
    // HAL_disablePwm(halHandle);
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_1);
}
#endif

#ifndef TZ_DISABLE
__interrupt void TripZoneISR(void) {
    HAL_Obj *obj = (HAL_Obj *)halHandle;

    for (int cnt = 0; cnt < 3; cnt++) {
        PWM_Obj *pwm = (PWM_Obj *)obj->pwmHandle[cnt];
        if (pwm->TZFLG & PWM_TripZoneFlag_Global) {

            if (pwm->TZFLG & PWM_TripZoneFlag_CBC) {
                PWM_clearTripZone(pwm, PWM_TripZoneFlag_CBC);
                gMotorVars.Flag_nFault = true;
            }
            if (pwm->TZFLG & PWM_TripZoneFlag_OST) {
                PWM_clearTripZone(pwm, PWM_TripZoneFlag_OST);
                pwm->TZCLR |= PWM_TripZoneFlag_OST;
#ifdef Fault_OST
                gMotorVars.Flag_nFault_OST = true;
#endif
            }
            PWM_clearTripZone(pwm, PWM_TripZoneFlag_Global);
        }
    }
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_2);
    return;
}
#endif

#ifdef DRV83XX
void FaultHandler(void) {
    if (gMotorVars.Flag_nFault) {
        switch (FaultState) {
            case PHASE_READING:
                gDrvSpi83xxVars.ReadCmd = 1;

                FaultState = PHASE_RESETTING;
                break;

            case PHASE_RESETTING:
                FAULT_RESET;
                gDrvSpi83xxVars.WriteCmd = 1;

                FaultState = PHASE_GO;
                break;
            case PHASE_GO:
                gMotorVars.Flag_nFault         = 0;
                gMotorVars.Flag_nFaultDroneCan = 1;

                FaultState = PHASE_READING;
                break;
            default:
                FaultState = PHASE_READING;
                break;
        }
    }
}
#else
void FaultHandler(void) {
    if (gMotorVars.Flag_nFault) {
        gMotorVars.Flag_nFault         = 0;
        gMotorVars.Flag_nFaultDroneCan = 1;
    }
}
#endif

#ifdef CPU_USAGE
void updateCPUusage(void) {
    // calculate the minimum cpu usage percentage
    gMotorVars.CpuUsagePercentageMin = (float_t)cpu_timeHandle->timer_delta_min / gCpu_usage_den * 100.0;
    // calculate the average cpu usage percentage
    gMotorVars.CpuUsagePercentageAvg = (float_t)cpu_timeHandle->timer_delta_avg / gCpu_usage_den * 100.0;
    // calculate the maximum cpu usage percentage
    gMotorVars.CpuUsagePercentageMax = (float_t)cpu_timeHandle->timer_delta_max / gCpu_usage_den * 100.0;
    return;
}    // end of updateCPUusage() function
#endif
//@} //defgroup
// end of file
