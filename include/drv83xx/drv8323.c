#include "user_hw_select.h"
#ifdef DRV8323_SPI
#include <math.h>
#include "drv8323.h"

DRV8323_Handle DRV8323_init(void *pMemory, const size_t numBytes) {
    DRV8323_Handle handle;

    if (numBytes < sizeof(DRV8323_Obj))
        return ((DRV8323_Handle)NULL);

    handle = (DRV8323_Handle)pMemory;

    DRV8323_resetRxTimeout(handle);
    DRV8323_resetEnableTimeout(handle);

    return (handle);
}

void DRV8323_enable(DRV8323_Handle handle) {
    DRV8323_Obj             *obj = (DRV8323_Obj *)handle;
    static volatile uint16_t enableWaitTimeOut;
    uint16_t                 n = 0;

    GPIO_setHigh(obj->gpioHandle, obj->enGateGpioNumber);

    enableWaitTimeOut = 0;

    while (((DRV8323_readSpi(handle, Address_Status_0) & DRV8323_STATUS00_FAULT_BITS) != 0) && (enableWaitTimeOut < 1000)) {
        if (++enableWaitTimeOut > 999) {
            obj->enableTimeOut = true;
        }
    }

    for (n = 0; n < 0xffff; n++)
        asm(" NOP");

    return;
}

DRV8323_CTRL03_PeakSourCurHS_e DRV8323_getPeakSourCurHS(DRV8323_Handle handle) {
    uint16_t data;

    data = DRV8323_readSpi(handle, Address_Control_3);

    data &= DRV8323_CTRL03_IDRIVEP_HS_BITS;

    return ((DRV8323_CTRL03_PeakSourCurHS_e)data);
}

DRV8323_CTRL03_PeakSinkCurHS_e DRV8323_getPeakSinkCurHS(DRV8323_Handle handle) {
    uint16_t data;

    data = DRV8323_readSpi(handle, Address_Control_3);

    data &= DRV8323_CTRL03_IDRIVEN_HS_BITS;

    return ((DRV8323_CTRL03_PeakSinkCurHS_e)data);
}

DRV8323_CTRL04_PeakTime_e DRV8323_getPeakSourTime(DRV8323_Handle handle) {
    uint16_t data;

    data = DRV8323_readSpi(handle, Address_Control_4);

    data &= DRV8323_CTRL04_TDRIVE_BITS;

    return ((DRV8323_CTRL04_PeakTime_e)data);
}

DRV8323_CTRL04_PeakSourCurLS_e DRV8323_getPeakSourCurLS(DRV8323_Handle handle) {
    uint16_t data;

    data = DRV8323_readSpi(handle, Address_Control_4);

    data &= DRV8323_CTRL04_IDRIVEP_LS_BITS;

    return ((DRV8323_CTRL04_PeakSourCurLS_e)data);
}

DRV8323_CTRL04_PeakSinkCurLS_e DRV8323_getPeakSinkCurLS(DRV8323_Handle handle) {
    uint16_t data;

    data = DRV8323_readSpi(handle, Address_Control_4);

    data &= DRV8323_CTRL04_IDRIVEN_LS_BITS;

    return ((DRV8323_CTRL04_PeakSinkCurLS_e)data);
}

DRV8323_CTRL05_OcpDeg_e DRV8323_getVDSDeglitch(DRV8323_Handle handle) {
    uint16_t data;

    data = DRV8323_readSpi(handle, Address_Control_5);

    data &= DRV8323_CTRL05_OCP_DEG_BITS;

    return ((DRV8323_CTRL05_OcpDeg_e)data);
}

DRV8323_CTRL05_DeadTime_e DRV8323_getDeadTime(DRV8323_Handle handle) {
    uint16_t data;

    data = DRV8323_readSpi(handle, Address_Control_5);

    data &= DRV8323_CTRL05_DEAD_TIME_BITS;

    return ((DRV8323_CTRL05_DeadTime_e)data);
}

DRV8323_CTRL02_PwmMode_e DRV8323_getPwmMode(DRV8323_Handle handle) {
    uint16_t data;

    data = DRV8323_readSpi(handle, Address_Control_2);

    data &= DRV8323_CTRL02_PWM_MODE_BITS;

    return ((DRV8323_CTRL02_PwmMode_e)data);
}

void DRV8323_setSpiHandle(DRV8323_Handle handle, SPI_Handle spiHandle) {
    DRV8323_Obj *obj = (DRV8323_Obj *)handle;

    obj->spiHandle = spiHandle;

    return;
}

void DRV8323_setGpioHandle(DRV8323_Handle handle, GPIO_Handle gpioHandle) {
    DRV8323_Obj *obj = (DRV8323_Obj *)handle;

    obj->gpioHandle = gpioHandle;

    return;
}

void DRV8323_setEnGateGpioNumber(DRV8323_Handle handle, GPIO_Number_e enGateGpioNumber) {
    DRV8323_Obj *obj = (DRV8323_Obj *)handle;

    obj->enGateGpioNumber = enGateGpioNumber;

    return;
}

void DRV8323_setupSpi(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars) {
    DRV8323_Address_e drvRegAddr;
    uint16_t          drvDataNew;

    Spi_8323_Vars->ManReadAddr  = 0;
    Spi_8323_Vars->ManReadData  = 0;
    Spi_8323_Vars->ManReadCmd   = false;
    Spi_8323_Vars->ManWriteAddr = 0;
    Spi_8323_Vars->ManWriteData = 0;
    Spi_8323_Vars->ManWriteCmd  = false;

    Spi_8323_Vars->ReadCmd  = false;
    Spi_8323_Vars->WriteCmd = false;

    drvRegAddr                         = Address_Status_0;
    drvDataNew                         = DRV8323_readSpi(handle, drvRegAddr);
    Spi_8323_Vars->Stat_Reg_00.VDS_LC  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_LC_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_00.VDS_HC  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_HC_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_00.VDS_LB  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_LB_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_00.VDS_HB  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_HB_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_00.VDS_LA  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_LA_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_00.VDS_HA  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_HA_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_00.OTSD    = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_OTSD_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_00.UVLO    = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_UVLO_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_00.GDF     = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_GDF_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_00.VDS_OCP = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_OCP_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_00.FAULT   = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_FAULT_BITS) ? 1 : 0;

    drvRegAddr                        = Address_Status_1;
    drvDataNew                        = DRV8323_readSpi(handle, drvRegAddr);
    Spi_8323_Vars->Stat_Reg_01.VGS_LC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_VGS_LC_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_01.VGS_HC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_VGS_HC_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_01.VGS_LB = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_VGS_LB_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_01.VGS_HB = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_VGS_HB_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_01.VGS_LA = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_VGS_LA_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_01.VGS_HA = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_VGS_HA_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_01.CPUV   = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_CPUV_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_01.OTW    = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_OTW_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_01.SC_OC  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_SC_OC_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_01.SB_OC  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_SB_OC_BITS) ? 1 : 0;
    Spi_8323_Vars->Stat_Reg_01.SA_OC  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_SA_OC_BITS) ? 1 : 0;

    drvRegAddr                             = Address_Control_2;
    drvDataNew                             = DRV8323_readSpi(handle, drvRegAddr);
    Spi_8323_Vars->Ctrl_Reg_02.CLR_FLT     = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_CLR_FLT_BITS);
    Spi_8323_Vars->Ctrl_Reg_02.BRAKE       = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_BRAKE_BITS);
    Spi_8323_Vars->Ctrl_Reg_02.COAST       = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_COAST_BITS);
    Spi_8323_Vars->Ctrl_Reg_02.PWM1_DIR    = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_PWM1_DIR_BITS);
    Spi_8323_Vars->Ctrl_Reg_02.PWM1_COM    = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_PWM1_COM_BITS);
    Spi_8323_Vars->Ctrl_Reg_02.PWM_MODE    = (DRV8323_CTRL02_PwmMode_e)(drvDataNew & (uint16_t)DRV8323_CTRL02_PWM_MODE_BITS);
    Spi_8323_Vars->Ctrl_Reg_02.OTW_REP     = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_OTW_REP_BITS);
    Spi_8323_Vars->Ctrl_Reg_02.DIS_GDF     = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_DIS_GDF_BITS);
    Spi_8323_Vars->Ctrl_Reg_02.DIS_CPUV    = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_DIS_CPUV_BITS);
    Spi_8323_Vars->Ctrl_Reg_02.CTRL02_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_RESERVED1_BITS) ? 1 : 0;

    drvRegAddr                            = Address_Control_3;
    drvDataNew                            = DRV8323_readSpi(handle, drvRegAddr);
    Spi_8323_Vars->Ctrl_Reg_03.IDRIVEN_HS = (DRV8323_CTRL03_PeakSinkCurHS_e)(drvDataNew & (uint16_t)DRV8323_CTRL03_IDRIVEN_HS_BITS);
    Spi_8323_Vars->Ctrl_Reg_03.IDRIVEP_HS = (DRV8323_CTRL03_PeakSourCurHS_e)(drvDataNew & (uint16_t)DRV8323_CTRL03_IDRIVEP_HS_BITS);
    Spi_8323_Vars->Ctrl_Reg_03.LOCK       = (DRV8323_CTRL03_Lock_e)(drvDataNew & (uint16_t)DRV8323_CTRL03_LOCK_BITS);

    drvRegAddr                            = Address_Control_4;
    drvDataNew                            = DRV8323_readSpi(handle, drvRegAddr);
    Spi_8323_Vars->Ctrl_Reg_04.IDRIVEN_LS = (DRV8323_CTRL04_PeakSinkCurLS_e)(drvDataNew & (uint16_t)DRV8323_CTRL04_IDRIVEN_LS_BITS);
    Spi_8323_Vars->Ctrl_Reg_04.IDRIVEP_LS = (DRV8323_CTRL04_PeakSourCurLS_e)(drvDataNew & (uint16_t)DRV8323_CTRL04_IDRIVEP_LS_BITS);
    Spi_8323_Vars->Ctrl_Reg_04.TDRIVE     = (DRV8323_CTRL04_PeakTime_e)(drvDataNew & (uint16_t)DRV8323_CTRL04_TDRIVE_BITS);
    Spi_8323_Vars->Ctrl_Reg_04.CBC        = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL04_CBC_BITS) ? 1 : 0;

    drvRegAddr                           = Address_Control_5;
    drvDataNew                           = DRV8323_readSpi(handle, drvRegAddr);
    Spi_8323_Vars->Ctrl_Reg_05.VDS_LVL   = (DRV8323_CTRL05_VDSLVL_e)(drvDataNew & (uint16_t)DRV8323_CTRL05_VDS_LVL_BITS);
    Spi_8323_Vars->Ctrl_Reg_05.OCP_DEG   = (DRV8323_CTRL05_OcpDeg_e)(drvDataNew & (uint16_t)DRV8323_CTRL05_OCP_DEG_BITS);
    Spi_8323_Vars->Ctrl_Reg_05.OCP_MODE  = (DRV8323_CTRL05_OcpMode_e)(drvDataNew & (uint16_t)DRV8323_CTRL05_OCP_MODE_BITS);
    Spi_8323_Vars->Ctrl_Reg_05.DEAD_TIME = (DRV8323_CTRL05_DeadTime_e)(drvDataNew & (uint16_t)DRV8323_CTRL05_DEAD_TIME_BITS);
    Spi_8323_Vars->Ctrl_Reg_05.TRETRY    = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL05_TRETRY_BITS);

    drvRegAddr                           = Address_Control_6;
    drvDataNew                           = DRV8323_readSpi(handle, drvRegAddr);
    Spi_8323_Vars->Ctrl_Reg_06.SEN_LVL   = (DRV8323_CTRL06_SENLevel_e)(drvDataNew & (uint16_t)DRV8323_CTRL06_SEN_LVL_BITS);
    Spi_8323_Vars->Ctrl_Reg_06.CSA_CAL_C = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_CSA_CAL_C_BITS);
    Spi_8323_Vars->Ctrl_Reg_06.CSA_CAL_B = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_CSA_CAL_B_BITS);
    Spi_8323_Vars->Ctrl_Reg_06.CSA_CAL_A = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_CSA_CAL_A_BITS);
    Spi_8323_Vars->Ctrl_Reg_06.DIS_SEN   = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_DIS_SEN_BITS);
    Spi_8323_Vars->Ctrl_Reg_06.CSA_GAIN  = (DRV8323_CTRL06_CSAGain_e)(drvDataNew & (uint16_t)DRV8323_CTRL06_CSA_GAIN_BITS);
    Spi_8323_Vars->Ctrl_Reg_06.LS_REF    = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_LS_REF_BITS);
    Spi_8323_Vars->Ctrl_Reg_06.VREF_DIV  = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_VREF_DIV_BITS);
    Spi_8323_Vars->Ctrl_Reg_06.CSA_FET   = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_CSA_FET_BITS);

    return;
}

uint16_t DRV8323_readSpi(DRV8323_Handle handle, const DRV8323_Address_e regAddr) {
    DRV8323_Obj              *obj = (DRV8323_Obj *)handle;
    uint16_t                  ctrlWord;
    uint16_t                  n;
    const uint16_t            data = 0;
    volatile uint16_t         readWord;
    static volatile uint16_t  WaitTimeOut = 0;
    volatile SPI_FifoStatus_e RxFifoCnt   = SPI_FifoStatus_Empty;

    ctrlWord = (uint16_t)DRV8323_buildCtrlWord(CtrlMode_Read, regAddr, data);

    SPI_resetRxFifo(obj->spiHandle);
    SPI_enableRxFifo(obj->spiHandle);

    GPIO_setLow(obj->gpioHandle, SPI_CS);


    for (n = 0; n < 0x03; n++)
        asm(" NOP");

    SPI_write(obj->spiHandle, ctrlWord);

    while ((RxFifoCnt < SPI_FifoStatus_1_Word) && (WaitTimeOut < 0xffff)) {
        RxFifoCnt = SPI_getRxFifoStatus(obj->spiHandle);
        if (++WaitTimeOut > 0xfffe) {
            obj->RxTimeOut = true;
        }
    }

    GPIO_setHigh(obj->gpioHandle, SPI_CS);

    readWord = SPI_readEmu(obj->spiHandle);

    return (readWord & DRV8323_DATA_MASK);
}

void DRV8323_writeSpi(DRV8323_Handle handle, const DRV8323_Address_e regAddr, const uint16_t data) {
    DRV8323_Obj *obj = (DRV8323_Obj *)handle;
    uint16_t     ctrlWord;
    uint16_t     n;

    ctrlWord = (uint16_t)DRV8323_buildCtrlWord(CtrlMode_Write, regAddr, data);

    SPI_resetRxFifo(obj->spiHandle);
    SPI_enableRxFifo(obj->spiHandle);

    GPIO_setLow(obj->gpioHandle, SPI_CS);

    for (n = 0; n < 0x03; n++)
        asm(" NOP");

    SPI_write(obj->spiHandle, ctrlWord);

    for (n = 0; n < 0x0CF; n++)
        asm(" NOP");

    GPIO_setHigh(obj->gpioHandle, SPI_CS);

    return;
}

void DRV8323_writeData(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars) {
    DRV8323_Address_e drvRegAddr;
    uint16_t          drvDataNew;

    if (Spi_8323_Vars->WriteCmd) {
        drvRegAddr = Address_Control_2;
        drvDataNew = (Spi_8323_Vars->Ctrl_Reg_02.CLR_FLT << 0) | (Spi_8323_Vars->Ctrl_Reg_02.BRAKE << 1) | (Spi_8323_Vars->Ctrl_Reg_02.COAST << 2) | (Spi_8323_Vars->Ctrl_Reg_02.PWM1_DIR << 3) | (Spi_8323_Vars->Ctrl_Reg_02.PWM1_COM << 4) | (Spi_8323_Vars->Ctrl_Reg_02.PWM_MODE) | (Spi_8323_Vars->Ctrl_Reg_02.OTW_REP << 7) | (Spi_8323_Vars->Ctrl_Reg_02.DIS_GDF << 8) | (Spi_8323_Vars->Ctrl_Reg_02.DIS_CPUV << 9) | (Spi_8323_Vars->Ctrl_Reg_02.CTRL02_RSV1 << 10);
        DRV8323_writeSpi(handle, drvRegAddr, drvDataNew);

        drvRegAddr = Address_Control_3;
        drvDataNew = (Spi_8323_Vars->Ctrl_Reg_03.IDRIVEN_HS) | (Spi_8323_Vars->Ctrl_Reg_03.IDRIVEP_HS) | (Spi_8323_Vars->Ctrl_Reg_03.LOCK);
        DRV8323_writeSpi(handle, drvRegAddr, drvDataNew);

        drvRegAddr = Address_Control_4;
        drvDataNew = (Spi_8323_Vars->Ctrl_Reg_04.IDRIVEN_LS) | (Spi_8323_Vars->Ctrl_Reg_04.IDRIVEP_LS) | (Spi_8323_Vars->Ctrl_Reg_04.TDRIVE) | (Spi_8323_Vars->Ctrl_Reg_04.CBC << 10);
        DRV8323_writeSpi(handle, drvRegAddr, drvDataNew);

        drvRegAddr = Address_Control_5;
        drvDataNew = (Spi_8323_Vars->Ctrl_Reg_05.VDS_LVL) | (Spi_8323_Vars->Ctrl_Reg_05.OCP_DEG) | (Spi_8323_Vars->Ctrl_Reg_05.OCP_MODE) | (Spi_8323_Vars->Ctrl_Reg_05.DEAD_TIME) | (Spi_8323_Vars->Ctrl_Reg_05.TRETRY << 10);
        DRV8323_writeSpi(handle, drvRegAddr, drvDataNew);

        drvRegAddr = Address_Control_6;
        drvDataNew = (Spi_8323_Vars->Ctrl_Reg_06.SEN_LVL) | (Spi_8323_Vars->Ctrl_Reg_06.CSA_CAL_C << 2) | (Spi_8323_Vars->Ctrl_Reg_06.CSA_CAL_B << 3) | (Spi_8323_Vars->Ctrl_Reg_06.CSA_CAL_A << 4) | (Spi_8323_Vars->Ctrl_Reg_06.DIS_SEN << 5) | (Spi_8323_Vars->Ctrl_Reg_06.CSA_GAIN) | (Spi_8323_Vars->Ctrl_Reg_06.LS_REF << 8) | (Spi_8323_Vars->Ctrl_Reg_06.VREF_DIV << 9) | (Spi_8323_Vars->Ctrl_Reg_06.CSA_FET << 10);
        DRV8323_writeSpi(handle, drvRegAddr, drvDataNew);

        Spi_8323_Vars->WriteCmd = false;
    }

    if (Spi_8323_Vars->ManWriteCmd) {
        drvRegAddr = (DRV8323_Address_e)(Spi_8323_Vars->ManWriteAddr << 11);
        drvDataNew = Spi_8323_Vars->ManWriteData;
        DRV8323_writeSpi(handle, drvRegAddr, drvDataNew);

        Spi_8323_Vars->ManWriteCmd = false;
    }

    return;
}

void DRV8323_readData(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars) {
    DRV8323_Address_e drvRegAddr;
    uint16_t          drvDataNew;

    if (Spi_8323_Vars->ReadCmd) {
        drvRegAddr                         = Address_Status_0;
        drvDataNew                         = DRV8323_readSpi(handle, drvRegAddr);
        Spi_8323_Vars->Stat_Reg_00.VDS_LC  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_LC_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_00.VDS_HC  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_HC_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_00.VDS_LB  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_LB_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_00.VDS_HB  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_HB_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_00.VDS_LA  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_LA_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_00.VDS_HA  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_HA_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_00.OTSD    = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_OTSD_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_00.UVLO    = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_UVLO_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_00.GDF     = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_GDF_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_00.VDS_OCP = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_VDS_OCP_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_00.FAULT   = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS00_FAULT_BITS) ? 1 : 0;

        drvRegAddr                        = Address_Status_1;
        drvDataNew                        = DRV8323_readSpi(handle, drvRegAddr);
        Spi_8323_Vars->Stat_Reg_01.VGS_LC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_VGS_LC_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_01.VGS_HC = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_VGS_HC_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_01.VGS_LB = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_VGS_LB_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_01.VGS_HB = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_VGS_HB_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_01.VGS_LA = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_VGS_LA_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_01.VGS_HA = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_VGS_HA_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_01.CPUV   = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_CPUV_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_01.OTW    = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_OTW_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_01.SC_OC  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_SC_OC_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_01.SB_OC  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_SB_OC_BITS) ? 1 : 0;
        Spi_8323_Vars->Stat_Reg_01.SA_OC  = (bool)(drvDataNew & (uint16_t)DRV8323_STATUS01_SA_OC_BITS) ? 1 : 0;

        drvRegAddr                             = Address_Control_2;
        drvDataNew                             = DRV8323_readSpi(handle, drvRegAddr);
        Spi_8323_Vars->Ctrl_Reg_02.CLR_FLT     = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_CLR_FLT_BITS);
        Spi_8323_Vars->Ctrl_Reg_02.BRAKE       = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_BRAKE_BITS);
        Spi_8323_Vars->Ctrl_Reg_02.COAST       = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_COAST_BITS);
        Spi_8323_Vars->Ctrl_Reg_02.PWM1_DIR    = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_PWM1_DIR_BITS);
        Spi_8323_Vars->Ctrl_Reg_02.PWM1_COM    = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_PWM1_COM_BITS);
        Spi_8323_Vars->Ctrl_Reg_02.PWM_MODE    = (DRV8323_CTRL02_PwmMode_e)(drvDataNew & (uint16_t)DRV8323_CTRL02_PWM_MODE_BITS);
        Spi_8323_Vars->Ctrl_Reg_02.OTW_REP     = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_OTW_REP_BITS);
        Spi_8323_Vars->Ctrl_Reg_02.DIS_GDF     = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_DIS_GDF_BITS);
        Spi_8323_Vars->Ctrl_Reg_02.DIS_CPUV    = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_DIS_CPUV_BITS);
        Spi_8323_Vars->Ctrl_Reg_02.CTRL02_RSV1 = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL02_RESERVED1_BITS) ? 1 : 0;

        drvRegAddr                            = Address_Control_3;
        drvDataNew                            = DRV8323_readSpi(handle, drvRegAddr);
        Spi_8323_Vars->Ctrl_Reg_03.IDRIVEN_HS = (DRV8323_CTRL03_PeakSinkCurHS_e)(drvDataNew & (uint16_t)DRV8323_CTRL03_IDRIVEN_HS_BITS);
        Spi_8323_Vars->Ctrl_Reg_03.IDRIVEP_HS = (DRV8323_CTRL03_PeakSourCurHS_e)(drvDataNew & (uint16_t)DRV8323_CTRL03_IDRIVEP_HS_BITS);
        Spi_8323_Vars->Ctrl_Reg_03.LOCK       = (DRV8323_CTRL03_Lock_e)(drvDataNew & (uint16_t)DRV8323_CTRL03_LOCK_BITS);

        drvRegAddr                            = Address_Control_4;
        drvDataNew                            = DRV8323_readSpi(handle, drvRegAddr);
        Spi_8323_Vars->Ctrl_Reg_04.IDRIVEN_LS = (DRV8323_CTRL04_PeakSinkCurLS_e)(drvDataNew & (uint16_t)DRV8323_CTRL04_IDRIVEN_LS_BITS);
        Spi_8323_Vars->Ctrl_Reg_04.IDRIVEP_LS = (DRV8323_CTRL04_PeakSourCurLS_e)(drvDataNew & (uint16_t)DRV8323_CTRL04_IDRIVEP_LS_BITS);
        Spi_8323_Vars->Ctrl_Reg_04.TDRIVE     = (DRV8323_CTRL04_PeakTime_e)(drvDataNew & (uint16_t)DRV8323_CTRL04_TDRIVE_BITS);
        Spi_8323_Vars->Ctrl_Reg_04.CBC        = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL04_CBC_BITS) ? 1 : 0;

        drvRegAddr                           = Address_Control_5;
        drvDataNew                           = DRV8323_readSpi(handle, drvRegAddr);
        Spi_8323_Vars->Ctrl_Reg_05.VDS_LVL   = (DRV8323_CTRL05_VDSLVL_e)(drvDataNew & (uint16_t)DRV8323_CTRL05_VDS_LVL_BITS);
        Spi_8323_Vars->Ctrl_Reg_05.OCP_DEG   = (DRV8323_CTRL05_OcpDeg_e)(drvDataNew & (uint16_t)DRV8323_CTRL05_OCP_DEG_BITS);
        Spi_8323_Vars->Ctrl_Reg_05.OCP_MODE  = (DRV8323_CTRL05_OcpMode_e)(drvDataNew & (uint16_t)DRV8323_CTRL05_OCP_MODE_BITS);
        Spi_8323_Vars->Ctrl_Reg_05.DEAD_TIME = (DRV8323_CTRL05_DeadTime_e)(drvDataNew & (uint16_t)DRV8323_CTRL05_DEAD_TIME_BITS);
        Spi_8323_Vars->Ctrl_Reg_05.TRETRY    = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL05_TRETRY_BITS);

        drvRegAddr                           = Address_Control_6;
        drvDataNew                           = DRV8323_readSpi(handle, drvRegAddr);
        Spi_8323_Vars->Ctrl_Reg_06.SEN_LVL   = (DRV8323_CTRL06_SENLevel_e)(drvDataNew & (uint16_t)DRV8323_CTRL06_SEN_LVL_BITS);
        Spi_8323_Vars->Ctrl_Reg_06.CSA_CAL_C = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_CSA_CAL_C_BITS);
        Spi_8323_Vars->Ctrl_Reg_06.CSA_CAL_B = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_CSA_CAL_B_BITS);
        Spi_8323_Vars->Ctrl_Reg_06.CSA_CAL_A = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_CSA_CAL_A_BITS);
        Spi_8323_Vars->Ctrl_Reg_06.DIS_SEN   = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_DIS_SEN_BITS);
        Spi_8323_Vars->Ctrl_Reg_06.CSA_GAIN  = (DRV8323_CTRL06_CSAGain_e)(drvDataNew & (uint16_t)DRV8323_CTRL06_CSA_GAIN_BITS);
        Spi_8323_Vars->Ctrl_Reg_06.LS_REF    = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_LS_REF_BITS);
        Spi_8323_Vars->Ctrl_Reg_06.VREF_DIV  = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_VREF_DIV_BITS);
        Spi_8323_Vars->Ctrl_Reg_06.CSA_FET   = (bool)(drvDataNew & (uint16_t)DRV8323_CTRL06_CSA_FET_BITS);

        Spi_8323_Vars->ReadCmd = false;
    }

    if (Spi_8323_Vars->ManReadCmd) {
        drvRegAddr                 = (DRV8323_Address_e)(Spi_8323_Vars->ManReadAddr << 11);
        drvDataNew                 = DRV8323_readSpi(handle, drvRegAddr);
        Spi_8323_Vars->ManReadData = drvDataNew;

        Spi_8323_Vars->ManReadCmd = false;
    }

    return;
}

#endif
