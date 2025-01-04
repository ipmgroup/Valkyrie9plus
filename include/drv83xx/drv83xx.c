#include "user_hw_select.h"
#ifdef DRV83XX

#include "drv83xx.h"

// Substitute function names to match the selected driver.
#define INIT(drv_type) INIT_(drv_type)
#define INIT_(drv_type) DRV##drv_type##_init

#define ENABLE(drv_type) ENABLE_(drv_type)
#define ENABLE_(drv_type) DRV##drv_type##_enable

#define SET_GPIO_HANDLE(drv_type) SET_GPIO_HANDLE_(drv_type)
#define SET_GPIO_HANDLE_(drv_type) DRV##drv_type##_setGpioHandle

#define SET_SPI_HANDLE(drv_type) SET_SPI_HANDLE_(drv_type)
#define SET_SPI_HANDLE_(drv_type) DRV##drv_type##_setSpiHandle

#define SET_EN_GATE_GPIO_NUMBER(drv_type) SET_EN_GATE_GPIO_NUMBER_(drv_type)
#define SET_EN_GATE_GPIO_NUMBER_(drv_type) DRV##drv_type##_setEnGateGpioNumber

#define SETUP_SPI(drv_type) SETUP_SPI_(drv_type)
#define SETUP_SPI_(drv_type) DRV##drv_type##_setupSpi

#define WRITE_DATA(drv_type) WRITE_DATA_(drv_type)
#define WRITE_DATA_(drv_type) DRV##drv_type##_writeData

#define READ_DATA(drv_type) READ_DATA_(drv_type)
#define READ_DATA_(drv_type) DRV##drv_type##_readData

DRV83XX_Handle DRV83XX_init(void *pMemory, const size_t numBytes) {
    DRV83XX_Handle handle;

    if (numBytes < sizeof(DRV83XX_Obj))
        return ((DRV83XX_Handle)NULL);

    handle = (DRV83XX_Handle)pMemory;

    // Assign actual functions to the pointers.
    handle->init                = INIT(DRV_TYPE);
    handle->enable              = ENABLE(DRV_TYPE);
    handle->setGpioHandle       = SET_GPIO_HANDLE(DRV_TYPE);
    handle->setSpiHandle        = SET_SPI_HANDLE(DRV_TYPE);
    handle->setEnGateGpioNumber = SET_EN_GATE_GPIO_NUMBER(DRV_TYPE);
    handle->setupSpi            = SETUP_SPI(DRV_TYPE);
    handle->writeData           = WRITE_DATA(DRV_TYPE);
    handle->readData            = READ_DATA(DRV_TYPE);

    handle->drv83xx_sub_handle = handle->init(&(handle->drv83xx_sub), sizeof(handle->drv83xx_sub));

    return handle;
}

void DRV83XX_enable(DRV83XX_Handle handle) {
    return handle->enable(handle->drv83xx_sub_handle);
}

void DRV83XX_setGpioHandle(DRV83XX_Handle handle, GPIO_Handle gpioHandle) {
    return handle->setGpioHandle(handle->drv83xx_sub_handle, gpioHandle);
}

void DRV83XX_setSpiHandle(DRV83XX_Handle handle, SPI_Handle spiHandle) {
    return handle->setSpiHandle(handle->drv83xx_sub_handle, spiHandle);
}

void DRV83XX_setEnGateGpioNumber(DRV83XX_Handle handle, GPIO_Number_e enGateGpioNumber) {
    return handle->setEnGateGpioNumber(handle->drv83xx_sub_handle, enGateGpioNumber);
}

void DRV83XX_setupSpi(DRV83XX_Handle handle, DRV_SPI_83XX_Vars_t *Spi_83XX_Vars) {
    return handle->setupSpi(handle->drv83xx_sub_handle, Spi_83XX_Vars);
}

void DRV83XX_writeData(DRV83XX_Handle handle, DRV_SPI_83XX_Vars_t *Spi_83XX_Vars) {
    return handle->writeData(handle->drv83xx_sub_handle, Spi_83XX_Vars);
}

void DRV83XX_readData(DRV83XX_Handle handle, DRV_SPI_83XX_Vars_t *Spi_83XX_Vars) {
    return handle->readData(handle->drv83xx_sub_handle, Spi_83XX_Vars);
}

#endif  // DRV83XX
