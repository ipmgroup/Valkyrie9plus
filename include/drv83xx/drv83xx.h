#ifndef INCLUDE_DRV83XX_DRV83XX_H_
#define INCLUDE_DRV83XX_DRV83XX_H_

#include "user_hw_select.h"

#ifdef DRV8301_SPI
#include "drv8301.h"
#define DRV_TYPE 8301
typedef DRV8301_Obj         DRV83XX_Sub_Obj;
typedef DRV8301_Handle      DRV83XX_Sub_Handle;
typedef DRV_SPI_8301_Vars_t DRV_SPI_83XX_Vars_t;
#elif defined DRV8305_SPI
#include "drv8305.h"
#define DRV_TYPE 8305
typedef DRV8305_Obj         DRV83XX_Sub_Obj;
typedef DRV8305_Handle      DRV83XX_Sub_Handle;
typedef DRV_SPI_8305_Vars_t DRV_SPI_83XX_Vars_t;
#elif defined DRV8323_SPI
#include "drv8323.h"
#define DRV_TYPE 8323
typedef DRV8323_Obj         DRV83XX_Sub_Obj;
typedef DRV8323_Handle      DRV83XX_Sub_Handle;
typedef DRV_SPI_8323_Vars_t DRV_SPI_83XX_Vars_t;
#endif

typedef struct _DRV83XX_Obj_ {
    DRV83XX_Sub_Handle (*init)(void *pMemory, const size_t numBytes);
    void (*enable)(DRV83XX_Sub_Handle handle);
    void (*setGpioHandle)(DRV83XX_Sub_Handle handle, GPIO_Handle gpioHandle);
    void (*setSpiHandle)(DRV83XX_Sub_Handle handle, SPI_Handle spiHandle);
    void (*setEnGateGpioNumber)(DRV83XX_Sub_Handle handle, GPIO_Number_e enGateGpioNumber);
    void (*setupSpi)(DRV83XX_Sub_Handle handle, DRV_SPI_83XX_Vars_t *Spi_83XX_Vars);
    void (*writeData)(DRV83XX_Sub_Handle handle, DRV_SPI_83XX_Vars_t *Spi_83XX_Vars);
    void (*readData)(DRV83XX_Sub_Handle handle, DRV_SPI_83XX_Vars_t *Spi_83XX_Vars);

    DRV83XX_Sub_Obj    drv83xx_sub;
    DRV83XX_Sub_Handle drv83xx_sub_handle;

} DRV83XX_Obj;

typedef struct _DRV83XX_Obj_ *DRV83XX_Handle;

extern DRV83XX_Handle DRV83XX_init(void *pMemory, const size_t numBytes);

extern void DRV83XX_enable(DRV83XX_Handle handle);

extern void DRV83XX_setGpioHandle(DRV83XX_Handle handle, GPIO_Handle gpioHandle);

extern void DRV83XX_setSpiHandle(DRV83XX_Handle handle, SPI_Handle spiHandle);

extern void DRV83XX_setEnGateGpioNumber(DRV83XX_Handle handle, GPIO_Number_e gpioNumber);

extern void DRV83XX_setupSpi(DRV83XX_Handle handle, DRV_SPI_83XX_Vars_t *Spi_83XX_Vars);

extern void DRV83XX_writeData(DRV83XX_Handle handle, DRV_SPI_83XX_Vars_t *Spi_83XX_Vars);

extern void DRV83XX_readData(DRV83XX_Handle handle, DRV_SPI_83XX_Vars_t *Spi_83XX_Vars);

#endif /* INCLUDE_DRV83XX_DRV83XX_H_ */
