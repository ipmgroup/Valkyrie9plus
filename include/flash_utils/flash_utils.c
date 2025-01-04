#include "flash_utils.h"
#include "Flash2806x_API_Library.h"

void Flash_Init() {
    Example_MemCopy(&Flash28_API_LoadStart, &Flash28_API_LoadEnd, &Flash28_API_RunStart);
    EALLOW;
    Flash_CPUScaleFactor = SCALE_FACTOR;
    Flash_CallbackPtr    = NULL;
    EDIS;
}

#pragma CODE_SECTION(Flash_SaveUserBaseParams, "ramfuncs");

bool Flash_SaveUserBaseParams(USER_Params_FLASH *pUserBaseParams) {
    FLASH_ST FlashStatus;

    if (Flash_APIVersionHex() != FLASH_API_VERSION) {
        return false;
    }

    if (Flash_Erase(SECTORH, &FlashStatus)) {
        return false;
    }

    if (Flash_Program((Uint16 *)&gUserBaseParams_flash, (Uint16 *)pUserBaseParams, sizeof(USER_Params_FLASH), &FlashStatus)) {
        return false;
    }

    if (Flash_Verify((Uint16 *)&gUserBaseParams_flash, (Uint16 *)pUserBaseParams, sizeof(USER_Params_FLASH), &FlashStatus)) {
        return false;
    }

    return true;
}

void Example_MemCopy(Uint16 *SourceAddr, Uint16 *SourceEndAddr, Uint16 *DestAddr) {
    while (SourceAddr < SourceEndAddr) {
        *DestAddr++ = *SourceAddr++;
    }
    return;
}
