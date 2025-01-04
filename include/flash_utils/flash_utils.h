#ifndef INCLUDE_FLASH_UTILS_FLASH_UTILS_H_
#define INCLUDE_FLASH_UTILS_FLASH_UTILS_H_

#include "Flash2806x_API_Library.h"
#include <user.h>
#include <hal.h>

#define FLASH_API_VERSION 0x100

extern const USER_Params_FLASH gUserBaseParams_flash;

void Flash_Init();

bool Flash_SaveUserBaseParams(USER_Params_FLASH *pUserBaseParams);
void Example_MemCopy(Uint16 *SourceAddr, Uint16 *SourceEndAddr, Uint16 *DestAddr);

#endif  // INCLUDE_FLASH_UTILS_FLASH_UTILS_H_
