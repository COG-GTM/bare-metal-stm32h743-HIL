/* Host stand-ins for the memory mapped blocks that src/system_stm32h7xx.c
   writes to, so SystemInit()/SystemCoreClockUpdate() can run under gcc.
   Build: make test */
#ifndef CMSIS_HOST_SHIM_H
#define CMSIS_HOST_SHIM_H

#include "stm32h7xx.h"
#include "system_stm32h7xx.h"

extern RCC_TypeDef       host_rcc;
extern SCB_Type          host_scb;
extern FLASH_TypeDef     host_flash;
extern DBGMCU_TypeDef    host_dbgmcu;
extern FMC_Bank1_TypeDef host_fmc_bank1;

/* Zero every stand-in block and load DBGMCU->IDCODE with a rev V device id. */
void cmsis_host_reset_peripherals(void);

#endif /* CMSIS_HOST_SHIM_H */
