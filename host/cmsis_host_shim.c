/* Compiles src/system_stm32h7xx.c for the host with the peripheral pointers
   redirected at plain memory, so the register writes of SystemInit() can be
   observed instead of faulting on 0x5xxxxxxx addresses.
   Build: make test */
#include <string.h>

#include "cmsis_host_shim.h"

RCC_TypeDef       host_rcc;
SCB_Type          host_scb;
FLASH_TypeDef     host_flash;
DBGMCU_TypeDef    host_dbgmcu;
FMC_Bank1_TypeDef host_fmc_bank1;

void cmsis_host_reset_peripherals(void)
{
  memset(&host_rcc, 0, sizeof(host_rcc));
  memset(&host_scb, 0, sizeof(host_scb));
  memset(&host_flash, 0, sizeof(host_flash));
  memset(&host_dbgmcu, 0, sizeof(host_dbgmcu));
  memset(&host_fmc_bank1, 0, sizeof(host_fmc_bank1));

  /* Rev V silicon: keeps SystemInit() away from the raw 0x51008108 write,
     which has no stand-in on the host. */
  host_dbgmcu.IDCODE = 0x20036450UL;
}

#undef RCC
#undef SCB
#undef FLASH
#undef DBGMCU
#undef FMC_Bank1_R

#define RCC          (&host_rcc)
#define SCB          (&host_scb)
#define FLASH        (&host_flash)
#define DBGMCU       (&host_dbgmcu)
#define FMC_Bank1_R  (&host_fmc_bank1)

/* core_cm7.h resolves __FPU_USED to 0 on a non-ARM compiler, which would
   compile out the CPACR write; keep that path in the host build. */
#undef __FPU_USED
#define __FPU_USED 1U

#include "../src/system_stm32h7xx.c"
