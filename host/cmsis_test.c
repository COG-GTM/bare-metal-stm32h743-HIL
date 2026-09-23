/* Host unit tests for the CMSIS support layer (src/system_stm32h7xx.c),
   driven through the register stand-ins of host/cmsis_host_shim.c.
   Build: make test */
#include <stdio.h>

#include "cmsis_host_shim.h"

static int fails = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); fails++; } else printf("ok:   %s\n", msg); } while (0)

/* PLL1 setup used by SystemClock_Config() in src/main.c:
   HSE 25 MHz / DIVM1 5 * (DIVN1 191 + 1) / (DIVP1 1 + 1) = 480 MHz. */
static void select_pll1_480mhz(void)
{
  host_rcc.CFGR      = RCC_CFGR_SWS_PLL1;
  host_rcc.PLLCKSELR = RCC_PLLCKSELR_PLLSRC_HSE | (5UL << RCC_PLLCKSELR_DIVM1_Pos);
  host_rcc.PLLCFGR   = 0;
  host_rcc.PLL1DIVR  = (191UL << 0U) | (1UL << 9U);
  host_rcc.PLL1FRACR = 0;
  host_rcc.D1CFGR    = 0;
}

static void test_system_init_registers(void)
{
  cmsis_host_reset_peripherals();
  SystemInit();

  CHECK((host_scb.CPACR & ((3UL << 20) | (3UL << 22))) == ((3UL << 20) | (3UL << 22)),
        "SystemInit: CP10/CP11 full access in SCB->CPACR");
  CHECK(host_scb.VTOR == FLASH_BANK1_BASE, "SystemInit: vector table at flash bank 1");
  CHECK((host_flash.ACR & FLASH_ACR_LATENCY) == FLASH_LATENCY_DEFAULT,
        "SystemInit: flash latency raised to the default 7 wait states");

  CHECK((host_rcc.CR & RCC_CR_HSION) == RCC_CR_HSION, "SystemInit: HSION set");
  CHECK((host_rcc.CR & (RCC_CR_HSEON | RCC_CR_CSION | RCC_CR_HSI48ON | RCC_CR_CSIKERON |
                        RCC_CR_PLL1ON | RCC_CR_PLL2ON | RCC_CR_PLL3ON | RCC_CR_HSEBYP)) == 0,
        "SystemInit: HSE/CSI/HSI48/PLL enables and HSEBYP cleared");
  CHECK(host_rcc.CFGR == 0, "SystemInit: CFGR reset");
  CHECK(host_rcc.D1CFGR == 0 && host_rcc.D2CFGR == 0 && host_rcc.D3CFGR == 0,
        "SystemInit: domain prescaler registers reset");
  CHECK(host_rcc.PLLCKSELR == 0x02020200UL, "SystemInit: PLLCKSELR reset value");
  CHECK(host_rcc.PLLCFGR == 0x01FF0000UL, "SystemInit: PLLCFGR reset value");
  CHECK(host_rcc.PLL1DIVR == 0x01010280UL && host_rcc.PLL2DIVR == 0x01010280UL &&
        host_rcc.PLL3DIVR == 0x01010280UL, "SystemInit: PLLxDIVR reset values");
  CHECK(host_rcc.PLL1FRACR == 0 && host_rcc.PLL2FRACR == 0 && host_rcc.PLL3FRACR == 0,
        "SystemInit: PLLxFRACR cleared");
  CHECK(host_rcc.CIER == 0, "SystemInit: clock interrupts disabled");
  CHECK(host_fmc_bank1.BTCR[0] == 0x000030D2UL, "SystemInit: FMC bank1 disabled");
}

static void test_system_init_lowers_flash_latency(void)
{
  cmsis_host_reset_peripherals();
  host_flash.ACR = FLASH_ACR_LATENCY; /* 15 wait states, above the default */
  SystemInit();

  CHECK((host_flash.ACR & FLASH_ACR_LATENCY) == FLASH_LATENCY_DEFAULT,
        "SystemInit: flash latency lowered to the default 7 wait states");
}

static void test_core_clock_default(void)
{
  cmsis_host_reset_peripherals();
  SystemCoreClock = 0;
  SystemInit();

  CHECK(SystemCoreClock == 0, "SystemInit: leaves SystemCoreClock to the caller");

  SystemCoreClockUpdate();
  CHECK(SystemCoreClock == 64000000UL, "after reset state: HSI selected, 64 MHz");
  CHECK(SystemD2Clock == 64000000UL, "after reset state: D2 clock 64 MHz");
}

static void test_core_clock_oscillator_sources(void)
{
  cmsis_host_reset_peripherals();
  SystemInit();
  host_rcc.CR |= RCC_CR_HSIDIV_4;
  SystemCoreClockUpdate();
  CHECK(SystemCoreClock == 16000000UL, "HSI with HSIDIV=4: 16 MHz");

  cmsis_host_reset_peripherals();
  SystemInit();
  host_rcc.CFGR = RCC_CFGR_SWS_CSI;
  SystemCoreClockUpdate();
  CHECK(SystemCoreClock == 4000000UL, "CSI selected: 4 MHz");

  cmsis_host_reset_peripherals();
  SystemInit();
  host_rcc.CFGR = RCC_CFGR_SWS_HSE;
  SystemCoreClockUpdate();
  CHECK(SystemCoreClock == 25000000UL, "HSE selected: 25 MHz");
}

static void test_core_clock_pll1(void)
{
  cmsis_host_reset_peripherals();
  SystemInit();
  select_pll1_480mhz();
  SystemCoreClockUpdate();
  CHECK(SystemCoreClock == 480000000UL, "PLL1 off HSE: 480 MHz");
  CHECK(SystemD2Clock == 480000000UL, "PLL1 off HSE, HPRE=1: D2 clock 480 MHz");

  /* D1CPRE = /2 on the core, HPRE = /2 on the AXI/AHB bus. */
  select_pll1_480mhz();
  host_rcc.D1CFGR = (0x8UL << RCC_D1CFGR_D1CPRE_Pos) | (0x8UL << RCC_D1CFGR_HPRE_Pos);
  SystemCoreClockUpdate();
  CHECK(SystemCoreClock == 240000000UL, "PLL1 with D1CPRE=/2: 240 MHz");
  CHECK(SystemD2Clock == 120000000UL, "PLL1 with D1CPRE=/2 and HPRE=/2: 120 MHz");

  /* Fractional divider: FRACN1 = 0x1000 adds half a multiplier step. */
  select_pll1_480mhz();
  host_rcc.PLLCFGR  = RCC_PLLCFGR_PLL1FRACEN;
  host_rcc.PLL1FRACR = (0x1000UL << 3U);
  SystemCoreClockUpdate();
  /* The VCO maths runs in single precision, so allow the last few Hz to drift. */
  CHECK(SystemCoreClock > 481249000UL && SystemCoreClock < 481251000UL,
        "PLL1 with FRACN1=0x1000: 481.25 MHz");

  /* DIVM1 = 0 disables the PLL input divider, so no clock can be derived. */
  select_pll1_480mhz();
  host_rcc.PLLCKSELR = RCC_PLLCKSELR_PLLSRC_HSE;
  SystemCoreClockUpdate();
  CHECK(SystemCoreClock == 0, "PLL1 with DIVM1=0: 0 Hz");
}

int main(void)
{
  test_system_init_registers();
  test_system_init_lowers_flash_latency();
  test_core_clock_default();
  test_core_clock_oscillator_sources();
  test_core_clock_pll1();

  printf("%s (%d failures)\n", fails ? "FAILED" : "PASSED", fails);
  return fails ? 1 : 0;
}
