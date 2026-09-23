/*==============================================================================
 * Name        : main.c
 * Author      : Martin Doff-Sotta (martin.doff-sotta@eng.ox.ac.uk) 
 * Description : HIL simulation for the stm32h743
 * Note        : Tested on stm32h743vit6 (version V) development board from DevEBox 
 -------------------------------------------------------------------------------
 * The MIT License (MIT)
 * Copyright (c) 2022 Martin Doff-Sotta
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
===============================================================================*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pid.h"
#include "hil_protocol.h"
#include <stdint.h>


// Uncomment this if you want to use the printf function (for debug purpose)
/*
#include <stdio.h>   // to use the printf function
#include <string.h>  // to use strings 

// Override the 'write' clib method to implement 'printf' over UART.
int _write(int handle, char* data, int size) {
  int count = size;
  
  // invariant: data[0:i] have been added to TDR register  
  while(count--) {
      while(!(UART5->ISR & USART_ISR_TXE_TXFNF)) {}; // wait for empty transmit register
      UART5->TDR = *data++; 
  
  }
  return size;
}
*/

/* Private constants ---------------------------------------------------------*/

// Spin budget for every RCC/PWR ready-bit poll. At the 64 MHz HSI clock that is
// running out of reset this is well over 100 ms, i.e. orders of magnitude more
// than the HSE start-up and PLL lock times.
#define CLOCK_WAIT_ITERATIONS  1200000UL

// UART5 kernel clock (rcc_pclk1) for each of the two clock configurations
#define PCLK1_PLL1_HZ          120000000UL  // 480 MHz sys_ck, HPRE /2, D2PPRE1 /2
#define PCLK1_HSI_HZ           64000000UL   // HSI fallback, all prescalers /1

// IWDG: 32 kHz LSI, /256 prescaler, maximum reload -> ~32 s time-out
#define IWDG_KEY_RELOAD        0x0000AAAAUL
#define IWDG_KEY_ENABLE        0x00005555UL
#define IWDG_KEY_START         0x0000CCCCUL
#define IWDG_PRESCALER_256     0x6UL
#define IWDG_RELOAD_MAX        0xFFFUL

/* Private function prototypes -----------------------------------------------*/
static void IWDG_Init(void);
static inline void IWDG_Refresh(void);
static int wait_bit(volatile uint32_t*, uint32_t, uint32_t);
static uint32_t SystemClock_Config(void);
static uint32_t SystemClock_Fallback_HSI(void);
static void Fault_Blink(uint32_t);
static void Fault_Reset(void);
static void LED_Init(void);
static void UART_Init(uint32_t);
static inline void UART_send_blocking(uint8_t*);
static inline void UART_rcv_blocking(uint8_t*);

/**
  * The application entry point.
  */
int main(void)
{

  /* Arm the watchdog before touching the clock tree so that a fault the bounded
     waits below cannot recover from still ends in a reset rather than a hang */
  IWDG_Init();

  /* Configure the system clock. Returns the resulting UART5 kernel clock: the
     nominal 120 MHz, or the HSI fallback value if the HSE/PLL never came up */
  uint32_t pclk1 = SystemClock_Config();

  /* Initialize all configured peripherals */
  LED_Init();
  UART_Init(pclk1);

  /* Signal a degraded clock on the LED before entering the control loop */
  if (pclk1 != PCLK1_PLL1_HZ)
  {
    Fault_Blink(6);
  }
  
  /* Initialise variables */
  float TAS = 0;
  float ref_TAS = 80;
  float u = 0;
  custom_float_t rcv;
  uint8_t frame[HIL_FRAME_BYTES];
  pid_ctrl_t pid;
  pid_init(&pid, 500.0f, 30.0f, 10.0f, 0.1f, 66.5f);
  
  /* Infinite loop */ 
  while (1)
  {

    	// Reception from Simulink
    	for (int i=0; i<HIL_FLOAT_BYTES; i++)
    	{
            UART_rcv_blocking(&rcv.bytes[i]);
    	}
    	                   
    	// Controller (PID)
    	TAS = rcv.single;                          // get true airspeed (TAS)
    	u = pid_step(&pid, ref_TAS, TAS);          // control law
    	
    	// Transmission to Simulink: header + float32 + terminator
    	hil_frame_encode(u, frame);
    	for (int i=0; i<HIL_FRAME_BYTES; i++)
    	{
            UART_send_blocking(&frame[i]);
    	}

    	IWDG_Refresh();
  }

}


/**
  * Configure peripherals for LED blink
  */
static void LED_Init(void)
{
  
  //Initialize all configured peripherals
  RCC_AHB4ENR |= RCC_AHB4ENR_GPIOAEN;

  // Set PA1 to output push-pull
  GPIOA->MODER   &=  ~(0x3UL << 2U);
  GPIOA->MODER   |=   (0x1UL << 2U);  //-> 01 in MODER1[1:0] (General purpose output mode)
  GPIOA->OTYPER  &=  ~(0x1UL << 1U);
  GPIOA->OTYPER  |=   (0x0UL << 1U);  //-> 0 in OT1 (Output push-pull)
  GPIOA->OSPEEDR &=  ~(0x3UL << 2U);
  GPIOA->OSPEEDR |=   (0x0UL << 2U);  //-> 00 in OSPEEDR1[1:0] (Low speed)
  GPIOA->PUPDR   &=  ~(0x3UL << 2U);
  GPIOA->PUPDR   |=   (0x0UL << 2U);  //-> 00 in PUPDR1[1:0] (No pull-up/-down)
}

/**
  * Configure UART5 peripherals
  * PB12: UART5_RX (receive), PB13: UART5_TX (transmit)
  */
static void UART_Init(uint32_t pclk1)
{
  
  // Enable peripheral clocks: GPIOB, UART5.
  RCC_AHB4ENR  |= RCC_AHB4ENR_GPIOBEN;
  RCC_APB1LENR |= RCC_APB1LENR_UART5EN;

  // Set PB13 for TX UART5 
  GPIOB->MODER   &=  ~(0x3UL << 26U);
  GPIOB->MODER   |=   (0x2UL << 26U); // MODER13[1:0] <- 0x10 for alternate function mode
  GPIOB->OTYPER  &=  ~(0x1UL << 13U);
  GPIOB->OTYPER  |=   (0x0UL << 13U); // OT13 <- 0x0 for output push-pull
  GPIOB->OSPEEDR &=  ~(0x3UL << 26U);
  GPIOB->OSPEEDR |=   (0x3UL << 26U); // OSPEEDER13[1:0] <- 0x11 for very high speed
  GPIOB->PUPDR   &=  ~(0x3UL << 26U);
  GPIOB->PUPDR   |=   (0x0UL << 26U); // PUPDR13[1:0] <- 0x00 for no pull-up/pull_down
  GPIOB->AFR[1]  &=  ~(0xFUL << 20U);
  GPIOB->AFR[1]  |=   (0xEUL << 20U); // AFR13[3:0] <- 0x1110 to set PB13 as AFR14 (UART5)
  
  // Set PB12 for RX UART5 
  GPIOB->MODER   &=  ~(0x3UL << 24U);
  GPIOB->MODER   |=   (0x2UL << 24U); // MODER12[1:0] <- 0x10 for alternate function mode
  GPIOB->OTYPER  &=  ~(0x1UL << 12U);
  GPIOB->OTYPER  |=   (0x0UL << 12U); // OT12 <- 0x0 for output push-pull
  GPIOB->OSPEEDR &=  ~(0x3UL << 24U);
  GPIOB->OSPEEDR |=   (0x3UL << 24U); // OSPEEDER12[1:0] <- 0x11 for very high speed
  GPIOB->PUPDR   &=  ~(0x3UL << 24U);
  GPIOB->PUPDR   |=   (0x0UL << 24U); // PUPDR12[1:0] <- 0x00 for no pull-up/pull_down
  GPIOB->AFR[1]  &=  ~(0xFUL << 16U);
  GPIOB->AFR[1]  |=   (0xEUL << 16U); // AFR12[3:0] <- 0x1110 to set PB12 as AFR14 (UART5)
  
  // Set baudrate (oversampling by 16) from the actual UART5 kernel clock
  uint16_t uartdiv = (uint16_t)(pclk1 / 38400U);
  UART5->BRR = uartdiv;
  
  
  // Enable the USART peripheral for transmission.
  UART5->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE ); 

}

/**
  * Start the independent watchdog (~32 s time-out)
  */
static void IWDG_Init(void)
{
    IWDG1->KR  = IWDG_KEY_START;   // start the watchdog (and the LSI with it)
    IWDG1->KR  = IWDG_KEY_ENABLE;  // unlock PR/RLR
    IWDG1->PR  = IWDG_PRESCALER_256;
    IWDG1->RLR = IWDG_RELOAD_MAX;
    wait_bit(&IWDG1->SR, 0x7UL, 0x0UL);  // registers updated (bounded)
    IWDG1->KR  = IWDG_KEY_RELOAD;
}

/**
  * Kick the independent watchdog
  */
static inline void IWDG_Refresh(void)
{
    IWDG1->KR = IWDG_KEY_RELOAD;
}

/**
  * Poll a register field until it reaches the expected value.
  * Return 0 on success, -1 once the spin budget is exhausted.
  */
static int wait_bit(volatile uint32_t* reg, uint32_t mask, uint32_t expected)
{
    for (uint32_t i = 0; i < CLOCK_WAIT_ITERATIONS; i++)
    {
        if ((*reg & mask) == expected)
        {
            return 0;
        }
        __asm__("nop");
    }
    return -1;
}

/**
  * Blink the fault LED on PA1 the requested number of times
  */
static void Fault_Blink(uint32_t count)
{
    LED_Init();
    for (uint32_t n = 0; n < 2 * count; n++)
    {
        GPIOA->ODR ^= GPIOA1;
        for (int i = 0; i < 1000000; i++) {__asm__("nop");}
        IWDG_Refresh();
    }
    GPIOA->ODR &= ~GPIOA1;
}

/**
  * Unrecoverable clock fault: signal it on the LED, then reset the part
  */
static void Fault_Reset(void)
{
    Fault_Blink(10);
    NVIC_SystemReset();
}

/**
  * Fall back to the HSI that is running out of reset when the HSE or the PLL
  * fails to come up. Return the resulting UART5 kernel clock.
  */
static uint32_t SystemClock_Fallback_HSI(void)
{
    RCC->CR |= RCC_CR_HSION;
    if (wait_bit(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY) != 0)
    {
        Fault_Reset();
    }

    RCC->CFGR &= ~RCC_CFGR_SW;  // HSI as system clock
    if (wait_bit(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSI) != 0)
    {
        Fault_Reset();
    }

    RCC->CR &= ~RCC_CR_HSEON;
    RCC->CR &= ~RCC_CR_PLLON;

    // Back to the reset value of the domain prescalers (no division)
    RCC->D1CFGR = 0;
    RCC->D2CFGR = 0;
    RCC->D3CFGR = 0;

    SystemCoreClock = PCLK1_HSI_HZ;
    SystemD2Clock   = PCLK1_HSI_HZ;

    return PCLK1_HSI_HZ;
}

/**
  * System Clock Configuration
  * Every ready-bit poll is bounded: on expiry the HSI fallback keeps the part
  * running at reduced speed instead of hanging forever in early boot.
  * Return the UART5 kernel clock (rcc_pclk1) in Hz.
  */
static uint32_t SystemClock_Config(void)
{
    uint32_t __attribute((unused)) tmpreg ; 

    /**  1) Boost the voltage scaling level to VOS0 to reach system maximum frequency **/
	
    // Supply configuration update enable
    MODIFY_REG(PWR->CR3, (PWR_CR3_SCUEN | PWR_CR3_LDOEN | PWR_CR3_BYPASS), PWR_CR3_LDOEN);
    for(int i=0; i<1500000;i++){__asm__("nop");}
  
    // Configure the Voltage Scaling 1 in order to modify ODEN bit 
    MODIFY_REG(PWR->D3CR, PWR_D3CR_VOS, (0x2UL << 14U));
    // Delay after setting the voltage scaling 
    tmpreg = READ_BIT(PWR->D3CR, PWR_D3CR_VOS);
    // Enable the PWR overdrive
    SET_BIT(SYSCFG->PWRCR, SYSCFG_PWRCR_ODEN);
    // Delay after setting the syscfg boost setting 
    tmpreg = READ_BIT(SYSCFG->PWRCR, SYSCFG_PWRCR_ODEN);

    // Wait for VOS to be ready
    if (wait_bit(&PWR->D3CR, PWR_D3CR_VOSRDY, PWR_D3CR_VOSRDY) != 0)
    {
        return SystemClock_Fallback_HSI();
    }

	/** 2) Oscillator initialisation **/

	//Enable HSE
	RCC->CR |= RCC_CR_HSEON;
	// Wait till HSE is ready
	if (wait_bit(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY) != 0)
	{
		return SystemClock_Fallback_HSI();
	}

	// Switch (disconnect)
	RCC->CFGR |= 0x2UL;                  // Swich to HSE temporarly
	if (wait_bit(&RCC->CFGR, RCC_CFGR_SWS, 0x00000010UL) != 0)
	{
		return SystemClock_Fallback_HSI();
	}
	RCC->CR   &= ~1;				 // Disable HSI
	RCC->CR   &= ~(0x1UL << 24U);	// Disable PLL
	// wait for PLL to be disabled
	if (wait_bit(&RCC->CR, RCC_CR_PLL1RDY, 0x0UL) != 0)
	{
		return SystemClock_Fallback_HSI();
	}

    // Config PLL
	//RCC -> PLLCKSELR |= RCC_PLLCKSELR_PLLSRC_HSE; //RCC -> PLLCKSELR |= (0x05UL << 4U);
	//MODIFY_REG(RCC->PLLCKSELR, (RCC_PLLCKSELR_PLLSRC ) , (RCC_PLLSOURCE_HSE) );
	RCC -> PLLCKSELR &= ~(0b111111UL << 4U); // reset bit
	RCC -> PLLCKSELR |= (0x05UL << 4U);
	RCC -> PLLCKSELR |= RCC_PLLCKSELR_PLLSRC_HSE;
	//MODIFY_REG(RCC->PLLCKSELR, ( RCC_PLLCKSELR_DIVM1) , ( (5) <<4U) );

	// DIVN = 192, DIVP = 2, DIVQ = 2, DIVR = 2.
	RCC -> PLL1DIVR  |= (0xBFUL << 0U);
	RCC -> PLL1DIVR  |= (0x01UL << 9U);
	RCC -> PLL1DIVR  |= (0x01UL << 16U);
	RCC -> PLL1DIVR  |= (0x01UL << 24U);

	// Disable PLLFRACN
	RCC->PLLCFGR &= ~(0x1UL << 0U);

	//  Configure PLL  PLL1FRACN 
	//__HAL_RCC_PLLFRACN_CONFIG(RCC_OscInitStruct->PLL.PLLFRACN);
	RCC -> PLL1FRACR = 0;

	//Select PLL1 input reference frequency range: VCI 
	//__HAL_RCC_PLL_VCIRANGE(RCC_OscInitStruct->PLL.PLLRGE) ;
	//RCC->PLLCFGR |= RCC_PLLCFGR_PLL1RGE_3;
	RCC->PLLCFGR |= (0x2UL << 2U);

	// Select PLL1 output frequency range : VCO 
	//__HAL_RCC_PLL_VCORANGE(RCC_OscInitStruct->PLL.PLLVCOSEL) ;
	//RCC->PLLCFGR &= ~RCC_PLLCFGR_PLL1VCOSEL;
	RCC->PLLCFGR |= (0x0UL << 1U);

	// Enable PLL System Clock output. // __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL1_DIVP);
	//Bit 16 DIVP1EN: PLL1 DIVP divider output enable
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVP1EN;

	// Enable PLL1Q Clock output. //__HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL1_DIVQ);
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVQ1EN;

	// Enable PLL1R  Clock output. // __HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL1_DIVR);
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVR1EN;

	// Enable PLL1FRACN . //__HAL_RCC_PLLFRACN_ENABLE();
	RCC->PLLCFGR |= RCC_PLLCFGR_PLL1FRACEN;

	// Enable the main PLL. //__HAL_RCC_PLL_ENABLE();
	RCC->CR |= RCC_CR_PLLON;
	if (wait_bit(&RCC->CR, RCC_CR_PLL1RDY, RCC_CR_PLL1RDY) != 0)
	{
		return SystemClock_Fallback_HSI();
	}

	/** 3) Clock initialisation **/

	//HPRE[3:0]: D1 domain AHB prescaler //1000: rcc_hclk3 = sys_d1cpre_ck / 2
	RCC -> D1CFGR |= (0x08UL << 0U);


	//D1CPRE[3:0]: D1 domain Core prescaler //0xxx: sys_ck not div. (default after reset)
	RCC -> D1CFGR |= (0x0UL << 8U);

	//SW[2:0]: System clock switch//011: PLL1 selected as system clock (pll1_p_ck)
	RCC->CFGR |= (0b011 << 0U);
	if (wait_bit(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL1) != 0)
	{
		return SystemClock_Fallback_HSI();
	}

	//D1PPRE[2:0]: D1 domain APB3 prescaler//100: rcc_pclk3 = rcc_hclk3 / 2
	RCC->D1CFGR   |= (0b100 << 4U);


	//D2PPRE1[2:0]: D2 domain APB1 prescaler//100: rcc_pclk1 = rcc_hclk1 / 2
	RCC -> D2CFGR |=  (0b100 << 4U);

	//D2PPRE2[2:0]: D2 domain APB2 prescaler//100: rcc_pclk2 = rcc_hclk1 / 2
	RCC -> D2CFGR |=  (0b100 << 8U);


	//D3PPRE[2:0]: D3 domain APB4 prescaler//100: rcc_pclk4 = rcc_hclk4 / 2
	RCC -> D3CFGR |=  (0b100 << 4U);

	//Update global variables
	const  uint8_t D1CorePrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
	SystemD2Clock = (480000000 >> ((D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_HPRE)
	                                                   >> RCC_D1CFGR_HPRE_Pos]) & 0x1FU));
	SystemCoreClock = 480000000;

	return PCLK1_PLL1_HZ;
}

/**
  * Send in blocking mode using UART5 peripheral
  */
static inline void UART_send_blocking(uint8_t* byte)
{
    // wait for empty transmit register, kicking the watchdog while idle
    while(!(UART5->ISR & USART_ISR_TXE_TXFNF)){IWDG_Refresh();};
    UART5->TDR = *byte;
}

/**
  * Receiving in blockin mode using UART5 peripheral 
  */
static inline void UART_rcv_blocking(uint8_t* byte)
{
    // wait for non empty read register, kicking the watchdog while idle
    while(!(UART5->ISR & USART_ISR_RXNE_RXFNE)){IWDG_Refresh();};
    *byte = UART5->RDR;

}