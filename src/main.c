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

// Maximum number of polling iterations spent on a single clock ready bit.
// At the 64 MHz HSI the core runs on at reset this is worth a few tens of ms,
// i.e. far more than the worst case start-up time of the HSE and of PLL1.
#define CLOCK_POLL_TIMEOUT  2000000UL

// IWDG: LSI (~32 kHz) / 256 with a full reload gives a ~32 s timeout, long
// enough for the boot sequence and for the HIL loop waiting on the host.
#define IWDG_KEY_RELOAD  0x0000AAAAUL
#define IWDG_KEY_ENABLE  0x0000CCCCUL
#define IWDG_KEY_WRITE   0x00005555UL
#define IWDG_PRESCALER   0x6UL
#define IWDG_RELOAD      0xFFFUL

#define HSI_FREQ_HZ  64000000UL   // HSI frequency after reset (HSIDIV = 1)
#define PLL_FREQ_HZ  480000000UL  // sysclk once PLL1 is the system clock

/* Private variables ---------------------------------------------------------*/

// Clock feeding UART5 (rcc_pclk1), set by SystemClock_Config().
static uint32_t UART_ClockHz = 120000000UL;

// Non zero when the boot clock configuration timed out and the MCU fell back
// to the HSI instead of running at 480 MHz.
static uint8_t Clock_Fault = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void IWDG_Init(void);
static inline void IWDG_Refresh(void);
static int  Clock_WaitBits(volatile uint32_t*, uint32_t, uint32_t);
static void SystemClock_FallbackHSI(void);
static void LED_Init(void);
static void LED_Blink_Fault(void);
static void UART_Init(void);
static inline void UART_send_blocking(uint8_t*);
static inline void UART_rcv_blocking(uint8_t*);

/**
  * The application entry point.
  */
int main(void)
{

  /* Arm the watchdog before touching the clock tree so that a hardware fault
     which no timeout can catch still resets the part instead of hanging it */
  IWDG_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  LED_Init();
  UART_Init();

  /* Signal a degraded clock (HSE or PLL1 never came up) on the LED */
  if (Clock_Fault)
  {
      LED_Blink_Fault();
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
static void UART_Init(void)
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
  
  // Set baudrate (oversampling by 16), from the clock the UART is fed with
  // (120 MHz nominally, 64 MHz when the boot fell back to the HSI)
  uint16_t uartdiv = (uint16_t)(UART_ClockHz / 38400);
  UART5->BRR = uartdiv;
  
  
  // Enable the USART peripheral for transmission.
  UART5->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE ); 

}

/**
  * System Clock Configuration
  */
static void IWDG_Init(void)
{
    IWDG1->KR  = IWDG_KEY_ENABLE;   // start the watchdog (also starts the LSI)
    IWDG1->KR  = IWDG_KEY_WRITE;    // unprotect PR and RLR
    IWDG1->PR  = IWDG_PRESCALER;
    IWDG1->RLR = IWDG_RELOAD;
    // Wait for the registers to be updated, bounded so a stuck LSI cannot hang
    (void) Clock_WaitBits(&IWDG1->SR, 0x7UL, 0UL);
    IWDG1->KR  = IWDG_KEY_RELOAD;
}

/**
  * Reload the independent watchdog counter
  */
static inline void IWDG_Refresh(void)
{
    IWDG1->KR = IWDG_KEY_RELOAD;
}

/**
  * Poll a register until (reg & mask) == match, giving up after
  * CLOCK_POLL_TIMEOUT iterations.
  * @return 1 if the bits reached the expected value, 0 on timeout
  */
static int Clock_WaitBits(volatile uint32_t* reg, uint32_t mask, uint32_t match)
{
    for (uint32_t i = 0; i < CLOCK_POLL_TIMEOUT; i++)
    {
        if ((*reg & mask) == match)
        {
            return 1;
        }
        IWDG_Refresh();
    }

    return 0;
}

/**
  * Limp mode: run the system clock from the HSI (64 MHz, on since reset) when
  * the HSE or PLL1 failed to come up, so that the application still boots.
  */
static void SystemClock_FallbackHSI(void)
{
    RCC->CR |= RCC_CR_HSION;
    (void) Clock_WaitBits(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY);

    // Select the HSI as system clock and restore the reset prescalers
    RCC->CFGR &= ~RCC_CFGR_SW;
    (void) Clock_WaitBits(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSI);
    RCC->D1CFGR = 0;
    RCC->D2CFGR = 0;
    RCC->D3CFGR = 0;

    SystemCoreClock = HSI_FREQ_HZ;
    SystemD2Clock   = HSI_FREQ_HZ;
    UART_ClockHz    = HSI_FREQ_HZ;
    Clock_Fault     = 1;
}

/**
  * Blink the LED to report that the MCU is running in HSI limp mode
  */
static void LED_Blink_Fault(void)
{
    for (int i = 0; i < 10; i++)
    {
        GPIOA->ODR ^= GPIOA1;
        for (int j = 0; j < 1000000; j++) {__asm__("nop");}
        IWDG_Refresh();
    }

    GPIOA->ODR &= ~GPIOA1;
}

/**
  * System Clock Configuration
  */
static void SystemClock_Config(void)
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
    if (!Clock_WaitBits(&PWR->D3CR, PWR_D3CR_VOSRDY, PWR_D3CR_VOSRDY))
    {
        SystemClock_FallbackHSI();
        return;
    }

	/** 2) Oscillator initialisation **/

	//Enable HSE
	RCC->CR |= RCC_CR_HSEON;
	// Wait till HSE is ready
	if (!Clock_WaitBits(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY))
	{
		RCC->CR &= ~RCC_CR_HSEON;   // give up on a missing/broken crystal
		SystemClock_FallbackHSI();
		return;
	}

	// Switch (disconnect)
	RCC->CFGR |= 0x2UL;                  // Swich to HSE temporarly
	if (!Clock_WaitBits(&RCC->CFGR, RCC_CFGR_SWS, (0x00000010UL)))
	{
		SystemClock_FallbackHSI();
		return;
	}
	RCC->CR   &= ~1;				 // Disable HSI
	RCC->CR   &= ~(0x1UL << 24U);	// Disable PLL
	// wait for PPL to be disabled
	if (!Clock_WaitBits(&RCC->CR, RCC_CR_PLL1RDY, 0UL))
	{
		SystemClock_FallbackHSI();
		return;
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
	if (!Clock_WaitBits(&RCC->CR, RCC_CR_PLL1RDY, RCC_CR_PLL1RDY))
	{
		RCC->CR &= ~RCC_CR_PLLON;   // PLL1 never locked
		SystemClock_FallbackHSI();
		return;
	}

	/** 3) Clock initialisation **/

	//HPRE[3:0]: D1 domain AHB prescaler //1000: rcc_hclk3 = sys_d1cpre_ck / 2
	RCC -> D1CFGR |= (0x08UL << 0U);


	//D1CPRE[3:0]: D1 domain Core prescaler //0xxx: sys_ck not div. (default after reset)
	RCC -> D1CFGR |= (0x0UL << 8U);

	//SW[2:0]: System clock switch//011: PLL1 selected as system clock (pll1_p_ck)
	RCC->CFGR |= (0b011 << 0U);
	if (!Clock_WaitBits(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL1))
	{
		SystemClock_FallbackHSI();
		return;
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
	SystemD2Clock = (PLL_FREQ_HZ >> ((D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_HPRE)
	                                                   >> RCC_D1CFGR_HPRE_Pos]) & 0x1FU));
	SystemCoreClock = PLL_FREQ_HZ;
	UART_ClockHz = 120000000UL;   // rcc_pclk1 = sysclk / 2 / 2
}

/**
  * Send in blocking mode using UART5 peripheral
  */
static inline void UART_send_blocking(uint8_t* byte)
{
    // wait for empty transmit register
    while(!(UART5->ISR & USART_ISR_TXE_TXFNF)){IWDG_Refresh();};
    UART5->TDR = *byte;
}

/**
  * Receiving in blockin mode using UART5 peripheral 
  */
static inline void UART_rcv_blocking(uint8_t* byte)
{
    // wait for non empty read register
    while(!(UART5->ISR & USART_ISR_RXNE_RXFNE)){IWDG_Refresh();};
    *byte = UART5->RDR;

}