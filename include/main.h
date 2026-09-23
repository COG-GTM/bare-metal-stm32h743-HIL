/*==============================================================================
 * Name        : main.h
 * Author      : Martin Doff-Sotta (martin.doff-sotta@eng.ox.ac.uk) 
 * Description : Header for main.c file.
 -------------------------------------------------------------------------------
 * The MIT License (MIT)
 * Copyright (c) 2021 Martin Doff-Sotta
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32h7xx.h"

/* Define registers */
#define RCC_AHB4ENR   *(volatile uint32_t *)(RCC_BASE   + 0x0E0)  // register AHB4
#define RCC_APB1LENR  *(volatile uint32_t *)(RCC_BASE   + 0x0E8)  // register APB1 for UART5

// Bit fields
#define GPIOA1  (1UL <<  1)

/* HIL timing configuration ---------------------------------------------------
 * The loop cadence is imposed by the host: the Simulink model sends one sample
 * every HIL_SAMPLE_TIME_S over UART5 at HIL_UART_BAUDRATE. Keep these values in
 * sync with the Simulink serial blocks (sample time and baud rate).
 */
#define HIL_UART_BAUDRATE     38400UL   // UART5 baud rate, must match Simulink
#define HIL_UART_KERNEL_CLK   120000000UL  // UART5 kernel clock (pclk1)
#define HIL_SAMPLE_TIME_S     0.1f      // nominal Simulink block sample time [s]
#define HIL_MIN_DT_S          (HIL_SAMPLE_TIME_S / 10.0f)  // dt sanity bounds
#define HIL_MAX_DT_S          (HIL_SAMPLE_TIME_S * 10.0f)


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */