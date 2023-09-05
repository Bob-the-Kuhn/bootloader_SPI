/*
 * Author: Aurelio Colosimo, 2017
 *
 * This file is part of kim-os project: https://github.com/colosimo/kim-os
 * According to kim-os license, you can do whatever you want with it,
 * as long as you retain this notice.
 */

#ifndef _LPC1769X_H
#define _LPC1769X_H

#pragma once
//#include "cpu.h"
#include <stdio.h>


volatile uint32_t* LPC_SC_SCS      ; // System Controls and Status register
volatile uint32_t* LPC_SC_RSID     ; // Reset Source Identification Register

volatile uint32_t* LPC_SC_CLKSRCSEL; // Clock Source Select Register R/W 0

volatile uint32_t* LPC_SC_PLL0CON  ; // PLL0 Control Register R/W 0
volatile uint32_t* LPC_SC_PLL0CFG  ; // PLL0 Configuration Register R/W 0
volatile uint32_t* LPC_SC_PLL0STAT ; // PLL0 Status Register RO 0
volatile uint32_t* LPC_SC_PLL0FEED ; // PLL0 Feed Register WO NA
volatile uint32_t* LPC_SC_PLL1CON  ; // PLL1 Control Register R/W 0
volatile uint32_t* LPC_SC_PLL1CFG  ; // PLL1 Configuration Register R/W 0
volatile uint32_t* LPC_SC_PLL1STAT ; // PLL1 Status Register RO 0
volatile uint32_t* LPC_SC_PLL1FEED ; // PLL1 Feed Register WO NA

volatile uint32_t* LPC_SC_CCLKCFG  ; // CPU Clock Configuration Register R/W 0
volatile uint32_t* LPC_SC_USBCLKCFG; // USB Clock Configuration Register R/W 0
volatile uint32_t* LPC_SC_PCLKSEL0 ; // Peripheral Clock Selection register 0. R/W 0
volatile uint32_t* LPC_SC_PCLKSEL1 ; // Peripheral Clock Selection register 1. R/W 0

volatile uint32_t* LPC_SC_PCON     ; // Power Control Register R/W 0
volatile uint32_t* LPC_SC_PCONP    ; // Power Control for Peripherals Register R/W 0x03BE

volatile uint32_t* LPC_SC_CLKOUTCFG; // Clock Output Configuration Register R/W 0

volatile uint32_t* FLASHCFG        ; // Flash Accelerator Configuration register
#define FLASHTIM      12  // Flash access time bit offset

#define _BV(bit)     (1<<(bit))
#define SET_BIT(x, pos) (x |= (1U << pos))
#define CLEAR_BIT(x, pos) (x &= (~(1U<< pos)))

#define OSCSTAT       6
#define OSCRANGE      4
#define OSCEN         5
#define PLLE0_STAT    24
#define PLLC0_STAT    25
#define PLOCK0        26
#define PLLE0         0
#define PLLC0         1
//PCLKSEL0 bits
#define PCLK_WDT      0
#define PCLK_TIMER0   2
#define PCLK_TIMER1   4
#define PCLK_UART0    6
#define PCLK_UART1    8
#define PCLK_PWM1     12
#define PCLK_I2C0     14
#define PCLK_SPI      16
#define PCLK_SSP1     20
#define PCLK_DAC      22
#define PCLK_ADC      24
#define PCLK_CAN1     26
#define PCLK_CAN2     28
#define PCLK_ACF      30
//PCLKSEL1 bits
#define PCLK_QEI      0
#define PCLK_GPIOINT  2
#define PCLK_PCB      4
#define PCLK_I2C1     6
#define PCLK_SSP0     10
#define PCLK_TIMER2   12
#define PCLK_TIMER3   14
#define PCLK_UART2    16
#define PCLK_UART3    18
#define PCLK_I2C2     20
#define PCLK_I2S      22
#define PCLK_RIT      26
#define PCLK_SYSCON   28
#define PCLK_MC       30


volatile uint32_t* R_PCONP;  //Power Control for Peripherals register
#define PCTIM0        1     // Timer/Counter 0 power/clock control bit.
#define PCTIM1        2     // Timer/Counter 1 power/clock control bit.
#define PCUART0       3     // UART0 power/clock control bit.
#define PCUART1       4     // UART1 power/clock control bit.
#define PCPWM1        6     // PWM1 power/clock control bit.
#define PCI2C0        7     // The I2C0 interface power/clock control
#define PCSPI         8     // The SPI interface power/clock control
#define PCRTC         9     // The RTC power/clock control bit.
#define PCSSP1        10    // The SSP 1 interface power/clock control
#define PCADC         12    // A/D converter (ADC) power/clock control
                            //   Note: Clear the PDN bit in the AD0CR
                            //   this bit before setting PDN.
#define PCCAN1        13    // CAN Controller 1 power/clock control
#define PCCAN2        14    // CAN Controller 2 power/clock control
#define PCGPIO        15    // Power/clock control bit for IOCON, GPIO,
#define PCRIT         16    // Repetitive Interrupt Timer power/clock
#define PCMCPWM       17    // Motor Control PWM
#define PCQEI         18    // Quadrature Encoder Interface power/
#define PCI2C1        19    // The I2C1 interface power/clock control
#define PCSSP0        21    // The SSP0 interface power/clock control
#define PCTIM2        22    // Timer 2 power/clock control bit.
#define PCTIM3        23    // Timer 3 power/clock control bit.
#define PCUART2       24    // UART 2 power/clock control bit.
#define PCUART3       25    // UART 3 power/clock control bit.
#define PCI2C2        26    // I2C interface 2 power/clock control bit.
#define PCI2S         27    // I2S interface power/clock control bit.
#define PCGPDMA       29    // GPDMA function power/clock control bit.
#define PCENET        30    // Ethernet block power/clock control bit.
#define PCUSB         31    // USB interface power/clock control bit.

#define PLLC1         1
#define PLLE1         0
#define PLLE1_STAT    8
#define PLLC1_STAT    9
#define PLOCK1        10

//PCLKSEL0 & PCLKSEL1 clock divisor settings
enum PCLK_peripheral {
      CCLK_DIV_4 = 0x00,
      CCLK_DIV_1 = 0x01,
      CCLK_DIV_2 = 0x02,
      CCLK_DIV_8 = 0x03
    };


// UART0
volatile uint32_t* LPC_UART0_U0RBR;  // UART0 Receiver Buffer Register
volatile uint32_t* LPC_UART0_U0THR;  // UART0 Transmit Holding Register
volatile uint32_t* LPC_UART0_U0DLL;  // UART0 Divisor Latch LSB register
volatile uint32_t* LPC_UART0_U0DLM;  // UART0 Divisor Latch MSB register
volatile uint32_t* LPC_UART0_U0IER;  // UART0 Interrupt Enable Register
volatile uint32_t* LPC_UART0_U0IIR;  // UART0 Interrupt Identification Register
volatile uint32_t* LPC_UART0_U0FCR;  // UART0 FIFO Control Register

#define SBIT_FIFO         0
#define SBIT_RxFIFO       1
#define SBIT_TxFIFO       2
#define DMA_Mode_Select   3
#define RX_Trigger_Level  6  // 00 - 1 character, 01 - 4 characters, 10 - 8 characters, 11 - 14 characters
volatile uint32_t* LPC_UART0_U0LCR; // UART0 Line Control Register
#define SBIT_WordLenght   0 // 00 5-bit character length. 01 6-bit character length. 10 7-bit character length. 11 8-bit character length.
#define SBIT_DLAB         7
volatile uint32_t* LPC_UART0_U0FDR;  // UART0 Fractional Divider Register
#define MULVAL            4
volatile uint32_t* LPC_UART0_U0LSR;  // UARTn Line Status Register1
#define THRE              5  // Transmitter Holding Register Empty






//System Tick Timer

volatile uint32_t* SysTick_STCTRL; // System Timer Control and status register
#define ENABLE           0
#define TICKINT          1
#define CLKSOURCE        2
#define COUNTFLAG        16
volatile uint32_t* SysTick_STRELOAD; // System Timer Reload value register
volatile uint32_t* SysTick_STCURR  ; // System Timer Current value register
volatile uint32_t* SysTick_STCALIB ; // System Timer Calibration value register


// misc
#define VTOR         = (uint32_t*)0xE000ED08)  // Vector Table Offset Register
volatile uint32_t* SCB_AIRCR;          // SCB AIRCR
#define SCB_AIRCR_VECTKEY_Pos 16U      // SCB AIRCR: VECTKEY Position
#define SCB_AIRCR_VECTKEY 0x5FAUL      // SCB AIRCR: VECTKEY value
#define SCB_AIRCR_SYSRESETREQ_Pos 2U   // SCB AIRCR: system reset Position 

// GPIO

volatile uint32_t* GPIO_PINSEL0   ;  // Pin Function Select register 0
volatile uint32_t* GPIO_PINSEL1   ;  // Pin Function Select Register 1

volatile uint32_t* GPIO_PINMODE0  ;  // Pin Mode select register 0
volatile uint32_t* GPIO_PINMODE1  ;  // Pin Mode select register 1

volatile uint32_t* GPIO_PINMODE_OD;  // Open Drain Pin Mode select register 0

volatile uint32_t* GPIO_FIODIR    ;         // Fast GPIO Port Direction control register
volatile uint32_t* GPIO_FIOMASK   ;         // Fast Mask register for port.
volatile uint32_t* GPIO_FIOPIN    ;         // Fast Port Pin value register using FIOMASK
volatile uint32_t* GPIO_FIOSET    ;         // Fast Port Output Set register using FIOMASK
volatile uint32_t* GPIO_FIOCLR    ;         // Fast Port Output Clear register using FIOMASK


// SPI

//volatile uint16_t* BASE_SPI0 = (uint16_t*)0x40020000UL;
//#define S0SPCR        *(BASE_SPI0 + 0x00)
//#define S0SPSR        *(BASE_SPI0 + 0x02) // pointer math 4 -> 2
//#define S0SPDR        *(BASE_SPI0 + 0x04) //  8 -> 4
//#define S0SPCCR       *(BASE_SPI0 + 0x06) //  C -> 6

#define SCR 8                 //  clock divider bits start in SSP1CR0
#define SSE 1                 //  SSP1 enable/disable bit in SSP1CR1
volatile uint16_t* SSP1CR0 ;  // SPI Control Register
volatile uint16_t* SSP1CR1 ;  // SPI Control Register
volatile uint16_t* SSP1DR  ;  // SPI Data Register (bi-directional)
volatile uint16_t* SSP1SR  ;  // SPI Status Register
volatile uint16_t* SSP1CPSR;  // Clock Counter Register
volatile uint16_t* SSP1IMSC;  // SPI Interrupt enable mask





#endif /* _LPC1769X_ */
