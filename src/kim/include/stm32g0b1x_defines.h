/*
 * Author: Aurelio Colosimo, 2017
 *
 * This file is part of kim-os project: https://github.com/colosimo/kim-os
 * According to kim-os license, you can do whatever you want with it,
 * as long as you retain this notice.
 */

#ifndef _STM32G0B1X__
#define _STM32G0B1X_

#include <cpu.h>

/* RCC registers */
#define RCC_CR          reg32(0x40021000)
#define RCC_ICSCR       reg32(0x40021004)
#define RCC_CFGR        reg32(0x40021008)
#define RCC_PLLCFGR     reg32(0x4002100C)
#define RCC_RESERVED0   reg32(0x40021010)
#define RCC_CRRCR       reg32(0x40021014)
#define RCC_CIER        reg32(0x40021018)
#define RCC_CIFR        reg32(0x4002101C)
#define RCC_CICR        reg32(0x40021020)
#define RCC_IOPRSTR     reg32(0x40021024)
#define RCC_AHBRSTR     reg32(0x40021028)
#define RCC_APBRSTR1    reg32(0x4002102C)
#define RCC_APBRSTR2    reg32(0x40021030)
#define RCC_IOPENR      reg32(0x40021034)
#define RCC_AHBENR      reg32(0x40021038)
#define RCC_APBENR1     reg32(0x4002103C)
#define RCC_APBENR2     reg32(0x40021040)
#define RCC_IOPSMENR    reg32(0x40021044)
#define RCC_AHBSMENR    reg32(0x40021048)
#define RCC_APBSMENR1   reg32(0x4002104C)
#define RCC_APBSMENR2   reg32(0x40021050)
#define RCC_CCIPR       reg32(0x40021054)
#define RCC_CCIPR2      reg32(0x40021058)
#define RCC_BDCR        reg32(0x4002105C)
#define RCC_CSR         reg32(0x40021060)


/* GPIO registers */
#define R_GPIOA_MODER   reg32(0x50000000)
#define R_GPIOA_OTYPER  reg32(0x50000004)
#define R_GPIOA_OSPEEDR reg32(0x50000008)
#define R_GPIOA_PUPDR   reg32(0x5000000c)
#define R_GPIOA_IDR     reg32(0x50000010)
#define R_GPIOA_ODR     reg32(0x50000014)
#define R_GPIOA_BSRR    reg32(0x50000018)
#define R_GPIOA_LCKR    reg32(0x5000001c)
#define R_GPIOA_AFRL    reg32(0x50000020)
#define R_GPIOA_AFRH    reg32(0x50000024)
#define R_GPIOA_BRR     reg32(0x50000028)

#define R_GPIOB_MODER   reg32(0x50000400)
#define R_GPIOB_OTYPER  reg32(0x50000404)
#define R_GPIOB_OSPEEDR reg32(0x50000408)
#define R_GPIOB_PUPDR   reg32(0x5000040c)
#define R_GPIOB_IDR     reg32(0x50000410)
#define R_GPIOB_ODR     reg32(0x50000414)
#define R_GPIOB_BSRR    reg32(0x50000418)
#define R_GPIOB_LCKR    reg32(0x5000041c)
#define R_GPIOB_AFRL    reg32(0x50000420)
#define R_GPIOB_AFRH    reg32(0x50000424)
#define R_GPIOB_BRR     reg32(0x50000428)

#define R_GPIOC_MODER   reg32(0x50000800)
#define R_GPIOC_OTYPER  reg32(0x50000804)
#define R_GPIOC_OSPEEDR reg32(0x50000808)
#define R_GPIOC_PUPDR   reg32(0x5000080c)
#define R_GPIOC_IDR     reg32(0x50000810)
#define R_GPIOC_ODR     reg32(0x50000814)
#define R_GPIOC_BSRR    reg32(0x50000818)
#define R_GPIOC_LCKR    reg32(0x5000081c)
#define R_GPIOC_AFRL    reg32(0x50000820)
#define R_GPIOC_AFRH    reg32(0x50000824)
#define R_GPIOC_BRR     reg32(0x50000828)

#define R_GPIOD_MODER   reg32(0x50000c00)
#define R_GPIOD_OTYPER  reg32(0x50000c04)
#define R_GPIOD_OSPEEDR reg32(0x50000c08)
#define R_GPIOD_PUPDR   reg32(0x50000c0c)
#define R_GPIOD_IDR     reg32(0x50000c10)
#define R_GPIOD_ODR     reg32(0x50000c14)
#define R_GPIOD_BSRR    reg32(0x50000c18)
#define R_GPIOD_LCKR    reg32(0x50000c1c)
#define R_GPIOD_AFRL    reg32(0x50000c20)
#define R_GPIOD_AFRH    reg32(0x50000c24)
#define R_GPIOD_BRR     reg32(0x50000c28)

#define R_GPIOE_MODER   reg32(0x50001000)
#define R_GPIOE_OTYPER  reg32(0x50001004)
#define R_GPIOE_OSPEEDR reg32(0x50001008)
#define R_GPIOE_PUPDR   reg32(0x5000100c)
#define R_GPIOE_IDR     reg32(0x50001010)
#define R_GPIOE_ODR     reg32(0x50001014)
#define R_GPIOE_BSRR    reg32(0x50001018)
#define R_GPIOE_LCKR    reg32(0x5000101c)
#define R_GPIOE_AFRL    reg32(0x50001020)
#define R_GPIOE_AFRH    reg32(0x50001024)
#define R_GPIOE_BRR     reg32(0x50001028)

#define R_GPIOF_MODER   reg32(0x50001400)
#define R_GPIOF_OTYPER  reg32(0x50001404)
#define R_GPIOF_OSPEEDR reg32(0x50001408)
#define R_GPIOF_PUPDR   reg32(0x5000140c)
#define R_GPIOF_IDR     reg32(0x50001410)
#define R_GPIOF_ODR     reg32(0x50001414)
#define R_GPIOF_BSRR    reg32(0x50001418)
#define R_GPIOF_LCKR    reg32(0x5000141c)
#define R_GPIOF_AFRL    reg32(0x50001420)
#define R_GPIOF_AFRH    reg32(0x50001424)
#define R_GPIOF_BRR     reg32(0x50001428)

/* USART registers */
#define R_USART1_CR1    reg32(0x40013800)    /*!< USART Control register 1,                 Address offset: 0x00  */
#define R_USART1_CR2    reg32(0x40013804)    /*!< USART Control register 2,                 Address offset: 0x04  */
#define R_USART1_CR3    reg32(0x40013808)    /*!< USART Control register 3,                 Address offset: 0x08  */
#define R_USART1_BRR    reg32(0x4001380C)    /*!< USART Baud rate register,                 Address offset: 0x0C  */
#define R_USART1_GTPR   reg32(0x40013810)    /*!< USART Guard time and prescaler register,  Address offset: 0x10  */
#define R_USART1_RTOR   reg32(0x40013814)    /*!< USART Receiver Time Out register,         Address offset: 0x14  */
#define R_USART1_RQR    reg32(0x40013818)    /*!< USART Request register,                   Address offset: 0x18  */
#define R_USART1_ISR    reg32(0x4001381C)    /*!< USART Interrupt and status register,      Address offset: 0x1C  */
#define R_USART1_ICR    reg32(0x40013820)    /*!< USART Interrupt flag Clear register,      Address offset: 0x20  */
#define R_USART1_RDR    reg32(0x40013824)    /*!< USART Receive Data register,              Address offset: 0x24  */
#define R_USART1_TDR    reg32(0x40013828)    /*!< USART Transmit Data register,             Address offset: 0x28  */
#define R_USART1_PRESC  reg32(0x4001382C)    /*!< USART Prescaler register,                 Address offset: 0x2C  */

#define R_USART2_CR1    reg32(0x40004400)    /*!< USART Control register 1,                 Address offset: 0x00  */
#define R_USART2_CR2    reg32(0x40004404)    /*!< USART Control register 2,                 Address offset: 0x04  */
#define R_USART2_CR3    reg32(0x40004408)    /*!< USART Control register 3,                 Address offset: 0x08  */
#define R_USART2_BRR    reg32(0x4000440C)    /*!< USART Baud rate register,                 Address offset: 0x0C  */
#define R_USART2_GTPR   reg32(0x40004410)    /*!< USART Guard time and prescaler register,  Address offset: 0x10  */
#define R_USART2_RTOR   reg32(0x40004414)    /*!< USART Receiver Time Out register,         Address offset: 0x14  */
#define R_USART2_RQR    reg32(0x40004418)    /*!< USART Request register,                   Address offset: 0x18  */
#define R_USART2_ISR    reg32(0x4000441C)    /*!< USART Interrupt and status register,      Address offset: 0x1C  */
#define R_USART2_ICR    reg32(0x40004420)    /*!< USART Interrupt flag Clear register,      Address offset: 0x20  */
#define R_USART2_RDR    reg32(0x40004424)    /*!< USART Receive Data register,              Address offset: 0x24  */
#define R_USART2_TDR    reg32(0x40004428)    /*!< USART Transmit Data register,             Address offset: 0x28  */
#define R_USART2_PRESC  reg32(0x4000442C)    /*!< USART Prescaler register,                 Address offset: 0x2C  */

#define R_USART3_CR1    reg32(0x40004800)    /*!< USART Control register 1,                 Address offset: 0x00  */
#define R_USART3_CR2    reg32(0x40004804)    /*!< USART Control register 2,                 Address offset: 0x04  */
#define R_USART3_CR3    reg32(0x40004808)    /*!< USART Control register 3,                 Address offset: 0x08  */
#define R_USART3_BRR    reg32(0x4000480C)    /*!< USART Baud rate register,                 Address offset: 0x0C  */
#define R_USART3_GTPR   reg32(0x40004810)    /*!< USART Guard time and prescaler register,  Address offset: 0x10  */
#define R_USART3_RTOR   reg32(0x40004814)    /*!< USART Receiver Time Out register,         Address offset: 0x14  */
#define R_USART3_RQR    reg32(0x40004818)    /*!< USART Request register,                   Address offset: 0x18  */
#define R_USART3_ISR    reg32(0x4000481C)    /*!< USART Interrupt and status register,      Address offset: 0x1C  */
#define R_USART3_ICR    reg32(0x40004820)    /*!< USART Interrupt flag Clear register,      Address offset: 0x20  */
#define R_USART3_RDR    reg32(0x40004824)    /*!< USART Receive Data register,              Address offset: 0x24  */
#define R_USART3_TDR    reg32(0x40004828)    /*!< USART Transmit Data register,             Address offset: 0x28  */
#define R_USART3_PRESC  reg32(0x4000482C)    /*!< USART Prescaler register,                 Address offset: 0x2C  */

#define R_USART4_CR1    reg32(0x40004C00)    /*!< USART Control register 1,                 Address offset: 0x00  */
#define R_USART4_CR2    reg32(0x40004C04)    /*!< USART Control register 2,                 Address offset: 0x04  */
#define R_USART4_CR3    reg32(0x40004C08)    /*!< USART Control register 3,                 Address offset: 0x08  */
#define R_USART4_BRR    reg32(0x40004C0C)    /*!< USART Baud rate register,                 Address offset: 0x0C  */
#define R_USART4_GTPR   reg32(0x40004C10)    /*!< USART Guard time and prescaler register,  Address offset: 0x10  */
#define R_USART4_RTOR   reg32(0x40004C14)    /*!< USART Receiver Time Out register,         Address offset: 0x14  */
#define R_USART4_RQR    reg32(0x40004C18)    /*!< USART Request register,                   Address offset: 0x18  */
#define R_USART4_ISR    reg32(0x40004C1C)    /*!< USART Interrupt and status register,      Address offset: 0x1C  */
#define R_USART4_ICR    reg32(0x40004C20)    /*!< USART Interrupt flag Clear register,      Address offset: 0x20  */
#define R_USART4_RDR    reg32(0x40004C24)    /*!< USART Receive Data register,              Address offset: 0x24  */
#define R_USART4_TDR    reg32(0x40004C28)    /*!< USART Transmit Data register,             Address offset: 0x28  */
#define R_USART4_PRESC  reg32(0x40004C2C)    /*!< USART Prescaler register,                 Address offset: 0x2C  */

#define R_USART5_CR1    reg32(0x40005000)    /*!< USART Control register 1,                 Address offset: 0x00  */
#define R_USART5_CR2    reg32(0x40005004)    /*!< USART Control register 2,                 Address offset: 0x04  */
#define R_USART5_CR3    reg32(0x40005008)    /*!< USART Control register 3,                 Address offset: 0x08  */
#define R_USART5_BRR    reg32(0x4000500C)    /*!< USART Baud rate register,                 Address offset: 0x0C  */
#define R_USART5_GTPR   reg32(0x40005010)    /*!< USART Guard time and prescaler register,  Address offset: 0x10  */
#define R_USART5_RTOR   reg32(0x40005014)    /*!< USART Receiver Time Out register,         Address offset: 0x14  */
#define R_USART5_RQR    reg32(0x40005018)    /*!< USART Request register,                   Address offset: 0x18  */
#define R_USART5_ISR    reg32(0x4000501C)    /*!< USART Interrupt and status register,      Address offset: 0x1C  */
#define R_USART5_ICR    reg32(0x40005020)    /*!< USART Interrupt flag Clear register,      Address offset: 0x20  */
#define R_USART5_RDR    reg32(0x40005024)    /*!< USART Receive Data register,              Address offset: 0x24  */
#define R_USART5_TDR    reg32(0x40005028)    /*!< USART Transmit Data register,             Address offset: 0x28  */
#define R_USART5_PRESC  reg32(0x4000502C)    /*!< USART Prescaler register,                 Address offset: 0x2C  */

#define R_USART6_CR1    reg32(0x40013C00)    /*!< USART Control register 1,                 Address offset: 0x00  */
#define R_USART6_CR2    reg32(0x40013C04)    /*!< USART Control register 2,                 Address offset: 0x04  */
#define R_USART6_CR3    reg32(0x40013C08)    /*!< USART Control register 3,                 Address offset: 0x08  */
#define R_USART6_BRR    reg32(0x40013C0C)    /*!< USART Baud rate register,                 Address offset: 0x0C  */
#define R_USART6_GTPR   reg32(0x40013C10)    /*!< USART Guard time and prescaler register,  Address offset: 0x10  */
#define R_USART6_RTOR   reg32(0x40013C14)    /*!< USART Receiver Time Out register,         Address offset: 0x14  */
#define R_USART6_RQR    reg32(0x40013C18)    /*!< USART Request register,                   Address offset: 0x18  */
#define R_USART6_ISR    reg32(0x40013C1C)    /*!< USART Interrupt and status register,      Address offset: 0x1C  */
#define R_USART6_ICR    reg32(0x40013C20)    /*!< USART Interrupt flag Clear register,      Address offset: 0x20  */
#define R_USART6_RDR    reg32(0x40013C24)    /*!< USART Receive Data register,              Address offset: 0x24  */
#define R_USART6_TDR    reg32(0x40013C28)    /*!< USART Transmit Data register,             Address offset: 0x28  */
#define R_USART6_PRESC  reg32(0x40013C2C)    /*!< USART Prescaler register,                 Address offset: 0x2C  */



/* Flash interface registers */
#define R_FLASH_ACR         reg32(0x40022000)    /*!< FLASH Access Control register,                     Address offset: 0x00 */
#define R_FLASH_KEYR        reg32(0x40022008)    /*!< FLASH Key register,                                Address offset: 0x08 */
#define R_FLASH_OPTKEYR     reg32(0x4002200C)    /*!< FLASH Option Key register,                         Address offset: 0x0C */
#define R_FLASH_SR          reg32(0x40022010)    /*!< FLASH Status register,                             Address offset: 0x10 */
#define R_FLASH_CR          reg32(0x40022014)    /*!< FLASH Control register,                            Address offset: 0x14 */
#define R_FLASH_ECCR        reg32(0x40022018)    /*!< FLASH ECC bank 1 register,                         Address offset: 0x18 */
#define R_FLASH_ECC2R       reg32(0x4002201C)    /*!< FLASH ECC bank 2 register,                         Address offset: 0x1C */
#define R_FLASH_OPTR        reg32(0x40022020)    /*!< FLASH Option register,                             Address offset: 0x20 */
#define R_FLASH_PCROP1ASR   reg32(0x40022024)    /*!< FLASH Bank PCROP area A Start address register,    Address offset: 0x24 */
#define R_FLASH_PCROP1AER   reg32(0x40022028)    /*!< FLASH Bank PCROP area A End address register,      Address offset: 0x28 */
#define R_FLASH_WRP1AR      reg32(0x4002202C)    /*!< FLASH Bank WRP area A address register,            Address offset: 0x2C */
#define R_FLASH_WRP1AR_STRT reg16(0x4002202C)    /*!< FLASH Bank WRP area A address register, start bits Address offset: 0x2C */
#define R_FLASH_WRP1BR      reg32(0x40022030)    /*!< FLASH Bank WRP area B address register,            Address offset: 0x30 */
#define R_FLASH_WRP1BR_STRT reg16(0x40022030)    /*!< FLASH Bank WRP area B address register,            Address offset: 0x30 */

#define R_FLASH_PCROP1BSR   reg32(0x40022034)    /*!< FLASH Bank PCROP area B Start address register,    Address offset: 0x34 */
#define R_FLASH_PCROP1BER   reg32(0x40022038)    /*!< FLASH Bank PCROP area B End address register,      Address offset: 0x38 */
#define R_FLASH_PCROP2ASR   reg32(0x40022044)    /*!< FLASH Bank2 PCROP area A Start address register,   Address offset: 0x44 */
#define R_FLASH_PCROP2AER   reg32(0x40022048)    /*!< FLASH Bank2 PCROP area A End address register,     Address offset: 0x48 */
#define R_FLASH_WRP2AR      reg32(0x4002204C)    /*!< FLASH Bank2 WRP area A address register,           Address offset: 0x4C */
#define R_FLASH_WRP2AR_STRT reg16(0x4002204C)    /*!< FLASH Bank2 WRP area A address register,           Address offset: 0x4C */
#define R_FLASH_WRP2BR      reg32(0x40022050)    /*!< FLASH Bank2 WRP area B address register,           Address offset: 0x50 */
#define R_FLASH_WRP2BR_STRT reg16(0x40022050)    /*!< FLASH Bank2 WRP area B address register,           Address offset: 0x50 */

#define R_FLASH_PCROP2BSR   reg32(0x40022054)    /*!< FLASH Bank2 PCROP area B Start address register,   Address offset: 0x54 */
#define R_FLASH_PCROP2BER   reg32(0x40022058)    /*!< FLASH Bank2 PCROP area B End address register,     Address offset: 0x58 */
#define R_FLASH_SECR        reg32(0x40022080)    /*!< FLASH security register ,                          Address offset: 0x80 */


/* TIM1 registers */
#define R_TIM1_CR1      reg16(0x40012C00)  /*!< TIM control register 1,                   Address offset: 0x00 */
#define R_TIM1_CR2      reg16(0x40012C04)  /*!< TIM control register 2,                   Address offset: 0x04 */
#define R_TIM1_SMCR     reg16(0x40012C08)  /*!< TIM slave mode control register,          Address offset: 0x08 */
#define R_TIM1_DIER     reg16(0x40012C0C)  /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
#define R_TIM1_SR       reg16(0x40012C10)  /*!< TIM status register,                      Address offset: 0x10 */
#define R_TIM1_EGR      reg16(0x40012C14)  /*!< TIM event generation register,            Address offset: 0x14 */
#define R_TIM1_CCMR1    reg16(0x40012C18)  /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
#define R_TIM1_CCMR2    reg16(0x40012C1C)  /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
#define R_TIM1_CCER     reg16(0x40012C20)  /*!< TIM capture/compare enable register,      Address offset: 0x20 */
#define R_TIM1_CNT      reg16(0x40012C24)  /*!< TIM counter register,                     Address offset: 0x24 */
#define R_TIM1_PSC      reg16(0x40012C28)  /*!< TIM prescaler register,                   Address offset: 0x28 */
#define R_TIM1_ARR      reg16(0x40012C2C)  /*!< TIM auto-reload register,                 Address offset: 0x2C */
#define R_TIM1_RCR      reg16(0x40012C30)  /*!< TIM repetition counter register,          Address offset: 0x30 */
#define R_TIM1_CCR1     reg16(0x40012C34)  /*!< TIM capture/compare register 1,           Address offset: 0x34 */
#define R_TIM1_CCR2     reg16(0x40012C38)  /*!< TIM capture/compare register 2,           Address offset: 0x38 */
#define R_TIM1_CCR3     reg16(0x40012C3C)  /*!< TIM capture/compare register 3,           Address offset: 0x3C */
#define R_TIM1_CCR4     reg16(0x40012C40)  /*!< TIM capture/compare register 4,           Address offset: 0x40 */
#define R_TIM1_BDTR     reg16(0x40012C44)  /*!< TIM break and dead-time register,         Address offset: 0x44 */
#define R_TIM1_DCR      reg16(0x40012C48)  /*!< TIM DMA control register,                 Address offset: 0x48 */
#define R_TIM1_DMAR     reg16(0x40012C4C)  /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
#define R_TIM1_OR1      reg16(0x40012C50)  /*!< TIM option register,                      Address offset: 0x50 */
#define R_TIM1_CCMR3    reg16(0x40012C54)  /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
#define R_TIM1_CCR5     reg16(0x40012C58)  /*!< TIM capture/compare register5,            Address offset: 0x58 */
#define R_TIM1_CCR6     reg16(0x40012C5C)  /*!< TIM capture/compare register6,            Address offset: 0x5C */
#define R_TIM1_AF1      reg16(0x40012C60)  /*!< TIM alternate function register 1,        Address offset: 0x60 */
#define R_TIM1_AF2      reg16(0x40012C64)  /*!< TIM alternate function register 2,        Address offset: 0x64 */
#define R_TIM1_TISEL    reg16(0x40012C68)  /*!< TIM Input Selection register,             Address offset: 0x68 */

#define R_TIM2_CR1      reg16(0x40000000)  /*!< TIM control register 1,                   Address offset: 0x00 */
#define R_TIM2_CR2      reg16(0x40000004)  /*!< TIM control register 2,                   Address offset: 0x04 */
#define R_TIM2_SMCR     reg16(0x40000008)  /*!< TIM slave mode control register,          Address offset: 0x08 */
#define R_TIM2_DIER     reg16(0x4000000C)  /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
#define R_TIM2_SR       reg16(0x40000010)  /*!< TIM status register,                      Address offset: 0x10 */
#define R_TIM2_EGR      reg16(0x40000014)  /*!< TIM event generation register,            Address offset: 0x14 */
#define R_TIM2_CCMR1    reg16(0x40000018)  /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
#define R_TIM2_CCMR2    reg16(0x4000001C)  /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
#define R_TIM2_CCER     reg16(0x40000020)  /*!< TIM capture/compare enable register,      Address offset: 0x20 */
#define R_TIM2_CNT      reg16(0x40000024)  /*!< TIM counter register,                     Address offset: 0x24 */
#define R_TIM2_PSC      reg16(0x40000028)  /*!< TIM prescaler register,                   Address offset: 0x28 */
#define R_TIM2_ARR      reg16(0x4000002C)  /*!< TIM auto-reload register,                 Address offset: 0x2C */
#define R_TIM2_RCR      reg16(0x40000030)  /*!< TIM repetition counter register,          Address offset: 0x30 */
#define R_TIM2_CCR1     reg16(0x40000034)  /*!< TIM capture/compare register 1,           Address offset: 0x34 */
#define R_TIM2_CCR2     reg16(0x40000038)  /*!< TIM capture/compare register 2,           Address offset: 0x38 */
#define R_TIM2_CCR3     reg16(0x4000003C)  /*!< TIM capture/compare register 3,           Address offset: 0x3C */
#define R_TIM2_CCR4     reg16(0x40000040)  /*!< TIM capture/compare register 4,           Address offset: 0x40 */
#define R_TIM2_BDTR     reg16(0x40000044)  /*!< TIM break and dead-time register,         Address offset: 0x44 */
#define R_TIM2_DCR      reg16(0x40000048)  /*!< TIM DMA control register,                 Address offset: 0x48 */
#define R_TIM2_DMAR     reg16(0x4000004C)  /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
#define R_TIM2_OR1      reg16(0x40000050)  /*!< TIM option register,                      Address offset: 0x50 */
#define R_TIM2_CCMR3    reg16(0x40000054)  /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
#define R_TIM2_CCR5     reg16(0x40000058)  /*!< TIM capture/compare register5,            Address offset: 0x58 */
#define R_TIM2_CCR6     reg16(0x4000005C)  /*!< TIM capture/compare register6,            Address offset: 0x5C */
#define R_TIM2_AF1      reg16(0x40000060)  /*!< TIM alternate function register 1,        Address offset: 0x60 */
#define R_TIM2_AF2      reg16(0x40000064)  /*!< TIM alternate function register 2,        Address offset: 0x64 */
#define R_TIM2_TISEL    reg16(0x40000068)  /*!< TIM Input Selection register,             Address offset: 0x68 */

#define R_TIM3_CR1      reg16(0x40000400)  /*!< TIM control register 1,                   Address offset: 0x00 */
#define R_TIM3_CR2      reg16(0x40000404)  /*!< TIM control register 2,                   Address offset: 0x04 */
#define R_TIM3_SMCR     reg16(0x40000408)  /*!< TIM slave mode control register,          Address offset: 0x08 */
#define R_TIM3_DIER     reg16(0x4000040C)  /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
#define R_TIM3_SR       reg16(0x40000410)  /*!< TIM status register,                      Address offset: 0x10 */
#define R_TIM3_EGR      reg16(0x40000414)  /*!< TIM event generation register,            Address offset: 0x14 */
#define R_TIM3_CCMR1    reg16(0x40000418)  /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
#define R_TIM3_CCMR2    reg16(0x4000041C)  /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
#define R_TIM3_CCER     reg16(0x40000420)  /*!< TIM capture/compare enable register,      Address offset: 0x20 */
#define R_TIM3_CNT      reg16(0x40000424)  /*!< TIM counter register,                     Address offset: 0x24 */
#define R_TIM3_PSC      reg16(0x40000428)  /*!< TIM prescaler register,                   Address offset: 0x28 */
#define R_TIM3_ARR      reg16(0x4000042C)  /*!< TIM auto-reload register,                 Address offset: 0x2C */
#define R_TIM3_RCR      reg16(0x40000430)  /*!< TIM repetition counter register,          Address offset: 0x30 */
#define R_TIM3_CCR1     reg16(0x40000434)  /*!< TIM capture/compare register 1,           Address offset: 0x34 */
#define R_TIM3_CCR2     reg16(0x40000438)  /*!< TIM capture/compare register 2,           Address offset: 0x38 */
#define R_TIM3_CCR3     reg16(0x4000043C)  /*!< TIM capture/compare register 3,           Address offset: 0x3C */
#define R_TIM3_CCR4     reg16(0x40000440)  /*!< TIM capture/compare register 4,           Address offset: 0x40 */
#define R_TIM3_BDTR     reg16(0x40000444)  /*!< TIM break and dead-time register,         Address offset: 0x44 */
#define R_TIM3_DCR      reg16(0x40000448)  /*!< TIM DMA control register,                 Address offset: 0x48 */
#define R_TIM3_DMAR     reg16(0x4000044C)  /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
#define R_TIM3_OR1      reg16(0x40000450)  /*!< TIM option register,                      Address offset: 0x50 */
#define R_TIM3_CCMR3    reg16(0x40000454)  /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
#define R_TIM3_CCR5     reg16(0x40000458)  /*!< TIM capture/compare register5,            Address offset: 0x58 */
#define R_TIM3_CCR6     reg16(0x4000045C)  /*!< TIM capture/compare register6,            Address offset: 0x5C */
#define R_TIM3_AF1      reg16(0x40000460)  /*!< TIM alternate function register 1,        Address offset: 0x60 */
#define R_TIM3_AF2      reg16(0x40000464)  /*!< TIM alternate function register 2,        Address offset: 0x64 */
#define R_TIM3_TISEL    reg16(0x40000468)  /*!< TIM Input Selection register,             Address offset: 0x68 */

#define R_TIM4_CR1      reg16(0x40000800)  /*!< TIM control register 1,                   Address offset: 0x00 */
#define R_TIM4_CR2      reg16(0x40000804)  /*!< TIM control register 2,                   Address offset: 0x04 */
#define R_TIM4_SMCR     reg16(0x40000808)  /*!< TIM slave mode control register,          Address offset: 0x08 */
#define R_TIM4_DIER     reg16(0x4000080C)  /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
#define R_TIM4_SR       reg16(0x40000810)  /*!< TIM status register,                      Address offset: 0x10 */
#define R_TIM4_EGR      reg16(0x40000814)  /*!< TIM event generation register,            Address offset: 0x14 */
#define R_TIM4_CCMR1    reg16(0x40000818)  /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
#define R_TIM4_CCMR2    reg16(0x4000081C)  /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
#define R_TIM4_CCER     reg16(0x40000820)  /*!< TIM capture/compare enable register,      Address offset: 0x20 */
#define R_TIM4_CNT      reg16(0x40000824)  /*!< TIM counter register,                     Address offset: 0x24 */
#define R_TIM4_PSC      reg16(0x40000828)  /*!< TIM prescaler register,                   Address offset: 0x28 */
#define R_TIM4_ARR      reg16(0x4000082C)  /*!< TIM auto-reload register,                 Address offset: 0x2C */
#define R_TIM4_RCR      reg16(0x40000830)  /*!< TIM repetition counter register,          Address offset: 0x30 */
#define R_TIM4_CCR1     reg16(0x40000834)  /*!< TIM capture/compare register 1,           Address offset: 0x34 */
#define R_TIM4_CCR2     reg16(0x40000838)  /*!< TIM capture/compare register 2,           Address offset: 0x38 */
#define R_TIM4_CCR3     reg16(0x4000083C)  /*!< TIM capture/compare register 3,           Address offset: 0x3C */
#define R_TIM4_CCR4     reg16(0x40000840)  /*!< TIM capture/compare register 4,           Address offset: 0x40 */
#define R_TIM4_BDTR     reg16(0x40000844)  /*!< TIM break and dead-time register,         Address offset: 0x44 */
#define R_TIM4_DCR      reg16(0x40000848)  /*!< TIM DMA control register,                 Address offset: 0x48 */
#define R_TIM4_DMAR     reg16(0x4000084C)  /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
#define R_TIM4_OR1      reg16(0x40000850)  /*!< TIM option register,                      Address offset: 0x50 */
#define R_TIM4_CCMR3    reg16(0x40000854)  /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
#define R_TIM4_CCR5     reg16(0x40000858)  /*!< TIM capture/compare register5,            Address offset: 0x58 */
#define R_TIM4_CCR6     reg16(0x4000085C)  /*!< TIM capture/compare register6,            Address offset: 0x5C */
#define R_TIM4_AF1      reg16(0x40000860)  /*!< TIM alternate function register 1,        Address offset: 0x60 */
#define R_TIM4_AF2      reg16(0x40000864)  /*!< TIM alternate function register 2,        Address offset: 0x64 */
#define R_TIM4_TISEL    reg16(0x40000868)  /*!< TIM Input Selection register,             Address offset: 0x68 */


/* Generic offset for TIM registers */
#define R_TIMx_CR1(base)   reg16((((uint32_t)base) + 0x00)
#define R_TIMx_CR2(base)   reg16((((uint32_t)base) + 0x04)
#define R_TIMx_SMCR(base)  reg16((((uint32_t)base) + 0x08)
#define R_TIMx_DIER(base)  reg16((((uint32_t)base) + 0x0c)
#define R_TIMx_SR(base)    reg16((((uint32_t)base) + 0x10)
#define R_TIMx_EGR(base)   reg16((((uint32_t)base) + 0x14)
#define R_TIMx_CCMR1(base) reg16(((uint32_t)base) + 0x18)
#define R_TIMx_CCMR2(base) reg16(((uint32_t)base) + 0x1c)
#define R_TIMx_CCER(base)  reg16(((uint32_t)base) + 0x20)
#define R_TIMx_CNT(base)   reg16(((uint32_t)base) + 0x24)
#define R_TIMx_PSC(base)   reg16(((uint32_t)base) + 0x28)
#define R_TIMx_ARR(base)   reg16(((uint32_t)base) + 0x2c)
#define R_TIMx_RCR(base)   reg16(((uint32_t)base) + 0x30)
#define R_TIMx_CCR1(base)  reg16(((uint32_t)base) + 0x34)
#define R_TIMx_CCR2(base)  reg16(((uint32_t)base) + 0x38)
#define R_TIMx_CCR3(base)  reg16(((uint32_t)base) + 0x3c)
#define R_TIMx_CCR4(base)  reg16(((uint32_t)base) + 0x40)
#define R_TIMx_BDTR(base)  reg16(((uint32_t)base) + 0x44)
#define R_TIMx_DCR(base)   reg16(((uint32_t)base) + 0x48)
#define R_TIMx_DMAR(base)  reg16(((uint32_t)base) + 0x4c)
#define R_TIMx_OR1(base)   reg16(((uint32_t)base) + 0x50)
#define R_TIMx_CCMR3(base) reg16(((uint32_t)base) + 0x54)
#define R_TIMx_CCR5(base)  reg16(((uint32_t)base) + 0x58)
#define R_TIMx_CCR6(base)  reg16(((uint32_t)base) + 0x5C)
#define R_TIMx_AF1(base)   reg16(((uint32_t)base) + 0x60)
#define R_TIMx_AF2(base)   reg16(((uint32_t)base) + 0x64)
#define R_TIMx_TISEL(base) reg16(((uint32_t)base) + 0x68)


/* I2C registers */
#define R_I2C1_CR1        reg16(0x40005400)  /*!< I2C Control register 1,            Address offset: 0x00 */
#define R_I2C1_CR2        reg16(0x40005404)  /*!< I2C Control register 2,            Address offset: 0x04 */
#define R_I2C1_OAR1       reg16(0x40005408)  /*!< I2C Own address 1 register,        Address offset: 0x08 */
#define R_I2C1_OAR2       reg16(0x4000540C)  /*!< I2C Own address 2 register,        Address offset: 0x0C */
#define R_I2C1_TIMINGR    reg16(0x40005410)  /*!< I2C Timing register,               Address offset: 0x10 */
#define R_I2C1_TIMEOUTR   reg16(0x40005414)  /*!< I2C Timeout register,              Address offset: 0x14 */
#define R_I2C1_ISR        reg16(0x40005418)  /*!< I2C Interrupt and status register, Address offset: 0x18 */
#define R_I2C1_ICR        reg16(0x4000541C)  /*!< I2C Interrupt clear register,      Address offset: 0x1C */
#define R_I2C1_PECR       reg16(0x40005420)  /*!< I2C PEC register,                  Address offset: 0x20 */
#define R_I2C1_RXDR       reg16(0x40005424)  /*!< I2C Receive data register,         Address offset: 0x24 */
#define R_I2C1_TXDR       reg16(0x40005428)  /*!< I2C Transmit data register,        Address offset: 0x28 */

#define R_I2C2_CR1        reg16(0x40005800)  /*!< I2C Control register 1,            Address offset: 0x00 */
#define R_I2C2_CR2        reg16(0x40005804)  /*!< I2C Control register 2,            Address offset: 0x04 */
#define R_I2C2_OAR1       reg16(0x40005808)  /*!< I2C Own address 1 register,        Address offset: 0x08 */
#define R_I2C2_OAR2       reg16(0x4000580C)  /*!< I2C Own address 2 register,        Address offset: 0x0C */
#define R_I2C2_TIMINGR    reg16(0x40005810)  /*!< I2C Timing register,               Address offset: 0x10 */
#define R_I2C2_TIMEOUTR   reg16(0x40005814)  /*!< I2C Timeout register,              Address offset: 0x14 */
#define R_I2C2_ISR        reg16(0x40005818)  /*!< I2C Interrupt and status register, Address offset: 0x18 */
#define R_I2C2_ICR        reg16(0x4000581C)  /*!< I2C Interrupt clear register,      Address offset: 0x1C */
#define R_I2C2_PECR       reg16(0x40005820)  /*!< I2C PEC register,                  Address offset: 0x20 */
#define R_I2C2_RXDR       reg16(0x40005824)  /*!< I2C Receive data register,         Address offset: 0x24 */
#define R_I2C2_TXDR       reg16(0x40005828)  /*!< I2C Transmit data register,        Address offset: 0x28 */

#define R_I2C3_CR1        reg16(0x40008800)  /*!< I2C Control register 1,            Address offset: 0x00 */
#define R_I2C3_CR2        reg16(0x40008804)  /*!< I2C Control register 2,            Address offset: 0x04 */
#define R_I2C3_OAR1       reg16(0x40008808)  /*!< I2C Own address 1 register,        Address offset: 0x08 */
#define R_I2C3_OAR2       reg16(0x4000880C)  /*!< I2C Own address 2 register,        Address offset: 0x0C */
#define R_I2C3_TIMINGR    reg16(0x40008810)  /*!< I2C Timing register,               Address offset: 0x10 */
#define R_I2C3_TIMEOUTR   reg16(0x40008814)  /*!< I2C Timeout register,              Address offset: 0x14 */
#define R_I2C3_ISR        reg16(0x40008818)  /*!< I2C Interrupt and status register, Address offset: 0x18 */
#define R_I2C3_ICR        reg16(0x4000881C)  /*!< I2C Interrupt clear register,      Address offset: 0x1C */
#define R_I2C3_PECR       reg16(0x40008820)  /*!< I2C PEC register,                  Address offset: 0x20 */
#define R_I2C3_RXDR       reg16(0x40008824)  /*!< I2C Receive data register,         Address offset: 0x24 */
#define R_I2C3_TXDR       reg16(0x40008828)  /*!< I2C Transmit data register,        Address offset: 0x28 */
  
#define R_I2C_CR1(x)    reg16(0x40005400 + x * 0x400)
#define R_I2C_CR2(x)    reg16(0x40005404 + x * 0x400)
#define R_I2C_OAR1(x)   reg16(0x40005408 + x * 0x400)
#define R_I2C_OAR2(x)   reg16(0x4000540c + x * 0x400)
#define R_I2C_DR(x)     reg16(0x40005410 + x * 0x400)
#define R_I2C_SR1(x)    reg16(0x40005414 + x * 0x400)
#define R_I2C_SR2(x)    reg16(0x40005418 + x * 0x400)
#define R_I2C_CCR(x)    reg16(0x4000541c + x * 0x400)
#define R_I2C_TRISE(x)  reg16(0x40005420 + x * 0x400)
#define R_I2C_FLTR(x)   reg16(0x40005424 + x * 0x400)

/* SPI1 registers */
#define RSPI1_CR1       reg16(0x40013000) /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
#define RSPI1_CR2       reg16(0x40013004) /*!< SPI control register 2,                             Address offset: 0x04 */
#define RSPI1_SR        reg16(0x40013008) /*!< SPI status register,                                Address offset: 0x08 */
#define RSPI1_DR        reg16(0x4001300C) /*!< SPI data register,                                  Address offset: 0x0C */
#define RSPI1_DR_8      0x4001300CUL      /*!< SPI data register,    (8 bit access)                Address offset: 0x0C */
#define RSPI1_CRCPR     reg16(0x40013010) /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
#define RSPI1_RXCRCR    reg16(0x40013014) /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
#define RSPI1_TXCRCR    reg16(0x40013018) /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
#define RSPI1_I2SCFGR   reg16(0x4001301C) /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
#define RSPI1_I2SPR     reg16(0x40013020) /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */

#define RSPI2_CR1       reg16(0x40003800) /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
#define RSPI2_CR2       reg16(0x40003804) /*!< SPI control register 2,                             Address offset: 0x04 */
#define RSPI2_SR        reg16(0x40003808) /*!< SPI status register,                                Address offset: 0x08 */
#define RSPI2_DR        reg16(0x4000380C) /*!< SPI data register,                                  Address offset: 0x0C */
#define RSPI2_DR_8      0x4000380CUL      /*!< SPI data register, (8 bit access)                   Address offset: 0x0C */
#define RSPI2_CRCPR     reg16(0x40003810) /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
#define RSPI2_RXCRCR    reg16(0x40003814) /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
#define RSPI2_TXCRCR    reg16(0x40003818) /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
#define RSPI2_I2SCFGR   reg16(0x4000381C) /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
#define RSPI2_I2SPR     reg16(0x40003820) /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */

#define RSPI3_CR1       reg16(0x40003c00) /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
#define RSPI3_CR2       reg16(0x40003c04) /*!< SPI control register 2,                             Address offset: 0x04 */
#define RSPI3_SR        reg16(0x40003c08) /*!< SPI status register,                                Address offset: 0x08 */
#define RSPI3_DR        reg16(0x40003c0C) /*!< SPI data register,                                  Address offset: 0x0C */
#define RSPI3_DR_8      0x40003c0CUL      /*!< SPI data register, (8 bit access)                   Address offset: 0x0C */
#define RSPI3_CRCPR     reg16(0x40003c10) /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
#define RSPI3_RXCRCR    reg16(0x40003c14) /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
#define RSPI3_TXCRCR    reg16(0x40003c18) /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
#define RSPI3_I2SCFGR   reg16(0x40003c1C) /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
#define RSPI3_I2SPR     reg16(0x40003c20) /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */


#endif /* _STM32G0B1X_ */
