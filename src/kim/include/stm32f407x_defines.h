/*
 * Author: Aurelio Colosimo, 2017
 *
 * This file is part of kim-os project: https://github.com/colosimo/kim-os
 * According to kim-os license, you can do whatever you want with it,
 * as long as you retain this notice.
 */

#ifndef _STM32F407X_
#define _STM32F407X_

#include <cpu.h>

/* RCC registers */
#define RCC_CR       reg32(0x40023800)
#define RCC_PLLCFGR  reg32(0x40023804)
#define RCC_CFGR     reg32(0x40023808)
#define RCC_AHB1ENR  reg32(0x40023830)
#define RCC_AHB2ENR  reg32(0x40023834)
#define RCC_APB1ENR  reg32(0x40023840)
#define RCC_APB2ENR  reg32(0x40023844)

/* GPIO registers */
#define R_GPIOA_MODER   reg32(0x40020000)
#define R_GPIOA_OTYPER  reg32(0x40020004)
#define R_GPIOA_OSPEEDR reg32(0x40020008)
#define R_GPIOA_PUPDR   reg32(0x4002000c)
#define R_GPIOA_IDR     reg32(0x40020010)
#define R_GPIOA_ODR     reg32(0x40020014)
#define R_GPIOA_BSRR    reg32(0x40020018)
#define R_GPIOA_LCKR    reg32(0x4002001c)
#define R_GPIOA_AFRL    reg32(0x40020020)
#define R_GPIOA_AFRH    reg32(0x40020024)

#define R_GPIOB_MODER   reg32(0x40020400)
#define R_GPIOB_OTYPER  reg32(0x40020404)
#define R_GPIOB_OSPEEDR reg32(0x40020408)
#define R_GPIOB_PUPDR   reg32(0x4002040c)
#define R_GPIOB_IDR     reg32(0x40020410)
#define R_GPIOB_ODR     reg32(0x40020414)
#define R_GPIOB_BSRR    reg32(0x40020418)
#define R_GPIOB_LCKR    reg32(0x4002041c)
#define R_GPIOB_AFRL    reg32(0x40020420)
#define R_GPIOB_AFRH    reg32(0x40020424)

/* USART registers */
#define R_USART1_SR     reg32(0x40011000)
#define R_USART1_DR     reg32(0x40011004)
#define R_USART1_BRR    reg32(0x40011008)
#define R_USART1_CR1    reg32(0x4001100c)
#define R_USART1_CR2    reg32(0x40011010)
#define R_USART1_CR3    reg32(0x40011014)

#define R_USART2_SR     reg32(0x40004400)
#define R_USART2_DR     reg32(0x40004404)
#define R_USART2_BRR    reg32(0x40004408)
#define R_USART2_CR1    reg32(0x4000440c)
#define R_USART2_CR2    reg32(0x40004410)
#define R_USART2_CR3    reg32(0x40004414)

#define R_USART3_SR     reg32(0x40004800)
#define R_USART3_DR     reg32(0x40004804)
#define R_USART3_BRR    reg32(0x40004808)
#define R_USART3_CR1    reg32(0x4000480c)
#define R_USART3_CR2    reg32(0x40004810)
#define R_USART3_CR3    reg32(0x40004814)

#define R_USART6_SR     reg32(0x40011400)
#define R_USART6_DR     reg32(0x40011404)
#define R_USART6_BRR    reg32(0x40011408)
#define R_USART6_CR1    reg32(0x4001140c)
#define R_USART6_CR2    reg32(0x40011410)
#define R_USART6_CR3    reg32(0x40011414)

/* Flash interface registers */
#define R_FLASH_ACR     reg32(0x40023c00)
#define R_FLASH_KEYR    reg32(0x40023c04)
#define R_FLASH_OPTKEYR reg32(0x40023c08)
#define R_FLASH_SR      reg32(0x40023c0c)
#define R_FLASH_CR      reg32(0x40023c10)
#define R_FLASH_OPTCR   reg32(0x40023c14)

/* TIM1 registers */
#define R_TIM1_CR1      reg16(0x40010000)
#define R_TIM1_CR2      reg16(0x40010004)
#define R_TIM1_SMCR     reg16(0x40010008)
#define R_TIM1_DIER     reg16(0x4001000c)
#define R_TIM1_SR       reg16(0x40010010)
#define R_TIM1_EGR      reg16(0x40010014)
#define R_TIM1_CCMR1    reg16(0x40010018)
#define R_TIM1_CCMR2    reg16(0x4001001c)
#define R_TIM1_CCER     reg16(0x40010020)
#define R_TIM1_CNT      reg16(0x40010024)
#define R_TIM1_PSC      reg16(0x40010028)
#define R_TIM1_ARR      reg16(0x4001002c)
#define R_TIM1_RCR      reg16(0x40010030)
#define R_TIM1_CCR1     reg16(0x40010034)
#define R_TIM1_CCR2     reg16(0x40010038)
#define R_TIM1_CCR3     reg16(0x4001003c)
#define R_TIM1_CCR4     reg16(0x40010040)
#define R_TIM1_BDTR     reg16(0x40010044)
#define R_TIM1_DCR      reg16(0x40010048)
#define R_TIM1_DMAR     reg16(0x4001004c)

/* TIM3 registers */
#define R_TIM3_CR1      reg16(0x40000400)
#define R_TIM3_CR2      reg16(0x40000404)
#define R_TIM3_SMCR     reg16(0x40000408)
#define R_TIM3_DIER     reg16(0x4000040c)
#define R_TIM3_SR       reg16(0x40000410)
#define R_TIM3_EGR      reg16(0x40000414)
#define R_TIM3_CCMR1    reg16(0x40000418)
#define R_TIM3_CCMR2    reg16(0x4000041c)
#define R_TIM3_CCER     reg16(0x40000420)
#define R_TIM3_CNT      reg16(0x40000424)
#define R_TIM3_PSC      reg16(0x40000428)
#define R_TIM3_ARR      reg16(0x4000042c)
#define R_TIM3_CCR1     reg16(0x40000434)
#define R_TIM3_CCR2     reg16(0x40000438)
#define R_TIM3_CCR3     reg16(0x4000043c)
#define R_TIM3_CCR4     reg16(0x40000440)
#define R_TIM3_DCR      reg16(0x40000448)
#define R_TIM3_DMAR     reg16(0x4000044c)

/* TIM4 registers */
#define R_TIM4_CR1      reg16(0x40000800)
#define R_TIM4_CR2      reg16(0x40000804)
#define R_TIM4_SMCR     reg16(0x40000808)
#define R_TIM4_DIER     reg16(0x4000080c)
#define R_TIM4_SR       reg16(0x40000810)
#define R_TIM4_EGR      reg16(0x40000814)
#define R_TIM4_CCMR1    reg16(0x40000818)
#define R_TIM4_CCMR2    reg16(0x4000081c)
#define R_TIM4_CCER     reg16(0x40000820)
#define R_TIM4_CNT      reg16(0x40000824)
#define R_TIM4_PSC      reg16(0x40000828)
#define R_TIM4_ARR      reg16(0x4000082c)
#define R_TIM4_CCR1     reg16(0x40000834)
#define R_TIM4_CCR2     reg16(0x40000838)
#define R_TIM4_CCR3     reg16(0x4000083c)
#define R_TIM4_CCR4     reg16(0x40000840)
#define R_TIM4_DCR      reg16(0x40000848)
#define R_TIM4_DMAR     reg16(0x4000084c)

/* TIM11 registers */
#define R_TIM11_CR1      reg16(0x40014800)
#define R_TIM11_SMCR     reg16(0x40014808)
#define R_TIM11_DIER     reg16(0x4001480c)
#define R_TIM11_SR       reg16(0x40014810)
#define R_TIM11_EGR      reg16(0x40014814)
#define R_TIM11_CCMR1    reg16(0x40014818)
#define R_TIM11_CCER     reg16(0x40014820)
#define R_TIM11_CNT      reg16(0x40014824)
#define R_TIM11_PSC      reg16(0x40014828)
#define R_TIM11_ARR      reg16(0x4001482c)
#define R_TIM11_CCR1     reg16(0x40014834)
#define R_TIM11_OR       reg16(0x40014850)

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

/* I2C registers */
#define R_I2C1_CR1      reg16(0x40005400)
#define R_I2C1_CR2      reg16(0x40005404)
#define R_I2C1_OAR1     reg16(0x40005408)
#define R_I2C1_OAR2     reg16(0x4000540c)
#define R_I2C1_DR       reg16(0x40005410)
#define R_I2C1_SR1      reg16(0x40005414)
#define R_I2C1_SR2      reg16(0x40005418)
#define R_I2C1_CCR      reg16(0x4000541c)
#define R_I2C1_TRISE    reg16(0x40005420)
#define R_I2C1_FLTR     reg16(0x40005424)
#define R_I2C2_CR1      reg16(0x40005800)
#define R_I2C2_CR2      reg16(0x40005804)
#define R_I2C2_OAR1     reg16(0x40005808)
#define R_I2C2_OAR2     reg16(0x4000580c)
#define R_I2C2_DR       reg16(0x40005810)
#define R_I2C2_SR1      reg16(0x40005814)
#define R_I2C2_SR2      reg16(0x40005818)
#define R_I2C2_CCR      reg16(0x4000581c)
#define R_I2C2_TRISE    reg16(0x40005820)
#define R_I2C2_FLTR     reg16(0x40005824)
#define R_I2C3_CR1      reg16(0x40005c00)
#define R_I2C3_CR2      reg16(0x40005c04)
#define R_I2C3_OAR1     reg16(0x40005c08)
#define R_I2C3_OAR2     reg16(0x40005c0c)
#define R_I2C3_DR       reg16(0x40005c10)
#define R_I2C3_SR1      reg16(0x40005c14)
#define R_I2C3_SR2      reg16(0x40005c18)
#define R_I2C3_CCR      reg16(0x40005c1c)
#define R_I2C3_TRISE    reg16(0x40005c20)
#define R_I2C3_FLTR     reg16(0x40005c24)
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

/* SDIO registers */
#define R_SDIO_POWER    reg32(0x40012c00)
#define R_SDIO_CLKCR    reg32(0x40012c04)
#define R_SDIO_ARG      reg32(0x40012c08)
#define R_SDIO_CMD      reg32(0x40012c0c)
#define R_SDIO_RESPCMD  reg32(0x40012c10)
#define R_SDIO_RESP1    reg32(0x40012c14)
#define R_SDIO_RESP2    reg32(0x40012c18)
#define R_SDIO_RESP3    reg32(0x40012c1c)
#define R_SDIO_RESP4    reg32(0x40012c20)
#define R_SDIO_DTIMER   reg32(0x40012c24)
#define R_SDIO_DLEN     reg32(0x40012c28)
#define R_SDIO_DCTRL    reg32(0x40012c2c)
#define R_SDIO_DCOUNT   reg32(0x40012c30)
#define R_SDIO_STA      reg32(0x40012c34)
#define R_SDIO_ICR      reg32(0x40012c38)
#define R_SDIO_MASK     reg32(0x40012c3c)
#define R_SDIO_FIFOCNT  reg32(0x40012c48)
#define R_SDIO_FIFO     reg32(0x40012c80)

/* SPI1 registers */
#define SPI1_BASE_ADDR  0x40013000
#define RSPI1_CR1       reg16(SPI1_BASE_ADDR + 0x00) /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
#define RSPI1_CR2       reg16(SPI1_BASE_ADDR + 0x04) /*!< SPI control register 2,                             Address offset: 0x04 */
#define RSPI1_SR        reg16(SPI1_BASE_ADDR + 0x08) /*!< SPI status register,                                Address offset: 0x08 */
#define RSPI1_DR        reg16(SPI1_BASE_ADDR + 0x0C) /*!< SPI data register,                                  Address offset: 0x0C */
#define RSPI1_CRCPR     reg16(SPI1_BASE_ADDR + 0x10) /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
#define RSPI1_RXCRCR    reg16(SPI1_BASE_ADDR + 0x14) /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
#define RSPI1_TXCRCR    reg16(SPI1_BASE_ADDR + 0x18) /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
#define RSPI1_I2SCFGR   reg16(SPI1_BASE_ADDR + 0x1C) /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
#define RSPI1_I2SPR     reg16(SPI1_BASE_ADDR + 0x20) /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */

/* ADC registers */
#define R_ADC_SR        reg32(0x40012000)
#define R_ADC_CR1       reg32(0x40012004)
#define R_ADC_CR2       reg32(0x40012008)
#define R_ADC_SMPR1     reg32(0x4001200c)
#define R_ADC_SMPR2     reg32(0x40012010)
#define R_ADC_JOFR1     reg32(0x40012014)
#define R_ADC_JOFR2     reg32(0x40012018)
#define R_ADC_JOFR3     reg32(0x4001201c)
#define R_ADC_JOFR4     reg32(0x40012020)
#define R_ADC_HTR       reg32(0x40012024)
#define R_ADC_LTR       reg32(0x40012028)
#define R_ADC_SQR1      reg32(0x4001202c)
#define R_ADC_SQR2      reg32(0x40012030)
#define R_ADC_SQR3      reg32(0x40012034)
#define R_ADC_JSQR      reg32(0x40012038)
#define R_ADC_JDR1      reg32(0x4001203c)
#define R_ADC_JDR2      reg32(0x40012040)
#define R_ADC_JDR3      reg32(0x40012044)
#define R_ADC_JDR4      reg32(0x40012048)
#define R_ADC_DR        reg32(0x4001204c)
#define R_ADC_CCR       reg32(0x40012304)

#endif /* _STM32F411X_ */
