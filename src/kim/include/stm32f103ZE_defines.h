/*
 * Author: Aurelio Colosimo, 2017
 *
 * This file is part of kim-os project: https://github.com/colosimo/kim-os
 * According to kim-os license, you can do whatever you want with it,
 * as long as you retain this notice.
 */

#ifndef _STM32F103ZE_
#define _STM32F103ZE_

#include <cpu.h>
#include "stm32f103xe.h"

/* RCC registers */
#define RCC_CR          reg32(0x40021000UL)          //     reg32(RCC_BASE + 0x00)
#define RCC_CFGR        reg32(0x40021004UL)          //     reg32(RCC_BASE + 0x04)
#define RCC_AHBENR      reg32(0x40021014UL)          //     reg32(RCC_BASE + 0x14)
#define RCC_APB2ENR     reg32(0x40021018UL)          //     reg32(RCC_BASE + 0x18)
#define RCC_APB1ENR     reg32(0x4002101cUL)          //     reg32(RCC_BASE + 0x1c)

/* AFIO_MAPR registers */
#define R_AFIO_MAPR     reg32(0x40010004UL)
#define R_AFIO_MAPR2    reg32(0x4001001cUL)

/* GPIO registers */
#define R_GPIOA_CRL     reg32(0x40010800UL)          //     reg32(GPIOA_BASE + 0x00)
#define R_GPIOA_CRH     reg32(0x40010804UL)          //     reg32(GPIOA_BASE + 0x04)
#define R_GPIOA_IDR     reg32(0x40010808UL)          //     reg32(GPIOA_BASE + 0x08)
#define R_GPIOA_ODR     reg32(0x4001080cUL)          //     reg32(GPIOA_BASE + 0x0c)
#define R_GPIOA_BSRR    reg32(0x40010810UL)          //     reg32(GPIOA_BASE + 0x10)
#define R_GPIOA_BRR     reg32(0x40010814UL)          //     reg32(GPIOA_BASE + 0x14)
#define R_GPIOA_LCKR    reg32(0x40010818UL)          //     reg32(GPIOA_BASE + 0x18)

#define R_GPIOB_CRL     reg32(0x40010C00UL)          //     reg32(GPIOB_BASE + 0x00)
#define R_GPIOB_CRH     reg32(0x40010C04UL)          //     reg32(GPIOB_BASE + 0x04)
#define R_GPIOB_IDR     reg32(0x40010C08UL)          //     reg32(GPIOB_BASE + 0x08)
#define R_GPIOB_ODR     reg32(0x40010C0cUL)          //     reg32(GPIOB_BASE + 0x0c)
#define R_GPIOB_BSRR    reg32(0x40010C10UL)          //     reg32(GPIOB_BASE + 0x10)
#define R_GPIOB_BRR     reg32(0x40010C14UL)          //     reg32(GPIOB_BASE + 0x14)
#define R_GPIOB_LCKR    reg32(0x40010C18UL)          //     reg32(GPIOB_BASE + 0x18)

#define R_GPIOC_CRL     reg32(0x40011000UL)          //     reg32(GPIOC_BASE + 0x00)
#define R_GPIOC_CRH     reg32(0x40011004UL)          //     reg32(GPIOC_BASE + 0x04)
#define R_GPIOC_IDR     reg32(0x40011008UL)          //     reg32(GPIOC_BASE + 0x08)
#define R_GPIOC_ODR     reg32(0x4001100cUL)          //     reg32(GPIOC_BASE + 0x0c)
#define R_GPIOC_BSRR    reg32(0x40011010UL)          //     reg32(GPIOC_BASE + 0x10)
#define R_GPIOC_BRR     reg32(0x40011014UL)          //     reg32(GPIOC_BASE + 0x14)
#define R_GPIOC_LCKR    reg32(0x40011018UL)          //     reg32(GPIOC_BASE + 0x18)

#define R_GPIOD_CRL     reg32(0x40011400UL)          //     reg32(GPIOD_BASE + 0x00)
#define R_GPIOD_CRH     reg32(0x40011404UL)          //     reg32(GPIOD_BASE + 0x04)
#define R_GPIOD_IDR     reg32(0x40011408UL)          //     reg32(GPIOD_BASE + 0x08)
#define R_GPIOD_ODR     reg32(0x4001140cUL)          //     reg32(GPIOD_BASE + 0x0c)
#define R_GPIOD_BSRR    reg32(0x40011410UL)          //     reg32(GPIOD_BASE + 0x10)
#define R_GPIOD_BRR     reg32(0x40011414UL)          //     reg32(GPIOD_BASE + 0x14)
#define R_GPIOD_LCKR    reg32(0x40011418UL)          //     reg32(GPIOD_BASE + 0x18)

#define R_GPIOE_CRL     reg32(0x40011800UL)          //     reg32(GPIOE_BASE + 0x00)
#define R_GPIOE_CRH     reg32(0x40011804UL)          //     reg32(GPIOE_BASE + 0x04)
#define R_GPIOE_IDR     reg32(0x40011808UL)          //     reg32(GPIOE_BASE + 0x08)
#define R_GPIOE_ODR     reg32(0x4001180cUL)          //     reg32(GPIOE_BASE + 0x0c)
#define R_GPIOE_BSRR    reg32(0x40011810UL)          //     reg32(GPIOE_BASE + 0x10)
#define R_GPIOE_BRR     reg32(0x40011814UL)          //     reg32(GPIOE_BASE + 0x14)
#define R_GPIOE_LCKR    reg32(0x40011818UL)          //     reg32(GPIOE_BASE + 0x18)

#define R_GPIOF_CRL     reg32(0x40011C00UL)          //     reg32(GPIOF_BASE + 0x00)
#define R_GPIOF_CRH     reg32(0x40011C04UL)          //     reg32(GPIOF_BASE + 0x04)
#define R_GPIOF_IDR     reg32(0x40011C08UL)          //     reg32(GPIOF_BASE + 0x08)
#define R_GPIOF_ODR     reg32(0x40011C0cUL)          //     reg32(GPIOF_BASE + 0x0c)
#define R_GPIOF_BSRR    reg32(0x40011C10UL)          //     reg32(GPIOF_BASE + 0x10)
#define R_GPIOF_BRR     reg32(0x40011C14UL)          //     reg32(GPIOF_BASE + 0x14)
#define R_GPIOF_LCKR    reg32(0x40011C18UL)          //     reg32(GPIOF_BASE + 0x18)

#define R_GPIOG_CRL     reg32(0x40012000UL)          //     reg32(GPIOG_BASE + 0x00)
#define R_GPIOG_CRH     reg32(0x40012004UL)          //     reg32(GPIOG_BASE + 0x04)
#define R_GPIOG_IDR     reg32(0x40012008UL)          //     reg32(GPIOG_BASE + 0x08)
#define R_GPIOG_ODR     reg32(0x4001200cUL)          //     reg32(GPIOG_BASE + 0x0c)
#define R_GPIOG_BSRR    reg32(0x40012010UL)          //     reg32(GPIOG_BASE + 0x10)
#define R_GPIOG_BRR     reg32(0x40012014UL)          //     reg32(GPIOG_BASE + 0x14)
#define R_GPIOG_LCKR    reg32(0x40012018UL)          //     reg32(GPIOG_BASE + 0x18)

/* USART registers */
#define R_USART1_SR     reg32(0x40013800UL)          //     reg32(USART1_BASE +0x00)
#define R_USART1_DR     reg32(0x40013804UL)          //     reg32(USART1_BASE +0x04)
#define R_USART1_BRR    reg32(0x40013808UL)          //     reg32(USART1_BASE +0x08)
#define R_USART1_CR1    reg32(0x4001380cUL)          //     reg32(USART1_BASE +0x0c)
#define R_USART1_CR2    reg32(0x40013810UL)          //     reg32(USART1_BASE +0x10)
#define R_USART1_CR3    reg32(0x40013814UL)          //     reg32(USART1_BASE +0x14)

#define R_USART2_SR     reg32(0x40004400UL)          //     reg32(USART2_BASE +0x00)
#define R_USART2_DR     reg32(0x40004404UL)          //     reg32(USART2_BASE +0x04)
#define R_USART2_BRR    reg32(0x40004408UL)          //     reg32(USART2_BASE +0x08)
#define R_USART2_CR1    reg32(0x4000440cUL)          //     reg32(USART2_BASE +0x0c)
#define R_USART2_CR2    reg32(0x40004410UL)          //     reg32(USART2_BASE +0x10)
#define R_USART2_CR3    reg32(0x40004414UL)          //     reg32(USART2_BASE +0x14)

#define R_USART3_SR     reg32(0x40004800UL)          //     reg32(USART3_BASE +0x00)
#define R_USART3_DR     reg32(0x40004804UL)          //     reg32(USART3_BASE +0x04)
#define R_USART3_BRR    reg32(0x40004808UL)          //     reg32(USART3_BASE +0x08)
#define R_USART3_CR1    reg32(0x4000480cUL)          //     reg32(USART3_BASE +0x0c)
#define R_USART3_CR2    reg32(0x40004810UL)          //     reg32(USART3_BASE +0x10)
#define R_USART3_CR3    reg32(0x40004814UL)          //     reg32(USART3_BASE +0x14)



/* Flash interface registers */
#define R_FLASH_ACR     reg32(0x40022000UL)        //     reg32(FLASH_R_BASE + 0x00)
#define R_FLASH_KEYR    reg32(0x40022004UL)        //     reg32(FLASH_R_BASE + 0x04)
#define R_FLASH_OPTKEYR reg32(0x40022008UL)        //     reg32(FLASH_R_BASE + 0x08)
#define R_FLASH_SR      reg32(0x4002200cUL)        //     reg32(FLASH_R_BASE + 0x0c)
#define R_FLASH_CR      reg32(0x40022010UL)        //     reg32(FLASH_R_BASE + 0x10)
#define R_FLASH_AR      reg32(0x40022014UL)        //     reg32(FLASH_R_BASE + 0x14)
#define R_FLASH_OBR     reg32(0x4002201cUL)        //     reg32(FLASH_R_BASE + 0x1c)
#define R_FLASH_WRPR    reg32(0x40022020UL)        //     reg32(FLASH_R_BASE + 0x20)

/* TIM1 registers */
#define R_TIM1_CR1      reg16(0x40012C00UL)           //     reg16(TIM1_BASE + 0x00)
#define R_TIM1_CR2      reg16(0x40012C04UL)           //     reg16(TIM1_BASE + 0x04)
#define R_TIM1_SMCR     reg16(0x40012C08UL)           //     reg16(TIM1_BASE + 0x08)
#define R_TIM1_DIER     reg16(0x40012C0cUL)           //     reg16(TIM1_BASE + 0x0c)
#define R_TIM1_SR       reg16(0x40012C10UL)           //     reg16(TIM1_BASE + 0x10)
#define R_TIM1_EGR      reg16(0x40012C14UL)           //     reg16(TIM1_BASE + 0x14)
#define R_TIM1_CCMR1    reg16(0x40012C18UL)           //     reg16(TIM1_BASE + 0x18)
#define R_TIM1_CCMR2    reg16(0x40012C1cUL)           //     reg16(TIM1_BASE + 0x1c)
#define R_TIM1_CCER     reg16(0x40012C20UL)           //     reg16(TIM1_BASE + 0x20)
#define R_TIM1_CNT      reg16(0x40012C24UL)           //     reg16(TIM1_BASE + 0x24)
#define R_TIM1_PSC      reg16(0x40012C28UL)           //     reg16(TIM1_BASE + 0x28)
#define R_TIM1_ARR      reg16(0x40012C2cUL)           //     reg16(TIM1_BASE + 0x2c)
#define R_TIM1_RCR      reg16(0x40012C30UL)           //     reg16(TIM1_BASE + 0x30)
#define R_TIM1_CCR1     reg16(0x40012C34UL)           //     reg16(TIM1_BASE + 0x34)
#define R_TIM1_CCR2     reg16(0x40012C38UL)           //     reg16(TIM1_BASE + 0x38)
#define R_TIM1_CCR3     reg16(0x40012C3cUL)           //     reg16(TIM1_BASE + 0x3c)
#define R_TIM1_CCR4     reg16(0x40012C40UL)           //     reg16(TIM1_BASE + 0x40)
#define R_TIM1_BDTR     reg16(0x40012C44UL)           //     reg16(TIM1_BASE + 0x44)
#define R_TIM1_DCR      reg16(0x40012C48UL)           //     reg16(TIM1_BASE + 0x48)
#define R_TIM1_DMAR     reg16(0x40012C4cUL)           //     reg16(TIM1_BASE + 0x4c)

/* TIM3 registers */
#define R_TIM3_CR1      reg16(0x40000400UL)           //     reg16(TIM3_BASE + 0x00)
#define R_TIM3_CR2      reg16(0x40000404UL)           //     reg16(TIM3_BASE + 0x04)
#define R_TIM3_SMCR     reg16(0x40000408UL)           //     reg16(TIM3_BASE + 0x08)
#define R_TIM3_DIER     reg16(0x4000040cUL)           //     reg16(TIM3_BASE + 0x0c)
#define R_TIM3_SR       reg16(0x40000410UL)           //     reg16(TIM3_BASE + 0x10)
#define R_TIM3_EGR      reg16(0x40000414UL)           //     reg16(TIM3_BASE + 0x14)
#define R_TIM3_CCMR1    reg16(0x40000418UL)           //     reg16(TIM3_BASE + 0x18)
#define R_TIM3_CCMR2    reg16(0x4000041cUL)           //     reg16(TIM3_BASE + 0x1c)
#define R_TIM3_CCER     reg16(0x40000420UL)           //     reg16(TIM3_BASE + 0x20)
#define R_TIM3_CNT      reg16(0x40000424UL)           //     reg16(TIM3_BASE + 0x24)
#define R_TIM3_PSC      reg16(0x40000428UL)           //     reg16(TIM3_BASE + 0x28)
#define R_TIM3_ARR      reg16(0x4000042cUL)           //     reg16(TIM3_BASE + 0x2c)
#define R_TIM3_CCR1     reg16(0x40000434UL)           //     reg16(TIM3_BASE + 0x34)
#define R_TIM3_CCR2     reg16(0x40000438UL)           //     reg16(TIM3_BASE + 0x38)
#define R_TIM3_CCR3     reg16(0x4000043cUL)           //     reg16(TIM3_BASE + 0x3c)
#define R_TIM3_CCR4     reg16(0x40000440UL)           //     reg16(TIM3_BASE + 0x40)
#define R_TIM3_DCR      reg16(0x40000448UL)           //     reg16(TIM3_BASE + 0x48)
#define R_TIM3_DMAR     reg16(0x4000044cUL)           //     reg16(TIM3_BASE + 0x4c)

/* TIM4 registers */
#define R_TIM4_CR1      reg16(0x40000800UL)           //     reg16(TIM4_BASE + 0x00)
#define R_TIM4_CR2      reg16(0x40000804UL)           //     reg16(TIM4_BASE + 0x04)
#define R_TIM4_SMCR     reg16(0x40000808UL)           //     reg16(TIM4_BASE + 0x08)
#define R_TIM4_DIER     reg16(0x4000080cUL)           //     reg16(TIM4_BASE + 0x0c)
#define R_TIM4_SR       reg16(0x40000810UL)           //     reg16(TIM4_BASE + 0x10)
#define R_TIM4_EGR      reg16(0x40000814UL)           //     reg16(TIM4_BASE + 0x14)
#define R_TIM4_CCMR1    reg16(0x40000818UL)           //     reg16(TIM4_BASE + 0x18)
#define R_TIM4_CCMR2    reg16(0x4000081cUL)           //     reg16(TIM4_BASE + 0x1c)
#define R_TIM4_CCER     reg16(0x40000820UL)           //     reg16(TIM4_BASE + 0x20)
#define R_TIM4_CNT      reg16(0x40000824UL)           //     reg16(TIM4_BASE + 0x24)
#define R_TIM4_PSC      reg16(0x40000828UL)           //     reg16(TIM4_BASE + 0x28)
#define R_TIM4_ARR      reg16(0x4000082cUL)           //     reg16(TIM4_BASE + 0x2c)
#define R_TIM4_CCR1     reg16(0x40000834UL)           //     reg16(TIM4_BASE + 0x34)
#define R_TIM4_CCR2     reg16(0x40000838UL)           //     reg16(TIM4_BASE + 0x38)
#define R_TIM4_CCR3     reg16(0x4000083cUL)           //     reg16(TIM4_BASE + 0x3c)
#define R_TIM4_CCR4     reg16(0x40000840UL)           //     reg16(TIM4_BASE + 0x40)
#define R_TIM4_DCR      reg16(0x40000848UL)           //     reg16(TIM4_BASE + 0x48)
#define R_TIM4_DMAR     reg16(0x4000084cUL)           //     reg16(TIM4_BASE + 0x4c)



/* Generic offset for TIM registers */
#define R_TIMx_CR1(base)   reg16((((u32)base) + 0x00)
#define R_TIMx_CR2(base)   reg16((((u32)base) + 0x04)
#define R_TIMx_SMCR(base)  reg16((((u32)base) + 0x08)
#define R_TIMx_DIER(base)  reg16((((u32)base) + 0x0c)
#define R_TIMx_SR(base)    reg16((((u32)base) + 0x10)
#define R_TIMx_EGR(base)   reg16((((u32)base) + 0x14)
#define R_TIMx_CCMR1(base) reg16(((u32)base) + 0x18)
#define R_TIMx_CCMR2(base) reg16(((u32)base) + 0x1c)
#define R_TIMx_CCER(base)  reg16(((u32)base) + 0x20)
#define R_TIMx_CNT(base)   reg16(((u32)base) + 0x24)
#define R_TIMx_PSC(base)   reg16(((u32)base) + 0x28)
#define R_TIMx_ARR(base)   reg16(((u32)base) + 0x2c)
#define R_TIMx_RCR(base)   reg16(((u32)base) + 0x30)
#define R_TIMx_CCR1(base)  reg16(((u32)base) + 0x34)
#define R_TIMx_CCR2(base)  reg16(((u32)base) + 0x38)
#define R_TIMx_CCR3(base)  reg16(((u32)base) + 0x3c)
#define R_TIMx_CCR4(base)  reg16(((u32)base) + 0x40)
#define R_TIMx_BDTR(base)  reg16(((u32)base) + 0x44)
#define R_TIMx_DCR(base)   reg16(((u32)base) + 0x48)
#define R_TIMx_DMAR(base)  reg16(((u32)base) + 0x4c)

/* I2C registers */
#define R_I2C1_CR1      reg16(0x40005400UL)                          //    reg16(I2C1_BASE + 0x00)
#define R_I2C1_CR2      reg16(0x40005404UL)                          //    reg16(I2C1_BASE + 0x04)
#define R_I2C1_OAR1     reg16(0x40005408UL)                          //    reg16(I2C1_BASE + 0x08)
#define R_I2C1_OAR2     reg16(0x4000540cUL)                          //    reg16(I2C1_BASE + 0x0c)
#define R_I2C1_DR       reg16(0x40005410UL)                          //    reg16(I2C1_BASE + 0x10)
#define R_I2C1_SR1      reg16(0x40005414UL)                          //    reg16(I2C1_BASE + 0x14)
#define R_I2C1_SR2      reg16(0x40005418UL)                          //    reg16(I2C1_BASE + 0x18)
#define R_I2C1_CCR      reg16(0x4000541cUL)                          //    reg16(I2C1_BASE + 0x1c)
#define R_I2C1_TRISE    reg16(0x40005420UL)                          //    reg16(I2C1_BASE + 0x20)
#define R_I2C1_FLTR     reg16(0x40005424UL)                          //    reg16(I2C1_BASE + 0x24)
#define R_I2C2_CR1      reg16(0x40005800UL)                          //    reg16(I2C2_BASE + 0x00)
#define R_I2C2_CR2      reg16(0x40005804UL)                          //    reg16(I2C2_BASE + 0x04)
#define R_I2C2_OAR1     reg16(0x40005808UL)                          //    reg16(I2C2_BASE + 0x08)
#define R_I2C2_OAR2     reg16(0x4000580cUL)                          //    reg16(I2C2_BASE + 0x0c)
#define R_I2C2_DR       reg16(0x40005810UL)                          //    reg16(I2C2_BASE + 0x10)
#define R_I2C2_SR1      reg16(0x40005814UL)                          //    reg16(I2C2_BASE + 0x14)
#define R_I2C2_SR2      reg16(0x40005818UL)                          //    reg16(I2C2_BASE + 0x18)
#define R_I2C2_CCR      reg16(0x4000581cUL)                          //    reg16(I2C2_BASE + 0x1c)
#define R_I2C2_TRISE    reg16(0x40005820UL)                          //    reg16(I2C2_BASE + 0x20)
#define R_I2C2_FLTR     reg16(0x40005824UL)                          //    reg16(I2C2_BASE + 0x24)
#define R_I2C_CR1(x)    reg16(0x40005400UL + (x * 0x400))            //    reg16(I2C1_BASE + 0x00 + (x * 0x400))
#define R_I2C_CR2(x)    reg16(0x40005404UL + (x * 0x400))            //    reg16(I2C1_BASE + 0x04 + (x * 0x400))
#define R_I2C_OAR1(x)   reg16(0x40005408UL + (x * 0x400))            //    reg16(I2C1_BASE + 0x08 + (x * 0x400))
#define R_I2C_OAR2(x)   reg16(0x4000540cUL + (x * 0x400))            //    reg16(I2C1_BASE + 0x0c + (x * 0x400))
#define R_I2C_DR(x)     reg16(0x40005410UL + (x * 0x400))            //    reg16(I2C1_BASE + 0x10 + (x * 0x400))
#define R_I2C_SR1(x)    reg16(0x40005414UL + (x * 0x400))            //    reg16(I2C1_BASE + 0x14 + (x * 0x400))
#define R_I2C_SR2(x)    reg16(0x40005418UL + (x * 0x400))            //    reg16(I2C1_BASE + 0x18 + (x * 0x400))
#define R_I2C_CCR(x)    reg16(0x4000541cUL + (x * 0x400))            //    reg16(I2C1_BASE + 0x1c + (x * 0x400))
#define R_I2C_TRISE(x)  reg16(0x40005420UL + (x * 0x400))            //    reg16(I2C1_BASE + 0x20 + (x * 0x400))
#define R_I2C_FLTR(x)   reg16(0x40005424UL + (x * 0x400))            //    reg16(I2C1_BASE + 0x24 + (x * 0x400))

/* SDIO registers */
#define R_SDIO_POWER    reg32(0x40018000UL)              //  reg32(SDIO_BASE + 0x00)
#define R_SDIO_CLKCR    reg32(0x40018004UL)              //  reg32(SDIO_BASE + 0x04)
#define R_SDIO_ARG      reg32(0x40018008UL)              //  reg32(SDIO_BASE + 0x08)
#define R_SDIO_CMD      reg32(0x4001800cUL)              //  reg32(SDIO_BASE + 0x0c)
#define R_SDIO_RESPCMD  reg32(0x40018010UL)              //  reg32(SDIO_BASE + 0x10)
#define R_SDIO_RESP1    reg32(0x40018014UL)              //  reg32(SDIO_BASE + 0x14)
#define R_SDIO_RESP2    reg32(0x40018018UL)              //  reg32(SDIO_BASE + 0x18)
#define R_SDIO_RESP3    reg32(0x4001801cUL)              //  reg32(SDIO_BASE + 0x1c)
#define R_SDIO_RESP4    reg32(0x40018020UL)              //  reg32(SDIO_BASE + 0x20)
#define R_SDIO_DTIMER   reg32(0x40018024UL)              //  reg32(SDIO_BASE + 0x24)
#define R_SDIO_DLEN     reg32(0x40018028UL)              //  reg32(SDIO_BASE + 0x28)
#define R_SDIO_DCTRL    reg32(0x4001802cUL)              //  reg32(SDIO_BASE + 0x2c)
#define R_SDIO_DCOUNT   reg32(0x40018030UL)              //  reg32(SDIO_BASE + 0x30)
#define R_SDIO_STA      reg32(0x40018034UL)              //  reg32(SDIO_BASE + 0x34)
#define R_SDIO_ICR      reg32(0x40018038UL)              //  reg32(SDIO_BASE + 0x38)
#define R_SDIO_MASK     reg32(0x4001803cUL)              //  reg32(SDIO_BASE + 0x3c)
#define R_SDIO_FIFOCNT  reg32(0x40018048UL)              //  reg32(SDIO_BASE + 0x48)
#define R_SDIO_FIFO     reg32(0x40018080UL)              //  reg32(SDIO_BASE + 0x80)

/* SPI1 registers */
//#define SPI1_BASE_ADDR  0x40013000
#define RSPI1_CR1       reg16(0x40013000UL)     // reg16(SPI1_BASE_ADDR + 0x00)  /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
#define RSPI1_CR2       reg16(0x40013004UL)     // reg16(SPI1_BASE_ADDR + 0x04)  /*!< SPI control register 2,                             Address offset: 0x04 */
#define RSPI1_SR        reg16(0x40013008UL)     // reg16(SPI1_BASE_ADDR + 0x08)  /*!< SPI status register,                                Address offset: 0x08 */
#define RSPI1_DR        reg16(0x4001300CUL)     // reg16(SPI1_BASE_ADDR + 0x0C)  /*!< SPI data register,                                  Address offset: 0x0C */
#define RSPI1_CRCPR     reg16(0x40013010UL)     // reg16(SPI1_BASE_ADDR + 0x10)  /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
#define RSPI1_RXCRCR    reg16(0x40013014UL)     // reg16(SPI1_BASE_ADDR + 0x14)  /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
#define RSPI1_TXCRCR    reg16(0x40013018UL)     // reg16(SPI1_BASE_ADDR + 0x18)  /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
#define RSPI1_I2SCFGR   reg16(0x4001301CUL)     // reg16(SPI1_BASE_ADDR + 0x1C)  /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
#define RSPI1_I2SPR     reg16(0x40013020UL)     // reg16(SPI1_BASE_ADDR + 0x20)  /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */

/* ADC registers */
#define R_ADC_SR        reg32(0x40012400UL)       //  reg32(ADC1_BASE + 0x000)
#define R_ADC_CR1       reg32(0x40012404UL)       //  reg32(ADC1_BASE + 0x004)
#define R_ADC_CR2       reg32(0x40012408UL)       //  reg32(ADC1_BASE + 0x008)
#define R_ADC_SMPR1     reg32(0x4001240cUL)       //  reg32(ADC1_BASE + 0x00c)
#define R_ADC_SMPR2     reg32(0x40012410UL)       //  reg32(ADC1_BASE + 0x010)
#define R_ADC_JOFR1     reg32(0x40012414UL)       //  reg32(ADC1_BASE + 0x014)
#define R_ADC_JOFR2     reg32(0x40012418UL)       //  reg32(ADC1_BASE + 0x018)
#define R_ADC_JOFR3     reg32(0x4001241cUL)       //  reg32(ADC1_BASE + 0x01c)
#define R_ADC_JOFR4     reg32(0x40012420UL)       //  reg32(ADC1_BASE + 0x020)
#define R_ADC_HTR       reg32(0x40012424UL)       //  reg32(ADC1_BASE + 0x024)
#define R_ADC_LTR       reg32(0x40012428UL)       //  reg32(ADC1_BASE + 0x028)
#define R_ADC_SQR1      reg32(0x4001242cUL)       //  reg32(ADC1_BASE + 0x02c)
#define R_ADC_SQR2      reg32(0x40012430UL)       //  reg32(ADC1_BASE + 0x030)
#define R_ADC_SQR3      reg32(0x40012434UL)       //  reg32(ADC1_BASE + 0x034)
#define R_ADC_JSQR      reg32(0x40012438UL)       //  reg32(ADC1_BASE + 0x038)
#define R_ADC_JDR1      reg32(0x4001243cUL)       //  reg32(ADC1_BASE + 0x03c)
#define R_ADC_JDR2      reg32(0x40012440UL)       //  reg32(ADC1_BASE + 0x040)
#define R_ADC_JDR3      reg32(0x40012444UL)       //  reg32(ADC1_BASE + 0x044)
#define R_ADC_JDR4      reg32(0x40012448UL)       //  reg32(ADC1_BASE + 0x048)
#define R_ADC_DR        reg32(0x4001244cUL)       //  reg32(ADC1_BASE + 0x04c)
#define R_ADC_CCR       reg32(0x40012704UL)       //  reg32(ADC1_BASE + 0x304)

#endif /* _STM32F411X_ */
