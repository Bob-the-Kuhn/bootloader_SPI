/*
 * Author: Aurelio Colosimo, 2017
 *
 * This file is part of kim-os project: https://github.com/colosimo/kim-os
 * According to kim-os license, you can do whatever you want with it,
 * as long as you retain this notice.
 */

#ifndef _STM32G474X__
#define _STM32G474X_

/* RCC registers */
#define RCC_CR          reg32(0x40021000)
#define RCC_ICSCR       reg32(0x40021004)
#define RCC_CFGR        reg32(0x40021008)
#define RCC_PLLCFGR     reg32(0x4002100C)
#define RCC_CIER        reg32(0x40021018)
#define RCC_CIFR        reg32(0x4002101C)
#define RCC_CICR        reg32(0x40021020)
#define RCC_AHB1RSTR    reg32(0x40021028)
#define RCC_AHB2RSTR    reg32(0x4002102C)
#define RCC_AHB3RSTR    reg32(0x40021030)
#define RCC_APB1RSTR1   reg32(0x40021038)
#define RCC_APB1RSTR2   reg32(0x4002103C)
#define RCC_APB2RSTR    reg32(0x40021040)
#define RCC_AHB1ENR     reg32(0x40021048)
#define RCC_AHB2ENR     reg32(0x4002104C)
#define RCC_AHB3ENR     reg32(0x40021050)
#define RCC_APB1ENR1    reg32(0x40021058)
#define RCC_APB1ENR2    reg32(0x4002105C)
#define RCC_APB2ENR     reg32(0x40021060)
#define RCC_AHB1SMENR   reg32(0x40021068)
#define RCC_AHB2SMENR   reg32(0x4002106C)
#define RCC_AHB3SMENR   reg32(0x40021070)
#define RCC_APB1SMENR1  reg32(0x40021078)
#define RCC_APB1SMENR2  reg32(0x4002107C)
#define RCC_APB2SMENR   reg32(0x40021080)
#define RCC_CCIPR       reg32(0x40021088)
#define RCC_BDCR        reg32(0x40021090)
#define RCC_CSR         reg32(0x40021094)
#define RCC_CRRCR       reg32(0x40021098)
#define RCC_CCIPR2      reg32(0x4002109C)

/* GPIO registers */
#define R_GPIOA_MODER   reg32(0x48000000)     
#define R_GPIOA_OTYPER  reg32(0x48000004)
#define R_GPIOA_OSPEEDR reg32(0x48000008)
#define R_GPIOA_PUPDR   reg32(0x4800000c)
#define R_GPIOA_IDR     reg32(0x48000010)
#define R_GPIOA_ODR     reg32(0x48000014)
#define R_GPIOA_BSRR    reg32(0x48000018)
#define R_GPIOA_LCKR    reg32(0x4800001c)
#define R_GPIOA_AFRL    reg32(0x48000020)
#define R_GPIOA_AFRH    reg32(0x48000024)
#define R_GPIOA_BRR     reg32(0x48000028)

#define R_GPIOB_MODER   reg32(0x48000400)
#define R_GPIOB_OTYPER  reg32(0x48000404)
#define R_GPIOB_OSPEEDR reg32(0x48000408)
#define R_GPIOB_PUPDR   reg32(0x4800040c)
#define R_GPIOB_IDR     reg32(0x48000410)
#define R_GPIOB_ODR     reg32(0x48000414)
#define R_GPIOB_BSRR    reg32(0x48000418)
#define R_GPIOB_LCKR    reg32(0x4800041c)
#define R_GPIOB_AFRL    reg32(0x48000420)
#define R_GPIOB_AFRH    reg32(0x48000424)
#define R_GPIOB_BRR     reg32(0x48000428)

#define R_GPIOC_MODER   reg32(0x48000800)
#define R_GPIOC_OTYPER  reg32(0x48000804)
#define R_GPIOC_OSPEEDR reg32(0x48000808)
#define R_GPIOC_PUPDR   reg32(0x4800080c)
#define R_GPIOC_IDR     reg32(0x48000810)
#define R_GPIOC_ODR     reg32(0x48000814)
#define R_GPIOC_BSRR    reg32(0x48000818)
#define R_GPIOC_LCKR    reg32(0x4800081c)
#define R_GPIOC_AFRL    reg32(0x48000820)
#define R_GPIOC_AFRH    reg32(0x48000824)
#define R_GPIOC_BRR     reg32(0x48000828)

#define R_GPIOD_MODER   reg32(0x48000c00)
#define R_GPIOD_OTYPER  reg32(0x48000c04)
#define R_GPIOD_OSPEEDR reg32(0x48000c08)
#define R_GPIOD_PUPDR   reg32(0x48000c0c)
#define R_GPIOD_IDR     reg32(0x48000c10)
#define R_GPIOD_ODR     reg32(0x48000c14)
#define R_GPIOD_BSRR    reg32(0x48000c18)
#define R_GPIOD_LCKR    reg32(0x48000c1c)
#define R_GPIOD_AFRL    reg32(0x48000c20)
#define R_GPIOD_AFRH    reg32(0x48000c24)
#define R_GPIOD_BRR     reg32(0x48000c28)

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

#define R_GPIOG_MODER   reg32(0x50001800)
#define R_GPIOG_OTYPER  reg32(0x50001804)
#define R_GPIOG_OSPEEDR reg32(0x50001808)
#define R_GPIOG_PUPDR   reg32(0x5000180c)
#define R_GPIOG_IDR     reg32(0x50001810)
#define R_GPIOG_ODR     reg32(0x50001814)
#define R_GPIOG_BSRR    reg32(0x50001818)
#define R_GPIOG_LCKR    reg32(0x5000181c)
#define R_GPIOG_AFRL    reg32(0x50001820)
#define R_GPIOG_AFRH    reg32(0x50001824)
#define R_GPIOG_BRR     reg32(0x50001828)
 

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


/* Flash interface registers */

#define R_FLASH_ACR         reg32(0x40022000)    /*!< FLASH access control register,            Address offset: 0x00 */
#define R_FLASH_PDKEYR      reg32(0x40022004)    /*!< FLASH power down key register,            Address offset: 0x04 */
#define R_FLASH_KEYR        reg32(0x40022008)    /*!< FLASH key register,                       Address offset: 0x08 */
#define R_FLASH_OPTKEYR     reg32(0x4002200C)    /*!< FLASH option key register,                Address offset: 0x0C */
#define R_FLASH_SR          reg32(0x40022010)    /*!< FLASH status register,                    Address offset: 0x10 */
#define R_FLASH_CR          reg32(0x40022014)    /*!< FLASH control register,                   Address offset: 0x14 */
#define R_FLASH_ECCR        reg32(0x40022018)    /*!< FLASH ECC register,                       Address offset: 0x18 */
#define R_FLASH_OPTR        reg32(0x40022020)    /*!< FLASH option register,                    Address offset: 0x20 */
#define R_FLASH_PCROP1SR    reg32(0x40022024)    /*!< FLASH bank1 PCROP start address register, Address offset: 0x24 */
#define R_FLASH_PCROP1ER    reg32(0x40022028)    /*!< FLASH bank1 PCROP end address register,   Address offset: 0x28 */
#define R_FLASH_WRP1AR      reg32(0x4002202C)    /*!< FLASH bank1 WRP area A address register,  Address offset: 0x2C */
#define R_FLASH_WRP1BR      reg32(0x40022030)    /*!< FLASH bank1 WRP area B address register,  Address offset: 0x30 */
#define R_FLASH_PCROP2SR    reg32(0x40022044)    /*!< FLASH bank2 PCROP start address register, Address offset: 0x44 */
#define R_FLASH_PCROP2ER    reg32(0x40022048)    /*!< FLASH bank2 PCROP end address register,   Address offset: 0x48 */
//#define R_FLASH_WRP2AR      reg32(0x4002204C)    /*!< FLASH bank2 WRP area A address register,  Address offset: 0x4C */
//#define R_FLASH_WRP2BR      reg32(0x40022050)    /*!< FLASH bank2 WRP area B address register,  Address offset: 0x50 */
#define R_FLASH_SEC1R       reg32(0x40022070)    /*!< FLASH Securable memory register bank1,    Address offset: 0x70 */
#define R_FLASH_SEC2R       reg32(0x40022074)    /*!< FLASH Securable memory register bank2,    Address offset: 0x74 */
                            
#define R_FLASH_WRP1AR_STRT reg16(0x4002202C)    /*!< FLASH Bank WRP area A address register, start bits Address offset: 0x2C */ 
#define R_FLASH_WRP1BR_STRT reg16(0x40022030)    /*!< FLASH Bank WRP area B address register,            Address offset: 0x30 */ 
                                                                                                                                 
//#define R_FLASH_WRP2AR_STRT reg16(0x4002204C)    /*!< FLASH Bank2 WRP area A address register,           Address offset: 0x4C */ 
//#define R_FLASH_WRP2BR_STRT reg16(0x40022050)    /*!< FLASH Bank2 WRP area B address register,           Address offset: 0x50 */
#define R_FLASH_WRP1AR_END  reg16(0x4002202E)    /*!< FLASH Bank WRP area A address register, start bits Address offset: 0x2C */ 
#define R_FLASH_WRP1BR_END  reg16(0x40022032)    /*!< FLASH Bank WRP area B address register,            Address offset: 0x30 */ 
                                                                                                      
//#define R_FLASH_WRP2AR_END  reg16(0x4002204E)    /*!< FLASH Bank2 WRP area A address register,           Address offset: 0x4C */ 
//#define R_FLASH_WRP2BR_END  reg16(0x40022052)    /*!< FLASH Bank2 WRP area B address register,           Address offset: 0x50 */


/* TIM1 registers */



/* I2C registers */


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


/* System Control Block registers */
#define R_SCB_CPUID     reg32(0xE000ED00)
#define R_SCB_ICSR      reg32(0xE000ED04)
#define R_SCB_AIRCR     reg32(0xE000ED0C)
#define R_SCB_SCR       reg32(0xE000ED10)
#define R_SCB_CCR       reg32(0xE000ED14)
#define R_SCB_SHPR2     reg32(0xE000ED1C)
#define R_SCB_SHPR3     reg32(0xE000ED20)

/* SysTick registers */
#define R_STK_CTRL      reg32(0xE000E010)
#define R_STK_LOAD      reg32(0xE000E014)
#define R_STK_VAL       reg32(0xE000E018)
#define R_STK_CALIB     reg32(0xE000E01C)

/* NVIC registers */
#define R_NVIC_ISER(x)    reg32(0xE000E100 + 4 * (x))
#define R_NVIC_ICER(x)    reg32(0xE000E180 + 4 * (x))
#define R_NVIC_ISPR(x)    reg32(0xE000E200 + 4 * (x))
#define R_NVIC_ICPR(x)    reg32(0xE000E280 + 4 * (x))
#define R_NVIC_IPR(x)     reg32(0xE000E400 + 4 * (x))

#endif /* _STM32G474X_ */
