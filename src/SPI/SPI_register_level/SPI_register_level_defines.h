/*
 * Author: Aurelio Colosimo, 2017
 *
 * This file is part of kim-os project: https://github.com/colosimo/kim-os
 * According to kim-os license, you can do whatever you want with it,
 * as long as you retain this notice.
 */

#ifndef _LPC1769_DEFINES_
#define _LPC1769_DEFINES_

#include <inttypes.h>



volatile uint32_t* GPIOA_BASE = (uint32_t*)0x40020000UL; 
volatile uint32_t* GPIOB_BASE = (uint32_t*)0x40020400UL;

#define GPIOA_AFRL     *(GPIOA_BASE + 0x08)  // 32 bit pointer arithmatic: 0x20 bytes = 0x08 words
#define GPIOA_BSRR     *(GPIOA_BASE + 0x06)  // 0x18 - > 0x06
#define GPIOA_CRL      *(GPIOA_BASE + 0)     // F103
#define GPIOA_MODER    *(GPIOA_BASE + 0x00)
#define GPIOA_OSPEEDR  *(GPIOA_BASE + 0x02)  // 8 -> 2

#define GPIOB_AFRL     *(GPIOB_BASE + 0x08)  // 32 bit pointer arithmatic: 0x20 bytes = 0x08 words
#define GPIOB_BSRR     *(GPIOB_BASE + 0x06)  // 0x18 - > 0x06
#define GPIOB_CRL      *(GPIOB_BASE + )      // F103
#define GPIOB_MODER    *(GPIOB_BASE + 0x00)
#define GPIOB_OSPEEDR  *(GPIOB_BASE + 0x02)  // 8 -> 2




#endif /* _LPC1769_DEFINES_ */
