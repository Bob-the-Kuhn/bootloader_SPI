/* 2016, Aurelio Colosimo, <aurelio@aureliocolosimo.it> */

#ifndef _GPIO_H_
#define _GPIO_H_

#include "stm32g0b1x_defines.h"
#include "basic.h"

#define PORTA 0
#define PORTB 1
#define PORTC 2
#define PORTD 3
#define PORTE 4
#define PORTF 5
#define PORTG 6

#define GPIO_INPUT                    0b00
#define GPIO_OUTPUT                   0b01
#define GPIO_ALTERNATIVE_FUNCTION     0b10
#define GPIO_ANALOG_INPUT             0b11

#define GPIO_OUTPUT_PUSH_PULL         0
#define GPIO_OUTPUT_OPEN_DRAIN        1

#define GPIO_OUTPUT_VERY_LOW_SPEED    0b00
#define GPIO_OUTPUT_LOW_SPEED         0b01
#define GPIO_OUTPUT_HIGH_SPEED        0b10
#define GPIO_OUTPUT_VERY_HIGH_SPEED   0b11

#define GPIO_NO_PULL_UP_DOWN          0b00
#define GPIO_PULL_UP                  0b01
#define GPIO_PULL_DOWN                0b10


#define IO(port, pin) (((port) << 8) | (pin))
#define PORT(io) ((io) >> 8)
#define PIN(io) ((io) & 0xff)

//#define R_GPIOA_CRL     reg32(0x40010800UL)          //     reg32(GPIOA_BASE + 0x00)
//#define R_GPIOA_CRH     reg32(0x40010804UL)          //     reg32(GPIOA_BASE + 0x04)
//#define R_GPIOA_IDR     reg32(0x40010808UL)          //     reg32(GPIOA_BASE + 0x08)
//#define R_GPIOA_ODR     reg32(0x4001080cUL)          //     reg32(GPIOA_BASE + 0x0c)
//#define R_GPIOA_BSRR    reg32(0x40010810UL)          //     reg32(GPIOA_BASE + 0x10)
//#define R_GPIOA_BRR     reg32(0x40010814UL)          //     reg32(GPIOA_BASE + 0x14)
//#define R_GPIOA_LCKR    reg32(0x40010818UL)          //     reg32(GPIOA_BASE + 0x18)



//#define GPIOx_MODER(x) (R_GPIOA_MODER + (0x400 * PORT(x)) / 4)         // F407 defines
//#define GPIOx_OTYPER(x) (R_GPIOA_OTYPER + (0x400 * PORT(x)) / 4)       // F407 defines
//#define GPIOx_AFRL(x)  (R_GPIOA_AFRL  + (0x400 * PORT(x)) / 4)         // F407 defines
//#define GPIOx_AFRH(x)  (R_GPIOA_AFRH  + (0x400 * PORT(x)) / 4)         // F407 defines
//#define GPIOx_AFR(x)   (PIN(x) < 8 ? GPIOx_AFRL(x) : GPIOx_AFRH(x))    // F407 defines
//#define GPIOx_PUPDR(x) (R_GPIOA_PUPDR + (0x400 * PORT(x)) / 4)         // F407 defines


#define GPIOx_IDR(x)   (R_GPIOA_IDR   + (0x100 * (x)))
#define GPIOx_BSRR(x)  (R_GPIOA_BSRR  + (0x100 * (x)))
#define GPIOx_CRL(x)   (R_GPIOA_CRL  + (0x100 * (x))) 
#define GPIOx_CRH(x)   (R_GPIOA_CRH  + (0x100 * (x))) 
#define GPIOx_CR(x)    (PIN(x) < 8 ? GPIOx_CRL(x) : GPIOx_CRH(x))
#define GPIOx_ODR(x)   (R_GPIOA_ODR  + (0x100 * (x))) 

#define GPIOx_AFRL(x)  (R_GPIOA_AFRL  + (0x100 * (x)))         // G0B & F407 defines
#define GPIOx_AFRH(x)  (R_GPIOA_AFRH  + (0x100 * (x)))         // G0B & F407 defines
#define GPIOx_AFR(y, x)   (PIN(x) < 8 ? GPIOx_AFRL(y) : GPIOx_AFRH(y))
#define GPIOx_MODER(x) (R_GPIOA_MODER + (0x100 * (x)))         // G0B & F407 defines
#define GPIOx_PUPDR(x) (R_GPIOA_PUPDR + (0x100 * (x)) )        // G0B & F407 defines
#define GPIOx_OTYPER(x) (R_GPIOA_OTYPER + (0x100 * (x)))       // G0B & F407 defines
#define GPIOx_OSPEEDR(x) (R_GPIOA_OSPEEDR + (0x100 * (x))) 


void gpio_alt_func( uint8_t port, uint8_t pin, uint8_t alt_func);
void gpio_mode( uint8_t port, uint8_t pin, uint8_t mode);
void gpio_pupd( uint8_t port, uint8_t pin, uint8_t pupd);
void gpio_otype( uint8_t port, uint8_t pin, uint8_t otype);
void gpio_ospeed( uint8_t port, uint8_t pin, uint8_t ospeed);

#define GPIO_CONFIG_OUTPUT(PORT, PIN, PUPD, OUT_TYPE, OSPEED) do { \
       gpio_alt_func(PORT, PIN, 0); \
       gpio_mode(PORT, PIN, GPIO_OUTPUT); \
       gpio_pupd(PORT, PIN, PUPD); \
       gpio_otype(PORT, PIN, OUT_TYPE); \
       gpio_ospeed(PORT, PIN, OSPEED); \
       } while(0);

#define GPIO_CONFIG_ALT(PORT, PIN, ALT_FUNC, PUPD, OUT_TYPE, OSPEED) do { \
       GPIO_CONFIG_OUTPUT(PORT, PIN, PUPD, OUT_TYPE, OSPEED); \
       gpio_alt_func(PORT, PIN, ALT_FUNC); \
       } while(0);

#define GPIO_CONFIG_INPUT(PORT, PIN, PUPD) do { \
       gpio_mode(PORT, PIN, GPIO_INPUT); \
       gpio_pupd(PORT, PIN, PUPD); \
       } while(0);
       
static inline int gpio_rd(uint8_t port, uint8_t pin)
{
	return (rd32(GPIOx_IDR(port)) >> pin) & 0x1;
}

static inline void gpio_wr(uint8_t port, uint8_t pin, int v)
{
	wr32(GPIOx_BSRR(port), 1 << (pin + (v ? 0 : 16)));
}



//#define PINSEL(io) (PINSEL0 + 2 * PORT(io) + PIN(io) / 16)
//#define PINMODE(io) (PINMODE0 + 2 * PORT(io) + PIN(io) / 16)


#endif /* _GPIO_H_ */
