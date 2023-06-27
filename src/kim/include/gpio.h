/* 2016, Aurelio Colosimo, <aurelio@aureliocolosimo.it> */

#ifndef _GPIO_H_
#define _GPIO_H_

#include "stm32f103ZE_defines.h"
#include "basic.h"

#define PORTA 0
#define PORTB 1
#define PORTC 2
#define PORTD 3
#define PORTE 4
#define PORTF 5
#define PORTG 6

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

#define GPIOx_IDR(x)   (R_GPIOA_IDR   + (0x100 * PORT(x)))
#define GPIOx_BSRR(x)  (R_GPIOA_BSRR  + (0x100 * PORT(x)))
#define GPIOx_CRL(x)   (R_GPIOA_CRL  + (0x100 * PORT(x))) 
#define GPIOx_CRH(x)   (R_GPIOA_CRH  + (0x100 * PORT(x))) 
#define GPIOx_CR(x)    (PIN(x) < 8 ? GPIOx_CRL(x) : GPIOx_CRH(x))
#define GPIOx_ODR(x)   (R_GPIOA_ODR  + (0x100 * PORT(x))) 


// mode bits for GPIO_CR registers 
//  from table 20 of the RM0008 Reference Manual 
#define Output_Push_Pull                          0b0001
#define Output_Open_Drain                         0b0101
#define Output_Push_Pull_Alternate_Function       0b1001
#define Output_Open_Drain_Alternate_Function      0b1101
#define Input_Analog                              0b0000
#define Input_Floating                            0b0100
#define Input_Pull_Down                           0b10000
#define Input_Pull_Up                             0b10001
#define Speed_Mid                                 0b01     // Maximum output speed 10 MHz  
#define Speed_Low                                 0b10     // Maximum output speed 2 MHz
#define Speed_High                                0b11     // Maximum output speed 50 MHz


#if 0
#define _GPIO_MODE(port_pin, mode, speed, ...) do {                \
  uint8_t mode_ = mode;                                            \
  volatile u32 *reg_cr;                                            \
  reg_cr = GPIOx_CR(port_pin);                                     \
  uint8_t pin = PIN(port_pin);                                     \
  pin  = (pin >= 8) ? pin - 8 : pin;                               \
  volatile u32 *reg_odr;                                           \
  reg_odr =GPIOx_ODR(port_pin);                                    \
  if ((mode == Input_Pull_Down) || (mode == Input_Pull_Up)) {      \
    and32(reg_cr, (0xFFFFFFFFUL & ~(0xF << pin)));                  \
    or32(reg_cr, ( ((mode & 0b11110) >> 1) << pin));               \
    and32(reg_odr, (0xFFFFFFFFUL & ~(0x1 << PIN(port_pin))));      \
    if (mode & 0b00001) or32(reg_odr, (1 << PIN(port_pin)));        \
  }                                                                \
  else {                                                           \
    if (mode & 1) {                                                \
      mode_ = ((mode & 0b1100) | speed);                            \
      if (!speed) mode_ = (mode_ | Speed_Low);                     \
    }                                                              \
    and32(reg_cr, (0xFFFFFFFFUL & ~(0xF << pin)));                  \
    or32(reg_cr, (mode_ << pin));                                  \
  }                                                                \
} while(0)
                                                                    
#define gpio_mode(...) _GPIO_MODE(__VA_ARGS__, 0, 0)
#endif

#define JUST3(a, b, c, ...) (a), (b), (c)
#define FUNC(...) func(JUST3(__VA_ARGS__, 0, 0))
  
void gpio_mode( uint16_t port_pin, uint8_t mode, uint8_t speed);
#if 0  // CPP version of GPIO_MODE

void gpio_mode( uint16_t port_pin, uint8_t mode, uint8_t speed) {
  uint8_t mode_ = mode;
  volatile u32 *reg_cr; 
  reg_cr = GPIOx_CR(port_pin);  // pointer to correct CRH/CRL register
  uint8_t pin = PIN(port_pin);
  pin  = (pin >= 8) ? pin - 8 : pin;  // pin (index) into CRH/CRL register
  volatile u32 *reg_odr; 
  reg_odr =GPIOx_ODR(port_pin);
  if ((mode == Input_Pull_Down) || (mode == Input_Pull_Up)) {
    and32(reg_cr, (0xFFFFFFFFUL & ~(0xF << pin)));  // clear out bits
    or32(reg_cr, ( ((mode & 0b11110) >> 1) << pin));  // or in mode bits (need to stip off pull up/down info)
    and32(reg_odr, (0xFFFFFFFFUL & ~(0x1 << PIN(port_pin))));  // clear out bit for pull up/down info (leaves it in pull down mode)
    if (mode & 0b00001) or32(reg_odr, (1 << PIN(port_pin)));    // set bit to pull up
  }
  else {
    if (mode & 1) {
      mode_ = ((mode & 0b1100) | speed); // add in speed bits for outputs
      if (!speed) mode_ = (mode_ | Speed_Low);  // default to low speed if speed isn't specified
    }
    and32(reg_cr, (0xFFFFFFFFUL & ~(0xF << pin)));  // clear out bits
    or32(reg_cr, (mode_ << pin));  // or in mode bits (need to stip off pull up/down info)
  }
}
#endif  
    
static inline int gpio_rd(u16 io)
{
	return (rd32(GPIOx_IDR(io)) >> PIN(io)) & 0x1;
}

static inline void gpio_wr(u16 io, int v)
{
	wr32(GPIOx_BSRR(io), 1 << (PIN(io) + (v ? 0 : 16)));
}



//#define PINSEL(io) (PINSEL0 + 2 * PORT(io) + PIN(io) / 16)
//#define PINMODE(io) (PINMODE0 + 2 * PORT(io) + PIN(io) / 16)


#endif /* _GPIO_H_ */
