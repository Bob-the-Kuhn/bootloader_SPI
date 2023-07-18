/* 2016, Aurelio Colosimo, <aurelio@aureliocolosimo.it> */

#ifndef _GPIO_CPP_
#define _GPIO_CPP_

#include "stm32g474x_defines.h"
#include "basic.h"
#include "gpio.h"

void gpio_alt_func( uint8_t port,  uint8_t pin, uint8_t alt_func) {
  volatile uint32_t *reg_afr; 
  reg_afr = GPIOx_AFR(port, pin);  // pointer to correct alt_func register
  uint8_t pin_  = (pin >= 8) ? pin - 8 : pin;  // pin (index) into alt_func H/L register
  and32(reg_afr, (0xFFFFFFFFUL & ~(0x0000000FUL << (pin * 4))));  // clear out bits
  or32(reg_afr, (alt_func << (pin_ * 4)));  // or in alt_func bits
  gpio_mode(port, pin, GPIO_ALTERNATIVE_FUNCTION);  // set MODER to alt function
}

void gpio_mode( uint8_t port,  uint8_t pin, uint8_t mode) {
  volatile uint32_t *reg_mode; 
  reg_mode = GPIOx_MODER(port);  // pointer to correct mode register
  and32(reg_mode, (0xFFFFFFFFUL & ~(0x00000003UL << (pin * 2))));  // clear out bits
  or32(reg_mode, (mode << (pin * 2)));  // or in mode bits
}

void gpio_pupd( uint8_t port,  uint8_t pin, uint8_t pupd) {
  volatile uint32_t *reg_pupd; 
  reg_pupd = GPIOx_PUPDR(port);  // pointer to correct pupd register
  and32(reg_pupd, (0xFFFFFFFFUL & ~(0x00000001UL << (pin ))));  // clear out bits
  or32(reg_pupd, (pupd << (pin)));  // or in pupd bits
}

void gpio_otype( uint8_t port,  uint8_t pin, uint8_t otype) {
  volatile uint32_t *reg_otype; 
  reg_otype = GPIOx_OTYPER(port);  // pointer to correct otype register
  and32(reg_otype, (0xFFFFFFFFUL & ~(0x00000001UL << (pin))));  // clear out bits
  or32(reg_otype, (otype << (pin)));  // or in otype bits
}

void gpio_ospeed( uint8_t port,  uint8_t pin, uint8_t ospeed) {
  volatile uint32_t *reg_ospeed;
  reg_ospeed = GPIOx_OSPEEDR(port);  // pointer to correct ospeed register
  and32(reg_ospeed, (0xFFFFFFFFUL & ~(0x00000003UL << (pin * 2))));  // clear out bits
  or32(reg_ospeed, (ospeed << (pin * 2)));  // or in ospeed bits
}

#endif /* _GPIO_CPP_ */
