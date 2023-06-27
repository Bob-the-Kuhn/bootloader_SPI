/* 2016, Aurelio Colosimo, <aurelio@aureliocolosimo.it> */

#ifndef _GPIO_CPP_
#define _GPIO_CPP_

#include "stm32f103ZE_defines.h"
#include "basic.h"
#include "gpio.h"

void gpio_mode( uint16_t port_pin, uint8_t mode, uint8_t speed) {
  uint32_t mode_ = mode;
  volatile u32 *reg_cr; 
  reg_cr = GPIOx_CR(port_pin);  // pointer to correct CRH/CRL register
  uint8_t pin = PIN(port_pin);
  pin  = (pin >= 8) ? pin - 8 : pin;  // pin (index) into CRH/CRL register
  volatile u32 *reg_odr; 
  reg_odr =GPIOx_ODR(port_pin);
  if ((mode == Input_Pull_Down) || (mode == Input_Pull_Up)) {
    and32(reg_cr, (0xFFFFFFFFUL & ~(0x0000000FUL << (pin * 4))));  // clear out bits
    or32(reg_cr, ( ((mode_ & 0b11110) >> 1) << (pin * 4)));  // or in mode bits (need to stip off pull up/down info)
    and32(reg_odr, (0xFFFFFFFFUL & ~(0x1 << PIN(port_pin))));  // clear out bit for pull up/down info (leaves it in pull down mode)
    if (mode & 0b00001) or32(reg_odr, (1 << PIN(port_pin)));    // set bit to pull up
  }
  else {
    if (mode & 1) {
      mode_ = ((mode & 0b1100) | speed); // add in speed bits for outputs
      if (!speed) mode_ = (mode_ | Speed_Low);  // default to low speed if speed isn't specified
    }
    
    //uint32_t mask_temp = 0xF << pin;
    //mask_temp = ~mask_temp;
    //mask_temp &= 0xFFFFFFFFUL;
    ////mask_temp = (0xFFFFFFFFUL & ~(0xF << pin));
    //uint32_t reg_temp = *reg_cr;
    //mask_temp &= reg_temp;
    //*reg_cr = mask_temp;
    //and32(reg_cr,  mask_temp);  // clear out bits
    and32(reg_cr, (0xFFFFFFFFUL & ~(0x0000000FUL << (pin * 4))));  // clear out bits
    or32(reg_cr, (mode_ << (pin * 4)));  // or in mode bits
  }
}

    

#endif /* _GPIO_CPP_ */
