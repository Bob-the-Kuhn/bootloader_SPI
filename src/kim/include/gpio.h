/* 2016, Aurelio Colosimo, <aurelio@aureliocolosimo.it> */

#ifndef _GPIO_H_
#define _GPIO_H_

#include "LPC1769x_defines.h"
#include "basic.h"

#define PORT0 0
#define PORT1 1
#define PORT2 2
#define PORT3 3
#define PORT4 4

#define IO(port, pin) (((port) << 8) | (pin))
#define PORT(io) ((io) >> 8)
#define PIN(io) ((io) & 0xff)

#define GPIOx_FIODIR(x) (GPIO_FIODIR + (0x20 * PORT(x)) / 4)

#define GPIOx_FIOPIN(x)   (GPIO_FIOPIN   + (0x20 * PORT(x)) / 4)
#define GPIOx_FIOSET(x)  (GPIO_FIOSET  + (0x20 * PORT(x)) / 4)
#define GPIOx_FIOCLR(x)  (GPIO_FIOCLR  + (0x20 * PORT(x)) / 4)
#define GPIOx_PINSEL_L(x)  (GPIO_PINSEL0 + (0x8 * PORT(x)) / 4)
#define GPIOx_PINSEL_H(x)  (GPIO_PINSEL1 + (0x8 * PORT(x)) / 4)
#define GPIOx_PINSEL(x)   (PIN(x) < 16 ? GPIOx_PINSEL_L(x) : GPIOx_PINSEL_H(x))


#define GPIOx_PINMODE_L(x)  (GPIO_PINMODE0 + (0x8 * PORT(x)) / 4)
#define GPIOx_PINMODE_H(x)  (GPIO_PINMODE1 + (0x8 * PORT(x)) / 4)
#define GPIOx_PINMODE(x)   (PIN(x) < 16 ? GPIOx_PINMODE_L(x) : GPIOx_PINMODE_H(x))

#define GPIOx_PINMODE_OD(x) (GPIO_PINMODE_OD + (0x4 * PORT(x)) / 4)

#define PULL_UP        0b00
#define REPEATER_MODE  0b01
#define PULL_NO        0b10
#define PULL_DOWN      0b11

#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

static inline void gpio_dir(u16 io, int out)
{
	volatile u32 *reg;
  reg = GPIOx_FIODIR(io);
	*reg &= (u32)~(1 << (PIN(io)));
	if (out)
    *reg |= (u32)(1 << (PIN(io)));
}

static inline int gpio_rd(u16 io)
{
  volatile u32 *reg;
  reg = (u32*)GPIOx_FIOPIN(io);
	return (*reg >> PIN(io)) & 0x1;
}

static inline void gpio_wr(u16 io, int v)
{
  volatile u32 *reg_SET;
	volatile u32 *reg_CLR;
	reg_SET = GPIOx_FIOSET(io);
	reg_CLR = GPIOx_FIOCLR(io);
	if(v)
    *reg_SET = (u32)(1 << PIN(io));
  else
    *reg_CLR = (u32)(1 << PIN(io));
}

static inline void gpio_func(u16 io, u8 f)
{
	volatile u32 *reg_PINSEL;
	volatile u32 *reg_FIODIR;
	reg_PINSEL = GPIOx_PINSEL(io);
	reg_FIODIR = GPIOx_FIODIR(io);
  *reg_PINSEL &= (u32)~(0b11 << (2 * (PIN(io) % 16)));
	if (f != 0) {
    *reg_FIODIR &= (u32)~(1 << (PIN(io)));
		*reg_FIODIR |= (u32)(1 << (PIN(io)));
		*reg_PINSEL |= ((u32)f & 0b11) << (2 * (PIN(io) % 16));
	}
}

static inline void gpio_mode(u16 io, u8 f)
{
	volatile u32 *reg;
  reg = GPIOx_PINMODE(io);
  *reg &= (u32)~(3 << (2 * (PIN(io) % 16)));
  *reg |= ((u32)f) << (2 * (PIN(io) % 16));
}

static inline void gpio_odrain(u16 io, int odrain)
{
	volatile u32 *reg;
  reg = GPIOx_PINMODE_OD(io);
  *reg &= (u32)~(1 << PIN(io));
	if (odrain)
    *reg |= (u32)(1 << PIN(io));
}

/* 
#define GPIOx_FIODIR(x) (R_FIODIR + (0x20 * PORT(x)) / 4)

#define GPIOx_FIOPIN(x)   (R_FIOPIN   + (0x20 * PORT(x)) / 4)
#define GPIOx_FIOSET(x)  (R_FIOSET  + (0x20 * PORT(x)) / 4)
#define GPIOx_FIOCLR(x)  (R_FIOCLR  + (0x20 * PORT(x)) / 4)
#define GPIOx_PINSEL_L(x)  (R_PINSEL0 + (0x8 * PORT(x)) / 4)
#define GPIOx_PINSEL_H(x)  (R_PINSEL1 + (0x8 * PORT(x)) / 4)
#define GPIOx_PINSEL(x)   (PIN(x) < 16 ? GPIOx_PINSEL_L(x) : GPIOx_PINSEL_H(x))


#define GPIOx_PINMODE_L(x)  (R_PINMODE0 + (0x8 * PORT(x)) / 4)
#define GPIOx_PINMODE_H(x)  (R_PINMODE1 + (0x8 * PORT(x)) / 4)
#define GPIOx_PINMODE(x)   (PIN(x) < 16 ? GPIOx_PINMODE_L(x) : GPIOx_PINMODE_H(x))

#define GPIOx_PINMODE_OD(x) (R_PINMODE_OD + (0x4 * PORT(x)) / 4)

#define PULL_UP        0b00
#define REPEATER_MODE  0b01
#define PULL_NO        0b10
#define PULL_DOWN      0b11

#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

static inline void gpio_dir(u16 io, int out)
{
	volatile u32 *reg = GPIOx_FIODIR(io);
	and32(reg, ~(1 << (PIN(io))));
	if (out)
		or32(reg, (1 << (PIN(io))));
}

static inline int gpio_rd(u16 io)
{
	return (rd32(GPIOx_FIOPIN(io)) >> PIN(io)) & 0x1;
}

static inline void gpio_wr(u16 io, int v)
{
	if(v)
    wr32(GPIOx_FIOSET(io), 1 << PIN(io));
  else
    wr32(GPIOx_FIOCLR(io), 1 << PIN(io));
}

static inline void gpio_func(u16 io, u8 f)
{
	volatile u32 *reg_PINSEL;
	volatile u32 *reg_FIODIR;
	reg_PINSEL = GPIOx_PINSEL(io);
	reg_FIODIR = GPIOx_FIODIR(io);
	and32(reg_PINSEL, ~(0b11 << (2 * (PIN(io) % 16))));
	if (f != 0) {
		and32(reg_FIODIR, ~(1 << (PIN(io))));
		or32(reg_FIODIR, (1 << (PIN(io))));
		or32(reg_PINSEL, ((u32)f & 0b11) << (2 * (PIN(io) % 16)));
	}
}

static inline void gpio_mode(u16 io, u8 f)
{
	volatile u32 *reg = GPIOx_PINMODE(io);
	and32(reg, ~(3 << (2 * (PIN(io) % 16))));
	or32(reg, ((u32)f) << (2 * (PIN(io) % 16)));
}

static inline void gpio_odrain(u16 io, int odrain)
{
	volatile u32 *reg = GPIOx_PINMODE_OD(io);
	and32(reg, ~(1 << PIN(io)));
	if (odrain)
		or32(reg, (1 << PIN(io)));
}
*/

#endif /* _GPIO_H_ */
