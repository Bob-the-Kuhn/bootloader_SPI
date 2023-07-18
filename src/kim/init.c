/*
 * Author: Aurelio Colosimo, 2016
 * Originally modified from kim-os project:
 * https://github.com/colosimo/kim-os
 */

#include <linker.h>
#include <basic.h>
#include <log.h>
#include <gpio.h>
#include "stm32g474x_defines.h"
#include <string.h>
#include <string.h>

#define STACK_TOP ((void*)(0x20004000))

#define CPU_FREQ      48000000
#define AHB_PRESCALER         1
#define HCLCK (CPU_FREQ / AHB_PRESCALER)
#define SYSTICKS_FREQ      1000

#define kprint(...) /* FIXME dummy */

static uint32_t ticks = 4294907296UL;

extern unsigned char _sdata_flash;
extern unsigned char _sdata;
extern unsigned char _edata;
extern unsigned char _sbss;
extern unsigned char _ebss;
extern void main(void);

void init(void);

void __attribute__ ((weak)) isr_reset(void)
{
  unsigned char *src, *dest;

  /* Load data to ram */
  src = &_sdata_flash;
  dest = &_sdata;
  while (dest != &_edata)
    *dest++ = *src++;

  /* Set bss section to 0 */
  dest = &_sbss;
  while (dest != &_ebss)
    *dest++ = 0;

  /* Skip to mach or board specific init */
  init();
}

static uint32_t attr_used ipsr(void)
{
  uint32_t res;
  __asm volatile ("mrs %0, ipsr" : "=r" (res));
  return res;
}


void attr_weak isr_none(void)
{
#if 0
  crt("Unhandled IPSR=%x ISPR=%x\n", (uint)ipsr(), (uint)rd32(R_NVIC_ISPR));
  while(1);
#endif
}

void attr_weak isr_nmi(void)
{
}

void attr_weak isr_hf(void)
{
}

void isr_systick(void)
{
  ticks++;
}

void attr_weak SysTick_Handler(void)
{
  ticks++;
}
// STM32G474vector table
static const void *attr_sect("isrv_sys") _isrv_sys[] = {
  /* Cortex-M4 system interrupts */
  STACK_TOP,    /* Stack top */
  isr_reset,    /* Reset_Handler */
  isr_nmi,      /* NMI_Handler */
  isr_hf,       /* HardFault_Handler */
  isr_none,     /* MemManage_Handler */
  isr_none,     /* BusFault_Handler */
  isr_none,     /* UsageFault_Handler */
  0,            /* Reserved */
  0,            /* Reserved */
  0,            /* Reserved */
  0,            /* Reserved */
  isr_none,     /* SVC_Handler */
  isr_none,     /* DebugMon_Handler */
  0,            /* Reserved */
  isr_none,     /* PendSV_Handler */
  isr_systick,  /* SysTick_Handler */
};

// STM32G474 Peripheral interrupt vector table
static const void *attr_sect("isrv_irq") _isrv_irq[102] = {isr_none};

uint32_t systicks_freq(void)
{
  return SYSTICKS_FREQ;
}

uint32_t systicks(void)
{
  return ticks;
}

inline void sleep()
{
  asm("wfi");
}

uint32_t k_ticks() attr_weak attr_alias("systicks");
uint32_t k_ticks_freq(void) attr_alias("systicks_freq");

void init_clock(void)
{
  
  /* Configure flash */
  // G474  must set FLASH latency before switching to higher speed
  //and32(R_FLASH_ACR, ~(0x0000000FUL)); // clear out wait state bits
  and32(R_FLASH_ACR, ~(0x0000070FUL)); // clear out wait state, prefetch, instruction cache & data cache bits
  or32(R_FLASH_ACR, 0x0000010FUL);  // 4 wait states, prefetch enabled.
  //or32(R_FLASH_ACR, 0x0000070FUL);  // 4 wait states, prefetch, instruction cache & data cache enabled.
  while ((rd32(R_FLASH_ACR) & 0x00000101U) != 0x00000101U);  // wait until the bits update
  
  or32(RCC_CR, (BIT8 | BIT9));  //turn on HSI 
  and32(RCC_CR, ~(BIT24 | BIT18 | BIT16) ); // turn off PLL so can update RCC_CFGR, turn HSE off
  
  while ((rd32(RCC_CR) & BIT25));  // wait for PLL to stop
  
  
  /* SYSCLOCK=48MHz, f_USB=48MHz */
  
  /* use PLL, AHB (HPRE): /1 , APB (PPRE): /1 ,  MCO:none */
  and32(RCC_CFGR, ~0x7f006ff3UL);  // set all bits to zero except reserved and hardware controlled
  or32(RCC_CFGR, 0xF); // use PLL as SYSCLK source
  
  and32(RCC_PLLCFGR, ~0xff737ff3UL);  // set all bits to zero except reserved and hardware controlled
  // R: /4, PLLRCLK enabled, Q: /4, PLLQCLK enabled, P: /2, PLLPCLK disabled, N: x12, M: /1 , HSint16_t is the input
  or32(RCC_PLLCFGR, (2 << 27) | (1 << 25) | (1 << 24) | (1 << 21) |  (1 << 20) | (12 << 8) | 2);  
  or32(RCC_CR, BIT24);  // turn on PLL
  while (!(rd32(RCC_CR) & BIT25)); 
  
  
  // set USART1 & 2 clocks to sysclk 
  and32(RCC_CCIPR, ~0b1111);  //clear out bits
  or32(RCC_CCIPR,   0b0101);    //select sysclk

  /* Enable clocks on AHB and APB peripherals */
  wr32(RCC_AHB2ENR, (BIT6 | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0));  /* GPIOA-GPIOG */
  wr32(RCC_APB1ENR1, (BIT17 | BIT14  | BIT5 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0));   /* USART2, SPI2, TIM2-7 */
  wr32(RCC_APB2ENR, ( BIT14  | BIT13 | BIT11 | BIT0)); /* USART1, TIM8, TIM1, SYSCFGEN */

}

void init_systick(void)
{
  ticks = 0;
  wr32(R_STK_LOAD, ((HCLCK / SYSTICKS_FREQ) * 4));
  wr32(R_STK_VAL, 0);
  wr32(R_STK_CTRL, BIT0 | BIT1 | BIT2);
}

int putchar(int c)
{
  if (c == '\n')
    putchar('\r');
  wr32(R_USART2_TDR, c);
  while (!(rd32(R_USART2_ISR) & BIT6));
  wr32(R_USART1_TDR, c);
  while (!(rd32(R_USART1_ISR) & BIT6));
  return c;
}

void k_delay(const uint32_t ms);

void init_uart(void)
{
 
 
 /* USART2 on PA2/PA3 */
 
  GPIO_CONFIG_ALT(PORTA, 2, 1, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);
  GPIO_CONFIG_ALT(PORTA, 3, 1, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);
  /* fPCLK=48MHz, br=115.2KBps, USARTDIV=22.8125, see table 80 pag. 519 */
  wr32(R_USART2_BRR, 0x19f);
  or32(R_USART2_CR1, BIT13 | BIT5 | BIT3 | BIT2 | BIT0);
  //or32(R_NVIC_ISER(1), BIT7); /* USART3 is irq 39 */
  //or32(R_NVIC_ISER(1), BIT6); /* USART2 is irq 38 */
  //or32(R_NVIC_ISER(1), BIT5); /* USART1 is irq 37 */
  
 
 /* USART1 on PC4/PC5 */
 
  GPIO_CONFIG_ALT(PORTC, 4, 7, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);
  GPIO_CONFIG_ALT(PORTC, 5, 1, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);
  /* fPCLK=48MHz, br=115.2KBps, USARTDIV=22.8125, see table 80 pag. 519 */
  wr32(R_USART1_BRR, 0x19f);
  or32(R_USART1_CR1, BIT13 | BIT5 | BIT3 | BIT2 | BIT0);
  //or32(R_NVIC_ISER(1), BIT7); /* USART3 is irq 39 */
  //or32(R_NVIC_ISER(1), BIT6); /* USART2 is irq 38 */
  //or32(R_NVIC_ISER(1), BIT5); /* USART1 is irq 37 */
}

void init(void)
{
  init_clock();
  init_systick();
  init_uart();
  main();
}
