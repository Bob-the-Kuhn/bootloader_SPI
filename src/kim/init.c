/*
 * Author: Aurelio Colosimo, 2016
 * Originally modified from kim-os project:
 * https://github.com/colosimo/kim-os
 */

#include <linker.h>
#include <basic.h>
#include <log.h>
#include <gpio.h>
#include <stm32f407x_defines.h>
#include <string.h>
#include <string.h>

#define STACK_TOP ((void*)(0x20004000))

#define CPU_FREQ      168000000
#define AHB_PRESCALER         4
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

void attr_weak isr_systick(void)
{
  ticks++;
}

// STM405/407 vector table
static const void *attr_sect("isrv_sys") _isrv_sys[] = {
  /* Cortex-M0 system interrupts */
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

// STM405/407 vector table
static const void *attr_sect("isrv_irq") _isrv_irq[] = {
  /* Peripheral interrupts */
  isr_none, /* Window WatchDog              */             
  isr_none, /* PVD through EXTI Line detection */          
  isr_none, /* Tamper and TimeStamps through the EXTI line */ 
  isr_none, /* RTC Wakeup through the EXTI line */         
  isr_none, /* FLASH                        */             
  isr_none, /* RCC                          */             
  isr_none, /* EXTI Line0                   */             
  isr_none, /* EXTI Line1                   */             
  isr_none, /* EXTI Line2                   */             
  isr_none, /* EXTI Line3                   */             
  isr_none, /* EXTI Line4                   */             
  isr_none, /* DMA1 Stream 0                */             
  isr_none, /* DMA1 Stream 1                */             
  isr_none, /* DMA1 Stream 2                */             
  isr_none, /* DMA1 Stream 3                */             
  isr_none, /* DMA1 Stream 4                */             
  isr_none, /* DMA1 Stream 5                */             
  isr_none, /* DMA1 Stream 6                */             
  isr_none, /* ADC1, ADC2 and ADC3s         */             
  isr_none, /* CAN1 TX                      */             
  isr_none, /* CAN1 RX0                     */             
  isr_none, /* CAN1 RX1                     */             
  isr_none, /* CAN1 SCE                     */             
  isr_none, /* External Line[9:5]s          */             
  isr_none, /* TIM1 Break and TIM9          */         
  isr_none, /* TIM1 Update and TIM10        */         
  isr_none, /* TIM1 Trigger and Commutation and TIM11 */
  isr_none, /* TIM1 Capture Compare         */             
  isr_none, /* TIM2                         */             
  isr_none, /* TIM3                         */             
  isr_none, /* TIM4                         */             
  isr_none, /* I2C1 Event                   */             
  isr_none, /* I2C1 Error                   */             
  isr_none, /* I2C2 Event                   */             
  isr_none, /* I2C2 Error                   */             
  isr_none, /* SPI1                         */             
  isr_none, /* SPI2                         */             
  isr_none, /* USART1                       */             
  isr_none, /* USART2                       */             
  isr_none, /* USART3                       */             
  isr_none, /* External Line[15:10]s        */             
  isr_none, /* RTC Alarm (A and B) through EXTI Line */    
  isr_none, /* USB OTG FS Wakeup through EXTI line */      
  isr_none, /* TIM8 Break and TIM12         */         
  isr_none, /* TIM8 Update and TIM13        */         
  isr_none, /* TIM8 Trigger and Commutation and TIM14 */
  isr_none, /* TIM8 Capture Compare         */             
  isr_none, /* DMA1 Stream7                 */             
  isr_none, /* FSMC                         */             
  isr_none, /* SDIO                         */             
  isr_none, /* TIM5                         */             
  isr_none, /* SPI3                         */             
  isr_none, /* UART4                        */             
  isr_none, /* UART5                        */             
  isr_none, /* TIM6 and DAC1&2 underrun errors */          
  isr_none, /* TIM7                         */
  isr_none, /* DMA2 Stream 0                */             
  isr_none, /* DMA2 Stream 1                */             
  isr_none, /* DMA2 Stream 2                */             
  isr_none, /* DMA2 Stream 3                */             
  isr_none, /* DMA2 Stream 4                */             
  isr_none, /* Ethernet                     */             
  isr_none, /* Ethernet Wakeup through EXTI line */        
  isr_none, /* CAN2 TX                      */             
  isr_none, /* CAN2 RX0                     */             
  isr_none, /* CAN2 RX1                     */             
  isr_none, /* CAN2 SCE                     */             
  isr_none, /* USB OTG FS                   */             
  isr_none, /* DMA2 Stream 5                */             
  isr_none, /* DMA2 Stream 6                */             
  isr_none, /* DMA2 Stream 7                */             
  isr_none, /* USART6                       */             
  isr_none, /* I2C3 event                   */             
  isr_none, /* I2C3 error                   */             
  isr_none, /* USB OTG HS End Point 1 Out   */             
  isr_none, /* USB OTG HS End Point 1 In    */             
  isr_none, /* USB OTG HS Wakeup through EXTI */           
  isr_none, /* USB OTG HS                   */             
  isr_none, /* DCMI                         */             
  isr_none, /* CRYP crypto                  */             
  isr_none, /* Hash and Rng                 */
  isr_none, /* FPU                          */
};

uint32_t systicks_freq(void)
{
  return SYSTICKS_FREQ;
}

u32 systicks(void)
{
  return ticks;
}

inline void sleep()
{
  asm("wfi");
}

u32 k_ticks() attr_weak attr_alias("systicks");
u32 k_ticks_freq(void) attr_alias("systicks_freq");

void init_clock(void)
{
  /* Enable HSE (8MHz external oscillator) */
  or32(RCC_CR, BIT16);
  while (!(rd32(RCC_CR) & BIT17));

  /* PLLM=8 PLLN=336, PLLP=00 (2), PLLQ=7; f_PLL=168MHz, f_USB=48MHz */
  // STMcubeIDE says this is impossible, SYSCLK reports 525MHz
  and32(RCC_PLLCFGR, ~0x0f037fff);
  or32(RCC_PLLCFGR, BIT22 | (7 << 24) | (336 << 6) | 8);
  or32(RCC_CR, BIT24);
  while (!(rd32(RCC_CR) & BIT25));
  
  ///* PLLM=4 PLLN=96, PLLP=00 (2), PLLQ=4; f_PLL=96MHz, AHB=2, f_USB=48MHz */
  //and32(RCC_PLLCFGR, ~0x0f037fff);
  //or32(RCC_PLLCFGR, BIT22 | (4 << 24) | (96 << 6) | 4);
  ////or32(RCC_PLLCFGR, (4 << 24) | (96 << 6) | 4); // use HSI clock - dead CPU
  //or32(RCC_CR, BIT24);
  //while (!(rd32(RCC_CR) & BIT25));

  /* Configure flash */
  wr32(R_FLASH_ACR, BIT10 | BIT9 | BIT8 | 1);

  /* Use PLL as system clock, with AHB prescaler set to 4 */
  wr32(RCC_CFGR, (0x9 << 4) | 0x2); // APB1 & APB2 set to 1
  
  /* Use PLL as system clock, with AHB prescaler set to 2 */
  //wr32(RCC_CFGR, (0x8 << 4) | 4 << 10 | 0x2);   // APB1 set to 2, APB2 to 1
  //while (((rd32(RCC_CFGR) >> 2) & 0x3) != 0x2);

  /* Enable clock on AHB and APB peripherals */
  wr32(RCC_AHB1ENR, BIT7 | BIT4 | BIT3 | BIT2 | BIT1 | BIT0); /* GPIO A,B,C,D,E,H*/
  wr32(RCC_APB1ENR, BIT1 | BIT2 | BIT17| BIT18); /* TIM3, TIM4, USART2 and USART3 */
  wr32(RCC_APB2ENR, BIT0 | BIT4 | BIT8 | BIT12| BIT18); /* TIM1/11, ADC1, SPI1, USART1 */
}

void init_systick(void)
{
  ticks = 0;
  wr32(R_SYST_RVR, HCLCK / SYSTICKS_FREQ);
  wr32(R_SYST_CVR, 0);
  wr32(R_SYST_CSR, BIT0 | BIT1 | BIT2);
}

int putchar(int c)
{
  if (c == '\n')
    putchar('\r');
  wr32(R_USART1_DR, c);
  wr32(R_USART3_DR, c);  // echo out USART 3
  while (!(rd32(R_USART1_SR) & BIT6));
  while (!(rd32(R_USART3_SR) & BIT6));
  return c;
}

void init_uart(void)
{
  /* USART3 on PD8/PD9 */                       
  /* USART2 on PD5/PD6 */
  /* USART1 on PA9/PA10 */
  gpio_func(IO(PORTD, 8), 7);
  gpio_func(IO(PORTD, 9), 7);
  gpio_mode(IO(PORTD, 8), PULL_NO);
  gpio_mode(IO(PORTD, 9), PULL_NO);
  /* fPCLK=42MHz, br=115.2KBps, USARTDIV=22.8125, see table 80 pag. 519 */
  wr32(R_USART3_BRR, (22 << 4) | 13);
  or32(R_USART3_CR1, BIT13 | BIT5 | BIT3 | BIT2);
  or32(R_NVIC_ISER(1), BIT7); /* USART3 is irq 39 */
  //or32(R_NVIC_ISER(1), BIT6); /* USART2 is irq 38 */
  //or32(R_NVIC_ISER(1), BIT5); /* USART1 is irq 37 */
  
  /* USART3 on PD8/PD9 */
  /* USART2 on PD5/PD6 */
  /* USART1 on PA9/PA10 */
  gpio_func(IO(PORTA, 9), 7);
  gpio_func(IO(PORTA, 10), 7);
  gpio_mode(IO(PORTA, 9), PULL_NO);
  gpio_mode(IO(PORTA, 10), PULL_NO);
  /* fPCLK=42MHz, br=115.2KBps, USARTDIV=22.8125, see table 80 pag. 519 */
  wr32(R_USART1_BRR, (22 << 4) | 13);
  or32(R_USART1_CR1, BIT13 | BIT5 | BIT3 | BIT2);
  //or32(R_NVIC_ISER(1), BIT7); /* USART3 is irq 39 */                                                    
  //or32(R_NVIC_ISER(1), BIT6); /* USART2 is irq 38 */
  or32(R_NVIC_ISER(1), BIT5); /* USART1 is irq 37 */
}

void init(void)
{
  init_clock();
  init_systick();
  init_uart();
  main();
}
