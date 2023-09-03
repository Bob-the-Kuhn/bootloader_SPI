/*
 * Author: Aurelio Colosimo, 2016
 * Originally modified from kim-os project:
 * https://github.com/colosimo/kim-os
 */

#include "linker.h"
#include "basic.h"
#include "log.h"
#include "gpio.h"
//#include "LPC1769x_defines.h"
#include <string.h>
#include <string.h>

#define STACK_TOP ((void*)(0x10007FFF))

#define CPU_FREQ      120000000
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

// LPC1769 exception table
static const void *attr_sect("isrv_sys") _isrv_sys[] = {
  /* Cortex-M0 system interrupts             */
  STACK_TOP,    /* Stack top            00     */
  isr_reset,    /* Reset_Handler        04     */
  isr_nmi,      /* NMI_Handler          08     */
  isr_hf,       /* HardFault_Handler    0C     */
  isr_none,     /* MemManage_Handler    10     */
  isr_none,     /* BusFault_Handler     14     */
  isr_none,     /* UsageFault_Handler   18     */
  0,            /* Reserved             1C     */
  0,            /* Reserved             20     */
  0,            /* Reserved             24     */
  0,            /* Reserved             28     */
  isr_none,     /* SVC_Handler          2C     */
  isr_none,     /* DebugMon_Handler     30     */
  0,            /* Reserved             34     */
  isr_none,     /* PendSV_Handler       38     */
  isr_systick,  /* SysTick_Handler      3C     */
};                                      

// LPC1769 vector table
static const void *attr_sect("isrv_irq") _isrv_irq[] = {
  /* Peripheral interrupts */
  isr_none, /* WDT                          40  */             
  isr_none, /* Timer 0                      44  */          
  isr_none, /* Timer 1                      48  */ 
  isr_none, /* Timer 2                      4C  */         
  isr_none, /* Timer 3                      50  */             
  isr_none, /* UART0                        54  */             
  isr_none, /* UART1                        58  */             
  isr_none, /* UART 2                       5C  */             
  isr_none, /* UART 3                       60  */             
  isr_none, /* PWM1                         64  */             
  isr_none, /* I2C0                         68  */             
  isr_none, /* I2C1                         6C  */             
  isr_none, /* I2C2                         70  */             
  isr_none, /* SPI                          74  */             
  isr_none, /* SSP0                         78  */             
  isr_none, /* SSP 1                        7C  */             
  isr_none, /* PLL0 (Main PLL)              80  */             
  isr_none, /* RTC                          84  */             
  isr_none, /* External Interrupt 0 (EINT0) 88  */             
  isr_none, /* External Interrupt 1 (EINT1) 8C  */             
  isr_none, /* External Interrupt 2 (EINT2) 90  */             
  isr_none, /* External Interrupt 3 (EINT3) 94  */             
  isr_none, /* ADC                          98  */             
  isr_none, /* BOD                          9C  */             
  isr_none, /* USB                          A0  */         
  isr_none, /* CAN                          A4  */         
  isr_none, /* GPDMAand TIM11               A8  */
  isr_none, /* I2S                          AC  */             
  isr_none, /* Ethernet                     B0  */             
  isr_none, /* Repetitive Interrupt         B4  */             
  isr_none, /* Timer                        B8  */             
  isr_none, /* Motor Control PWM            BC  */             
  isr_none, /* Quadrature Encoder           C0  */             
  isr_none, /* PLL1 (USB PLL)               C4  */             
  isr_none, /* USB Activity Interrupt       C8  */             
  isr_none, /* CAN Activity Interrupt       CC  */ 
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


#define PLL0feed do{ *LPC_SC_PLL0FEED = 0xAA; *LPC_SC_PLL0FEED = 0x55;}while(0)
#define PLL1feed do{ *LPC_SC_PLL1FEED = 0xAA; *LPC_SC_PLL1FEED = 0x55;}while(0)

void init_clock(void)
{
  /* 120 MHz LPC1769 */
  /* Enable External Oscillator (12MHz external oscillator) */
  
  *LPC_SC_SCS = 0x020;  // enable low range external crystal oscillator
  while(!(*LPC_SC_SCS & _BV(OSCSTAT)));  // wait for external crystal oscillator to be functional
  
  
  
  // 1. Disconnect PLL0 with one feed sequence if PLL0 is already connected.
  PLL0feed;  // force contents of R_PLL0STAT to be valid
  if (*LPC_SC_PLL0STAT & _BV(PLLC0_STAT)) {
    CLEAR_BIT(*LPC_SC_PLL0CON, PLLC0); // disconnect PLL0
    PLL0feed;  // activate above action
  }
  // 2. Disable PLL0 with one feed sequence.
  CLEAR_BIT(*LPC_SC_PLL0CON, PLLE0); // disable PLL0
  PLL0feed;  // activate above action
  // 3. Change the CPU Clock Divider setting to speed up operation without PLL0, if desired.
  // 4. Write to the Clock Source Selection Control register to change the clock source if
  //    needed.
  *LPC_SC_CLKSRCSEL = 1; // use main oscillator (external crystal) to drive PLL0
  // 5. Write to the PLL0CFG and make it effective with one feed sequence. The PLL0CFG
  //    can only be updated when PLL0 is disabled.
  *LPC_SC_PLL0CFG = 0x0E; // PLLCLK = 360MHz (N = 1, M =15 (N must be less than 120 (12MHz/100KHz)))
  /* Periphral clock must be selected before PLL0 enabling and connecting
   * - according errata.lpc1768-16.March.2010 -
   */
  *LPC_SC_PCLKSEL0 = (_BV(PCLK_TIMER0) | _BV(PCLK_TIMER1) | _BV(PCLK_UART0) | _BV(PCLK_UART1) | _BV(PCLK_SSP1) ); // set to divide by 1 
  *LPC_SC_PCLKSEL1 = (_BV(PCLK_GPIOINT) | _BV(PCLK_PCB) | _BV(PCLK_TIMER2) | _BV(PCLK_TIMER3) | _BV(PCLK_TIMER2) | _BV(PCLK_TIMER2) | _BV(PCLK_TIMER3) | _BV(PCLK_UART2) | _BV(PCLK_UART3) | _BV(PCLK_SYSCON));

  // 6. Enable PLL0 with one feed sequence.
  *LPC_SC_PLL0CON = _BV(PLLE0);
  PLL0feed;  // activate above action
  // 7. Change the CPU Clock Divider setting for the operation with PLL0. It is critical to do
  //    this before connecting PLL0.
  *LPC_SC_CCLKCFG = 2; // divide PLL0CLK by 3
  // 8. Wait for PLL0 to achieve lock by monitoring the PLOCK0 bit in the PLL0STAT register,
  //    or using the PLOCK0 interrupt, or wait for a fixed time when the input clock to PLL0 is
  //    slow (i.e. 32 kHz). The value of PLOCK0 may not be stable when the PLL reference
  //    frequency (FREF, the frequency of REFCLK, which is equal to the PLL input
  //    frequency divided by the pre-divider value) is less than 100 kHz or greater than
  //    20 MHz. In these cases, the PLL may be assumed to be stable after a start-up time
  //    has passed. This time is 500 μs when FREF is greater than 400 kHz and 200 / FREF
  //    seconds when FREF is less than 400 kHz.
  while (!(*LPC_SC_PLL0STAT & _BV(PLOCK0)));// wait for PLL0 to lock
  // 9. Connect PLL0 with one feed sequence.
  SET_BIT(*LPC_SC_PLL0CON, PLLC0); // connect PLL0
  PLL0feed;  // activate above action
  while (!(*LPC_SC_PLL0STAT & (_BV(PLLE0_STAT) | _BV(PLLC0_STAT))));/* Wait for PLLC0_STAT & PLLE0_STAT */
                                
  
  // setup PLL1 for 48MHz USB
  // M = 4, P= 2
    // 1. Disconnect PLL1 with one feed sequence if PLL1 is already connected.
  PLL1feed;  // force contents of R_PLL1STAT to be valid
  if (*LPC_SC_PLL1STAT & _BV(PLLC1_STAT)) {
    CLEAR_BIT(*LPC_SC_PLL1CON, PLLC1); // disconnect PLL1
    PLL1feed;  // activate above action
  }
  // 2. Disable PLL1 with one feed sequence.
  CLEAR_BIT(*LPC_SC_PLL1CON, PLLE1); // disable PLL1
  PLL0feed;  // activate above action
  // 3. Change the CPU Clock Divider setting to speed up operation without PLL1, if desired.
  // N/A
  // 4. Write to the Clock Source Selection Control register to change the clock source if
  //    needed.
  // N/A
  // 5. Write to the PLL1CFG and make it effective with one feed sequence. The PLL1CFG
  //    can only be updated when PLL1 is disabled.
  *LPC_SC_PLL1CFG = 0x023; // PLLCLK = 48MHz (M = 4, P = 2) 
  // 6. Enable PLL1 with one feed sequence.
  SET_BIT(*LPC_SC_PLL1CON, PLLE1); 
  PLL1feed;  // activate above action
  // 7. Change the CPU Clock Divider setting for the operation with PLL1. It is critical to do
  //    this before connecting PLL1.
  // N/A
  // 8. Wait for PLL1 to achieve lock by monitoring the PLOCK1 bit in the PLL1STAT register
  while (!(*LPC_SC_PLL1STAT & _BV(PLOCK1)));  // wait for PLL1 to lock
  // 9. Connect PLL1 with one feed sequence.
  SET_BIT(*LPC_SC_PLL1CON, PLLC1); // connect PLL1
  PLL1feed;  // activate above action
  while (!(*LPC_SC_PLL1STAT & (_BV(PLLE1_STAT) | _BV(PLLC1_STAT))));/* Wait for PLLC1_STAT & PLLE1_STAT */
  
  *LPC_SC_PCONP |= (_BV(PCTIM0) | _BV(PCTIM1) | _BV(PCUART0) | _BV(PCSSP1) | _BV(PCGPIO) | _BV(PCRIT) | _BV(PCTIM2) | _BV(PCTIM3)); // enable modules
  
  
}

void init_systick(void)
{
  ticks = 0;
  *SysTick_STRELOAD = 0x01D46F;  // 1mS (120MHz/1000 -1)
  *SysTick_STCURR = 0;
  *SysTick_STCTRL = 7;  // enable, enable interrupt, use CPU clock
}

int putchar(int c)
{
  if (c == '\n')
    putchar('\r');
  *LPC_UART0_U0THR = c;
  while (!(*LPC_UART0_U0LSR & _BV(THRE)));  // wait for transmit buffer to empty 
  return c;
}

void init_uart(void)
{
  /* UART0 on P0_2/P0_3 */
  
  *GPIO_PINSEL0 &= ~0x000000F0;
  *GPIO_PINSEL0 |= 0x00000050;            // Enable TxD0 P0.2 and RxD0 p0.3 

  *LPC_UART0_U0FCR = (1<<SBIT_FIFO) | (1<<SBIT_RxFIFO) | (1<<SBIT_TxFIFO); // Enable FIFO and reset Rx/Tx FIFO buffers    
  *LPC_UART0_U0LCR = (0x03<<SBIT_WordLenght) | (1<<SBIT_DLAB); // 8bit data, 1Stop bit, No parity, enable DLL & DLM registers
  //DivAddVal - 5
  //MulVal - 13
  //DLest - 47 - 0x02F
  //DLM = DLest [15:8] = 0
  //DLL = DLest [7:0] = 0x2F
  // calculated baud rate 115,248.23 (PCLK = 120MHz)
  *LPC_UART0_U0FDR = 5 + (13<<MULVAL);
  *LPC_UART0_U0DLL =  0x2F; 
  *LPC_UART0_U0DLM =  0; 
  CLEAR_BIT(*LPC_UART0_U0LCR, SBIT_DLAB);  // disable DLL & DLM registers
}

void init(void)
{
  init_clock();
  init_systick();
  init_uart();
  main();
}
