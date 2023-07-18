/**
  ******************************************************************************
  * @file      startup_stm32g474xx.s
  * @author    MCD Application Team
  * @brief     STM32G474xx devices vector table GCC toolchain.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address,
  *                - Configure the clock system
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M4 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  
  /**
  ******************************************************************************
  *  Modified for use in a bootloader
  *   1: isr_reset is NOT weak so it'll replace the one in the system
  *      startup_xxxx.s file.
  *   2: Code is added at the very beginning that decides if the bootloader
  *      start sequence is to be executed or if we should start the application.
  *   3. The normal Reset_Handler code follows the added code section.  The
  *      Reset_Handler code is a copy of the code in:
  *        .platformio\packages\framework-cmsis-stm32f4\Source\Templates\gcc\startup_stm32f407xx.s
  *   4. The added code is based on Piranna's comment in the following page:
  *        https://community.st.com/s/question/0D50X0000AFpTmUSQV/using-nvicsystemreset-in-bootloaderapplication-jumps
  *
  ******************************************************************************
  * 
  * Copyright (c) 2023 Bob Kuhn
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <https://www.gnu.org/licenses/>.
  *
  ******************************************************************************
  */

 
.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global g_pfnVectors_b
.global Default_Handler_IRQ

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

.equ  BootRAM,        0xF1E0F85F
/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/

  .section .text.Reset_Handler
  .global Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:


/* start code adopted from the post by Piranha */

      LDR R0, = 0x08000000UL // set VTOR to the beginning of main FLASH
      LDR R1, =0xE000ED00UL   // SCB
	    STR R0, [R1, #8]      // VTOR
      LDR R0, = 0                  //  default to boot application
      LDR R1, = Magic_Location
      LDR R1, [R1, R0]             // read magic value  (LDR R1, [R1, #0])
      CMP R1, #0
      BEQ SKIP                          // if 0 then have not yet been through the bootloader
      LDR R2, = MagicApplication
      CMP R1, #0
      BEQ SKIP                          // if 0 then have not yet been through the bootloader
      LDR R2, [R2, R0]           // LDR R2, [R2, #0]
      CMP R1, R2
      BNE SKIP                     
      LDR R1, = Magic_Location
      STR R0, [R1, #0]             // force next boot to go thru the bootloader
      LDR R0, = APP_ADDR           // point this boot at application
      LDR R2, = 0           // set R2 to offset
      LDR R0, [R0, R2]      // (LDR R0, [R0, #0])
      LDR R1, =0xE000ED00UL   // SCB
	    STR R0, [R1, #8]      // VTOR
      LDR R2, = 0           // set R2 to offset
      LDR R3, [R0, R2]
      MOV SP, R3            // set SP from application vector table (LDR SP, [R0, #0] )
      LDR R2, = 4           // set R2 to offset
      LDR R0, [R0, R2]      // get application reset vector address  (LDR R0, [R0, #4])
      DSB                   // Ensure the VTOR and SP operations are complete
      ISB                   // Flush the pipeline because of SP change
      BX   R0               // Start the application - mimic reset (jump to application reset handler)
 SKIP:

/* end code adopted from the post by Piranha */

/* code copied from the STM32cubeIDE startup_stm32g474retx.s file */

  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b	LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit
  
/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss
  
/* end code copied from the STM32cubeIDE startup_stm32g474retx.s file */


/* Call the clock system intitialization function.*/
/*   bl  SystemInit   */
/* Call static constructors */
/*     bl __libc_init_array */
/* Call the application's entry point.*/

/*  bx  init */
      LDR R0, = 0x08000000   // point at bootloader vector table
      LDR R1, = 0
      LDR R2, [R0, R1]      // set SP from bootloader vector table (LDR SP, [R0, #0])
      MOV SP, R2
      DSB                   // Ensure the SP operation is complete
      LDR R0, = init        // point this boot at bootloader
      BX   R0               // Start the bootloader code
 
.size Reset_Handler, .-Reset_Handler


/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval : None
*/
    .section .text.Default_Handler_IRQ,"ax",%progbits
Default_Handler_IRQ:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler_IRQ, .-Default_Handler_IRQ
  
  .section .text.NMI_Handler,"ax",%progbits
NMI_Handler:
Infinite_Loop_NMI_Handler:
  b Infinite_Loop_NMI_Handler
  .size NMI_Handler, .-NMI_Handler
  
  .section .text.HardFault_Handler,"ax",%progbits
HardFault_Handler:
Infinite_Loop_HardFault_Handler:
  b Infinite_Loop_HardFault_Handler
  .size HardFault_Handler, .-HardFault_Handler
  
  .section .text.BusFault_Handler,"ax",%progbits
BusFault_Handler:
Infinite_Loop_BusFault_Handler:
  b Infinite_Loop_BusFault_Handler
  .size BusFault_Handler, .-BusFault_Handler
  
  .section .text.UsageFault_Handler,"ax",%progbits
UsageFault_Handler:
Infinite_Loop_UsageFault_Handler:
  b Infinite_Loop_UsageFault_Handler
  .size UsageFault_Handler, .-UsageFault_Handler
  
  .section .text.SVC_Handler,"ax",%progbits
SVC_Handler:
Infinite_Loop_SVC_Handler:
  b Infinite_Loop_SVC_Handler
  .size SVC_Handler, .-SVC_Handler
  
  .section .text.DebugMon_Handler,"ax",%progbits
DebugMon_Handler:
Infinite_Loop_DebugMon_Handler:
  b Infinite_Loop_DebugMon_Handler
  .size DebugMon_Handler, .-DebugMon_Handler
  
  .section .text.PendSV_Handler,"ax",%progbits
PendSV_Handler:
Infinite_Loop_PendSV_Handler:
  b Infinite_Loop_PendSV_Handler
  .size PendSV_Handler, .-PendSV_Handler
  
  .section .text.WWDG_IRQHandler,"ax",%progbits
WWDG_IRQHandler:
Infinite_Loop_WWDG_IRQHandler:
  b Infinite_Loop_WWDG_IRQHandler
  .size WWDG_IRQHandler, .-WWDG_IRQHandler
  
  .section .text.PVD_PVM_IRQHandler,"ax",%progbits
PVD_PVM_IRQHandler:
Infinite_Loop_PVD_PVM_IRQHandler:
  b Infinite_Loop_PVD_PVM_IRQHandler
  .size PVD_PVM_IRQHandler, .-PVD_PVM_IRQHandler
  
  .section .text.RTC_TAMP_LSECSS_IRQHandler,"ax",%progbits
RTC_TAMP_LSECSS_IRQHandler:
Infinite_Loop_RTC_TAMP_LSECSS_IRQHandler:
  b Infinite_Loop_RTC_TAMP_LSECSS_IRQHandler
  .size RTC_TAMP_LSECSS_IRQHandler, .-RTC_TAMP_LSECSS_IRQHandler
  
  .section .text.RTC_WKUP_IRQHandler,"ax",%progbits
RTC_WKUP_IRQHandler:
Infinite_Loop_RTC_WKUP_IRQHandler:
  b Infinite_Loop_RTC_WKUP_IRQHandler
  .size RTC_WKUP_IRQHandler, .-RTC_WKUP_IRQHandler
  
  .section .text.FLASH_IRQHandler,"ax",%progbits
FLASH_IRQHandler:
Infinite_Loop_FLASH_IRQHandler:
  b Infinite_Loop_FLASH_IRQHandler
  .size FLASH_IRQHandler, .-FLASH_IRQHandler
  
  .section .text.RCC_IRQHandler,"ax",%progbits
RCC_IRQHandler:
Infinite_Loop_RCC_IRQHandler:
  b Infinite_Loop_RCC_IRQHandler
  .size RCC_IRQHandler, .-RCC_IRQHandler
  
  .section .text.EXTI0_IRQHandler,"ax",%progbits
EXTI0_IRQHandler:
Infinite_Loop_EXTI0_IRQHandler:
  b Infinite_Loop_EXTI0_IRQHandler
  .size EXTI0_IRQHandler, .-EXTI0_IRQHandler
  
  .section .text.EXTI1_IRQHandler,"ax",%progbits
EXTI1_IRQHandler:
Infinite_Loop_EXTI1_IRQHandler:
  b Infinite_Loop_EXTI1_IRQHandler
  .size EXTI1_IRQHandler, .-EXTI1_IRQHandler
  
  .section .text.EXTI2_IRQHandler,"ax",%progbits
EXTI2_IRQHandler:
Infinite_Loop_EXTI2_IRQHandler:
  b Infinite_Loop_EXTI2_IRQHandler
  .size EXTI2_IRQHandler, .-EXTI2_IRQHandler
  
  .section .text.EXTI3_IRQHandler,"ax",%progbits
EXTI3_IRQHandler:
Infinite_Loop_EXTI3_IRQHandler:
  b Infinite_Loop_EXTI3_IRQHandler
  .size EXTI3_IRQHandler, .-EXTI3_IRQHandler
  
  .section .text.EXTI4_IRQHandler,"ax",%progbits
EXTI4_IRQHandler:
Infinite_Loop_EXTI4_IRQHandler:
  b Infinite_Loop_EXTI4_IRQHandler
  .size EXTI4_IRQHandler, .-EXTI4_IRQHandler
  
  .section .text.DMA1_Channel1_IRQHandler,"ax",%progbits
DMA1_Channel1_IRQHandler:
Infinite_Loop_DMA1_Channel1_IRQHandler:
  b Infinite_Loop_DMA1_Channel1_IRQHandler
  .size DMA1_Channel1_IRQHandler, .-DMA1_Channel1_IRQHandler
  
  .section .text.DMA1_Channel2_IRQHandler,"ax",%progbits
DMA1_Channel2_IRQHandler:
Infinite_Loop_DMA1_Channel2_IRQHandler:
  b Infinite_Loop_DMA1_Channel2_IRQHandler
  .size DMA1_Channel2_IRQHandler, .-DMA1_Channel2_IRQHandler
  
  .section .text.DMA1_Channel3_IRQHandler,"ax",%progbits
DMA1_Channel3_IRQHandler:
Infinite_Loop_DMA1_Channel3_IRQHandler:
  b Infinite_Loop_DMA1_Channel3_IRQHandler
  .size DMA1_Channel3_IRQHandler, .-DMA1_Channel3_IRQHandler
  
  .section .text.DMA1_Channel4_IRQHandler,"ax",%progbits
DMA1_Channel4_IRQHandler:
Infinite_Loop_DMA1_Channel4_IRQHandler:
  b Infinite_Loop_DMA1_Channel4_IRQHandler
  .size DMA1_Channel4_IRQHandler, .-DMA1_Channel4_IRQHandler
  
  .section .text.DMA1_Channel5_IRQHandler,"ax",%progbits
DMA1_Channel5_IRQHandler:
Infinite_Loop_DMA1_Channel5_IRQHandler:
  b Infinite_Loop_DMA1_Channel5_IRQHandler
  .size DMA1_Channel5_IRQHandler, .-DMA1_Channel5_IRQHandler
  
  .section .text.DMA1_Channel6_IRQHandler,"ax",%progbits
DMA1_Channel6_IRQHandler:
Infinite_Loop_DMA1_Channel6_IRQHandler:
  b Infinite_Loop_DMA1_Channel6_IRQHandler
  .size DMA1_Channel6_IRQHandler, .-DMA1_Channel6_IRQHandler
  
  .section .text.DMA1_Channel7_IRQHandler,"ax",%progbits
DMA1_Channel7_IRQHandler:
Infinite_Loop_DMA1_Channel7_IRQHandler:
  b Infinite_Loop_DMA1_Channel7_IRQHandler
  .size DMA1_Channel7_IRQHandler, .-DMA1_Channel7_IRQHandler
  
  .section .text.ADC1_2_IRQHandler,"ax",%progbits
ADC1_2_IRQHandler:
Infinite_Loop_ADC1_2_IRQHandler:
  b Infinite_Loop_ADC1_2_IRQHandler
  .size ADC1_2_IRQHandler, .-ADC1_2_IRQHandler
  
  .section .text.USB_HP_IRQHandler,"ax",%progbits
USB_HP_IRQHandler:
Infinite_Loop_USB_HP_IRQHandler:
  b Infinite_Loop_USB_HP_IRQHandler
  .size USB_HP_IRQHandler, .-USB_HP_IRQHandler
  
  .section .text.USB_LP_IRQHandler,"ax",%progbits
USB_LP_IRQHandler:
Infinite_Loop_USB_LP_IRQHandler:
  b Infinite_Loop_USB_LP_IRQHandler
  .size USB_LP_IRQHandler, .-USB_LP_IRQHandler
  
  .section .text.FDCAN1_IT0_IRQHandler,"ax",%progbits
FDCAN1_IT0_IRQHandler:
Infinite_Loop_FDCAN1_IT0_IRQHandler:
  b Infinite_Loop_FDCAN1_IT0_IRQHandler
  .size FDCAN1_IT0_IRQHandler, .-FDCAN1_IT0_IRQHandler
  
  .section .text.FDCAN1_IT1_IRQHandler,"ax",%progbits
FDCAN1_IT1_IRQHandler:
Infinite_Loop_FDCAN1_IT1_IRQHandler:
  b Infinite_Loop_FDCAN1_IT1_IRQHandler
  .size FDCAN1_IT1_IRQHandler, .-FDCAN1_IT1_IRQHandler
  
  .section .text.EXTI9_5_IRQHandler,"ax",%progbits
EXTI9_5_IRQHandler:
Infinite_Loop_EXTI9_5_IRQHandler:
  b Infinite_Loop_EXTI9_5_IRQHandler
  .size EXTI9_5_IRQHandler, .-EXTI9_5_IRQHandler
  
  .section .text.TIM1_BRK_TIM15_IRQHandler,"ax",%progbits
TIM1_BRK_TIM15_IRQHandler:
Infinite_Loop_TIM1_BRK_TIM15_IRQHandler:
  b Infinite_Loop_TIM1_BRK_TIM15_IRQHandler
  .size TIM1_BRK_TIM15_IRQHandler, .-TIM1_BRK_TIM15_IRQHandler
  
  .section .text.TIM1_UP_TIM16_IRQHandler,"ax",%progbits
TIM1_UP_TIM16_IRQHandler:
Infinite_Loop_TIM1_UP_TIM16_IRQHandler:
  b Infinite_Loop_TIM1_UP_TIM16_IRQHandler
  .size TIM1_UP_TIM16_IRQHandler, .-TIM1_UP_TIM16_IRQHandler
  
  .section .text.TIM1_TRG_COM_TIM17_IRQHandler,"ax",%progbits
TIM1_TRG_COM_TIM17_IRQHandler:
Infinite_Loop_TIM1_TRG_COM_TIM17_IRQHandler:
  b Infinite_Loop_TIM1_TRG_COM_TIM17_IRQHandler
  .size TIM1_TRG_COM_TIM17_IRQHandler, .-TIM1_TRG_COM_TIM17_IRQHandler
  
  .section .text.TIM1_CC_IRQHandler,"ax",%progbits
TIM1_CC_IRQHandler:
Infinite_Loop_TIM1_CC_IRQHandler:
  b Infinite_Loop_TIM1_CC_IRQHandler
  .size TIM1_CC_IRQHandler, .-TIM1_CC_IRQHandler
  
  .section .text.TIM2_IRQHandler,"ax",%progbits
TIM2_IRQHandler:
Infinite_Loop_TIM2_IRQHandler:
  b Infinite_Loop_TIM2_IRQHandler
  .size TIM2_IRQHandler, .-TIM2_IRQHandler
  
  .section .text.TIM3_IRQHandler,"ax",%progbits
TIM3_IRQHandler:
Infinite_Loop_TIM3_IRQHandler:
  b Infinite_Loop_TIM3_IRQHandler
  .size TIM3_IRQHandler, .-TIM3_IRQHandler
  
  .section .text.TIM4_IRQHandler,"ax",%progbits
TIM4_IRQHandler:
Infinite_Loop_TIM4_IRQHandler:
  b Infinite_Loop_TIM4_IRQHandler
  .size TIM4_IRQHandler, .-TIM4_IRQHandler
  
  .section .text.I2C1_EV_IRQHandler,"ax",%progbits
I2C1_EV_IRQHandler:
Infinite_Loop_I2C1_EV_IRQHandler:
  b Infinite_Loop_I2C1_EV_IRQHandler
  .size I2C1_EV_IRQHandler, .-I2C1_EV_IRQHandler
  
  .section .text.I2C1_ER_IRQHandler,"ax",%progbits
I2C1_ER_IRQHandler:
Infinite_Loop_I2C1_ER_IRQHandler:
  b Infinite_Loop_I2C1_ER_IRQHandler
  .size I2C1_ER_IRQHandler, .-I2C1_ER_IRQHandler
  
  .section .text.I2C2_EV_IRQHandler,"ax",%progbits
I2C2_EV_IRQHandler:
Infinite_Loop_I2C2_EV_IRQHandler:
  b Infinite_Loop_I2C2_EV_IRQHandler
  .size I2C2_EV_IRQHandler, .-I2C2_EV_IRQHandler
  
  .section .text.I2C2_ER_IRQHandler,"ax",%progbits
I2C2_ER_IRQHandler:
Infinite_Loop_I2C2_ER_IRQHandler:
  b Infinite_Loop_I2C2_ER_IRQHandler
  .size I2C2_ER_IRQHandler, .-I2C2_ER_IRQHandler
  
  .section .text.SPI1_IRQHandler,"ax",%progbits
SPI1_IRQHandler:
Infinite_Loop_SPI1_IRQHandler:
  b Infinite_Loop_SPI1_IRQHandler
  .size SPI1_IRQHandler, .-SPI1_IRQHandler
  
  .section .text.SPI2_IRQHandler,"ax",%progbits
SPI2_IRQHandler:
Infinite_Loop_SPI2_IRQHandler:
  b Infinite_Loop_SPI2_IRQHandler
  .size SPI2_IRQHandler, .-SPI2_IRQHandler
  
  .section .text.USART1_IRQHandler,"ax",%progbits
USART1_IRQHandler:
Infinite_Loop_USART1_IRQHandler:
  b Infinite_Loop_USART1_IRQHandler
  .size USART1_IRQHandler, .-USART1_IRQHandler
  
  .section .text.USART2_IRQHandler,"ax",%progbits
USART2_IRQHandler:
Infinite_Loop_USART2_IRQHandler:
  b Infinite_Loop_USART2_IRQHandler
  .size USART2_IRQHandler, .-USART2_IRQHandler
  
  .section .text.USART3_IRQHandler,"ax",%progbits
USART3_IRQHandler:
Infinite_Loop_USART3_IRQHandler:
  b Infinite_Loop_USART3_IRQHandler
  .size USART3_IRQHandler, .-USART3_IRQHandler
  
  .section .text.RTC_Alarm_IRQHandler,"ax",%progbits
RTC_Alarm_IRQHandler:
Infinite_Loop_RTC_Alarm_IRQHandler:
  b Infinite_Loop_RTC_Alarm_IRQHandler
  .size RTC_Alarm_IRQHandler, .-RTC_Alarm_IRQHandler
  
  .section .text.USBWakeUp_IRQHandler,"ax",%progbits
USBWakeUp_IRQHandler:
Infinite_Loop_USBWakeUp_IRQHandler:
  b Infinite_Loop_USBWakeUp_IRQHandler
  .size USBWakeUp_IRQHandler, .-USBWakeUp_IRQHandler
  
  .section .text.TIM8_BRK_IRQHandler,"ax",%progbits
TIM8_BRK_IRQHandler:
Infinite_Loop_TIM8_BRK_IRQHandler:
  b Infinite_Loop_TIM8_BRK_IRQHandler
  .size TIM8_BRK_IRQHandler, .-TIM8_BRK_IRQHandler
  
  .section .text.TIM8_UP_IRQHandler,"ax",%progbits
TIM8_UP_IRQHandler:
Infinite_Loop_TIM8_UP_IRQHandler:
  b Infinite_Loop_TIM8_UP_IRQHandler
  .size TIM8_UP_IRQHandler, .-TIM8_UP_IRQHandler
  
  .section .text.TIM8_TRG_COM_IRQHandler,"ax",%progbits
TIM8_TRG_COM_IRQHandler:
Infinite_Loop_TIM8_TRG_COM_IRQHandler:
  b Infinite_Loop_TIM8_TRG_COM_IRQHandler
  .size TIM8_TRG_COM_IRQHandler, .-TIM8_TRG_COM_IRQHandler
  
  .section .text.TIM8_CC_IRQHandler,"ax",%progbits
TIM8_CC_IRQHandler:
Infinite_Loop_TIM8_CC_IRQHandler:
  b Infinite_Loop_TIM8_CC_IRQHandler
  .size TIM8_CC_IRQHandler, .-TIM8_CC_IRQHandler
  
  .section .text.ADC3_IRQHandler,"ax",%progbits
ADC3_IRQHandler:
Infinite_Loop_ADC3_IRQHandler:
  b Infinite_Loop_ADC3_IRQHandler
  .size ADC3_IRQHandler, .-ADC3_IRQHandler
  
  .section .text.FMC_IRQHandler,"ax",%progbits
FMC_IRQHandler:
Infinite_Loop_FMC_IRQHandler:
  b Infinite_Loop_FMC_IRQHandler
  .size FMC_IRQHandler, .-FMC_IRQHandler
  
  .section .text.LPTIM1_IRQHandler,"ax",%progbits
LPTIM1_IRQHandler:
Infinite_Loop_LPTIM1_IRQHandler:
  b Infinite_Loop_LPTIM1_IRQHandler
  .size LPTIM1_IRQHandler, .-LPTIM1_IRQHandler
  
  .section .text.TIM5_IRQHandler,"ax",%progbits
TIM5_IRQHandler:
Infinite_Loop_TIM5_IRQHandler:
  b Infinite_Loop_TIM5_IRQHandler
  .size TIM5_IRQHandler, .-TIM5_IRQHandler
  
  .section .text.SPI3_IRQHandler,"ax",%progbits
SPI3_IRQHandler:
Infinite_Loop_SPI3_IRQHandler:
  b Infinite_Loop_SPI3_IRQHandler
  .size SPI3_IRQHandler, .-SPI3_IRQHandler
  
  .section .text.UART4_IRQHandler,"ax",%progbits
UART4_IRQHandler:
Infinite_Loop_UART4_IRQHandler:
  b Infinite_Loop_UART4_IRQHandler
  .size UART4_IRQHandler, .-UART4_IRQHandler
  
  .section .text.UART5_IRQHandler,"ax",%progbits
UART5_IRQHandler:
Infinite_Loop_UART5_IRQHandler:
  b Infinite_Loop_UART5_IRQHandler
  .size UART5_IRQHandler, .-UART5_IRQHandler
  
  .section .text.TIM6_DAC_IRQHandler,"ax",%progbits
TIM6_DAC_IRQHandler:
Infinite_Loop_TIM6_DAC_IRQHandler:
  b Infinite_Loop_TIM6_DAC_IRQHandler
  .size TIM6_DAC_IRQHandler, .-TIM6_DAC_IRQHandler
  
  .section .text.TIM7_DAC_IRQHandler,"ax",%progbits
TIM7_DAC_IRQHandler:
Infinite_Loop_TIM7_DAC_IRQHandler:
  b Infinite_Loop_TIM7_DAC_IRQHandler
  .size TIM7_DAC_IRQHandler, .-TIM7_DAC_IRQHandler
  
  .section .text.DMA2_Channel2_IRQHandler,"ax",%progbits
DMA2_Channel2_IRQHandler:
Infinite_Loop_DMA2_Channel2_IRQHandler:
  b Infinite_Loop_DMA2_Channel2_IRQHandler
  .size DMA2_Channel2_IRQHandler, .-DMA2_Channel2_IRQHandler
  
  .section .text.DMA2_Channel1_IRQHandler,"ax",%progbits
DMA2_Channel1_IRQHandler:
Infinite_Loop_DMA2_Channel1_IRQHandler:
  b Infinite_Loop_DMA2_Channel1_IRQHandler
  .size DMA2_Channel1_IRQHandler, .-DMA2_Channel1_IRQHandler
  
  .section .text.DMA2_Channel3_IRQHandler,"ax",%progbits
DMA2_Channel3_IRQHandler:
Infinite_Loop_DMA2_Channel3_IRQHandler:
  b Infinite_Loop_DMA2_Channel3_IRQHandler
  .size DMA2_Channel3_IRQHandler, .-DMA2_Channel3_IRQHandler
  
  .section .text.DMA2_Channel4_IRQHandler,"ax",%progbits
DMA2_Channel4_IRQHandler:
Infinite_Loop_DMA2_Channel4_IRQHandler:
  b Infinite_Loop_DMA2_Channel4_IRQHandler
  .size DMA2_Channel4_IRQHandler, .-DMA2_Channel4_IRQHandler
  
  .section .text.DMA2_Channel5_IRQHandler,"ax",%progbits
DMA2_Channel5_IRQHandler:
Infinite_Loop_DMA2_Channel5_IRQHandler:
  b Infinite_Loop_DMA2_Channel5_IRQHandler
  .size DMA2_Channel5_IRQHandler, .-DMA2_Channel5_IRQHandler
  
  .section .text.ADC4_IRQHandler,"ax",%progbits
ADC4_IRQHandler:
Infinite_Loop_ADC4_IRQHandler:
  b Infinite_Loop_ADC4_IRQHandler
  .size ADC4_IRQHandler, .-ADC4_IRQHandler
  
  .section .text.ADC5_IRQHandler,"ax",%progbits
ADC5_IRQHandler:
Infinite_Loop_ADC5_IRQHandler:
  b Infinite_Loop_ADC5_IRQHandler
  .size ADC5_IRQHandler, .-ADC5_IRQHandler
  
  .section .text.UCPD1_IRQHandler,"ax",%progbits
UCPD1_IRQHandler:
Infinite_Loop_UCPD1_IRQHandler:
  b Infinite_Loop_UCPD1_IRQHandler
  .size UCPD1_IRQHandler, .-UCPD1_IRQHandler
  
  .section .text.COMP1_2_3_IRQHandler,"ax",%progbits
COMP1_2_3_IRQHandler:
Infinite_Loop_COMP1_2_3_IRQHandler:
  b Infinite_Loop_COMP1_2_3_IRQHandler
  .size COMP1_2_3_IRQHandler, .-COMP1_2_3_IRQHandler
  
  .section .text.COMP4_5_6_IRQHandler,"ax",%progbits
COMP4_5_6_IRQHandler:
Infinite_Loop_COMP4_5_6_IRQHandler:
  b Infinite_Loop_COMP4_5_6_IRQHandler
  .size COMP4_5_6_IRQHandler, .-COMP4_5_6_IRQHandler
  
  .section .text.COMP7_IRQHandler,"ax",%progbits
COMP7_IRQHandler:
Infinite_Loop_COMP7_IRQHandler:
  b Infinite_Loop_COMP7_IRQHandler
  .size COMP7_IRQHandler, .-COMP7_IRQHandler
  
  .section .text.HRTIM1_Master_IRQHandler,"ax",%progbits
HRTIM1_Master_IRQHandler:
Infinite_Loop_HRTIM1_Master_IRQHandler:
  b Infinite_Loop_HRTIM1_Master_IRQHandler
  .size HRTIM1_Master_IRQHandler, .-HRTIM1_Master_IRQHandler
  
  .section .text.HRTIM1_TIMA_IRQHandler,"ax",%progbits
HRTIM1_TIMA_IRQHandler:
Infinite_Loop_HRTIM1_TIMA_IRQHandler:
  b Infinite_Loop_HRTIM1_TIMA_IRQHandler
  .size HRTIM1_TIMA_IRQHandler, .-HRTIM1_TIMA_IRQHandler
  
  .section .text.HRTIM1_TIMB_IRQHandler,"ax",%progbits
HRTIM1_TIMB_IRQHandler:
Infinite_Loop_HRTIM1_TIMB_IRQHandler:
  b Infinite_Loop_HRTIM1_TIMB_IRQHandler
  .size HRTIM1_TIMB_IRQHandler, .-HRTIM1_TIMB_IRQHandler
  
  .section .text.HRTIM1_TIMC_IRQHandler,"ax",%progbits
HRTIM1_TIMC_IRQHandler:
Infinite_Loop_HRTIM1_TIMC_IRQHandler:
  b Infinite_Loop_HRTIM1_TIMC_IRQHandler
  .size HRTIM1_TIMC_IRQHandler, .-HRTIM1_TIMC_IRQHandler
  
  .section .text.HRTIM1_TIMD_IRQHandler,"ax",%progbits
HRTIM1_TIMD_IRQHandler:
Infinite_Loop_HRTIM1_TIMD_IRQHandler:
  b Infinite_Loop_HRTIM1_TIMD_IRQHandler
  .size HRTIM1_TIMD_IRQHandler, .-HRTIM1_TIMD_IRQHandler
  
  .section .text.HRTIM1_TIME_IRQHandler,"ax",%progbits
HRTIM1_TIME_IRQHandler:
Infinite_Loop_HRTIM1_TIME_IRQHandler:
  b Infinite_Loop_HRTIM1_TIME_IRQHandler
  .size HRTIM1_TIME_IRQHandler, .-HRTIM1_TIME_IRQHandler
  
  .section .text.HRTIM1_FLT_IRQHandler,"ax",%progbits
HRTIM1_FLT_IRQHandler:
Infinite_Loop_HRTIM1_FLT_IRQHandler:
  b Infinite_Loop_HRTIM1_FLT_IRQHandler
  .size HRTIM1_FLT_IRQHandler, .-HRTIM1_FLT_IRQHandler
  
  .section .text.HRTIM1_TIMF_IRQHandler,"ax",%progbits
HRTIM1_TIMF_IRQHandler:
Infinite_Loop_HRTIM1_TIMF_IRQHandler:
  b Infinite_Loop_HRTIM1_TIMF_IRQHandler
  .size HRTIM1_TIMF_IRQHandler, .-HRTIM1_TIMF_IRQHandler
  
  .section .text.CRS_IRQHandler,"ax",%progbits
CRS_IRQHandler:
Infinite_Loop_CRS_IRQHandler:
  b Infinite_Loop_CRS_IRQHandler
  .size CRS_IRQHandler, .-CRS_IRQHandler
  
  .section .text.SAI1_IRQHandler,"ax",%progbits
SAI1_IRQHandler:
Infinite_Loop_SAI1_IRQHandler:
  b Infinite_Loop_SAI1_IRQHandler
  .size SAI1_IRQHandler, .-SAI1_IRQHandler
  
  .section .text.TIM20_BRK_IRQHandler,"ax",%progbits
TIM20_BRK_IRQHandler:
Infinite_Loop_TIM20_BRK_IRQHandler:
  b Infinite_Loop_TIM20_BRK_IRQHandler
  .size TIM20_BRK_IRQHandler, .-TIM20_BRK_IRQHandler
  
  .section .text.TIM20_UP_IRQHandler,"ax",%progbits
TIM20_UP_IRQHandler:
Infinite_Loop_TIM20_UP_IRQHandler:
  b Infinite_Loop_TIM20_UP_IRQHandler
  .size TIM20_UP_IRQHandler, .-TIM20_UP_IRQHandler
  
  .section .text.TIM20_TRG_COM_IRQHandler,"ax",%progbits
TIM20_TRG_COM_IRQHandler:
Infinite_Loop_TIM20_TRG_COM_IRQHandler:
  b Infinite_Loop_TIM20_TRG_COM_IRQHandler
  .size TIM20_TRG_COM_IRQHandler, .-TIM20_TRG_COM_IRQHandler
  
  .section .text.TIM20_CC_IRQHandler,"ax",%progbits
TIM20_CC_IRQHandler:
Infinite_Loop_TIM20_CC_IRQHandler:
  b Infinite_Loop_TIM20_CC_IRQHandler
  .size TIM20_CC_IRQHandler, .-TIM20_CC_IRQHandler
  
  .section .text.FPU_IRQHandler,"ax",%progbits
FPU_IRQHandler:
Infinite_Loop_FPU_IRQHandler:
  b Infinite_Loop_FPU_IRQHandler
  .size FPU_IRQHandler, .-FPU_IRQHandler
  
  .section .text.I2C4_EV_IRQHandler,"ax",%progbits
I2C4_EV_IRQHandler:
Infinite_Loop_I2C4_EV_IRQHandler:
  b Infinite_Loop_I2C4_EV_IRQHandler
  .size I2C4_EV_IRQHandler, .-I2C4_EV_IRQHandler
  
  .section .text.I2C4_ER_IRQHandler,"ax",%progbits
I2C4_ER_IRQHandler:
Infinite_Loop_I2C4_ER_IRQHandler:
  b Infinite_Loop_I2C4_ER_IRQHandler
  .size I2C4_ER_IRQHandler, .-I2C4_ER_IRQHandler
  
  .section .text.SPI4_IRQHandler,"ax",%progbits
SPI4_IRQHandler:
Infinite_Loop_SPI4_IRQHandler:
  b Infinite_Loop_SPI4_IRQHandler
  .size SPI4_IRQHandler, .-SPI4_IRQHandler
  
  .section .text.FDCAN2_IT0_IRQHandler,"ax",%progbits
FDCAN2_IT0_IRQHandler:
Infinite_Loop_FDCAN2_IT0_IRQHandler:
  b Infinite_Loop_FDCAN2_IT0_IRQHandler
  .size FDCAN2_IT0_IRQHandler, .-FDCAN2_IT0_IRQHandler
  
  .section .text.FDCAN2_IT1_IRQHandler,"ax",%progbits
FDCAN2_IT1_IRQHandler:
Infinite_Loop_FDCAN2_IT1_IRQHandler:
  b Infinite_Loop_FDCAN2_IT1_IRQHandler
  .size FDCAN2_IT1_IRQHandler, .-FDCAN2_IT1_IRQHandler
  
  .section .text.FDCAN3_IT0_IRQHandler,"ax",%progbits
FDCAN3_IT0_IRQHandler:
Infinite_Loop_FDCAN3_IT0_IRQHandler:
  b Infinite_Loop_FDCAN3_IT0_IRQHandler
  .size FDCAN3_IT0_IRQHandler, .-FDCAN3_IT0_IRQHandler
  
  .section .text.FDCAN3_IT1_IRQHandler,"ax",%progbits
FDCAN3_IT1_IRQHandler:
Infinite_Loop_FDCAN3_IT1_IRQHandler:
  b Infinite_Loop_FDCAN3_IT1_IRQHandler
  .size FDCAN3_IT1_IRQHandler, .-FDCAN3_IT1_IRQHandler
  
  .section .text.RNG_IRQHandler,"ax",%progbits
RNG_IRQHandler:
Infinite_Loop_RNG_IRQHandler:
  b Infinite_Loop_RNG_IRQHandler
  .size RNG_IRQHandler, .-RNG_IRQHandler
  
  .section .text.LPUART1_IRQHandler,"ax",%progbits
LPUART1_IRQHandler:
Infinite_Loop_LPUART1_IRQHandler:
  b Infinite_Loop_LPUART1_IRQHandler
  .size LPUART1_IRQHandler, .-LPUART1_IRQHandler
  
  .section .text.I2C3_EV_IRQHandler,"ax",%progbits
I2C3_EV_IRQHandler:
Infinite_Loop_I2C3_EV_IRQHandler:
  b Infinite_Loop_I2C3_EV_IRQHandler
  .size I2C3_EV_IRQHandler, .-I2C3_EV_IRQHandler
  
  .section .text.I2C3_ER_IRQHandler,"ax",%progbits
I2C3_ER_IRQHandler:
Infinite_Loop_I2C3_ER_IRQHandler:
  b Infinite_Loop_I2C3_ER_IRQHandler
  .size I2C3_ER_IRQHandler, .-I2C3_ER_IRQHandler
  
  .section .text.DMAMUX_OVR_IRQHandler,"ax",%progbits
DMAMUX_OVR_IRQHandler:
Infinite_Loop_DMAMUX_OVR_IRQHandler:
  b Infinite_Loop_DMAMUX_OVR_IRQHandler
  .size DMAMUX_OVR_IRQHandler, .-DMAMUX_OVR_IRQHandler
  
  .section .text.QUADSPI_IRQHandler,"ax",%progbits
QUADSPI_IRQHandler:
Infinite_Loop_QUADSPI_IRQHandler:
  b Infinite_Loop_QUADSPI_IRQHandler
  .size QUADSPI_IRQHandler, .-QUADSPI_IRQHandler
  
  .section .text.DMA1_Channel8_IRQHandler,"ax",%progbits
DMA1_Channel8_IRQHandler:
Infinite_Loop_DMA1_Channel8_IRQHandler:
  b Infinite_Loop_DMA1_Channel8_IRQHandler
  .size DMA1_Channel8_IRQHandler, .-DMA1_Channel8_IRQHandler
  
  .section .text.DMA2_Channel6_IRQHandler,"ax",%progbits
DMA2_Channel6_IRQHandler:
Infinite_Loop_DMA2_Channel6_IRQHandler:
  b Infinite_Loop_DMA2_Channel6_IRQHandler
  .size DMA2_Channel6_IRQHandler, .-DMA2_Channel6_IRQHandler
  
  .section .text.DMA2_Channel7_IRQHandler,"ax",%progbits
DMA2_Channel7_IRQHandler:
Infinite_Loop_DMA2_Channel7_IRQHandler:
  b Infinite_Loop_DMA2_Channel7_IRQHandler
  .size DMA2_Channel7_IRQHandler, .-DMA2_Channel7_IRQHandler
  
  .section .text.DMA2_Channel8_IRQHandler,"ax",%progbits
DMA2_Channel8_IRQHandler:
Infinite_Loop_DMA2_Channel8_IRQHandler:
  b Infinite_Loop_DMA2_Channel8_IRQHandler
  .size DMA2_Channel8_IRQHandler, .-DMA2_Channel8_IRQHandler
  
  .section .text.CORDIC_IRQHandler,"ax",%progbits
CORDIC_IRQHandler:
Infinite_Loop_CORDIC_IRQHandler:
  b Infinite_Loop_CORDIC_IRQHandler
  .size CORDIC_IRQHandler, .-CORDIC_IRQHandler
  
  .section .text.FMAC_IRQHandler,"ax",%progbits
FMAC_IRQHandler:
Infinite_Loop_FMAC_IRQHandler:
  b Infinite_Loop_FMAC_IRQHandler
  .size FMAC_IRQHandler, .-FMAC_IRQHandler

  
  
/******************************************************************************
*
* The minimal vector table for a Cortex-M4.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
 	.section	.isr_vector,"a",%progbits
	.type	g_pfnVectors_b, %object
	.size	g_pfnVectors_b, .-g_pfnVectors_b

/*
g_pfnVectors_mod:
	.word	_estack
	.word	Reset_Handler
	.word	NMI_Handler
	.word	Default_Handler_IRQ  // HardFault_Handler
	.word	Default_Handler_IRQ  // MemManage_Handler
	.word	Default_Handler_IRQ  // BusFault_Handler
	.word	Default_Handler_IRQ  // UsageFault_Handler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	Default_Handler_IRQ  // SVC_Handler
	.word	Default_Handler_IRQ  // DebugMon_Handler
	.word	0
	.word	Default_Handler_IRQ  // PendSV_Handler
	.word	isr_systick       // use routine from init.c
	.word	Default_Handler_IRQ  // WWDG_IRQHandler
	.word	Default_Handler_IRQ  // PVD_PVM_IRQHandler
	.word	Default_Handler_IRQ  // RTC_TAMP_LSECSS_IRQHandler
	.word	Default_Handler_IRQ  // RTC_WKUP_IRQHandler
	.word	Default_Handler_IRQ  // FLASH_IRQHandler
	.word	Default_Handler_IRQ  // RCC_IRQHandler
	.word	Default_Handler_IRQ  // EXTI0_IRQHandler
	.word	Default_Handler_IRQ  // EXTI1_IRQHandler
	.word	Default_Handler_IRQ  // EXTI2_IRQHandler
	.word	Default_Handler_IRQ  // EXTI3_IRQHandler
	.word	Default_Handler_IRQ  // EXTI4_IRQHandler
	.word	Default_Handler_IRQ  // DMA1_Channel1_IRQHandler
	.word	Default_Handler_IRQ  // DMA1_Channel2_IRQHandler
	.word	Default_Handler_IRQ  // DMA1_Channel3_IRQHandler
	.word	Default_Handler_IRQ  // DMA1_Channel4_IRQHandler
	.word	Default_Handler_IRQ  // DMA1_Channel5_IRQHandler
	.word	Default_Handler_IRQ  // DMA1_Channel6_IRQHandler
	.word	Default_Handler_IRQ  // DMA1_Channel7_IRQHandler
	.word	Default_Handler_IRQ  // ADC1_2_IRQHandler
	.word	Default_Handler_IRQ  // USB_HP_IRQHandler
	.word	Default_Handler_IRQ  // USB_LP_IRQHandler
	.word	Default_Handler_IRQ  // FDCAN1_IT0_IRQHandler
	.word	Default_Handler_IRQ  // FDCAN1_IT1_IRQHandler
	.word	Default_Handler_IRQ  // EXTI9_5_IRQHandler
	.word	Default_Handler_IRQ  // TIM1_BRK_TIM15_IRQHandler
	.word	Default_Handler_IRQ  // TIM1_UP_TIM16_IRQHandler
	.word	Default_Handler_IRQ  // TIM1_TRG_COM_TIM17_IRQHandler
	.word	Default_Handler_IRQ  // TIM1_CC_IRQHandler
	.word	Default_Handler_IRQ  // TIM2_IRQHandler
	.word	Default_Handler_IRQ  // TIM3_IRQHandler
	.word	Default_Handler_IRQ  // TIM4_IRQHandler
	.word	Default_Handler_IRQ  // I2C1_EV_IRQHandler
	.word	Default_Handler_IRQ  // I2C1_ER_IRQHandler
	.word	Default_Handler_IRQ  // I2C2_EV_IRQHandler
	.word	Default_Handler_IRQ  // I2C2_ER_IRQHandler
	.word	Default_Handler_IRQ  // SPI1_IRQHandler
	.word	Default_Handler_IRQ  // SPI2_IRQHandler
	.word	Default_Handler_IRQ  // USART1_IRQHandler
	.word	Default_Handler_IRQ  // USART2_IRQHandler
	.word	Default_Handler_IRQ  // USART3_IRQHandler
	.word	Default_Handler_IRQ  // EXTI15_10_IRQHandler
	.word	Default_Handler_IRQ  // RTC_Alarm_IRQHandler
	.word	Default_Handler_IRQ  // USBWakeUp_IRQHandler
	.word	Default_Handler_IRQ  // TIM8_BRK_IRQHandler
	.word	Default_Handler_IRQ  // TIM8_UP_IRQHandler
	.word	Default_Handler_IRQ  // TIM8_TRG_COM_IRQHandler
	.word	Default_Handler_IRQ  // TIM8_CC_IRQHandler
	.word	Default_Handler_IRQ  // ADC3_IRQHandler
	.word	Default_Handler_IRQ  // FMC_IRQHandler
	.word	Default_Handler_IRQ  // LPTIM1_IRQHandler
	.word	Default_Handler_IRQ  // TIM5_IRQHandler
	.word	Default_Handler_IRQ  // SPI3_IRQHandler
	.word	Default_Handler_IRQ  // UART4_IRQHandler
	.word	Default_Handler_IRQ  // UART5_IRQHandler
	.word	Default_Handler_IRQ  // TIM6_DAC_IRQHandler
	.word	Default_Handler_IRQ  // TIM7_DAC_IRQHandler
	.word	Default_Handler_IRQ  // DMA2_Channel1_IRQHandler
	.word	Default_Handler_IRQ  // DMA2_Channel2_IRQHandler
	.word	Default_Handler_IRQ  // DMA2_Channel3_IRQHandler
	.word	Default_Handler_IRQ  // DMA2_Channel4_IRQHandler
	.word	Default_Handler_IRQ  // DMA2_Channel5_IRQHandler
	.word	Default_Handler_IRQ  // ADC4_IRQHandler
	.word	Default_Handler_IRQ  // ADC5_IRQHandler
	.word	Default_Handler_IRQ  // UCPD1_IRQHandler
	.word	Default_Handler_IRQ  // COMP1_2_3_IRQHandler
	.word	Default_Handler_IRQ  // COMP4_5_6_IRQHandler
	.word	Default_Handler_IRQ  // COMP7_IRQHandler
	.word	Default_Handler_IRQ  // HRTIM1_Master_IRQHandler
	.word	Default_Handler_IRQ  // HRTIM1_TIMA_IRQHandler
	.word	Default_Handler_IRQ  // HRTIM1_TIMB_IRQHandler
	.word	Default_Handler_IRQ  // HRTIM1_TIMC_IRQHandler
	.word	Default_Handler_IRQ  // HRTIM1_TIMD_IRQHandler
	.word	Default_Handler_IRQ  // HRTIM1_TIME_IRQHandler
	.word	Default_Handler_IRQ  // HRTIM1_FLT_IRQHandler
	.word	Default_Handler_IRQ  // HRTIM1_TIMF_IRQHandler
	.word	Default_Handler_IRQ  // CRS_IRQHandler
	.word	Default_Handler_IRQ  // SAI1_IRQHandler
	.word	Default_Handler_IRQ  // TIM20_BRK_IRQHandler
	.word	Default_Handler_IRQ  // TIM20_UP_IRQHandler
	.word	Default_Handler_IRQ  // TIM20_TRG_COM_IRQHandler
	.word	Default_Handler_IRQ  // TIM20_CC_IRQHandler
	.word	Default_Handler_IRQ  // FPU_IRQHandler
	.word	Default_Handler_IRQ  // I2C4_EV_IRQHandler
	.word	Default_Handler_IRQ  // I2C4_ER_IRQHandler
	.word	Default_Handler_IRQ  // SPI4_IRQHandler
	.word	0
	.word	Default_Handler_IRQ  // FDCAN2_IT0_IRQHandler
	.word	Default_Handler_IRQ  // FDCAN2_IT1_IRQHandler
	.word	Default_Handler_IRQ  // FDCAN3_IT0_IRQHandler
	.word	Default_Handler_IRQ  // FDCAN3_IT1_IRQHandler
	.word	Default_Handler_IRQ  // RNG_IRQHandler
	.word	Default_Handler_IRQ  // LPUART1_IRQHandler
	.word	Default_Handler_IRQ  // I2C3_EV_IRQHandler
	.word	Default_Handler_IRQ  // I2C3_ER_IRQHandler
	.word	Default_Handler_IRQ  // DMAMUX_OVR_IRQHandler
	.word	Default_Handler_IRQ  // QUADSPI_IRQHandler
	.word	Default_Handler_IRQ  // DMA1_Channel8_IRQHandler
	.word	Default_Handler_IRQ  // DMA2_Channel6_IRQHandler
	.word	Default_Handler_IRQ  // DMA2_Channel7_IRQHandler
	.word	Default_Handler_IRQ  // DMA2_Channel8_IRQHandler
	.word	Default_Handler_IRQ  // CORDIC_IRQHandler
	.word	Default_Handler_IRQ  // FMAC_IRQHandler
*/
 
/*  original vector table */
g_pfnVectors_b:
	.word	_estack
	.word	Reset_Handler
	.word	NMI_Handler
	.word	HardFault_Handler
	.word	MemManage_Handler
	.word	BusFault_Handler
	.word	UsageFault_Handler
	.word	0
	.word	0
	.word	0
	.word	0
	.word	SVC_Handler
	.word	DebugMon_Handler
	.word	0
	.word	PendSV_Handler
	.word	isr_systick       // use routine from init.c
	.word	WWDG_IRQHandler
	.word	PVD_PVM_IRQHandler
	.word	RTC_TAMP_LSECSS_IRQHandler
	.word	RTC_WKUP_IRQHandler
	.word	FLASH_IRQHandler
	.word	RCC_IRQHandler
	.word	EXTI0_IRQHandler
	.word	EXTI1_IRQHandler
	.word	EXTI2_IRQHandler
	.word	EXTI3_IRQHandler
	.word	EXTI4_IRQHandler
	.word	DMA1_Channel1_IRQHandler
	.word	DMA1_Channel2_IRQHandler
	.word	DMA1_Channel3_IRQHandler
	.word	DMA1_Channel4_IRQHandler
	.word	DMA1_Channel5_IRQHandler
	.word	DMA1_Channel6_IRQHandler
	.word	DMA1_Channel7_IRQHandler
	.word	ADC1_2_IRQHandler
	.word	USB_HP_IRQHandler
	.word	USB_LP_IRQHandler
	.word	FDCAN1_IT0_IRQHandler
	.word	FDCAN1_IT1_IRQHandler
	.word	EXTI9_5_IRQHandler
	.word	TIM1_BRK_TIM15_IRQHandler
	.word	TIM1_UP_TIM16_IRQHandler
	.word	TIM1_TRG_COM_TIM17_IRQHandler
	.word	TIM1_CC_IRQHandler
	.word	TIM2_IRQHandler
	.word	TIM3_IRQHandler
	.word	TIM4_IRQHandler
	.word	I2C1_EV_IRQHandler
	.word	I2C1_ER_IRQHandler
	.word	I2C2_EV_IRQHandler
	.word	I2C2_ER_IRQHandler
	.word	SPI1_IRQHandler
	.word	SPI2_IRQHandler
	.word	USART1_IRQHandler
	.word	USART2_IRQHandler
	.word	USART3_IRQHandler
	.word	USART3_IRQHandler
	.word	RTC_Alarm_IRQHandler
	.word	USBWakeUp_IRQHandler
	.word	TIM8_BRK_IRQHandler
	.word	TIM8_UP_IRQHandler
	.word	TIM8_TRG_COM_IRQHandler
	.word	TIM8_CC_IRQHandler
	.word	ADC3_IRQHandler
	.word	FMC_IRQHandler
	.word	LPTIM1_IRQHandler
	.word	TIM5_IRQHandler
	.word	SPI3_IRQHandler
	.word	UART4_IRQHandler
	.word	UART5_IRQHandler
	.word	TIM6_DAC_IRQHandler
	.word	TIM7_DAC_IRQHandler
	.word	DMA2_Channel1_IRQHandler
	.word	DMA2_Channel2_IRQHandler
	.word	DMA2_Channel3_IRQHandler
	.word	DMA2_Channel4_IRQHandler
	.word	DMA2_Channel5_IRQHandler
	.word	ADC4_IRQHandler
	.word	ADC5_IRQHandler
	.word	UCPD1_IRQHandler
	.word	COMP1_2_3_IRQHandler
	.word	COMP4_5_6_IRQHandler
	.word	COMP7_IRQHandler
	.word	HRTIM1_Master_IRQHandler
	.word	HRTIM1_TIMA_IRQHandler
	.word	HRTIM1_TIMB_IRQHandler
	.word	HRTIM1_TIMC_IRQHandler
	.word	HRTIM1_TIMD_IRQHandler
	.word	HRTIM1_TIME_IRQHandler
	.word	HRTIM1_FLT_IRQHandler
	.word	HRTIM1_TIMF_IRQHandler
	.word	CRS_IRQHandler
	.word	SAI1_IRQHandler
	.word	TIM20_BRK_IRQHandler
	.word	TIM20_UP_IRQHandler
	.word	TIM20_TRG_COM_IRQHandler
	.word	TIM20_CC_IRQHandler
	.word	FPU_IRQHandler
	.word	I2C4_EV_IRQHandler
	.word	I2C4_ER_IRQHandler
	.word	SPI4_IRQHandler
	.word	0
	.word	FDCAN2_IT0_IRQHandler
	.word	FDCAN2_IT1_IRQHandler
	.word	FDCAN3_IT0_IRQHandler
	.word	FDCAN3_IT1_IRQHandler
	.word	RNG_IRQHandler
	.word	LPUART1_IRQHandler
	.word	I2C3_EV_IRQHandler
	.word	I2C3_ER_IRQHandler
	.word	DMAMUX_OVR_IRQHandler
	.word	QUADSPI_IRQHandler
	.word	DMA1_Channel8_IRQHandler
	.word	DMA2_Channel6_IRQHandler
	.word	DMA2_Channel7_IRQHandler
	.word	DMA2_Channel8_IRQHandler
	.word	CORDIC_IRQHandler
	.word	FMAC_IRQHandler
 

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler_IRQ.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/


/*
  .weak	NMI_Handler
	.thumb_set NMI_Handler,Default_Handler_IRQ

	.weak	HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler_IRQ

	.weak	MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler_IRQ

	.weak	BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler_IRQ

	.weak	UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler_IRQ

	.weak	SVC_Handler
	.thumb_set SVC_Handler,Default_Handler_IRQ

	.weak	DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler_IRQ

	.weak	PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler_IRQ

	.weak	SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler_IRQ

	.weak	WWDG_IRQHandler
	.thumb_set WWDG_IRQHandler,Default_Handler_IRQ

	.weak	PVD_PVM_IRQHandler
	.thumb_set PVD_PVM_IRQHandler,Default_Handler_IRQ

	.weak	RTC_TAMP_LSECSS_IRQHandler
	.thumb_set RTC_TAMP_LSECSS_IRQHandler,Default_Handler_IRQ

	.weak	RTC_WKUP_IRQHandler
	.thumb_set RTC_WKUP_IRQHandler,Default_Handler_IRQ

	.weak	FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,Default_Handler_IRQ

	.weak	RCC_IRQHandler
	.thumb_set RCC_IRQHandler,Default_Handler_IRQ

	.weak	EXTI0_IRQHandler
	.thumb_set EXTI0_IRQHandler,Default_Handler_IRQ

	.weak	EXTI1_IRQHandler
	.thumb_set EXTI1_IRQHandler,Default_Handler_IRQ

	.weak	EXTI2_IRQHandler
	.thumb_set EXTI2_IRQHandler,Default_Handler_IRQ

	.weak	EXTI3_IRQHandler
	.thumb_set EXTI3_IRQHandler,Default_Handler_IRQ

	.weak	EXTI4_IRQHandler
	.thumb_set EXTI4_IRQHandler,Default_Handler_IRQ

	.weak	DMA1_Channel1_IRQHandler
	.thumb_set DMA1_Channel1_IRQHandler,Default_Handler_IRQ

	.weak	DMA1_Channel2_IRQHandler
	.thumb_set DMA1_Channel2_IRQHandler,Default_Handler_IRQ

	.weak	DMA1_Channel3_IRQHandler
	.thumb_set DMA1_Channel3_IRQHandler,Default_Handler_IRQ

	.weak	DMA1_Channel4_IRQHandler
	.thumb_set DMA1_Channel4_IRQHandler,Default_Handler_IRQ

	.weak	DMA1_Channel5_IRQHandler
	.thumb_set DMA1_Channel5_IRQHandler,Default_Handler_IRQ

	.weak	DMA1_Channel6_IRQHandler
	.thumb_set DMA1_Channel6_IRQHandler,Default_Handler_IRQ

	.weak	DMA1_Channel7_IRQHandler
	.thumb_set DMA1_Channel7_IRQHandler,Default_Handler_IRQ

	.weak	ADC1_2_IRQHandler
	.thumb_set ADC1_2_IRQHandler,Default_Handler_IRQ

	.weak	USB_HP_IRQHandler
	.thumb_set USB_HP_IRQHandler,Default_Handler_IRQ

	.weak	USB_LP_IRQHandler
	.thumb_set USB_LP_IRQHandler,Default_Handler_IRQ

	.weak	FDCAN1_IT0_IRQHandler
	.thumb_set FDCAN1_IT0_IRQHandler,Default_Handler_IRQ

	.weak	FDCAN1_IT1_IRQHandler
	.thumb_set FDCAN1_IT1_IRQHandler,Default_Handler_IRQ

	.weak	EXTI9_5_IRQHandler
	.thumb_set EXTI9_5_IRQHandler,Default_Handler_IRQ

	.weak	TIM1_BRK_TIM15_IRQHandler
	.thumb_set TIM1_BRK_TIM15_IRQHandler,Default_Handler_IRQ

	.weak	TIM1_UP_TIM16_IRQHandler
	.thumb_set TIM1_UP_TIM16_IRQHandler,Default_Handler_IRQ

	.weak	TIM1_TRG_COM_TIM17_IRQHandler
	.thumb_set TIM1_TRG_COM_TIM17_IRQHandler,Default_Handler_IRQ

	.weak	TIM1_CC_IRQHandler
	.thumb_set TIM1_CC_IRQHandler,Default_Handler_IRQ

	.weak	TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler_IRQ

	.weak	TIM3_IRQHandler
	.thumb_set TIM3_IRQHandler,Default_Handler_IRQ

	.weak	TIM4_IRQHandler
	.thumb_set TIM4_IRQHandler,Default_Handler_IRQ

	.weak	I2C1_EV_IRQHandler
	.thumb_set I2C1_EV_IRQHandler,Default_Handler_IRQ

	.weak	I2C1_ER_IRQHandler
	.thumb_set I2C1_ER_IRQHandler,Default_Handler_IRQ

	.weak	I2C2_EV_IRQHandler
	.thumb_set I2C2_EV_IRQHandler,Default_Handler_IRQ

	.weak	I2C2_ER_IRQHandler
	.thumb_set I2C2_ER_IRQHandler,Default_Handler_IRQ

	.weak	SPI1_IRQHandler
	.thumb_set SPI1_IRQHandler,Default_Handler_IRQ

	.weak	SPI2_IRQHandler
	.thumb_set SPI2_IRQHandler,Default_Handler_IRQ

	.weak	USART1_IRQHandler
	.thumb_set USART1_IRQHandler,Default_Handler_IRQ

	.weak	USART2_IRQHandler
	.thumb_set USART2_IRQHandler,Default_Handler_IRQ

	.weak	USART3_IRQHandler
	.thumb_set USART3_IRQHandler,Default_Handler_IRQ

	.weak	EXTI15_10_IRQHandler
	.thumb_set EXTI15_10_IRQHandler,Default_Handler_IRQ

	.weak	RTC_Alarm_IRQHandler
	.thumb_set RTC_Alarm_IRQHandler,Default_Handler_IRQ

	.weak	USBWakeUp_IRQHandler
	.thumb_set USBWakeUp_IRQHandler,Default_Handler_IRQ

	.weak	TIM8_BRK_IRQHandler
	.thumb_set TIM8_BRK_IRQHandler,Default_Handler_IRQ

	.weak	TIM8_UP_IRQHandler
	.thumb_set TIM8_UP_IRQHandler,Default_Handler_IRQ

	.weak	TIM8_TRG_COM_IRQHandler
	.thumb_set TIM8_TRG_COM_IRQHandler,Default_Handler_IRQ

	.weak	TIM8_CC_IRQHandler
	.thumb_set TIM8_CC_IRQHandler,Default_Handler_IRQ

	.weak	ADC3_IRQHandler
	.thumb_set ADC3_IRQHandler,Default_Handler_IRQ

	.weak	FMC_IRQHandler
	.thumb_set FMC_IRQHandler,Default_Handler_IRQ

	.weak	LPTIM1_IRQHandler
	.thumb_set LPTIM1_IRQHandler,Default_Handler_IRQ

	.weak	TIM5_IRQHandler
	.thumb_set TIM5_IRQHandler,Default_Handler_IRQ

	.weak	SPI3_IRQHandler
	.thumb_set SPI3_IRQHandler,Default_Handler_IRQ

	.weak	UART4_IRQHandler
	.thumb_set UART4_IRQHandler,Default_Handler_IRQ

	.weak	UART5_IRQHandler
	.thumb_set UART5_IRQHandler,Default_Handler_IRQ

	.weak	TIM6_DAC_IRQHandler
	.thumb_set TIM6_DAC_IRQHandler,Default_Handler_IRQ

	.weak	TIM7_DAC_IRQHandler
	.thumb_set TIM7_DAC_IRQHandler,Default_Handler_IRQ

	.weak	DMA2_Channel1_IRQHandler
	.thumb_set DMA2_Channel1_IRQHandler,Default_Handler_IRQ

	.weak	DMA2_Channel2_IRQHandler
	.thumb_set DMA2_Channel2_IRQHandler,Default_Handler_IRQ

	.weak	DMA2_Channel3_IRQHandler
	.thumb_set DMA2_Channel3_IRQHandler,Default_Handler_IRQ

	.weak	DMA2_Channel4_IRQHandler
	.thumb_set DMA2_Channel4_IRQHandler,Default_Handler_IRQ

	.weak	DMA2_Channel5_IRQHandler
	.thumb_set DMA2_Channel5_IRQHandler,Default_Handler_IRQ

	.weak	ADC4_IRQHandler
	.thumb_set ADC4_IRQHandler,Default_Handler_IRQ

	.weak	ADC5_IRQHandler
	.thumb_set ADC5_IRQHandler,Default_Handler_IRQ

	.weak	UCPD1_IRQHandler
	.thumb_set UCPD1_IRQHandler,Default_Handler_IRQ

	.weak	COMP1_2_3_IRQHandler
	.thumb_set COMP1_2_3_IRQHandler,Default_Handler_IRQ

	.weak	COMP4_5_6_IRQHandler
	.thumb_set COMP4_5_6_IRQHandler,Default_Handler_IRQ

	.weak	COMP7_IRQHandler
	.thumb_set COMP7_IRQHandler,Default_Handler_IRQ

	.weak	HRTIM1_Master_IRQHandler
	.thumb_set HRTIM1_Master_IRQHandler,Default_Handler_IRQ

	.weak	HRTIM1_TIMA_IRQHandler
	.thumb_set HRTIM1_TIMA_IRQHandler,Default_Handler_IRQ

	.weak	HRTIM1_TIMB_IRQHandler
	.thumb_set HRTIM1_TIMB_IRQHandler,Default_Handler_IRQ

	.weak	HRTIM1_TIMC_IRQHandler
	.thumb_set HRTIM1_TIMC_IRQHandler,Default_Handler_IRQ

	.weak	HRTIM1_TIMD_IRQHandler
	.thumb_set HRTIM1_TIMD_IRQHandler,Default_Handler_IRQ

	.weak	HRTIM1_TIME_IRQHandler
	.thumb_set HRTIM1_TIME_IRQHandler,Default_Handler_IRQ

	.weak	HRTIM1_FLT_IRQHandler
	.thumb_set HRTIM1_FLT_IRQHandler,Default_Handler_IRQ

	.weak	HRTIM1_TIMF_IRQHandler
	.thumb_set HRTIM1_TIMF_IRQHandler,Default_Handler_IRQ

	.weak	CRS_IRQHandler
	.thumb_set CRS_IRQHandler,Default_Handler_IRQ

	.weak	SAI1_IRQHandler
	.thumb_set SAI1_IRQHandler,Default_Handler_IRQ

	.weak	TIM20_BRK_IRQHandler
	.thumb_set TIM20_BRK_IRQHandler,Default_Handler_IRQ

	.weak	TIM20_UP_IRQHandler
	.thumb_set TIM20_UP_IRQHandler,Default_Handler_IRQ

	.weak	TIM20_TRG_COM_IRQHandler
	.thumb_set TIM20_TRG_COM_IRQHandler,Default_Handler_IRQ

	.weak	TIM20_CC_IRQHandler
	.thumb_set TIM20_CC_IRQHandler,Default_Handler_IRQ

	.weak	FPU_IRQHandler
	.thumb_set FPU_IRQHandler,Default_Handler_IRQ

	.weak	I2C4_EV_IRQHandler
	.thumb_set I2C4_EV_IRQHandler,Default_Handler_IRQ

	.weak	I2C4_ER_IRQHandler
	.thumb_set I2C4_ER_IRQHandler,Default_Handler_IRQ

	.weak	SPI4_IRQHandler
	.thumb_set SPI4_IRQHandler,Default_Handler_IRQ

	.weak	FDCAN2_IT0_IRQHandler
	.thumb_set FDCAN2_IT0_IRQHandler,Default_Handler_IRQ

	.weak	FDCAN2_IT1_IRQHandler
	.thumb_set FDCAN2_IT1_IRQHandler,Default_Handler_IRQ

	.weak	FDCAN3_IT0_IRQHandler
	.thumb_set FDCAN3_IT0_IRQHandler,Default_Handler_IRQ

	.weak	FDCAN3_IT1_IRQHandler
	.thumb_set FDCAN3_IT1_IRQHandler,Default_Handler_IRQ

	.weak	RNG_IRQHandler
	.thumb_set RNG_IRQHandler,Default_Handler_IRQ

	.weak	LPUART1_IRQHandler
	.thumb_set LPUART1_IRQHandler,Default_Handler_IRQ

	.weak	I2C3_EV_IRQHandler
	.thumb_set I2C3_EV_IRQHandler,Default_Handler_IRQ

	.weak	I2C3_ER_IRQHandler
	.thumb_set I2C3_ER_IRQHandler,Default_Handler_IRQ

	.weak	DMAMUX_OVR_IRQHandler
	.thumb_set DMAMUX_OVR_IRQHandler,Default_Handler_IRQ

	.weak	QUADSPI_IRQHandler
	.thumb_set QUADSPI_IRQHandler,Default_Handler_IRQ

	.weak	DMA1_Channel8_IRQHandler
	.thumb_set DMA1_Channel8_IRQHandler,Default_Handler_IRQ

	.weak	DMA2_Channel6_IRQHandler
	.thumb_set DMA2_Channel6_IRQHandler,Default_Handler_IRQ

	.weak	DMA2_Channel7_IRQHandler
	.thumb_set DMA2_Channel7_IRQHandler,Default_Handler_IRQ

	.weak	DMA2_Channel8_IRQHandler
	.thumb_set DMA2_Channel8_IRQHandler,Default_Handler_IRQ

	.weak	CORDIC_IRQHandler
	.thumb_set CORDIC_IRQHandler,Default_Handler_IRQ

	.weak	FMAC_IRQHandler
	.thumb_set FMAC_IRQHandler,Default_Handler_IRQ

*/