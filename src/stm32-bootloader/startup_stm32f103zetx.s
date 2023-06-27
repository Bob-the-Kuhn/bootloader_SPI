/**
  *************** (C) COPYRIGHT 2017 STMicroelectronics ************************
  * @file      startup_stm32f103xe.s
  * @author    MCD Application Team
  * @brief     STM32F103xE Devices vector table for Atollic toolchain.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Configure the clock system   
  *                - Configure external SRAM mounted on STM3210E-EVAL board
  *                  to be used as data memory (optional, to be enabled by user)
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M3 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
  .cpu cortex-m3
  .fpu softvfp
  .thumb

.global g_pfnVectors
.global Default_Handler

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

      LDR R0, = 0                  //  default to boot application
      LDR R1, = Magic_Location
      LDR R1, [R1, #0]                  // read magic value
      CMP R1, #0
      BEQ SKIP                          // if 0 then have not yet been through the bootloader
      LDR R2, = MagicApplication
      CMP R1, #0
      BEQ SKIP                          // if 0 then have not yet been through the bootloader
      LDR R2, [R2, #0]
      CMP R1, R2
      BNE SKIP                     
      LDR R1, = Magic_Location
      STR R0, [R1, #0]             // force next boot to go thru the bootloader
      LDR R0, = APP_ADDR           // point this boot at application
      LDR R0, [R0, #0]
      LDR R1, =0xE000ED00   // SCB
	    STR R0, [R1, #8]      // VTOR
      LDR SP, [R0, #0]      // set SP from application vector table
      LDR R0, [R0, #4]      // get application reset vector address
      DSB                   // Ensure the VTOR and SP operations are complete
      ISB                   // Flush the pipeline because of SP change
      BX   R0               // Start the application - mimic reset (jump to application reset handler)
 SKIP:

/* end code adopted from the post by Piranha */

/* code copied from the startup_stm32f407xx.s file */

  ldr   sp, =_estack     /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */  
  movs  r1, #0
  b  LoopCopyDataInit

CopyDataInit:
  ldr  r3, =_sidata
  ldr  r3, [r3, r1]
  str  r3, [r0, r1]
  adds  r1, r1, #4
    
LoopCopyDataInit:
  ldr  r0, =_sdata
  ldr  r3, =_edata
  adds  r2, r0, r1
  cmp  r2, r3
  bcc  CopyDataInit
  ldr  r2, =_sbss
  b  LoopFillZerobss
/* Zero fill the bss segment. */  
FillZerobss:
  movs  r3, #0
  str  r3, [r2], #4
    
LoopFillZerobss:
  ldr  r3, = _ebss
  cmp  r2, r3
  bcc  FillZerobss

/* Call the clock system intitialization function.*/
/*   bl  SystemInit   */
/* Call static constructors */
/*     bl __libc_init_array */
/* Call the application's entry point.*/
/*  bx  init */
      LDR R0, =0x08000000   // point at bootloader vector table
      LDR SP, [R0, #0]      // set SP from bootloader vector table
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
    .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler
/******************************************************************************
*
* The minimal vector table for a Cortex M3.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors


g_pfnVectors:

  .word _estack
  .word Reset_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SysTick_Handler
  .word SysTick_Handler
  .word 0
  .word SysTick_Handler
  .word SysTick_Handler
  .word WWDG_IRQHandler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word SysTick_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word BootRAM       /* @0x1E0. This is for boot in RAM mode for
                         STM32F10x High Density devices. */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  .weak NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak MemManage_Handler
  .thumb_set MemManage_Handler,Default_Handler

  .weak BusFault_Handler
  .thumb_set BusFault_Handler,Default_Handler

  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler,Default_Handler

  .weak SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler

  .weak PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

  .weak WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler,Default_Handler

  .weak PVD_IRQHandler
  .thumb_set PVD_IRQHandler,Default_Handler

  .weak TAMPER_IRQHandler
  .thumb_set TAMPER_IRQHandler,Default_Handler

  .weak RTC_IRQHandler
  .thumb_set RTC_IRQHandler,Default_Handler

  .weak FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler,Default_Handler

  .weak RCC_IRQHandler
  .thumb_set RCC_IRQHandler,Default_Handler

  .weak EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler,Default_Handler

  .weak EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler,Default_Handler

  .weak EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler,Default_Handler

  .weak EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler,Default_Handler

  .weak EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler,Default_Handler

  .weak DMA1_Channel1_IRQHandler
  .thumb_set DMA1_Channel1_IRQHandler,Default_Handler

  .weak DMA1_Channel2_IRQHandler
  .thumb_set DMA1_Channel2_IRQHandler,Default_Handler

  .weak DMA1_Channel3_IRQHandler
  .thumb_set DMA1_Channel3_IRQHandler,Default_Handler

  .weak DMA1_Channel4_IRQHandler
  .thumb_set DMA1_Channel4_IRQHandler,Default_Handler

  .weak DMA1_Channel5_IRQHandler
  .thumb_set DMA1_Channel5_IRQHandler,Default_Handler

  .weak DMA1_Channel6_IRQHandler
  .thumb_set DMA1_Channel6_IRQHandler,Default_Handler

  .weak DMA1_Channel7_IRQHandler
  .thumb_set DMA1_Channel7_IRQHandler,Default_Handler

  .weak ADC1_2_IRQHandler
  .thumb_set ADC1_2_IRQHandler,Default_Handler

  .weak USB_HP_CAN1_TX_IRQHandler
  .thumb_set USB_HP_CAN1_TX_IRQHandler,Default_Handler

  .weak USB_LP_CAN1_RX0_IRQHandler
  .thumb_set USB_LP_CAN1_RX0_IRQHandler,Default_Handler

  .weak CAN1_RX1_IRQHandler
  .thumb_set CAN1_RX1_IRQHandler,Default_Handler

  .weak CAN1_SCE_IRQHandler
  .thumb_set CAN1_SCE_IRQHandler,Default_Handler

  .weak EXTI9_5_IRQHandler
  .thumb_set EXTI9_5_IRQHandler,Default_Handler

  .weak TIM1_BRK_IRQHandler
  .thumb_set TIM1_BRK_IRQHandler,Default_Handler

  .weak TIM1_UP_IRQHandler
  .thumb_set TIM1_UP_IRQHandler,Default_Handler

  .weak TIM1_TRG_COM_IRQHandler
  .thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler

  .weak TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler,Default_Handler

  .weak TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler,Default_Handler

  .weak TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler,Default_Handler

  .weak TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler,Default_Handler

  .weak I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler,Default_Handler

  .weak I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler,Default_Handler

  .weak I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler,Default_Handler

  .weak I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler,Default_Handler

  .weak SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler,Default_Handler

  .weak USART1_IRQHandler
  .thumb_set USART1_IRQHandler,Default_Handler

  .weak USART2_IRQHandler
  .thumb_set USART2_IRQHandler,Default_Handler

  .weak USART3_IRQHandler
  .thumb_set USART3_IRQHandler,Default_Handler

  .weak EXTI15_10_IRQHandler
  .thumb_set EXTI15_10_IRQHandler,Default_Handler

  .weak RTC_Alarm_IRQHandler
  .thumb_set RTC_Alarm_IRQHandler,Default_Handler

  .weak USBWakeUp_IRQHandler
  .thumb_set USBWakeUp_IRQHandler,Default_Handler

  .weak TIM8_BRK_IRQHandler
  .thumb_set TIM8_BRK_IRQHandler,Default_Handler

  .weak TIM8_UP_IRQHandler
  .thumb_set TIM8_UP_IRQHandler,Default_Handler

  .weak TIM8_TRG_COM_IRQHandler
  .thumb_set TIM8_TRG_COM_IRQHandler,Default_Handler

  .weak TIM8_CC_IRQHandler
  .thumb_set TIM8_CC_IRQHandler,Default_Handler

  .weak ADC3_IRQHandler
  .thumb_set ADC3_IRQHandler,Default_Handler

  .weak FSMC_IRQHandler
  .thumb_set FSMC_IRQHandler,Default_Handler

  .weak SDIO_IRQHandler
  .thumb_set SDIO_IRQHandler,Default_Handler

  .weak TIM5_IRQHandler
  .thumb_set TIM5_IRQHandler,Default_Handler

  .weak SPI3_IRQHandler
  .thumb_set SPI3_IRQHandler,Default_Handler

  .weak UART4_IRQHandler
  .thumb_set UART4_IRQHandler,Default_Handler

  .weak UART5_IRQHandler
  .thumb_set UART5_IRQHandler,Default_Handler

  .weak TIM6_IRQHandler
  .thumb_set TIM6_IRQHandler,Default_Handler

  .weak TIM7_IRQHandler
  .thumb_set TIM7_IRQHandler,Default_Handler

  .weak DMA2_Channel1_IRQHandler
  .thumb_set DMA2_Channel1_IRQHandler,Default_Handler

  .weak DMA2_Channel2_IRQHandler
  .thumb_set DMA2_Channel2_IRQHandler,Default_Handler

  .weak DMA2_Channel3_IRQHandler
  .thumb_set DMA2_Channel3_IRQHandler,Default_Handler

  .weak DMA2_Channel4_5_IRQHandler
  .thumb_set DMA2_Channel4_5_IRQHandler,Default_Handler

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
