/**
  ******************************************************************************
  * @file      startup_stm32f746xx.s
  * @author    MCD Application Team
  * @brief     STM32F746xx Devices vector table for GCC based toolchain. 
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M7 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
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
  .cpu cortex-m7
  .fpu softvfp
  .thumb

.global  g_pfnVectors
.global  Default_Handler

/* start address for the initialization values of the .data section. 
defined in linker script */
.word  _sidata
/* start address for the .data section. defined in linker script */  
.word  _sdata
/* end address for the .data section. defined in linker script */
.word  _edata
/* start address for the .bss section. defined in linker script */
.word  _sbss
/* end address for the .bss section. defined in linker script */
.word  _ebss
/* stack used for SystemInit_ExtMemCtl; always internal RAM used */

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called. 
 * @param  None
 * @retval : None
*/

    .section  .text.Reset_Handler
  .global  Reset_Handler
  .type  Reset_Handler, %function
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

  ldr   sp, =_estack      /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */  
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

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

/* Call the clock system initialization function.*/
  bl  SystemInit   
/* Call static constructors */
    bl __libc_init_array
/* Call the application's entry point.*/
  bl  main
  bx  lr    
.size  Reset_Handler, .-Reset_Handler


/* end code copied from the startup_stm32f407vetx.s file */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
