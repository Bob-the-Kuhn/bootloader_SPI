/**
  ******************************************************************************
  * @file      startup_stm32f407xx.s
  * @author    MCD Application Team
  * @brief     STM32F407xx Devices vector table for GCC based toolchains. 
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M4 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

  .section .text.isr_reset
  .global isr_reset
  .type isr_reset, %function
isr_reset:

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
  
.size  isr_reset, .-isr_reset


/* end code copied from the startup_stm32f407vetx.s file */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
