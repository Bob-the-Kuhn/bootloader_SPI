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
  *   1: Reset_Handler is NOT weak so it'll replace the one in the system
  *      startup_xxxx.s file.
  *   2: Code is added at the very beginning that decides if the bootloader
  *      start sequence is to be executed or if we should start the application.
  *   3. The normal Reset_Handler code follows the added code section.  The
  *      Reset_Handler code is a copy of the code in:
  *        .platformio\packages\framework-cmsis-stm32f1\Source\Templates\gcc\startup_stm32f103xe.s
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

/* start code copied from the post by Piranha */

	BL   Boot_GetApplicationAddress
	CBZ  R0, StartBootloader
	LDR  R1, =0xE000ED00  // SCB
	STR  R0, [R1, #8]     // VTOR
	LDR  SP, [R0, #0]     // Stack pointer
	LDR  R0, [R0, #4]     // Reset handler
	DSB                   // Ensure the VTOR and SP operations are complete
	ISB                   // Flush the pipeline because of SP change
	BX   R0               // Start the application
StartBootloader:

/* end code copied from the post by Piranha */

/* code copied from the startup_stm32f103xe.s file */

/* Copy the data segment initializers from flash to SRAM */
  movs r1, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r3, =_sidata
  ldr r3, [r3, r1]
  str r3, [r0, r1]
  adds r1, r1, #4

LoopCopyDataInit:
  ldr r0, =_sdata
  ldr r3, =_edata
  adds r2, r0, r1
  cmp r2, r3
  bcc CopyDataInit
  ldr r2, =_sbss
  b LoopFillZerobss
/* Zero fill the bss segment. */
FillZerobss:
  movs r3, #0
  str r3, [r2], #4

LoopFillZerobss:
  ldr r3, = _ebss
  cmp r2, r3
  bcc FillZerobss

/* Call the clock system intitialization function.*/
    bl  SystemInit
/* Call static constructors */
    bl __libc_init_array
/* Call the application entry point.*/
  bl main
  bx lr
.size Reset_Handler, .-Reset_Handler


/**
 * @brief  Checks if the magic numbers are in RAM
 *          location: Magic_Location
 *          Magic_BootLoader  (boot bootloader)
 *          Magic_Application (boot application)
 * @param  None
 * @retval : R0 - 0 - if Magic_Location is not Magic_Application
 *           R0 - VTOR base address - if Magic_Location is Magic_Application
*/

  .section .text.Boot_GetApplicationAddress
  .type Boot_GetApplicationAddress, %function
Boot_GetApplicationAddress:
  LDR R0, = 0                  //  default to boot application
  LDR R1, = Magic_Location
  LDR R1, [R1, #0]                  // read magic value
  LDR R2, = MagicApplication
  LDR R2, [R2, #0]
  CMP R1, R2
  BNE SKIP
  LDR R1, = Magic_Location
  STR R0, [R1, #0]             // force next boot to go thru the bootloader
  LDR R0, = APP_ADDR           // point this boot at application
  LDR R0, [R0, #0]
SKIP:
  MOV PC, LR  // return
  .size   Boot_GetApplicationAddress, .-Boot_GetApplicationAddress





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
