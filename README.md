## STM32F746ZG bootloader using SPI interface

Functionality:

Functionality:

    - Copies image from the file firmware.bin to FLASH
    - Loading starts at 0x0800 8000 (can be changed)
    - Write protection is automatically removed and restored
    - File/image checksum/crc checking is not done
    - USART3 @115200 is used to monitor the boot process
    - Supports FAT32 & exFAT file systems with 512 - 4096 byte AUs

Load point is set by APP_ADDRESS in bootloader.h.

Image filename is set by CONF_FILENAME in main.h.

Target card was a nucleo_F746ZG.

Pinout is at: UM1974   https://www.st.com/content/ccc/resource/technical/document/user_manual/group0/26/49/90/2e/33/0d/4a/da/DM00244518/files/DM00244518.pdf/jcr:content/translations/en.DM00244518.pdf

The bootloader code is based on the bootloader and main routines from:
  https://akospasztor.github.io/stm32-bootloader

STM32CubeIDE was used to generate the bulk of the code.

The reset handler modifications are based on Piranna's comment in the following page:
  https://community.st.com/s/question/0D50X0000AFpTmUSQV/using-nvicsystemreset-in-bootloaderapplication-jumps

## Building the image:

This repository builds in VSCode/platformio as is.

There is a build.bat file that was used to automate the build steps in my debug environment.  It can be adapted to suit your needs.

## Hardware notes:

On the nucleo card, USART3 is connected to the virtual com port on the ST-Link.  

Erasing is done via pages. All pages that don't have the boot loader image in it are erased.

This program comes in at about 30K bytes which means it almost fills up pages 1. That means the lowest load point is the beginning of page 2 (0x0800 8000).  

APP_ADDRESS can be set to any 512 byte aligned address in any erased page.

Changing the write protection status of the page actually occurs during hardware reset.  The code initiates the reset sequence.

## Software notes:

There are four main files that implement the bootloader functionality:
    main_boot.c
    main_boot.h
    bootloader.c
    bootloader.h

STM32CubeIDE was used to create the non-bootloader code.  After generating code, modify the file main.c to invoke main_boot(). Then the folders Core, Drivers, FATFS and Middlewares are copied over to the main working directory along with the file STM32F746ZGTX_FLASH.ld.

The first sector on a STM32F746 is huge.  No attempt was made to minimize code. exFAT support was enabled because it didn't cost anything. 

STM32CubeIDE can be used to build & debug the program. Copy the files in directories "stm32-bootloader" and "SPI" into the same directories as main.c and main.h or else STM32CubeIDE won't find them during the compile/build process.

The software is a state machine.  There are two main state cycles:
- Bootloader mode vs. application mode
- Write protection

The flags/variables used to control the state machine have to survive the CPU going through reset and the reset handler.  This is accomplished via:
- A custom reset handler
- Some of the flags/variables are used directly by the custom reset handler.  These can go into the standard bss and data sections because they are used before the bss section of RAM gets zeroed out.
- Some of the flags/variables are used in the main C routines. Some can be initialized on reset.  These can go into the standard data section.  The ones that can't be initialized are put into the "no_init" section.  Allowing them to go into the bss section would result in them getting zeroed out by the reset handler.       

Interactions/dependencies between the bootloader and the application are minimized by a custom reset handler. Sending the CPU through a reset between finishing the bootloader activities and starting the application means that the two can have independent startup code.  Not going through reset requires trying to put the CPU back into a state that doesn't interfere with the application. The "non-reset" approach is, to be polite, "problematic". Thanks to Piranna for showing a better method.

Further down is a simplified flow chart of the software.

## Porting to another processor:

Porting to other STM32F746 chips is very easy.  The only difference is the size of the FLASH and SRAM.  You'll need to modify the FLASH defines.

Porting to other processors requires looking at the page layout, erase mechanisms, protect mechanisms and FLASH programming mechanism.

Steps to create project:
1) copy already existing project to a new folder
2) save main.c and main.h
3) delete the Core, Drivers, FATFS and Middlewares folders
4) create base system in STM32cubeIDE & generate code
5) copy Core, Drivers, FATFS and Middlewares over to the new folder
6) delete the files "startup_stm32f103zetx.s" and "stm32f1xx.h" (they conflict with ones are supplied by the cmsis framework)
7) update platformio.ini and build.bat
8) update main.h and main.c (compare saved files vs. new files & copy items as needed)
9) update bootloader files for the new CPU


## Simplified Flow Chart

![Simplified Flow Chart png](/readme_files/bootloader_flow_chart.png)

## Related bootloaders

This bootloader is one of a family of bootloaders aimed at various platforms.

All are based on the bootloader and main routines from akospasztor.

        https://akospasztor.github.io/stm32-bootloader
        
Most are optimized for size.  These are hased on the FAT system, low level routines & print routines from here:
https://github.com/colosimo/fatfs-stm32
        
The larger sized ones are based on STM32cubeIDE code.

![Bootloader Family Summary png](/readme_files/bootloader_summary.png)