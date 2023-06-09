## STM32F103ZE bootloader using SPI interface

Functionality:

    - Copies image from the file firmware.bin to FLASH
    - Loading starts at 0x0800 8000 (can be changed)
    - Write protection is automatically removed
    - File/image checksum/crc checking is not done
    - UART1 @115200 can be used to monitor the boot process.
    - FAT32 file system with 512 - 4096 byte AUs.

Load point is set by APP_ADDRESS in bootloader.h.

Image filename is set by CONF_FILENAME in main.h.

Target card was a F103EZ_PRO.

Pinout is at:  https://stm32-base.org/boards/STM32F103ZET6-F103ZE-Board

Only able to find a very fuzzy schematic.
       https://preview.redd.it/9syoqui0mtd61.jpg?width=2327&format=pjpg&auto=webp&v=enabled&s=20620982106121516a1e88c9ff20ead48f68aa95

USART1 drives the CH340 USB chip (left side USB port).

Apparently the target card DOES have an SDIO interface to the SD card.  Can't tell if it has an SD detect signal.  Definitely has a pullup on it so it should be present.

Ran into serious issues with the SysTick system.  
In polling mode it worked OK.
With the interrupt enabled, the system went off into the weeds.  
Filled the entire exception & interrupt vector table with a capture routine and never had an interrupt.
No idea of what the root cause of the SysTick issues is.

This code is based on the booloader and main routines from:
  https://akospasztor.github.io/stm32-bootloader

Steps to create project:
1) copy already existing project to the new folder
2) delete the Core, FATFS and Middlewares folders
3) create base in STM32cubeIDE & generate code
4) copy Core, FATFS and Middlewares over to new folder
5) update platformio.ini and build.bat
6) update main.h and main.c (compare already existing project vs. new files & copy items as needed)
7) update bootloader files for the new CPU



## Hardware notes:

SDIO is used in low speed polling mode.  SDIO DMA mode worked for cards with
512 byte allocation units but not with 4096 byte allocation units.

This board does NOT have a hardware "SD card is present" pin so some
code was commented out.

Erasing is done via sectors. All sectors that don't have the boot loader image in it are erased. 

This program comes in at about 26,800 bytes which means it fills up sector 0 and extends in to
sector 1.  That means the lowest load point is the beginning of sector 2 (0x0800 8000). 

APP_ADDRESS can be set to any 512 byte aligned address in any erased sector.

## Building the image:

I used platformio within VSCode.  You'll need to set the workspace to the top
directory.  In the terminal window set the directory to the same one as the workspace.
Use the command "platformio run" command to build the image.  


## Porting to another processor:

Porting to the STM32F407ZG is very easy.  The only difference is the
larger FLASH.  You'll need to modify the FLASH defines to add the larger
number of sectors. 

Porting to other processors requires looking at the sector layout, erase
mechanisms and FLASH programming mechanisms. 