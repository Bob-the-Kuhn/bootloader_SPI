## LPC1769 bootloader using SPI interface

Functionality:

    - Copies image from the file firmware.bin to FLASH
    - Loading starts at 0x0000 4000 (can be changed)
    - Write protection is automatically removed and restored
    - File/image checksum/crc checking is not done
    - UART0 @115200 can be used to monitor the boot process.
    - FAT32 file system with 512 - 4096 byte AUs. exFAT can be enabled

Load point is set by APP_ADDRESS in bootloader.h.

Image filename is set by CONF_FILENAME in main.h.

Target card was a Cohesion3D Remix.  This is a LPC1769 based board with a hardware SPI interface to the SD card.

Schematic is at:  https://github.com/KevinOConnor/klipper/files/4513839/C3D.ReMix.Design.Files.zip

This code is based on:

    bootloader and main routines:

        https://akospasztor.github.io/stm32-bootloader

    FAT system, low level routines & print routines:

        https://github.com/colosimo/fatfs-stm32
        
    Hard SPI routines:
        https://controllerstech.com/spi-using-registers-in-stm32/
        
The reset handler modifications are based on Piranna's comment in the following page:

    https://community.st.com/s/question/0D50X0000AFpTmUSQV/using-nvicsystemreset-in-bootloaderapplication-jumps

The low level routines & print routines significantly reduced the image size
vs. using STM32CubeIDE generated code (15,900 vs. 22,500). That allows The lowest 
load point to be the beginning of sector 4 (0x0000 4000).

EXFAT support can be enabled.  It has been successfully tested.  The drawback with
enabling EXFAT (and the required LFN support) is the huge image size. 

## Hardware notes:

The hardware SPI module is used to provide the SPI interface,

Erasing is done via sectors. All sectors that don't have the boot loader image in it are erased. 

APP_ADDRESS can be set to any 512 byte aligned address in any erased sector.

The LPC176x processors do not have a FLASH write protection mechanism.  That makes the software portion much simpler.

## Software notes:

The software is a state machine.  There are two main states:
- Bootloader mode vs. application mode

The flags/variables used to control the state machine have to survive the CPU going through reset and the reset handler.  This is accomplished via:
- A custom reset handler
- Some of the flags/variables are used directly by the custom reset handler.  These can go into the standard bss and data sections because they are used before the bss section of RAM gets zeroed out.
- Some of the flags/variables are used in the main C routines. Some can be initialized on reset.  These can go into the standard data section.  The ones that can't be initialized are put into the "no_init" section.  Allowing them to go into the bss section would result in them getting zeroed out by the reset handler.       

Interactions/dependencies between the bootloader and the application are minimized by a custom reset handler. Sending the CPU through a reset between finishing the bootloader activities and starting the application means that the two can have independent startup code.  Not going through reset requires trying to put the CPU back into a state that doesn't interfere with the application. The "non-reset" approach is, to be polite, "problematic". Thanks to Piranna for showing a better method.

Further down is a simplified flow chart of the software.

In order to get the image size small enough to fit into the first 4K, almost all debug/status prints have been commented out.  

## Building the image:

I used platformio within VSCode.  You'll need to set the workspace to the top
directory.  In the terminal window set the directory to the same one as the workspace.
Use the command "platformio run" command to build the image.  


## Porting to another processor:

Porting to the LPC1768 is very easy.  The only difference is the
lower max clock frequency which affects multiple clock/prescaler registers.

Porting to other processors requires looking at the sector layout, erase
mechanisms and FLASH programming mechanisms.

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