platformio run
cd C:\work\debug\ARM_tools
copy C:\work\temp\bootloader_platformio\bootloader_SPI\.pio\build\bootloader_BTT_SKR_V1_1_STM32F407ZGT6\firmware.elf
.\objdump.exe -d -S -l -C -t firmware.elf >C:\work\debug\ARM_tools\ARM_disassemble_with_line_numbers.txt
cd C:\work\temp\bootloader_platformio\bootloader_SPI

