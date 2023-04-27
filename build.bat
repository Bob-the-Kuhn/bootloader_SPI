platformio run
cd C:\work\debug\ARM_tools
copy C:\work\temp\bootloader_platformio\bootloader_SPI\.pio\build\genericSTM32F103ZE\firmware.elf
.\objdump.exe -d -S -l -C -t firmware.elf >C:\work\debug\ARM_tools\ARM_disassemble_with_line_numbers.txt
cd C:\work\temp\bootloader_platformio\bootloader_SPI

