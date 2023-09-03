platformio run
cd C:\work\debug\ARM_tools
copy C:\Users\bobku\Documents\GitHub\bootloader_SPI\.pio\build\bootloader_SPI_LPC1769\firmware.elf
.\objdump.exe -d -S -l -C -t firmware.elf >C:\work\debug\ARM_tools\ARM_disassemble_with_line_numbers.txt
cd C:\Users\bobku\Documents\GitHub\bootloader_SPI