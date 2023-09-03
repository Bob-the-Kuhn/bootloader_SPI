
#include "basic.h"
//#include "SPI_register_level_defines.h"
#include "gpio.h"
#include "LPC1769x_defines.h"

//volatile uint16_t* BASE_SPI0 = (uint16_t*)0x40020000UL;
//
//#define S0SPCR        *(BASE_SPI1 + 0x00)  
//#define S0SPSR        *(BASE_SPI1 + 0x02) // pointer math 4 -> 2
//#define S0SPDR        *(BASE_SPI1 + 0x04) //  8 -> 4
//#define S0SPCCR       *(BASE_SPI1 + 0x06) //  C -> 6


// P0_6 SS
// P0_7 CLK
// P0_8 MISO
// P0_9 MOSI
// N/C  SD_detect
//                                                                                            measured 

#define FCLK_SLOW() do { *SSP1CR0 &= ~(0x0FF00);\
                         *SSP1CR0 |= (1<<SCR);\
                         *SSP1CPSR &= ~(0xFF);\
                         *SSP1CPSR |= 0xC8; }while(0)    // 300K baud rate (120MHz/200/2/1)
#define FCLK_FAST()do {  *SSP1CR0 &= ~(0x0FF00);\
                         *SSP1CPSR &= ~(0xFF);\
                         *SSP1CPSR |= 0x1E; }while(0)    // 4M baud rate (120MHz/30/1/1)

#define CS_HIGH()	{gpio_wr(  IO(PORT0,6), 1);}
#define CS_LOW()	{gpio_wr(  IO(PORT0,6), 0);}
