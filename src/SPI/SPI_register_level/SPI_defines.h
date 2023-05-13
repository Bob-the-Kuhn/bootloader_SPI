
#include "basic.h"
//#include "SPI_register_level_defines.h"
#include "gpio.h"

volatile uint16_t* BASE_SPI1 = (uint16_t*)0x40013000UL;

#define SPI_1_CR1       *(BASE_SPI1 + 0x00)  
#define SPI_1_CR2       *(BASE_SPI1 + 0x02) // pointer math 4 -> 2
#define SPI_1_DR        *(BASE_SPI1 + 0x06) //  C -> 6
#define SPI_1_SR        *(BASE_SPI1 + 0x04) //  8 -> 4


// PA4 SS
// PA5 CLK
// PA6 MISO
// PB5 MOSI
// PB11 SD_detect
//                                                                                            measured 
// SPI_1_CR1 |= (0<<3);  // BR[2:0] = 000: fPCLK/16, PCLK2 = 80MHz, SPI clk = 40MHz            21.3MHz
// SPI_1_CR1 |= (1<<3);  // BR[2:0] = 001: fPCLK/16, PCLK2 = 80MHz, SPI clk = 20MHz            10.7MHz 
// SPI_1_CR1 |= (2<<3);  // BR[2:0] = 010: fPCLK/16, PCLK2 = 80MHz, SPI clk = 10MHz             5.3MHz
// SPI_1_CR1 |= (3<<3);  // BR[2:0] = 011: fPCLK/16, PCLK2 = 80MHz, SPI clk = 5MHz            2.667MHz
// SPI_1_CR1 |= (4<<3);  // BR[2:0] = 100: fPCLK/16, PCLK2 = 80MHz, SPI clk = 2.5MHz            1.33MHz
// SPI_1_CR1 |= (5<<3);  // BR[2:0] = 101: fPCLK/16, PCLK2 = 80MHz, SPI clk = 1.25MHz           0.625MHz  
// SPI_1_CR1 |= (6<<3);  // BR[2:0] = 110: fPCLK/16, PCLK2 = 80MHz, SPI clk = 0.625MHz          0.328MHz
// SPI_1_CR1 |= (7<<3);  // BR[2:0] = 111: fPCLK/16, PCLK2 = 80MHz, SPI clk = 0.3125MHz       0.164MHz


#define FCLK_SLOW() { SPI_1_CR1 |= (6<<3); }	/* Set SCLK = slow, approx 300 KBits/s*/
#define FCLK_FAST() { SPI_1_CR1 |= (3<<3); }	/* Set SCLK = fast, approx 2.5 MBits/s */

#define CS_HIGH()	{gpio_wr(  IO(PORTA, 4), 1);}
#define CS_LOW()	{gpio_wr(  IO(PORTA, 4), 0);}
