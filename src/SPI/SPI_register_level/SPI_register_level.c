// https://controllerstech.com/spi-using-registers-in-stm32/

#include "integer.h" //from FatFs middleware library
//#include "SPI_register_level_defines.h"
#include "stm32g0b1x_defines.h"
#include "basic.h"
#include "gpio.h"
#include "main.h"

//#include "RccConfig_F446.h"

void SPIConfig (void)
{
  /************** STEPS TO FOLLOW *****************
	1. Enable SPI clock
	2. Configure the Control Register 1
	3. Configure the CR2
	************************************************/	
  //RCC_APB2ENR |= (1<<12);  // Enable SPI3 CLock  (already done in init.c)
  //SPI3_CR1 |= (1<<0)|(1<<1);   // CPOL=1, CPHA=1 - SD cards are usually 0,0
  or16(RSPI3_CR1, (1<<2));  // Master Mode
  or16(RSPI3_CR1, (3<<3));  // BR[2:0] = 011: fPCLK/16, PCLK2 = 80MHz, SPI clk = 5MHz
  and16(RSPI3_CR1, ~(1<<7));  // LSBFIRST = 0, MSB first
  or16(RSPI3_CR1, (1<<8) | (1<<9));  // SSM=1, SSi=1 _ Software Slave Management
  and16(RSPI3_CR1, ~(1<<10));  // RXONLY = 0, full-duplex
  and16(RSPI3_CR1, ~(1<<11));  // DFF=0, 8 bit data
  //or16(RSPI3_CR1, (1<<14));  // BIDIOE output enable
  wr16(RSPI3_CR2, (7<<8));   // 8 bit transfer, Motorola format
}


void k_delay(const uint32_t ms);;

void SPI_GPIOConfig (void)
{ 
  
  // SPI 3
  // PA4 SS
  // PC10 CLK
  // PC11 MISO
  // PC12 MOSI
  // PB11 SD_detect
	//RCC_AHB1ENR |= (1<<0);  // Enable GPIO Clock (already done in init.c)
	
  GPIO_CONFIG_OUTPUT(PORTA, 4,  GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);
  GPIO_CONFIG_ALT(PORTC, 10, 4, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);
  GPIO_CONFIG_ALT(PORTC, 11, 4, GPIO_PULL_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);
  GPIO_CONFIG_ALT(PORTC, 12, 4, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);
  GPIO_CONFIG_INPUT(PORTB, 11,  GPIO_NO_PULL_UP_DOWN)
  
  
  gpio_wr(PORTA, 4,  0);
  k_delay(5);

  gpio_wr(PORTA, 4,  1);
  k_delay(5);
  CS_HIGH();
  k_delay(10);
  CS_LOW();
  k_delay(10);
  
  
}

// void GPIOConfig (void)  // F103
// {
// 	RCC_APB2ENR |=  (1<<2);  // Enable GPIOA clock
// 	
// 	GPIOA_CRL = 0;
// 	GPIOA_CRL |= (11U<<20);   // PA5 (SCK) AF output Push Pull
// 	GPIOA_CRL |= (11U<<28);   // PA7 (MOSI) AF output Push Pull
// 	GPIOA_CRL |= (1<<26);    // PA6 (MISO) Input mode (floating)
// 	GPIOA_CRL |= (3<<16);    // PA4 used for CS, GPIO Output 
// 	
// }

void SPI_Enable (void)
{
	or16(RSPI3_CR1, (1<<6));   // SPE=1, Peripheral enabled
}

void SPI_Disable (void)
{
	and16(RSPI3_CR1,  ~(1<<6));   // SPE=0, Peripheral Disabled
}

void CS_Enable (void)
{
  or32(R_GPIOA_BSRR, (1<<9)<<16);
}

void CS_Disable (void)
{
  or32(R_GPIOA_BSRR, (1<<9));
}


void SPI_Transmit (BYTE *data, UINT size)
{
	
	/************** STEPS TO FOLLOW *****************
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	************************************************/		
	
	UINT i=0;
	while (i<size)
	{
	   while (!((rd16(RSPI3_SR)&(1<<1)))) {};  // wait for TXE bit to set _ This will indicate that the buffer is empty
	   wr16(RSPI3_DR, data[i]);  // load the data into the Data Register
	   i++;
	}	
	
	
/*During discontinuous communications, there is a 2 APB clock period delay between the
write operation to the SPI_DR register and BSY bit setting. As a consequence it is
mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
data.
*/
	while (!((rd16(RSPI3_SR)&(1<<1)))) {};  // wait for TXE bit to set _ This will indicate that the buffer is empty
	while ((rd16(RSPI3_SR)&(1<<7))) {};  // wait for BSY bit to Reset _ This will indicate that SPI is not busy in communication	
	
	//  Clear the Overrun flag by reading DR and SR
	BYTE temp = rd16(RSPI3_DR);
					temp = rd16(RSPI3_SR);
	
}


void SPI_Receive (BYTE *data, UINT size)
{
	/************** STEPS TO FOLLOW *****************
	1. Wait for the BSY bit to reset in Status Register
	2. Send some Dummy data before reading the DATA
	3. Wait for the RXNE bit to Set in the status Register
	4. Read data from Data Register
	************************************************/		

	while (size)
	{
		while (((rd16(RSPI3_SR))&(1<<7))) {};  // wait for BSY bit to Reset _ This will indicate that SPI is not busy in communication
		wr16(RSPI3_DR, 0);  // send dummy data
		while (!((rd16(RSPI3_SR)) &(1<<0))){};  // Wait for RXNE to set _ This will indicate that the Rx buffer is not empty
	  *data++ = (rd16(RSPI3_DR));
		size--;
	}	
}

BYTE SPI_Transfer(BYTE data) {
  while (!((rd16(RSPI3_SR))&(1<<1))) {};  // wait for TXE bit to set _ This will indicate that the buffer is empty
	wr16(RSPI3_DR, data);  // load the data into the Data Register
  while (!((rd16(RSPI3_SR)) &(1<<0))){};  // Wait for RXNE to set _ This will indicate that the Rx buffer is not empty
  return rd16(RSPI3_DR);
}
 
  