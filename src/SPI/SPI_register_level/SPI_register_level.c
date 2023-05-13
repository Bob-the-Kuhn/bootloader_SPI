// https://controllerstech.com/spi-using-registers-in-stm32/

#include "integer.h" //from FatFs middleware library
#include "SPI_register_level_defines.h"
#include "stm32f407x_defines.h"
//#include "RccConfig_F446.h"

void SPIConfig (void)
{
  /************** STEPS TO FOLLOW *****************
	1. Enable SPI clock
	2. Configure the Control Register 1
	3. Configure the CR2
	************************************************/	
  //RCC_APB2ENR |= (1<<12);  // Enable SPI1 CLock  (already done in init.c)
  //SPI1_CR1 |= (1<<0)|(1<<1);   // CPOL=1, CPHA=1 - SD cards are usually 0,0
  SPI1_CR1 |= (1<<2);  // Master Mode
  SPI1_CR1 |= (3<<3);  // BR[2:0] = 011: fPCLK/16, PCLK2 = 80MHz, SPI clk = 5MHz
  SPI1_CR1 &= ~(1<<7);  // LSBFIRST = 0, MSB first
  SPI1_CR1 |= (1<<8) | (1<<9);  // SSM=1, SSi=1 _ Software Slave Management
  SPI1_CR1 &= ~(1<<10);  // RXONLY = 0, full-duplex
  SPI1_CR1 &= ~(1<<11);  // DFF=0, 8 bit data
  SPI1_CR2 = 0;
}

void SPI_GPIOConfig (void)
{
  // PA4 SS
  // PA5 CLK
  // PA6 MISO
  // PB5 MOSI
  // PB11 SD_detect
	//RCC_AHB1ENR |= (1<<0);  // Enable GPIO Clock (already done in init.c)
	
  GPIOA_MODER |= ((1<< 8|2<<10)|(2<<12));  // Output for PA4, Alternate functions for PA5 & PA6
  GPIOB_MODER |= ((2<<10)|(1<<22));        // Alternate functions for PB5, Output for PB11
	
	GPIOA_OSPEEDR |= ((3<<8)|(3<<10)|(3<<12));  // Very HIGH Speed for PA4, PA5, PA6
  GPIOB_OSPEEDR |= (3<<10);                // Very HIGH Speed for PB5
	
	GPIOA_AFRL |= ((5<<20)|(5<<24));         // AF5(SPI1) for PA5, PA6
  GPIOB_AFRL |= (5<<20);                   // AF5(SPI1) for PB5
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
	SPI1_CR1 |= (1<<6);   // SPE=1, Peripheral enabled
}

void SPI_Disable (void)
{
	SPI1_CR1 &= ~(1<<6);   // SPE=0, Peripheral Disabled
}

void CS_Enable (void)
{
	GPIOA_BSRR |= (1<<9)<<16;
}

void CS_Disable (void)
{
	GPIOA_BSRR |= (1<<9);
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
	   while (!((SPI1_SR)&(1<<1))) {};  // wait for TXE bit to set _ This will indicate that the buffer is empty
	   SPI1_DR = data[i];  // load the data into the Data Register
	   i++;
	}	
	
	
/*During discontinuous communications, there is a 2 APB clock period delay between the
write operation to the SPI_DR register and BSY bit setting. As a consequence it is
mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
data.
*/
	while (!((SPI1_SR)&(1<<1))) {};  // wait for TXE bit to set _ This will indicate that the buffer is empty
	while (((SPI1_SR)&(1<<7))) {};  // wait for BSY bit to Reset _ This will indicate that SPI is not busy in communication	
	
	//  Clear the Overrun flag by reading DR and SR
	BYTE temp = SPI1_DR;
					temp = SPI1_SR;
	
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
		while (((SPI1_SR)&(1<<7))) {};  // wait for BSY bit to Reset _ This will indicate that SPI is not busy in communication
		SPI1_DR = 0;  // send dummy data
		while (!((SPI1_SR) &(1<<0))){};  // Wait for RXNE to set _ This will indicate that the Rx buffer is not empty
	  *data++ = (SPI1_DR);
		size--;
	}	
}

BYTE SPI_Transfer(BYTE data) {
  while (!((SPI1_SR)&(1<<1))) {};  // wait for TXE bit to set _ This will indicate that the buffer is empty
	SPI1_DR = data;  // load the data into the Data Register
  while (!((SPI1_SR) &(1<<0))){};  // Wait for RXNE to set _ This will indicate that the Rx buffer is not empty
  return SPI1_DR;
}
 
  