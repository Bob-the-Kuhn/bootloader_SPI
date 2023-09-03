// https://controllerstech.com/spi-using-registers-in-stm32/

#include "integer.h" //from FatFs middleware library
#include "SPI_register_level_defines.h"
#include "LPC1769x_defines.h"
#include "gpio.h"
//#include "RccConfig_F446.h"

void SPIConfig (void)
{
  /************** STEPS TO FOLLOW *****************
	1. Enable SPI clock
	2. Configure the Control Register 1
	3. Configure the CR2
	************************************************/	
  
  *SSP1CR0 &= ~(0x0FFFF);     // clear register (SPI mode, CPOL=0, CPHA=0, SCR=0)
  *SSP1CR0 |= 7;              // send/receive 8 bits
  *SSP1CR0 |= (1<<SCR);       // divide clock by 2
  *SSP1CR1 &= ~(0xF);         // clear register (Master mode, disable SSP1)
  *SSP1CPSR &= ~(0xFF);       // clear register
  *SSP1CPSR |= 0xC8;          // 300K baud rate (120MHz/200/2/1)
  *SSP1IMSC &= ~(0xF);        // interrupts disabled
  *SSP1CR1 |= (1<<SSE);       // enable SSP1
  
}

void SPI_GPIOConfig (void)
{
  // SD_SCK1_PIN    P0_07 Function code: 0b10
  // SD_MISO1_PIN   P0_08 Function code: 0b10
  // SD_MOSI1_PIN   P0_09 Function code: 0b10
  // SD_CS_PIN      P0_06 Function code: 0b10
  // no SD_detect
	// Enable GPIO Clock (already done in init.c)
  
    /*Configure GPIO pins : SD_SCK_PIN */
  gpio_wr(   IO(PORT0, 7), 0);
  gpio_func( IO(PORT0, 7), 0b10);
  gpio_dir(  IO(PORT0, 7), GPIO_OUTPUT);
  gpio_mode( IO(PORT0, 7), PULL_NO);
  
    /*Configure GPIO pins : SD_MISO_PIN */
  gpio_func( IO(PORT0, 8), 0b10);
  gpio_dir(  IO(PORT0, 8), GPIO_INPUT);
  gpio_mode( IO(PORT0, 8), PULL_NO);
  
    /*Configure GPIO pins : SD_MOSI_PIN */
  gpio_wr(   IO(PORT0, 9), 1);
  gpio_func( IO(PORT0, 9), 0b10);
  gpio_dir(  IO(PORT0, 9), GPIO_OUTPUT);
  gpio_mode( IO(PORT0, 9), PULL_NO);
  
    /*Configure GPIO pins : SD_CS_PIN (application controlled)*/
  gpio_wr(   IO(PORT0, 6), 1);
  gpio_func( IO(PORT0, 6), 0);
  gpio_dir(  IO(PORT0, 6), GPIO_OUTPUT);
  gpio_mode( IO(PORT0, 6), PULL_NO);

}

void SPI_Enable (void)
{
  volatile u32 *reg;
  reg = LPC_SC_PCONP;
	*reg |=  _BV(PCSSP1); // enable SPI module 
}

void SPI_Disable (void)
{
  volatile u32 *reg;
  reg = LPC_SC_PCONP;
	CLEAR_BIT(*reg ,PCSSP1);  // Peripheral Disabled
}

void CS_Enable (void)
{
  gpio_wr(IO(PORT0, 06), 0);
}

void CS_Disable (void)
{
  gpio_wr(IO(PORT0, 06), 1);
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
	   
	   *SSP1DR = data[i];  // load the data into the Data Register
     while (!((*SSP1SR) & 1)){};          // wait for Transmit FIFO to go empty
	   i++;
	}	
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
		*SSP1DR = 0;  // send dummy data
		while (!((*SSP1SR) &(1<<2))){};   // wait for Receive FIFO to have something in it
	  *data++ = (*SSP1DR);
		size--;
	}	
}

BYTE SPI_Transfer(BYTE data) {
  *SSP1DR = data;  // load the data into the Data Register
  while (!((*SSP1SR) & 1)){};          // wait for Transmit FIFO to go empty
  while (!((*SSP1SR) & (1<<2))){};   // wait for Receive FIFO to have something in it
  return *SSP1DR;
}
 
  