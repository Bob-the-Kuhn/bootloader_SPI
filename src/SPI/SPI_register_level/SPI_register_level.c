// https://controllerstech.com/spi-using-registers-in-stm32/

#include "integer.h" //from FatFs middleware library
//#include "SPI_register_level_defines.h"
#include "stm32g0b1x_defines.h"
#include "basic.h"
#include "gpio.h"
#include "main.h"
#include <stdint.h>

extern uint32_t k_ticks(void);

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
  and16(RSPI3_CR1,  ~(1<<6));   // SPE=0, Peripheral Disabled - some bits can't be set when enabled
  or16(RSPI3_CR1, (1<<2));  // Master Mode
  or16(RSPI3_CR1, (3<<3));  // BR[2:0] = 011: fPCLK/16, PCLK2 = 80MHz, SPI clk = 5MHz
  and16(RSPI3_CR1, ~(1<<7));  // LSBFIRST = 0, MSB first
  or16(RSPI3_CR1, (1<<8) | (1<<9));  // SSM=1, SSi=1 _ Software Slave Management
  and16(RSPI3_CR1, ~(1<<10));  // RXONLY = 0, full-duplex
  and16(RSPI3_CR1, ~(1<<11));  // DFF=0, 8 bit data
  //or16(RSPI3_CR1, (1<<14));  // BIDIOE output enable
  wr16(RSPI3_CR2, ((1 << 12) | (7<<8)));   // 8 bit transfer, Motorola format
  //wr16(RSPI3_CR2,  (7<<8));   // 16 bit transfer, Motorola format
  
  or16(RSPI3_CR1, (1<<6));   // SPE=1, Peripheral enabled
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
  GPIO_CONFIG_INPUT(PORTB, 11,  GPIO_NO_PULL_UP_DOWN);
  
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




//void SPI_Transmit (BYTE *data, UINT size)
void xmit_spi_multi (BYTE *data, UINT size)
{
	
	/************** STEPS TO FOLLOW *****************
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
	************************************************/		
	//  SPI data register is 16 bits but incoming data is 8 bit
  
  //wr16(RSPI3_CR2,  (7<<8));   // 16 bit transfer, Motorola format
  
	UINT i=0;
  uint16_t* RD_pointer = (uint16_t*)RSPI3_DR_8;  // pointer to SPI data register
  uint16_t data_16;
	while (i<size)
	{ 
     if (i == (size - 1)) {
       data_16 = 0;
       data_16 = data[i];
     }
     else {
       data_16 = 0;
       data_16 = (data[i] | (data[i+1] << 8));
       i += 2;
     }
     
     while (!((rd16(RSPI3_SR)&(1<<1)))) {};  // wait for TXE bit to set _ This will indicate that the buffer is empty
     *RD_pointer = data_16;  // send the data
	}	
	
	
/*During discontinuous communications, there is a 2 APB clock period delay between the
write operation to the SPI_DR_8 register and BSY bit setting. As a consequence it is
mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
data.
*/
	//while (!((rd16(RSPI3_SR)&(1<<1)))) {};  // wait for TXE bit to set _ This will indicate that the buffer is empty
	//while ((rd16(RSPI3_SR)&(1<<7))) {};  // wait for BSY bit to Reset _ This will indicate that SPI is not busy in communication	
	
	//  Clear the Overrun flag by reading DR and SR
	data_16 = *RD_pointer;
	data_16 = rd16(RSPI3_SR);
  
  //wr16(RSPI3_CR2, ((1 << 12) | (7<<8)));   // back to 8 bit transfer mode, Motorola format
	
}


//void SPI_Receive (BYTE *data, UINT size)
void rcvr_spi_multi (BYTE *data, UINT size)
{
	/************** STEPS TO FOLLOW *****************
	1. Wait for the BSY bit to reset in Status Register
	2. Send some Dummy data before reading the DATA
	3. Wait for the RXNE bit to Set in the status Register
	4. Read data from Data Register
	************************************************/		
 // wr16(RSPI3_CR2,  (7<<8));   // 16 bit transfer, Motorola format
  uint8_t* RD_pointer = (uint8_t*)RSPI3_DR_8;
  // input array is data[0] ... data[size-1]
	while (size)
	{
		while (((rd16(RSPI3_SR))&(1<<7))) {};  // wait for BSY bit to Reset _ This will indicate that SPI is not busy in communication
		*RD_pointer = 0;  // send dummy data
//		while (!((rd16(RSPI3_SR)) &(1<<0))){};  // Wait for RXNE to set _ This will indicate that the Rx buffer is not empty
	  data[size - 1] = RD_pointer[0];
    size--;
    if (size != 0) {
      data[size - 1] = RD_pointer[1];  // move MSB into data buffer if past the end
      size--;
    }
    
	}	
  wr16(RSPI3_CR2, ((1 << 12) | (7<<8)));   // back to 8 bit transfer mode, Motorola format
}

__attribute__((weak))
BYTE SPI_Transfer(uint16_t data) {
  uint32_t timeout = k_ticks() + 10;  // 10 ms timeout
  uint8_t* RD_pointer = (uint8_t*)RSPI3_DR_8;
//  while (!((rd16(RSPI3_SR))&(1<<1)) && (timeout > k_ticks())) {};  // wait for TXE bit to set _ This will indicate that the buffer is empty
  *RD_pointer =  data;  // load the data into the Data Register
//  while (!((rd16(RSPI3_SR)) &(1<<0)) && (timeout > k_ticks())){};  // Wait for RXNE to set _ This will indicate that the Rx buffer is not empty
  if(timeout <= k_ticks()) return 0;  
  return *RD_pointer ;
}
  