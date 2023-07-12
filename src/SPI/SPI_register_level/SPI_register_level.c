// https://controllerstech.com/spi-using-registers-in-stm32/

#include "integer.h" //from FatFs middleware library
//#include "SPI_register_level_defines.h"
#include "stm32g0b1x_defines.h"
#include "basic.h"
#include "gpio.h"
#include "main.h"
#include <stdint.h>

extern uint32_t k_ticks(void);
void k_delay(const uint32_t ms);

//#include "RccConfig_F446.h"

void SPIConfig (void)
{
  /************** STEPS TO FOLLOW *****************
	1. Enable SPI clock
	2. Configure the Control Register 1
	3. Configure the CR2
	************************************************/	
  
  
  // enable 
  or16(RSPI3_CR2, (1<<2));  //  Set SSOE bit to 1
  wr16(RSPI3_CR1, 0); //  Set all bits (including SSM) to 0 
  //or16(RSPI3_CR2, (1<<3));  //  Set NSSP bit to 1  (optional)
  
  while (SPI3->SR & (1<<5)) {  // writing to some registers is disabled if Mode fault (MODF) bit is active
     wr16(RSPI3_CR1, 0);      // clearing sequence is to read or write the status register and then write CR1
  }                               //  SDSS (NSSP) must be inactive for this to work
    
  //RCC_APB2ENR |= (1<<12);  // Enable SPI3 CLock  (already done in init.c)
  //SPI3->CR1 |= (3);   // CPOL=1, CPHA=1 - SD cards are usually 0,0
  //SPI3->CR1 |= (2);   // CPOL=1, CPHA=1 - SD cards are usually 0,0
  //SPI3->CR1 |= (1);   // CPOL=1, CPHA=1 - SD cards are usually 0,0
  or16(RSPI3_CR1, (1<<8)); // set internal SSI bit to 1 before enabling Master mode or will get a Mode fault (MODF) error
  or16(RSPI3_CR1, (1<<9)); // enable SSM before enabling Master mode or will get a Mode fault (MODF) error
  or16(RSPI3_CR1, (12<<3)); // add in bit rate
  //or16(RSPI3_CR1, (1<<2));  // Master Mode
  //and16(RSPI3_CR1, ~(1<<7));  // LSBFIRST = 0, MSB first
  //or16(RSPI3_CR1, (1<<8) | (1<<9));  // SSM=1, SSi=1 _ Software Slave Management
  //and16(RSPI3_CR1, ~(1<<10));  // RXONLY = 0, full-duplex
  //and16(RSPI3_CR1, ~(1<<11));  // DFF=0, 8 bit data
  //or16(RSPI3_CR1, (1<<14));  // BIDIOE output enable
  wr16(RSPI3_CR2, ((1 << 12) | (7<<8)));   // 8 bit transfer, Motorola format
  //wr16(RSPI3_CR2,  (7<<8));   // 16 bit transfer, Motorola format
  
  k_delay(2);  // try to keep optimizing compiler from executing the following out of order
 // and16(RSPI3_CR2, ~(1<<2));  //  Set SSOE bit to 0
  or16(RSPI3_CR1, 1<<2); // enable Master mode
  
  k_delay(2);  // try to keep optimizing compiler from executing the following out of order
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
	
  gpio_wr(PORTA, 4, GPIO_PIN_SET); // set SDSS high (inactive)
  
  GPIO_CONFIG_OUTPUT(PORTA, 4,  GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);    // let software contol SDSS pin
  //GPIO_CONFIG_ALT(PORTA,  4, 9, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);  // let SPI control SDSS pin
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
  or32(R_GPIOA_BSRR, (1<<4)<<16);
 or16(RSPI3_CR1, (1<<8)); // set internal SSI bit  high
 //and16(RSPI3_CR1, ~((1<<8)));  // set internal SSI bit low which should force the physican pin low
}

void CS_Disable (void)
{
  or32(R_GPIOA_BSRR, (1<<4));
  //or16(RSPI3_CR1, (1<<8)); // set internal SSI bit  high
  and16(RSPI3_CR1, ~((1<<8)));  // set internal SSI bit low which should force the physican pin low
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

uint16_t spi_status_temp1[256] = {0xFF};
uint16_t spi_status_temp2[256] = {0xFF};
uint16_t spi_status_temp3[256] = {0xFF};
uint16_t spi_status_temp4[256] = {0xFF};
uint16_t spi_status_timeout1[256] = {0xFF};
uint16_t spi_status_timeout2[256] = {0xFF};

uint16_t rcv_count = 0;

uint8_t rcv_data_buf[1024];
uint16_t rcv_data_status[1024];

__attribute__((weak))
BYTE SPI_Transfer(uint16_t data) {
  uint32_t timeout = k_ticks() + 10;  // 10 ms timeout
  uint8_t* RD_pointer = (uint8_t*)RSPI3_DR_8;
//  spi_status_temp1[(uint8_t) data] = rd16(RSPI3_SR); // save a copy of the status registers
  //while(rd16(RSPI3_SR)&(3<<11));  //wait if xmit fifo is not empty
  //while (!((rd16(RSPI3_SR))&(3<<11)) && (timeout > k_ticks())) {};  // wait for TX FIFO to be empty.
  while (!((rd16(RSPI3_SR))&(0x82)) && (timeout > k_ticks())) {};  // wait for TXE bit to set low. This will indicate that the buffer is empty
//  spi_status_temp2[(uint8_t) data] = rd16(RSPI3_SR); // save a copy of the status registers
//  if (timeout <= k_ticks()) spi_status_timeout1[(uint8_t) data] = 0;
  *RD_pointer =  (data & 0x00FF);  // load the data into the Data Register
//  spi_status_temp3[(uint8_t) data] = rd16(RSPI3_SR); // save a copy of the status registers
  timeout = k_ticks() + 10;  // 10 ms timeout
  while (!((rd16(RSPI3_SR)) &(1<<0)) && (timeout > k_ticks())){};  // Wait for RXNE to set low. This will indicate that the Rx buffer is not empty
 // if (timeout <= k_ticks()) spi_status_timeout2[(uint8_t) data] = 0;
 // spi_status_temp4[(uint8_t) data] = rd16(RSPI3_SR); // save a copy of the status registers
  if(timeout <= k_ticks()) return 0; 
  //rcv_data_buf[rcv_count] = SPI3->DR;
  rcv_data_buf[rcv_count] = *RD_pointer ;
  rcv_data_status[rcv_count]= SPI3->SR; 
  uint8_t rcv_data = rcv_data_buf[rcv_count++];
  if (rcv_count >1023) rcv_count = 1023;
    //return *RD_pointer ;
return (rcv_data);
  
 // if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXE))
 //     {
 //       *((__IO uint8_t *)&hspi->Instance->DR) = (*hspi->pTxBuffPtr);  // transmit 8 bits
}
  