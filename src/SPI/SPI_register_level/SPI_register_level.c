// https://controllerstech.com/spi-using-registers-in-stm32/

#include "integer.h" //from FatFs middleware library
//#include "SPI_register_level_defines.h"
#include "stm32g474x_defines.h"
#include "basic.h"
#include "gpio.h"
#include "main.h"
#include <stdint.h>

#ifndef SOFT_SPI  // enable hard spi routines
  
extern uint32_t k_ticks(void);
void k_delay(const uint32_t ms);

BYTE SPI_Transfer(uint16_t data);




void SPIConfig (void)
{
  /************** STEPS TO FOLLOW *****************
	1. Enable SPI clock
	2. Configure the Control Register 1
	3. Configure the CR2
	************************************************/	
  
  
  // enable 
  or16(RSPI2_CR2, (1<<2));  //  Set SSOE bit to 1
  wr16(RSPI2_CR1, 0); //  Set all bits (including SSM) to 0 
  //or16(RSPI2_CR2, (1<<3));  //  Set NSSP bit to 1  (optional)
  
  while (rd16(RSPI2_SR) & (1<<5)) {  // writing to some registers is disabled if Mode fault (MODF) bit is active
     wr16(RSPI2_CR1, 0);      // clearing sequence is to read or write the status register and then write CR1
  }                               //  SDSS (NSSP) must be inactive for this to work
    
  //RCC_APB2ENR |= (1<<12);  // Enable SPI2 CLock  (already done in init.c)
  //SPI2->CR1 |= (3);   // CPOL=1, CPHA=1 - SD cards are usually 0,0
  //SPI2->CR1 |= (2);   // CPOL=1, CPHA=1 - SD cards are usually 0,0
  //SPI2->CR1 |= (1);   // CPOL=1, CPHA=1 - SD cards are usually 0,0
  or16(RSPI2_CR1, (1<<8)); // set internal SSI bit to 1 before enabling Master mode or will get a Mode fault (MODF) error
  or16(RSPI2_CR1, (1<<9)); // enable SSM before enabling Master mode or will get a Mode fault (MODF) error
  or16(RSPI2_CR1, (7<<3)); // add in bit rate
  //or16(RSPI2_CR1, (1<<2));  // Master Mode
  //and16(RSPI2_CR1, ~(1<<7));  // LSBFIRST = 0, MSB first
  //or16(RSPI2_CR1, (1<<8) | (1<<9));  // SSM=1, SSi=1 _ Software Slave Management
  //and16(RSPI2_CR1, ~(1<<10));  // RXONLY = 0, full-duplex
  //and16(RSPI2_CR1, ~(1<<11));  // DFF=0, 8 bit data
  //or16(RSPI2_CR1, (1<<14));  // BIDIOE output enable
  wr16(RSPI2_CR2, ((1 << 12) | (7<<8)));   // 8 bit transfer, Motorola format
  //wr16(RSPI2_CR2,  (7<<8));   // 16 bit transfer, Motorola format
  
  k_delay(2);  // try to keep optimizing compiler from executing the following out of order
 // and16(RSPI2_CR2, ~(1<<2));  //  Set SSOE bit to 0
  or16(RSPI2_CR1, 1<<2); // enable Master mode
  
  k_delay(2);  // try to keep optimizing compiler from executing the following out of order
  or16(RSPI2_CR1, (1<<6));   // SPE=1, Peripheral enabled
}


void k_delay(const uint32_t ms);;

void SPI_GPIOConfig (void)
{ 
  
  // don't forget to update main.h
  
   //               virtual com port     
  // USART 3        USART 1       SPI 2         
  // PB9  AF7 TX    PA9  AF7 TX   PB12 AF5 SS   
  // PB8  AF7 Rx    PA10 AF7 Rx   PB13 AF5 CLK  
  //                              PB14 AF5 MISO 
  //                              PB15 AF5 MOSI 
  //
  // PB5 SD_detect
  //
  // PA5 LED, green (built in)
  // PA7 LED, yellow 
  //
  // PC13 button
  //
  // PA13 SWDIO  AF0
  // PA14 SWCLK  AF0
  // PB3  SWO    AF0
  
	//RCC_AHB1ENR |= (1<<0);  // Enable GPIO Clock (already done in init.c)
	
  gpio_wr(PORTB, 12, GPIO_PIN_SET); // set SDSS high (inactive)
  
 GPIO_CONFIG_OUTPUT(PORTB, 12,  GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);    // let software contol SDSS pin
 //GPIO_CONFIG_ALT(PORTB,  12, 5, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);  // let SPI control SDSS pin
 GPIO_CONFIG_ALT(PORTB,  13, 5, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);
 GPIO_CONFIG_ALT(PORTB,  14, 5, GPIO_PULL_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);
 GPIO_CONFIG_ALT(PORTB,  15, 5, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);
 GPIO_CONFIG_INPUT(PORTB, 5,  GPIO_NO_PULL_UP_DOWN);
  
  //GPIO_CONFIG_OUTPUT(PORTB, 9,  GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);  
  //GPIO_CONFIG_OUTPUT(PORTB, 8,  GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);  
  //GPIO_CONFIG_OUTPUT(PORTB, 5,  GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);  
  //GPIO_CONFIG_OUTPUT(PORTB, 2,  GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED); 
  //
  //gpio_wr(PORTB, 9, GPIO_PIN_RESET); // set SDSS low
  //k_delay(20);                               
  //gpio_wr(PORTB, 8, GPIO_PIN_RESET); // set SCK  low
  //k_delay(20);                             
  //gpio_wr(PORTB, 5, GPIO_PIN_RESET); // set MOSIlow
  //k_delay(20);                           
  //gpio_wr(PORTB, 2, GPIO_PIN_RESET); // set SDSS low
  //k_delay(20);
  //
  //gpio_wr(PORTB, 9, GPIO_PIN_SET); // set SDSS high
  //k_delay(20);                          
  //gpio_wr(PORTB, 8, GPIO_PIN_SET); // set SCK  high
  //k_delay(20);                              
  //gpio_wr(PORTB, 5, GPIO_PIN_SET); // set MOSI high
  //k_delay(20);                          
  //gpio_wr(PORTB, 2, GPIO_PIN_SET); // set SDSS high
  //
  //k_delay(20);
  //gpio_wr(PORTB, 9, GPIO_PIN_RESET); // set SDSS low
  //k_delay(20);                               
  //gpio_wr(PORTB, 8, GPIO_PIN_RESET); // set SCK  low
  //k_delay(20);                             
  //gpio_wr(PORTB, 5, GPIO_PIN_RESET); // set MOSIlow
  //k_delay(20);                           
  //gpio_wr(PORTB, 2, GPIO_PIN_RESET); // set SDSS low
  
}

void SPI_Enable (void)
{
	or16(RSPI2_CR1, (1<<6));   // SPE=1, Peripheral enabled
}

void SPI_Disable (void)
{
	and16(RSPI2_CR1,  ~(1<<6));   // SPE=0, Peripheral Disabled
}

void CS_Enable (void)
{
  wr32(R_GPIOD_BSRR, (1<<9)<<16);
  //or16(RSPI2_CR1, (1<<8)); // set internal SSI bit  high
  and16(RSPI2_CR1, ~((1<<8)));  // set internal SSI bit low which should force the physical pin low
}

void CS_Disable (void)
{
  wr32(R_GPIOD_BSRR, (1<<9));
  or16(RSPI2_CR1, (1<<8)); // set internal SSI bit  high
  //and16(RSPI2_CR1, ~((1<<8)));  // set internal SSI bit low which should force the physical pin low
}

//void SPI_Transmit (BYTE *data, UINT size)
void xmit_spi_multi (BYTE *data, UINT size)
{
	
	/************** STEPS TO FOLLOW *****************
	1. Wait for the TXE bit to set in the Status Register
	2. Write the data to the Data Register
	3. After the data has been transmitted, wait for the BSY bit to reset in Status Register
	4. Clear the Overrun flag by reading DR and SR
  
  Note that the orignal code had the buffer filling from back to front
  
	************************************************/		
	#if 0
  
  //  SPI data register is 16 bits but incoming data is 8 bit
  
  //wr16(RSPI2_CR2,  (7<<8));   // 16 bit transfer, Motorola format
  
	UINT i=0;
  uint16_t* RD_pointer = (uint16_t*)RSPI2_DR_8;  // pointer to SPI data register
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
     
     while (!((rd16(RSPI2_SR)&(1<<1)))) {};  // wait for TXE bit to set _ This will indicate that the buffer is empty
     *RD_pointer = data_16;  // send the data
	}	
	
	
/*During discontinuous communications, there is a 2 APB clock period delay between the
write operation to the SPI_DR_8 register and BSY bit setting. As a consequence it is
mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
data.
*/
	//while (!((rd16(RSPI2_SR)&(1<<1)))) {};  // wait for TXE bit to set _ This will indicate that the buffer is empty
	//while ((rd16(RSPI2_SR)&(1<<7))) {};  // wait for BSY bit to Reset _ This will indicate that SPI is not busy in communication	
	
	//  Clear the Overrun flag by reading DR and SR
	data_16 = *RD_pointer;
	data_16 = rd16(RSPI2_SR);
  
  //wr16(RSPI2_CR2, ((1 << 12) | (7<<8)));   // back to 8 bit transfer mode, Motorola format
#endif
  
  
 for (uint16_t i = 0; i < size; i++) {  // try putting data into buffer in order received
	{
		while (((rd16(RSPI2_SR))&(1<<7))) {};  // wait for BSY bit to Reset _ This will indicate that SPI is not busy in communication
		  // send dummy data
//		while (!((rd16(RSPI2_SR)) &(1<<0))){};  // Wait for RXNE to set _ This will indicate that the Rx buffer is not empty
	  SPI_Transfer(data[i]);
  }
 }
 
  
}


//void SPI_Receive (BYTE *data, UINT size)
void rcvr_spi_multi (BYTE *data, UINT size)
{
	/************** STEPS TO FOLLOW *****************
	1. Wait for the BSY bit to reset in Status Register
	2. Send some Dummy data before reading the DATA
	3. Wait for the RXNE bit to Set in the status Register
	4. Read data from Data Register
  
  Note that the orignal code had the buffer filling from back to front
  
	************************************************/		
#if 0  // original code
// wr16(RSPI2_CR2,  (7<<8));   // 16 bit transfer, Motorola format
  uint8_t* RD_pointer = (uint8_t*)RSPI2_DR_8;
  // input array is data[0] ... data[size-1]
	while (size)
	{
		while (((rd16(RSPI2_SR))&(1<<7))) {};  // wait for BSY bit to Reset _ This will indicate that SPI is not busy in communication
		*RD_pointer = 0;  // send dummy data
//		while (!((rd16(RSPI2_SR)) &(1<<0))){};  // Wait for RXNE to set _ This will indicate that the Rx buffer is not empty
	  data[size - 1] = RD_pointer[0];
    size--;
    if (size != 0) {
      data[size - 1] = RD_pointer[1];  // move MSB into data buffer if past the end
      size--;
    }
    
	}	
  wr16(RSPI2_CR2, ((1 << 12) | (7<<8)));   // back to 8 bit transfer mode, Motorola format
#endif

 for (uint16_t i = 0; i < size; i++) {  // try putting data into buffer in order received
	{
		while (((rd16(RSPI2_SR))&(1<<7))) {};  // wait for BSY bit to Reset _ This will indicate that SPI is not busy in communication
		  // send dummy data
//		while (!((rd16(RSPI2_SR)) &(1<<0))){};  // Wait for RXNE to set _ This will indicate that the Rx buffer is not empty
	  data[i] = SPI_Transfer(0);
  }
    
 }

}


__attribute__((weak))
BYTE SPI_Transfer(uint16_t data) {
  uint32_t timeout = k_ticks() + 10;  // 10 ms timeout
  uint8_t* RD_pointer = (uint8_t*)RSPI2_DR_8;
  while (!((rd16(RSPI2_SR))&(0x82)) && (timeout > k_ticks())) {};  // wait for TXE bit to set low. This will indicate that the buffer is empty
  if(timeout <= k_ticks()) {
    return 0;
  }
  *RD_pointer =  (data & 0x00FF);  // load the data into the Data Register
  timeout = k_ticks() + 10;  // 10 ms timeout
  while (!((rd16(RSPI2_SR)) &(1<<0)) && (timeout > k_ticks())){};  // Wait for RXNE to set low. This will indicate that the Rx buffer is not empty
  uint8_t rcv_data = *RD_pointer ;
  if(timeout <= k_ticks()) {
    return 0; 
  }
  return (rcv_data);
  
}
#endif // hard spi
  