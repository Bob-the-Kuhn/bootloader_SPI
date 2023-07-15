/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2017 Victor Perez
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

// STM32F103 is so slow can only run at the slow speed

// In order to do that, special single instruction defines were created to
// eliminate all subroutine calls when setting/resetting the soft SPI pins
// in the SOFT_SPI_STM32_SpiTransfer_Mode_3 routine.  Even then it only hits
// about 300KHz.

// The framework was left in place incase this routine is ever used with a
// faster processor.


#include "main.h"
#include "stdint.h"
#include "gpio.h"

#ifdef SOFT_SPI

void spiInit(uint8_t spiRate);
// ------------------------
// Public Variables
// ------------------------


// ------------------------
// Public functions
// ------------------------

  // ------------------------
  // Software SPI
  // ------------------------

  //void delay_ns (uint16_t ns)
  //{
  //  uint16_t delay = ns/125;  // 8MHz clock
  //  __HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
  //  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);  // wait for the counter to reach the delay input in the parameter
  //}

  // SPI 3             SPI 2          SPI 1
  // PA4  AF9 SS       PB9 AF5 SS     PD9 AF5 SS
  // PC10 AF4 CLK      PB8 AF5 CLK    PD8 AF1 CLK
  // PC11 AF4 MISO     PC2 AF1 MISO   PD5 AF1 MISO
  // PC12 AF4 MOSI     PC3 AF1 MOSI   PA2 AF0 MOSI
  
  // PB11 SD_detect

void k_delay(const uint32_t ms);

  void spiBegin(void) {

    WRITE_SD_SS_PIN(GPIO_PIN_SET);     // set initial states
    WRITE_SD_SCK_PIN(GPIO_PIN_RESET);
    WRITE_SD_MOSI_PIN(GPIO_PIN_SET);    
    
    GPIO_CONFIG_OUTPUT(PORTD, 8, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);  // SCK
    GPIO_CONFIG_INPUT(PORTD,  5, GPIO_NO_PULL_UP_DOWN);                                                      // MISO
    GPIO_CONFIG_OUTPUT(PORTA, 2, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);  // MOSI
    GPIO_CONFIG_OUTPUT(PORTD, 9, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);  // SDSS
    GPIO_CONFIG_INPUT(PORTB, 11,  GPIO_NO_PULL_UP_DOWN);                                                     // SD detect
  
  }

  


  uint8_t SPI_Transfer(uint16_t data_in) {
  //uint8_t SOFT_SPI_STM32_SpiTransfer_Mode_0(uint8_t b) { // using Mode 0     
    uint8_t data_out = 0;
    for (uint8_t bits = 8; bits--;) {                                        // 
      
                                               
      if (data_in & 0x80) WRITE_SD_MOSI_PIN_SET;                        //  WRITE_SD_MOSI
      else WRITE_SD_MOSI_PIN_RESET;
 
        WRITE_SD_SCK_PIN_RESET;                                               //  WRITE_SD_SCK_PIN_RESET (delay for setup)
        if (data_in & 0x80) WRITE_SD_MOSI_PIN_SET;                            //  WRITE_SD_MOSI          (delay for setup)
        else WRITE_SD_MOSI_PIN_RESET;
        WRITE_SD_SCK_PIN_RESET;                                               //  WRITE_SD_SCK_PIN_RESET (delay for setup)
        if (data_in & 0x80) WRITE_SD_MOSI_PIN_SET;                            //  WRITE_SD_MOSI          (delay for setup)
        else WRITE_SD_MOSI_PIN_RESET;
        WRITE_SD_SCK_PIN_RESET;                                               //  WRITE_SD_SCK_PIN_RESET (delay for setup)
        if (data_in & 0x80) WRITE_SD_MOSI_PIN_SET;                            //  WRITE_SD_MOSI          (delay for setup)
        else WRITE_SD_MOSI_PIN_RESET;
        WRITE_SD_SCK_PIN_RESET;                                               //  WRITE_SD_SCK_PIN_RESET (delay for setup)
        if (data_in & 0x80) WRITE_SD_MOSI_PIN_SET;                            //  WRITE_SD_MOSI          (delay for setup)
        else WRITE_SD_MOSI_PIN_RESET;
        WRITE_SD_SCK_PIN_RESET;                                               //  WRITE_SD_SCK_PIN_RESET (delay for setup)
        if (data_in & 0x80) WRITE_SD_MOSI_PIN_SET;                            //  WRITE_SD_MOSI          (delay for setup)
        else WRITE_SD_MOSI_PIN_RESET;
        WRITE_SD_SCK_PIN_RESET;                                               //  WRITE_SD_SCK_PIN_RESET (delay for setup)
        if (data_in & 0x80) WRITE_SD_MOSI_PIN_SET;                            //  WRITE_SD_MOSI          (delay for setup)
        else WRITE_SD_MOSI_PIN_RESET;
        WRITE_SD_SCK_PIN_RESET;                                               //  WRITE_SD_SCK_PIN_RESET (delay for setup)
        if (data_in & 0x80) WRITE_SD_MOSI_PIN_SET;                            //  WRITE_SD_MOSI          (delay for setup)
        else WRITE_SD_MOSI_PIN_RESET;
        
        
      data_in <<= 1; 
      data_out <<= 1; 
      WRITE_SD_SCK_PIN_SET;                                            // WRITE_SD_SCK_PIN_SET
      

        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)     
        data_out |= (SD_MISO_PIN_READ != 0);                                  //  READ_SD_MISO_PIN
        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)
        data_out |= (SD_MISO_PIN_READ != 0);                                  
        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)
        data_out |= (SD_MISO_PIN_READ != 0);                                  
        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)
        data_out |= (SD_MISO_PIN_READ != 0);                                  
        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)
        data_out |= (SD_MISO_PIN_READ != 0);                                  
        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)
        data_out |= (SD_MISO_PIN_READ != 0);                                  
        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)
        data_out |= (SD_MISO_PIN_READ != 0);                                  
        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)
        data_out |= (SD_MISO_PIN_READ != 0);                                  
        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)
        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)
        data_out |= (SD_MISO_PIN_READ != 0);                                  
        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)
        data_out |= (SD_MISO_PIN_READ != 0);                                  
        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)
        data_out |= (SD_MISO_PIN_READ != 0);                                  
        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)
        data_out |= (SD_MISO_PIN_READ != 0);                                  
        WRITE_SD_SCK_PIN_SET;                                                 // WRITE_SD_SCK_PIN_SET (delay)
        
      data_out |= (SD_MISO_PIN_READ != 0);                             // read MISO
      WRITE_SD_SCK_PIN_RESET;                                          //  WRITE_SD_SCK_PIN_RESET
      

 
                                                                       
      //delaySPIFunc();                                                      //
      //delaySPIFunc();
    }
    //delay_ns(125);
    return data_out;
  }




  uint8_t SOFT_SPI_STM32_SpiTransfer_Mode_3(uint8_t b) { // using Mode 3     //  WRITE_SD_SCK_PIN_RESET
    for (uint8_t bits = 8; bits--;) {                                        //  WRITE_SD_SCK_PIN_SET
      //WRITE_SD_SCK_PIN(GPIO_PIN_RESET);                                    //
      WRITE_SD_SCK_PIN_RESET;                                                //

      //WRITE_SD_MOSI_PIN(b & 0x80);                                         //  WRITE_SD_MOSI_PIN_RESET
      if (b & 0x80) WRITE_SD_MOSI_PIN_SET;
      else WRITE_SD_MOSI_PIN_RESET;
                                                                             //  WRITE_SD_MOSI_PIN_SET
      //delaySPIFunc();                                                      //
      WRITE_SD_SCK_PIN_SET;
      //delaySPIFunc();

      b <<= 1;        // little setup time
      //b |= (READ_SD_MISO_PIN() != 0);                                     //  READ_SD_MISO_PIN
      b |= (SD_MISO_PIN_READ != 0);
    }
    //delay_ns(125);
    return b;
  }
#endif  // softspi
