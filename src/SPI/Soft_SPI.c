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


  void spiBegin(void) {
    WRITE_SD_SS_PIN(GPIO_PIN_SET);
    WRITE_SD_SCK_PIN(GPIO_PIN_SET);
    WRITE_SD_MOSI_PIN(GPIO_PIN_SET);
    //HAL_TIM_Base_Start(&htim1); // start delay counter
    //spiInit(SPI_SPEED_6);
  }

  // Use function with compile-time value so we can actually reach the desired frequency
  //void (*delaySPIFunc)();
  //void delaySPI_125()  { delay_ns(125); }
  //void delaySPI_250()  { delay_ns(250); }
  //void delaySPI_500()  { delay_ns(500); }
  //void delaySPI_1000() { delay_ns(1000); }
  //void delaySPI_2000() { delay_ns(2000); }
  //void delaySPI_4000() { delay_ns(4000); }
  //
  //void spiInit(uint8_t spiRate) {
  //  // Use datarates Marlin uses
  //  switch (spiRate) {
  //    case SPI_FULL_SPEED:   delaySPIFunc =  &delaySPI_125; break;  // desired: 8,000,000  actual: ~1.1M
  //    case SPI_HALF_SPEED:   delaySPIFunc =  &delaySPI_125; break;  // desired: 4,000,000  actual: ~1.1M
  //    case SPI_QUARTER_SPEED:delaySPIFunc =  &delaySPI_250; break;  // desired: 2,000,000  actual: ~890K
  //    case SPI_EIGHTH_SPEED: delaySPIFunc =  &delaySPI_500; break;  // desired: 1,000,000  actual: ~590K
  //    case SPI_SPEED_5:      delaySPIFunc = &delaySPI_1000; break;  // desired:   500,000  actual: ~360K
  //    case SPI_SPEED_6:      delaySPIFunc = &delaySPI_2000; break;  // desired:   250,000  actual: ~210K
  //    default:               delaySPIFunc = &delaySPI_4000; break;  // desired:   125,000  actual: ~123K
  //  }
  //}



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


