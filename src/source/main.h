/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "stm32g474xx_hal.h"
#include "kprint.h"
#include "gpio.h"
#include "stm32g4xx_hal_def.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

void Error_Handler(void);

/*** Application-Specific Configuration ***************************************/
/* File name of application located on SD card */
#define CONF_FILENAME "FIRMWARE.bin"
#define FILE_EXT_CHANGE "CUR"
/* For development/debugging: print messages to ST-LINK VCP */
#define USE_VCP 1
/******************************************************************************/


/* Hardware Defines ----------------------------------------------------------*/
                          



// D2_LED_G1_Pin PE5
// Detect_SDIO_Pin PC1
// SD_MISO_Pin PC8
// SDSS_Pin PC11
// SD_SCK_Pin PC12
// SD_MOSI_Pin PD2
// D4_LED_G2_Pin PB5

#define GPIO_PIN__0    0
#define GPIO_PIN__1    1
#define GPIO_PIN__2    2
#define GPIO_PIN__3    3
#define GPIO_PIN__4    4
#define GPIO_PIN__5    5
#define GPIO_PIN__6    6
#define GPIO_PIN__7    7
#define GPIO_PIN__8    8
#define GPIO_PIN__9    9
#define GPIO_PIN__10    10
#define GPIO_PIN__11    11
#define GPIO_PIN__12    12
#define GPIO_PIN__13    13
#define GPIO_PIN__14    14
#define GPIO_PIN__15    15

#define D2_LED_G1_Pin GPIO_PIN__5
#define D2_LED_G1_GPIO_Port PORTA
#define D4_LED_G2_Pin GPIO_PIN__6
#define D4_LED_G2_GPIO_Port PORTA

//#define SOFT_SPI       // select if use soft spi or hard spi routines  

#define Detect_SDIO_Pin GPIO_PIN__5
#define Detect_SDIO_GPIO_Port PORTB
#define SD_MISO_Pin GPIO_PIN__14
#define SD_MISO_GPIO_Port PORTB
#define SDSS_Pin GPIO_PIN__12
#define SDSS_GPIO_Port PORTB
#define SD_SCK_Pin GPIO_PIN__13
#define SD_SCK_GPIO_Port PORTB
#define SD_MOSI_Pin GPIO_PIN__15
#define SD_MOSI_GPIO_Port PORTB


  // USART 2       USART 1       SPI 2          
  // PA2  AF7 TX   PA9  AF7 TX   PB12 AF5 SS    
  // PA3  AF7 Rx   PA10 AF7 Rx   PB13 AF5 CLK   
  //                             PB14 AF5 MISO  
  //                             PB15 AF5 MOSI  
  //
  // PB5 SD_detect
  //
  // PA5 LED, green (built in)
  // PA6 LED, yellow 
  //
  // PC13 button
  //
  // PA13 SWDIO  AF0
  // PA14 SWCLK  AF0
  // PB3  SWO    AF0

/* LD2 */
#define LED_G1_Port D2_LED_G1_GPIO_Port
#define LED_G1_Pin  D2_LED_G1_Pin

/* LD4 */
#define LED_G2_Port D4_LED_G2_GPIO_Port
#define LED_G2_Pin  D4_LED_G2_Pin


/* Enumerations --------------------------------------------------------------*/
/* Error codes */
enum eApplicationErrorCodes
{
    ERR_OK = 0,
    ERR_WRP_ACTIVE,
    ERR_SD_INIT,
    ERR_SD_MOUNT,
    ERR_SD_FILE,
    ERR_APP_LARGE,
    ERR_FLASH,
    ERR_VERIFY,
    ERR_OBP,
};

/* Hardware Macros -----------------------------------------------------------*/
#define LED_G1_ON()  gpio_wr(D2_LED_G1_GPIO_Port, D2_LED_G1_Pin, GPIO_PIN_RESET)
#define LED_G1_OFF() gpio_wr(D2_LED_G1_GPIO_Port, D2_LED_G1_Pin, GPIO_PIN_SET)
#define LED_G1_TG()  gpio_wr(D2_LED_G1_GPIO_Port, D2_LED_G1_Pin, !gpio_rd(D2_LED_G1_GPIO_Port, D2_LED_G1_Pin))
#define LED_G2_ON()  gpio_wr(D4_LED_G2_GPIO_Port, D4_LED_G2_Pin, GPIO_PIN_RESET)
#define LED_G2_OFF() gpio_wr(D4_LED_G2_GPIO_Port, D4_LED_G2_Pin, GPIO_PIN_SET)
#define LED_G2_TG()  gpio_wr(D4_LED_G2_GPIO_Port, D4_LED_G2_Pin, !gpio_rd(D4_LED_G2_GPIO_Port, D4_LED_G2_Pin))

#define LED_ALL_ON() \
    do               \
    {                \
        LED_G1_ON(); \
        LED_G2_ON(); \
    } while(0)
#define LED_ALL_OFF() \
    do                \
    {                 \
        LED_G1_OFF(); \
        LED_G2_OFF(); \
    } while(0)
#define LED_ALL_TG() \
    do               \
    {                \
        LED_G1_TG(); \
        LED_G2_TG(); \
    } while(0)

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

// Soft SPI

void spiBegin(void);
void spiInit(uint8_t spiRate);
uint8_t SOFT_SPI_STM32_SpiTransfer_Mode_3(uint8_t b);


//#define WRITE(PIN, DATA)  {HAL_GPIO_WritePin(PIN, DATA);}
//#define READ(PIN)         {HAL_GPIO_ReadPin(PIN);}
//#define SD_MOSI_PIN (SD_MOSI_GPIO_Port, SD_MOSI_Pin)
//#define SD_SCK_PIN (GPIOC, SD_SCK_Pin)
//#define SD_SS_PIN (GPIOC, SDSS_Pin)
//#define SD_MISO_PIN (GPIOC, SD_MISO_Pin)

#define WRITE_SD_MOSI_PIN(DATA)  gpio_wr(SD_MOSI_GPIO_Port, SD_MOSI_Pin, (DATA))
#define WRITE_SD_SCK_PIN(DATA)   gpio_wr(SD_SCK_GPIO_Port, SD_SCK_Pin, (DATA))
#define WRITE_SD_SS_PIN(DATA)    gpio_wr(SDSS_GPIO_Port, SDSS_Pin, (DATA))
#define READ_SD_MISO_PIN()       gpio_rd(SD_MISO_GPIO_Port, SD_MISO_Pin)


#define WRITE_SD_SCK_PIN_RESET  GPIOD->BSRR = 0x01000000 // PD8
#define WRITE_SD_SCK_PIN_SET    GPIOD->BSRR = 0x00000100 // PD8

#define WRITE_SD_MOSI_PIN_RESET GPIOA->BSRR = 0x00040000 // PA2
#define WRITE_SD_MOSI_PIN_SET   GPIOA->BSRR = 0x00000004 // PA2

#define SD_MISO_PIN_READ     ((GPIOD->IDR & 0x00000020) ? 1:0) // PD5

#define CS_HIGH()	{WRITE_SD_SS_PIN(GPIO_PIN_SET);}
#define CS_LOW()	{WRITE_SD_SS_PIN(GPIO_PIN_RESET);}

#define FCLK_SLOW() { or16(RSPI1_CR1, (6<<3)); }	/* Set SCLK = slow, approx 300 KBits/s*/
#define FCLK_FAST() { or16(RSPI1_CR1, (3<<3)); }	/* Set SCLK = fast, approx 2.5 MBits/s */


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
