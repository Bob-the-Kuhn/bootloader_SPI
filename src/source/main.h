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
#include "stm32f1xx_hal.h"
#include "kprint.h"
#include "gpio.h"

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
                          
// BTT SKR PRO has only one LED at PA7
// The 407 black board has two at PA6 and PA7
// The 103 black board has two at PE5 and PB5


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
#define D2_LED_G1_GPIO_Port PORTE
#define Detect_SDIO_Pin GPIO_PIN__1
#define Detect_SDIO_GPIO_Port PORTC
#define SD_MISO_Pin GPIO_PIN__8
#define SD_MISO_GPIO_Port PORTC
#define SDSS_Pin GPIO_PIN__11
#define SDSS_GPIO_Port PORTC
#define SD_SCK_Pin GPIO_PIN__12
#define SD_SCK_GPIO_Port PORTC
#define SD_MOSI_Pin GPIO_PIN__2
#define SD_MOSI_GPIO_Port PORTD
#define D4_LED_G2_Pin GPIO_PIN__5
#define D4_LED_G2_GPIO_Port PORTB


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
#define LED_G1_ON()  gpio_wr(IO(D2_LED_G1_GPIO_Port, D2_LED_G1_Pin), GPIO_PIN_RESET)
#define LED_G1_OFF() gpio_wr(IO(D2_LED_G1_GPIO_Port, D2_LED_G1_Pin), GPIO_PIN_SET)
#define LED_G1_TG()  gpio_wr(IO(D2_LED_G1_GPIO_Port, D2_LED_G1_Pin), !gpio_rd(IO(D2_LED_G1_GPIO_Port, D2_LED_G1_Pin)))
#define LED_G2_ON()  gpio_wr(IO(D4_LED_G2_GPIO_Port, D4_LED_G2_Pin), GPIO_PIN_RESET)
#define LED_G2_OFF() gpio_wr(IO(D4_LED_G2_GPIO_Port, D4_LED_G2_Pin), GPIO_PIN_SET)
#define LED_G2_TG()  gpio_wr(IO(D4_LED_G2_GPIO_Port, D4_LED_G2_Pin), !gpio_rd(IO(D4_LED_G2_GPIO_Port, D4_LED_G2_Pin)))

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

#define WRITE_SD_MOSI_PIN(DATA)  gpio_wr(IO(SD_MOSI_GPIO_Port, SD_MOSI_Pin), (DATA))
#define WRITE_SD_SCK_PIN(DATA)   gpio_wr(IO(SD_SCK_GPIO_Port, SD_SCK_Pin), (DATA))
#define WRITE_SD_SS_PIN(DATA)    gpio_wr(IO(SDSS_GPIO_Port, SDSS_Pin), (DATA))
#define READ_SD_MISO_PIN()       gpio_rd(IO(SD_MISO_GPIO_Port, SD_MISO_Pin))


#define WRITE_SD_SCK_PIN_RESET  GPIOC->BSRR = 0x10000000 // PC12
#define WRITE_SD_SCK_PIN_SET    GPIOC->BSRR = 0x00001000 // PC12

#define WRITE_SD_MOSI_PIN_RESET GPIOD->BSRR = 0x00040000 // PD2
#define WRITE_SD_MOSI_PIN_SET   GPIOD->BSRR = 0x00000004 // PD2

#define SD_MISO_PIN_READ     ((GPIOC->IDR & 0x00000100) ? 1:0) // PC8

#define CS_HIGH()	{WRITE_SD_SS_PIN(GPIO_PIN_SET);}
#define CS_LOW()	{WRITE_SD_SS_PIN(GPIO_PIN_RESET);}

#define FCLK_SLOW() { }  // soft spi only runs at one (slow) speed
#define FCLK_FAST() { }  // soft spi only runs at one (slow) speed

enum Spi_Speed
{
  SPI_FULL_SPEED = 0,
  SPI_HALF_SPEED,
  SPI_QUARTER_SPEED,
  SPI_EIGHTH_SPEED,
  SPI_SPEED_5,
  SPI_SPEED_6,
};






/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
