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
#ifndef __MAIN_BOOT_H
#define __MAIN_BOOT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"

extern UART_HandleTypeDef huart1;

extern char msg[64];

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/*** Application-Specific Configuration ***************************************/
/* File name of application located on SD card */
#define CONF_FILENAME "firmware.bin"
#define FILE_EXT_CHANGE "CUR"
/* For development/debugging: print messages to ST-LINK VCP */
#define USE_VCP 1
/******************************************************************************/

/* Hardware Defines ----------------------------------------------------------*/


//#define WORK_LED_Pin GPIO_PIN_2
//#define WORK_LED_GPIO_Port GPIOB

/* LD2 */
#define LED_G1_Port D2_LED_G2_GPIO_Port
#define LED_G1_Pin  D2_LED_G2_Pin

/* LD3 */
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
#define LED_G1_ON()  HAL_GPIO_WritePin(LED_G1_Port, LED_G1_Pin, GPIO_PIN_RESET)
#define LED_G1_OFF() HAL_GPIO_WritePin(LED_G1_Port, LED_G1_Pin, GPIO_PIN_SET)
#define LED_G1_TG()  HAL_GPIO_TogglePin(LED_G1_Port, LED_G1_Pin)
#define LED_G2_ON()  HAL_GPIO_WritePin(LED_G2_Port, LED_G2_Pin, GPIO_PIN_RESET)
#define LED_G2_OFF() HAL_GPIO_WritePin(LED_G2_Port, LED_G2_Pin, GPIO_PIN_SET)
#define LED_G2_TG()  HAL_GPIO_TogglePin(LED_G2_Port, LED_G2_Pin)

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

#define WRITE_SD_MOSI_PIN(DATA)  HAL_GPIO_WritePin(SD_MOSI_GPIO_Port, SD_MOSI_Pin, (DATA))
#define WRITE_SD_SCK_PIN(DATA)   HAL_GPIO_WritePin(GPIOC, SD_SCK_Pin, (DATA))
#define WRITE_SD_SS_PIN(DATA)    HAL_GPIO_WritePin(GPIOC, SDSS_Pin, (DATA))
#define READ_SD_MISO_PIN()       HAL_GPIO_ReadPin(GPIOC, SD_MISO_Pin)


#define WRITE_SD_SCK_PIN_RESET  GPIOC->BSRR = 0x10000000 // PC12
#define WRITE_SD_SCK_PIN_SET    GPIOC->BSRR = 0x00001000 // PC12

#define WRITE_SD_MOSI_PIN_RESET GPIOD->BSRR = 0x00040000 // PD2
#define WRITE_SD_MOSI_PIN_SET   GPIOD->BSRR = 0x00000004 // PD2

#define SD_MISO_PIN_READ     ((GPIOC->IDR & 0x00000100) ? 1:0) // PC8

#define CS_HIGH()	{WRITE_SD_SS_PIN(GPIO_PIN_SET);}
#define CS_LOW()	{WRITE_SD_SS_PIN(GPIO_PIN_RESET);}

enum Spi_Speed
{
  SPI_FULL_SPEED = 0,
  SPI_HALF_SPEED,
  SPI_QUARTER_SPEED,
  SPI_EIGHTH_SPEED,
  SPI_SPEED_5,
  SPI_SPEED_6,
};


//#define FCLK_SLOW() { spiInit(SPI_QUARTER_SPEED); }	/* Set SCLK = slow, approx 280 KBits/s*/
//#define FCLK_FAST() { spiInit(SPI_FULL_SPEED); }	/* Set SCLK = fast, approx 1 MBits/s */
#define FCLK_SLOW() {}  // STM32F103 is so slow can only run at the slow speed
#define FCLK_FAST() {}	// STM32F103 is so slow can only run at the slow speed

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler_Boot(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_BOOT_H */
