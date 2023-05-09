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
#include "ff.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
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
                          
// BTT SKR PRO has only one LED at PA7
// The 407 black board has two at PA6 and PA7


//#define WORK_LED_Pin GPIO_PIN_2
//#define WORK_LED_GPIO_Port GPIOB

/* D2 */
#define LED_G1_Port GPIOA
#define LED_G1_Pin  GPIO_PIN_7

/* D4 */
#define LED_G2_Port GPIOA
#define LED_G2_Pin  GPIO_PIN_7

/* D2 */
#define D2_LED_G2_GPIO_Port LED_G1_Port 
#define D2_LED_G2_Pin       LED_G1_Pin  

/* D4 */
#define D4_LED_G2_GPIO_Port LED_G2_Port 
#define D4_LED_G2_Pin       LED_G2_Pin  

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
    
// SD card pins

// SD_detect: PB11 
#define SD_detect_Pin  GPIO_PIN_11
#define SD_detect_Port GPIOB
// SDSS: PA4
#define SDSS_Pin  GPIO_PIN_4
#define SDSS_Port GPIOA
// SCK:  PA5
#define SCK_Pin  GPIO_PIN_5
#define SCK_Port GPIOA
// MOSI: PB5
#define MOSI_Pin  GPIO_PIN_5
#define MOSI_Port GPIOB
// MISO: PA6
#define MISO_Pin  GPIO_PIN_6
#define MISO_Port GPIOA

// defines for use in user_diskio_spi.c
#define SD_SPI_HANDLE hspi1
#define SD_CS_GPIO_Port SDSS_Port
#define SD_CS_Pin SDSS_Pin

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
