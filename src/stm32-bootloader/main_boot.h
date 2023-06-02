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
//#include "stm32f4xx_hal.h"
//#include "kprint.h"

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

//Nucleo_F746  has three LEDs
// LD1_PIN                          PB0  // green
// LD2_PIN                          PB7  // blue
// LD3_PIN                          PB14  // red

//#define WORK_LED_Pin GPIO_PIN_2
//#define WORK_LED_GPIO_Port GPIOB

/* LD1 */
#define LED_G1_Port GPIOB
#define LED_G1_Pin  GPIO_PIN_0

/* LD2 */
#define LED_G2_Port GPIOB
#define LED_G2_Pin  GPIO_PIN_7

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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

// SD_DETECT_PIN   PG2
#define Detect_SD_Pin GPIO_PIN_2
#define Detect_SD_GPIO_Port GPIOG

#define SD_PRESENT               ((uint8_t)0x01)  /* also in bsp_driver_sd.h */
#define SD_NOT_PRESENT           ((uint8_t)0x00)  /* also in bsp_driver_sd.h */
#define SD_DETECT_PIN         Detect_SD_Pin
#define SD_DETECT_GPIO_PORT   Detect_SD_GPIO_Port

#define SD_TIMEOUT 300   // how long to wait for a SD operation


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_BOOT_H */
