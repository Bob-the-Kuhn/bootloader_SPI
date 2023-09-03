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
//#include "stm32f4xx_hal.h"
#include "kprint.h"

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
                          
//  LED_PIN      P4_28  // Play LED
//  LED1_PIN     P1_18  // green LED
//  LED2_PIN     P1_19  // green LED
//  LED3_PIN     P1_20  // green LED
//  LED4_PIN     P1_21  // green LED

/* LD2 */
#define LED_G1_Port PORT1
#define LED_G1_Pin  19

/* LD3 */
#define LED_G2_Port PORT1
#define LED_G2_Pin  21

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
#define LED_G1_ON()  gpio_wr(IO(LED_G1_Port, LED_G1_Pin), 0)
#define LED_G1_OFF() gpio_wr(IO(LED_G1_Port, LED_G1_Pin), 1)
#define LED_G1_TG()  gpio_wr(IO(LED_G1_Port, LED_G1_Pin), !(gpio_rd(IO(LED_G1_Port, LED_G1_Pin))));
#define LED_G2_ON()  gpio_wr(IO(LED_G2_Port, LED_G2_Pin), 0)
#define LED_G2_OFF() gpio_wr(IO(LED_G2_Port, LED_G2_Pin), 1)
#define LED_G2_TG()  gpio_wr(IO(LED_G2_Port, LED_G2_Pin), !(gpio_rd(IO(LED_G2_Port, LED_G2_Pin))));

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

//#define Detect_SDIO_Pin GPIO_PIN_11  // no SD detect on this board
//#define Detect_SDIO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
