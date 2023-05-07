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


/* Define to prevent recursive inclusion -------------------------------------*/
//#ifndef __LEDS_H
//#define __LEDS_H

#ifdef __cplusplus
extern "C" {
#endif

// couldn't get past "undefined reference to `HAL_GPIO_WritePin'" compile
// errors so went with hard coded defines


void LED_G1_ON(void);

void LED_G1_OFF(void);

void LED_G1_TG(void);

void LED_G2_ON(void);

void LED_G2_OFF(void);

void LED_G2_TG(void);


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



#ifdef __cplusplus
}
#endif

//#endif /* __LEDS_H */
