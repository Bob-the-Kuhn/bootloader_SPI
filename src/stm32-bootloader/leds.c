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

#if 0
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LEDS_C
#define __LEDS_C

#ifdef __cplusplus
extern "C" {
#endif

// couldn't get past "undefined reference to `HAL_GPIO_WritePin'" compile
// errors so went with hard coded defines


/* LD2 */
//#define LED_G1_Port GPIOA
//#define LED_G1_Pin  GPIO_PIN_6

/* LD3 */
//#define LED_G2_Port GPIOA
//#define LED_G2_Pin  GPIO_PIN_7



void LED_G1_ON(void) {  // set bit low
  volatile uint32_t* GPIOA_BSSR = (uint32_t*)0x40020018CUL;
  *GPIOA_BSSR = (1 << (16+6));
}

void LED_G1_OFF(void) { // set bit high
  volatile uint32_t* GPIOA_BSSR = (uint32_t*)0x40020018CUL;
  *GPIOA_BSSR = (1 << (   6));
}

void LED_G1_TG(void) {
  volatile uint16_t* GPIOA_ODR = (uint16_t*)0x40020014CUL;
  if (*GPIOA_ODR & (1 << 6))
    LED_G1_ON();
  else
    LED_G1_OFF();
}

void LED_G2_ON(void) { // set bit low
  volatile uint32_t* GPIOA_BSSR = (uint32_t*)0x40020018CUL;
  *GPIOA_BSSR = (1 << (16+7));
}

void LED_G2_OFF(void) { // set bit high
  volatile uint32_t* GPIOA_BSSR = (uint32_t*)0x40020018CUL;
  *GPIOA_BSSR = (1 << (   7));
}

void LED_G2_TG(void) {
  volatile uint16_t* GPIOA_ODR = (uint16_t*)0x40020014CUL;
  if (*GPIOA_ODR & (1 << 7))
    LED_G2_ON();
  else
    LED_G2_OFF();
}


#ifdef __cplusplus
}
#endif

#endif /* __LEDS_C */

#endif