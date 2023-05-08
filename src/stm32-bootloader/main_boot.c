/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main_boot.h"
#include "bootloader.h"
#include "fatfs.h"
//#include "ff.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "ffconf.h"
#include <ctype.h>
#include <stdint.h>
//#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define HAS_SD_DETECT  // enable if has SD detect capability

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

void main_boot(void);
uint8_t Enter_Bootloader(void);
void SD_Eject(void) {};
void UART1_Init(void);
void UART1_DeInit(void);
void Error_Handler_Boot(void);
void GPIO_Init(void);
void print(const char* str);   // debug
//void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi);  // GPIO init of SPI pins
extern SPI_HandleTypeDef hspi1;

void NVIC_System_Reset(void);

#define PGM_READ_WORD(x) *(x)

char msg[64];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main_boot_init(void)
{
  /* USER CODE BEGIN 1 */

    sprintf(msg, "SYSCLK_Frequency %08lu\n", HAL_RCC_GetSysClockFreq());
    print(msg);
    sprintf(msg, "HCLK_Frequency   %08lu\n", HAL_RCC_GetHCLKFreq());
    print(msg);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */



  /* USER CODE BEGIN Init */

  HAL_NVIC_EnableIRQ(SysTick_IRQn);  // enable Systick irq
  /* USER CODE END Init */

  GPIO_Init();
  HAL_SPI_MspInit(&hspi1);               // setup the SPI pins
  LED_G1_OFF();
  LED_G2_OFF();             

  //print("debug test\n");  

  LED_G1_ON();
  LED_G2_OFF();    /* USER CODE BEGIN 2 */

  main_boot();

/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
*/
void GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */
  

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D2_LED_G2_GPIO_Port, D2_LED_G2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(D4_LED_G2_GPIO_Port, D4_LED_G2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);  // default to SD card not selected

  /*Configure GPIO pin : D2_LED_G2_Pin */
  GPIO_InitStruct.Pin = D2_LED_G2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D2_LED_G2_GPIO_Port, &GPIO_InitStruct);


  /*Configure GPIO pin : D4_LED_G2_Pin */
  GPIO_InitStruct.Pin = D4_LED_G2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D4_LED_G2_GPIO_Port, &GPIO_InitStruct);
  
  /*Configure GPIO pin : SD_CS (also called SDSS and SPI1_NSS) */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);
 
  
  /*Configure GPIO pin : Detect_SDIO_Pin */
  #ifdef HAS_SD_DETECT
  /*Configure GPIO pin : Detect_SDIO_Pin */
  GPIO_InitStruct.Pin = Detect_SDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Detect_SDIO_GPIO_Port, &GPIO_InitStruct);
  #endif  
  
}

/* USER CODE BEGIN 4 */

/*
  * @brief  This is the main program. Does some setup, calls the
  *         bootloader and then jumps to the application.
  * @param  None
  * @retval None
  *
*/
void main_boot(void)
{

  print("\nPower up, Boot started.\n");

  /* Check system reset flags */
  if(__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
  {
    print("POR/PDR reset flag is active.\n");
    #if(CLEAR_RESET_FLAGS)
      /* Clear system reset flags */
      __HAL_RCC_CLEAR_RESET_FLAGS();
      print("Reset flags cleared.\n");
    #endif
  }

  print("Entering Bootloader...\n");
  Bootloader_Init();
  uint8_t temp_stat = Enter_Bootloader();

  if((temp_stat == ERR_FLASH) || (temp_stat == ERR_VERIFY)) Error_Handler_Boot();

  /* Check if there is application in user flash area */
  if(Bootloader_CheckForApplication() == BL_OK)
  {
    #if(USE_CHECKSUM)
      /* Verify application checksum */
      if(Bootloader_VerifyChecksum() != BL_OK)
      {
        print("Checksum Error.\n");
        Error_Handler_Boot();
      }
      else
      {
        print("Checksum OK.\n");
      }
    #endif

    print("Launching Application.\n");

    LED_G1_ON();
    LED_G2_ON();

    WRITE_Prot_Old_Flag = 0;  //  set "restore write protect" state machine back to unitialized

    /* Launch application */
    Bootloader_JumpToApplication();
  }

  /* No application found */
  print("No application in flash.\n");
  while(1)
  {
    Error_Handler_Boot();
  }
}

/**
  * @brief  This function executes the bootloader sequence.
  * @param  None
  * @retval Application error code ::eApplicationErrorCodes
  *
*/
uint8_t Enter_Bootloader(void)
{
  FRESULT fr;
  
  FATFS SDFatFS;  // create file system object
  
  FIL SDFile;     //  file name handle
  
  UINT num;

  //    uint8_t i;
  uint8_t status;
  uint64_t data;
  uint32_t cntr;
//  uint32_t addr;
  char SDPath[4] = {0x00};   /* SD logical drive path */

  /* Mount SD card */
  fr = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
  if(fr != FR_OK)
  {
    /* f_mount failed */
    print("SD card cannot be mounted.\n");
    //        sprintf(msg, "FatFs error code: %u\n", fr);
    //        print(msg);
    return ERR_SD_MOUNT;
  }
  print("SD mounted.\n");

  /* Open file for programming */
  fr = f_open(&SDFile, CONF_FILENAME, FA_READ);
  if(fr != FR_OK)
  {
    /* f_open failed */
    print("File cannot be opened.\n");
    //        sprintf(msg, "FatFs error code: %u\n", fr);
    //        print(msg);

    SD_Eject();
    print("SD ejected.\n");
    return ERR_SD_FILE;
  }
  print("Software found on SD.\n");

  /* Check size of application found on SD card */
  if(Bootloader_CheckSize(f_size(&SDFile)) != BL_OK)
  {
    print("Error: app on SD card is too large.\n");

    f_close(&SDFile);
    SD_Eject();
    print("SD ejected.\n");
    return ERR_APP_LARGE;
  }
  print("App size OK.\n");

  /* Step 1: Init Bootloader and Flash */

  /* Check for flash write protection of application area*/
  if(~Bootloader_GetProtectionStatus() & WRITE_protection) {
    print("Application space in flash is write protected.\n");
    if (IGNORE_WRITE_PROTECTION) {
      //        print("Press button to disable flash write protection...\n");
      //        LED_ALL_ON();
      //        for(i = 0; i < 100; ++i)
      //        {
      //            LED_ALL_TG();
      //            HAL_Delay(50);
      //            if(IS_BTN_PRESSED())
      //            {
      //                print("Disabling write protection and generating system "
      //                      "reset...\n");
      //                Bootloader_ConfigProtection(BL_PROTECTION_NONE);
      //            }
      //        }
      //        LED_ALL_OFF();
      //        print("Button was not pressed, write protection is still active.\n");
      if (!(WRITE_Prot_Old_Flag == WRITE_Prot_Old_Flag_Restored_flag)) {   // already restored original protection so don't initiate the process again
        print("Disabling write protection and generating system reset...\n");
        print("  May require power cycle to recover.\n");
        
        /* Eject SD card */
        SD_Eject();
        Magic_Location = Magic_BootLoader;  // flag that we should load the bootloader
                                            // after the next reset
        if (Bootloader_ConfigProtection(WRITE_protection, WP_CLEAR) != BL_OK)   // sends system though reset - no more code executed unless there's an error  // apparently not on a STM32F107
          {
            print("Failed to clear write protection.\n");
            print("Exiting Bootloader.\n");
            return ERR_OK;
          }

        print("write protection removed\n");

                WRITE_Prot_Old_Flag = WRITE_Prot_Original_flag;  // flag that protection was removed so can
                                                       // restore write protection after next reset)

        Magic_Location = Magic_BootLoader;  // flag that we should load the bootloader
                                            // after the next reset
        // NVIC_System_Reset();  // send system through reset
      }
    }
  }

  /* Step 2: Erase Flash */
  print("Erasing flash...\n");
  LED_G2_ON();
  Bootloader_Erase();
  LED_G2_OFF();
  print("Flash erase finished.\n");

  /* If BTN is pressed, then skip programming */
  //   if(IS_BTN_PRESSED())
  //   {
  //       print("Programming skipped.\n");
  //
  //       f_close(&SDFile);
  //       SD_Eject();
  //       print("SD ejected.");
  //       return ERR_OK;
  //   }

  /* Step 3: Programming */
  print("Starting programming...\n");
  LED_G2_ON();
  cntr = 0;
  
#if 1  // F407 code - may be slow  
    Bootloader_FlashBegin();
  do
  {
    data = 0xFFFFFFFFFFFFFFFF;
    //      fr   = f_read(&SDFile, &data, 8, &num);
    fr   = f_read(&SDFile, &data, 4, &num);
    if(num)
    {
      status = Bootloader_FlashNext(data);
      if(status == BL_OK)
      {
        cntr++;
      }
      else
      {
        //                sprintf(msg, "Programming error at: %lu byte\n", (cntr * 8));
        sprintf(msg, "  offset in file (byte):   %08lX\n", (cntr * 4));
        print(msg);
        
        f_close(&SDFile);
        SD_Eject();
        print("SD ejected.\n");
        
        LED_ALL_OFF();
        return ERR_FLASH;
      }
    }
    if(cntr % 256 == 0)
    {
      /* Toggle green LED during programming */
      LED_G1_TG();
    }
  } while((fr == FR_OK) && (num > 0));
#endif  // F407 code  
 
  
#if 0  // F103 code so needs to be tested  
  UINT num;
  
  Bootloader_FlashBegin();
  do
  {
    uint8_t buffer[512];
        
    fr   = f_read(&SDFile, buffer, 512, &num);  
    if(num)
    {
      cntr += num;
      status = Bootloader_FlashNext_Buf(buffer, num);
      if(status != BL_OK)

      {
        f_close(&SDFile);
        SD_Eject();
        print("SD ejected.\n");

        LED_ALL_OFF();
        return ERR_FLASH;
      }
    }
    /* Toggle green LED during programming */
    LED_G1_TG();
  } while((fr == FR_OK) && (num > 0));
#endif // F103 code    
  

  /* Step 4: Finalize Programming */
  Bootloader_FlashEnd();
  f_close(&SDFile);
  LED_ALL_OFF();
  print("Programming finished.\n");
  sprintf(msg, "Flashed: %lu bytes.\n", cntr);
  print(msg);

  /* Open file for verification */
  fr = f_open(&SDFile, CONF_FILENAME, FA_READ);
  if(fr != FR_OK)
  {
    /* f_open failed */
    print("File cannot be opened.\n");
    sprintf(msg, "FatFs error code: %u\n", fr);
    print(msg);

    SD_Eject();
    print("SD ejected.");
    return ERR_SD_FILE;
  }

  /* Step 5: Verify Flash Content */
#if 0  // adds 25-26 seconds but doesn't add any value (verify during programming only costs 60mS)
  print("Verifying ...\n");
  addr = APP_ADDRESS;
  cntr = 0;
  do
  {
    data = 0xFFFFFFFFFFFFFFFF;
    fr   = f_read(&SDFile, &data, 8, &num);
    if(num)
    {
      if(*(uint32_t*)addr == (uint32_t)data)
      {
        addr += 8;
        cntr++;
      }
      else
      {
        sprintf(msg, "Verification error at: %lu byte (%08lX)\n", (cntr * 8), (cntr * 8));
        print(msg);

        f_close(&SDFile);
        SD_Eject();
        print("SD ejected.\n");

        LED_ALL_OFF();
        return ERR_VERIFY;
      }
    }
    if(cntr % 256 == 0)
    {
      /* Toggle green LED during verification */
      LED_G2_TG();
    }
  } while((fr == FR_OK) && (num > 0));
  print("Verification passed.\n");
#endif  

  f_close(&SDFile);

  LED_G1_OFF();

  #if defined(FILE_EXT_CHANGE) && (_LFN_UNICODE == 0)   // rename file if using ANSI/OEM strings
    TCHAR new_filename[strlen(CONF_FILENAME) + 1];
    new_filename[strlen(CONF_FILENAME)] = '\0';  // terminate the string
    strncpy(new_filename, PGM_READ_WORD(&(CONF_FILENAME)), strlen(CONF_FILENAME) );  // copy FLASH into ram
    for (int x = 0; x < strlen(CONF_FILENAME); x++)  // convert to upper case
      new_filename[x] = toupper(new_filename[x]);
    char * pos = strrchr(new_filename, '.') + 1;  // find start of extension
    strncpy(pos, PGM_READ_WORD(&(FILE_EXT_CHANGE)), strlen(FILE_EXT_CHANGE) );  // copy FLASH into ram
    
    fr = f_unlink (new_filename); // if file already exists - delete it

    if ((fr == FR_OK) || (fr == FR_NO_FILE)) {

      fr = f_rename(CONF_FILENAME, new_filename);  // rename file to .CUR
      if (fr != FR_OK)
      {
        /* f_open failed */
        print("File cannot be renamed.\n");
        sprintf(msg, "FatFs error code: %u\n", fr);
        print(msg);

        // allow loading application even if can't rename
        Magic_Location = Magic_Application;  // flag that we should load application
                                             // after the next reset
      }
    }
    else {
          /* f_open failed */
            print("removing .CUR failed.\n");
            sprintf(msg, "FatFs error code: %u\n", fr);
            print(msg);

            // allow loading application even if can't rename
            Magic_Location = Magic_Application;  // flag that we should load application
                                                 // after the next reset
    }
        
  #endif

  /* Eject SD card */
  SD_Eject();
  print("SD ejected.\n");

  /* Enable flash write protection on application area */
  #if(USE_WRITE_PROTECTION && !RESTORE_WRITE_PROTECTION)
    print("Enabling flash write protection and generating system reset...\n");

    if (Bootloader_ConfigProtection(WRITE_protection, WP_SET) != BL_OK)  // sends system though reset - no more code executed unless there's an error
    {
      print("Failed to enable write protection.\n");
    }
  #endif

  /* Restore flash write protection */
  #if(!USE_WRITE_PROTECTION && RESTORE_WRITE_PROTECTION && IGNORE_WRITE_PROTECTION)
    if (WRITE_Prot_Old_Flag == WRITE_Prot_Original_flag) {
      WRITE_Prot_Old_Flag = WRITE_Prot_Old_Flag_Restored_flag;  // indicate we've restored the protection
      print("Restoring flash write protection and generating system reset...\n");
      print("  May require power cycle to recover.\n");

      if (Bootloader_ConfigProtection(Write_Prot_Old, WP_SET) != BL_OK)  // sends system though reset - no more code executed unless there's an error
      {
        print("Failed to restore write protection.\n");
      }
    }
  #endif

  return ERR_OK;
}


/**
  * @brief  Debug over UART3 -> ST-LINK -> USB Virtual Com Port
  * @param  str: string to be written to UART2
  * @retval None
*/
void print(const char* str)
{
  #if(USE_VCP)
    HAL_UART_Transmit(&huart3, (uint8_t*)str, (uint16_t)strlen(str), 100);
  #endif /* USE_VCP */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

void Error_Handler_Boot(void)
{
  /* USER CODE BEGIN Error_Handler_Boot_Debug */
  /* User can add his own implementation to report the HAL error return state */
  //__disable_irq();   //  HAL_Delay doesn't work if IRQs are disabled
  while (1)
  {
    LED_G1_ON();
    HAL_Delay(250);
    LED_G1_OFF();
    LED_G2_ON();
    HAL_Delay(250);
    LED_G2_OFF();
  }
  /* USER CODE END Error_Handler_Boot_Debug */
}
