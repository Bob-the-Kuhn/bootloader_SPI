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
#include "main.h"
#include "main_boot.h"
#include "bootloader.h"                       
#include "ff.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "ffconf.h"
#include <ctype.h>
//#include "gpio.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define HAS_SD_DETECT  // enable if has SD detect capability

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
void main_boot_init(void); 
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
static void main_boot(void);
uint8_t Enter_Bootloader(void);
void SD_Eject(void) {};
void Error_Handler_boot(void);
void GPIO_Init(void);
void print(const char* str);   // debug

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
void main_boot_init(void)
{
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */
  
  /* MCU Configuration--------------------------------------------------------*/
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  
  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */
  
  /* Configure the system clock */
  
  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */
  
  /* Initialize all configured peripherals */
  GPIO_Init();
  LED_G1_OFF();
  LED_G2_OFF();             

  sprintf(msg, "\nSYSCLK_Frequency %09lu\n", HAL_RCC_GetSysClockFreq());
  print(msg);
  sprintf(msg, "HCLK_Frequency   %09lu\n", HAL_RCC_GetHCLKFreq());
  print(msg);
  
  sprintf(msg, "\ncurrent protection: %08lX\n", Bootloader_GetProtectionStatus());
  print(msg);

  //print("debug test\n");  

  LED_G1_ON();
  LED_G2_OFF();  

  /* USER CODE BEGIN 2 */
  

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

  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */
  
  /*Configure GPIO pin Output Level */
 
  
   
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();


  /*Configure GPIO pin : LED_G1: PB0, LED_G2: PB7*/
  GPIO_InitStruct.Pin = (LED_G1_Pin | LED_G2_Pin);
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LED_G1_Port, &GPIO_InitStruct);
  

  #ifdef HAS_SD_DETECT
	/*Configure GPIO pin : Detect_SD_Pin PG2*/
	GPIO_InitStruct.Pin = Detect_SD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Detect_SD_GPIO_Port, &GPIO_InitStruct);
  #endif  
  
  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
  * @brief  This is the main program. Does some setup, calls the 
  *         bootloader and then jumps to the application.
  * @param  None
  * @retval None
  *
*/
static void main_boot(void)
{
  
  print("\nPower up, Boot started.\n");
  
  /* Check system reset flags */
  if(__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
  {
    //print("POR/PDR reset flag is active.\n");
    #if(CLEAR_RESET_FLAGS)
      /* Clear system reset flags */
      __HAL_RCC_CLEAR_RESET_FLAGS();
      //print("Reset flags cleared.\n");
    #endif
  }
  
  print("Entering Bootloader...\n");
  Bootloader_Init();
  uint8_t temp_stat = Enter_Bootloader();
  if((temp_stat == ERR_FLASH) || (temp_stat == ERR_VERIFY)) Error_Handler_boot();
  
  /* Check if there is application in user flash area */
  if(Bootloader_CheckForApplication() == BL_OK)
  {
    #if(USE_CHECKSUM)
      /* Verify application checksum */
      if(Bootloader_VerifyChecksum() != BL_OK)
      {
        print("Checksum Error.\n");
        Error_Handler_boot();
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
    
    /* De-initialize bootloader hardware & peripherals */
    //        SD_DeInit();
    //        GPIO_DeInit();
    #if(USE_VCP)
      //        UART1_DeInit();
    #endif /* USE_VCP */
    
    /* Launch application */
    Bootloader_JumpToApplication();
  }
  
  /* No application found */
  print("No application in flash.\n");
  while(1)
  {
    Error_Handler_boot();
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
  FATFS SDFatFS;
  
  FIL SDFile;

  
  FRESULT fr;
  UINT num;
  //    uint8_t i;
  uint8_t status;
  uint64_t data;
  uint32_t cntr;
  //uint32_t addr;
  char SDPath[4] = {0x00};   /* SD logical drive path */
  
  /* Mount SD card */
  fr = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
  if(fr != FR_OK)
  {
    /* f_mount failed */
    print("SD card cannot be mounted.\n");
    //        kprint("FatFs error code: %d\n", (const char *)fr);
    //      //print(msg);
    return ERR_SD_MOUNT;
  }
  print("SD mounted.\n");
  
  /* Open file for programming */
  fr = f_open(&SDFile, CONF_FILENAME, FA_READ);
  if(fr != FR_OK)
  {
    /* f_open failed */
    print("File cannot be opened.\n");
    //        kprint("FatFs error code: %u\n", fr);
    //      //print(msg);
    
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
  if(~Bootloader_GetProtectionStatus() & WRITE_protection & APP_sector_mask) {  // F407 high bit says sector is protected
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
      /* Eject SD card */
      SD_Eject();
      Magic_Location = Magic_BootLoader;  // flag that we should load the bootloader
                                          // after the next reset
      if (Bootloader_ConfigProtection(WRITE_protection, APP_sector_mask, WP_SAVE) != BL_OK)   // sends system though reset - no more code executed unless there's an error 
        {
          print("Failed to set write protection.\n");
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
      else {
        return ERR_OK;  // already programmed FLASH & protection restored so it's time to launch the application
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
        //                kprint("Programming error at: %lu byte\n", (cntr * 8));
        sprintf(msg, "Programming error at offset in file (byte):   %08lX\n", (cntr * 4));
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
  
  /* Step 4: Finalize Programming */
  Bootloader_FlashEnd();
  f_close(&SDFile);
  LED_ALL_OFF();
  print("Programming finished.\n");
  sprintf(msg, "Flashed: %ld bytes.\n", (cntr * 4));
  print(msg);
  
  
#if 0  // adds 25-26 seconds but doesn't add any value (verify during programming only costs 60mS)  
  /* Step 5: Verify Flash Content */  
  /* Open file for verification */
  fr = f_open(&SDFile, CONF_FILENAME, FA_READ);
  if(fr != FR_OK)
  {
    /* f_open failed */
    print("File cannot be opened.\n");
    sprintf(msg, "FatFs error code: %d\n", (const char *)fr);
    print(msg);
    
    SD_Eject();
    print("SD ejected.");
    return ERR_SD_FILE;
  }
  
  print("Verifying ...\n");
  addr = APP_ADDRESS;
  cntr = 0;
  do
  {
    data = 0xFFFFFFFFFFFFFFFF;
    fr   = f_read(&SDFile, &data, 4, &num);
    if(num)
    {
      if(*(uint32_t*)addr == (uint32_t)data)
      {
        addr += 4;
        cntr++;
      }
      else
      {
        sprintf(msg, "Verification error at: %lu byte.\n", (cntr * 4));
        print(msg);
        
        f_close(&SDFile);
        SD_Eject();
        print("SD ejected.\n");
        
        LED_G1_OFF();
        return ERR_VERIFY;
      }
    }
    if(cntr % 256 == 0)
    {
      /* Toggle green LED during verification */
      LED_G1_TG();
    }
  } while((fr == FR_OK) && (num > 0));

  print("Verification passed.\n");
  f_close(&SDFile);
#endif          
                   
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
        sprintf(msg, "FatFs error code: %s\n", (const char *)fr);
        print(msg);
        
        //SD_Eject();               // allow loading application even if can't rename
        Magic_Location = Magic_Application;  // flag that we should load application
                                             // after the next reset
      }
    }
    else {
    /* f_open failed */
      print("removing .CUR failed.\n");
      sprintf(msg, "FatFs error code: %s\n", (const char *)fr);
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
    if(Bootloader_ConfigProtection(BL_PROTECTION_WRP, APP_sector_mask, WP_DONT_SAVE) != BL_OK)   // sends system though reset - no more code executed unless there's an error
    {
      print("Failed to enable write protection.\n");

    }
  #endif

  /* Restore flash write protection */
  #if(!USE_WRITE_PROTECTION && RESTORE_WRITE_PROTECTION && IGNORE_WRITE_PROTECTION)
    if (WRITE_Prot_Old_Flag == WRITE_Prot_Original_flag) {
      WRITE_Prot_Old_Flag = WRITE_Prot_Old_Flag_Restored_flag;  // indicate we've restored the protection
      print("Restoring flash write protection and generating system reset...\n");
      //print("  May require power cycle to recover.\n");

      if (Bootloader_ConfigProtection(Write_Prot_Old, APP_sector_mask, WP_DONT_SAVE) != BL_OK)  // sends system though reset - no more code executed unless there's an error
      {
        print("Failed to restore write protection.\n");
      }
    }
  #endif
  
  return ERR_OK;
}

/**
  * @brief  Debug over UART1 -> ST-LINK -> USB Virtual Com Port
  * @param  str: string to be written to UART2
  * @retval None
*/
//void print(const char* str)
//{
//  print(str);
//}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
*/
void Error_Handler_boot(void)
{
  /* USER CODE BEGIN Error_Handler_boot_Debug */
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
  /* USER CODE END Error_Handler_boot_Debug */
}

#ifdef  USE_FULL_ASSERT
  /**
    * @brief  Reports the name of the source file and the source line number
    *         where the assert_param error has occurred.
    * @param  file: pointer to the source file name
    * @param  line: assert_param error line source number
    * @retval None
  */
  void assert_failed(uint8_t *file, uint32_t line)
  {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
  }
#endif /* USE_FULL_ASSERT */
