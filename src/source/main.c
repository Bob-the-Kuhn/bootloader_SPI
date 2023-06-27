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
#include "bootloader.h"                       
#include "ff.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "bootloader.h"
#include "ffconf.h"
#include <ctype.h>
#include "gpio.h"
uint32_t __attribute__((section("no_init"))) init_1;  // debug
uint32_t __attribute__((section("no_init"))) init_2;


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
static void main_boot(void);
uint8_t Enter_Bootloader(void);
void SD_Eject(void) {};
void Error_Handler(void);
void GPIO_Init(void);
//void kprint(const char* str);   // debug
void k_delay(const uint32_t ms);

void SPI_GPIOConfig(void);
//void SPIConfig(void);
//void SPI_Enable(void);
void report_WP_ConfigProtection(void);

void SPI_Transmit (BYTE *data, UINT size);

void gpio_mode( uint16_t port_pin, uint8_t mode, uint8_t speed);

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
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* MCU Configuration--------------------------------------------------------*/
  
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  
  /* USER CODE BEGIN Init */
  
  HAL_NVIC_EnableIRQ(SysTick_IRQn);  // enable Systick irq
  
  /* USER CODE END Init */
  
  /* Configure the system clock */
  
  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */
  
  /* Initialize all configured peripherals */
  GPIO_Init();
  
  LED_G1_ON();
  LED_G2_OFF(); 
  
  SPI_GPIOConfig();
  //SPIConfig();
  //SPI_Enable();
  
  //sprintf(msg, "\nSYSCLK_Frequency %08lu\n", HAL_RCC_GetSysClockFreq());
  //kprint(msg);
  //sprintf(msg, "HCLK_Frequency   %08lu\n", HAL_RCC_GetHCLKFreq());
  //kprint(msg);
  
  LED_G1_OFF();
  LED_G2_OFF();             

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
  
// D2_LED_G1_Pin PE5
// Detect_SDIO_Pin PC1
// SD_MISO_Pin PC8
// SDSS_Pin PC11
// SD_SCK_Pin PC12
// SD_MOSI_Pin PD2
// D4_LED_G2_Pin PB5 
  
  
  /*Configure GPIO pin Output Level */
 
   /*Configure GPIO pins : LED_D2_Pin */
  gpio_wr(  IO(D2_LED_G1_GPIO_Port, D2_LED_G1_Pin), 0);
  gpio_mode(IO(D2_LED_G1_GPIO_Port, D2_LED_G1_Pin), Output_Push_Pull, Speed_Low);
  
  /*Configure GPIO pins : LED_D4_Pin */
//  gpio_wr(  IO(D4_LED_G2_GPIO_Port, D4_LED_G2_Pin), 0);
//  gpio_mode(IO(D4_LED_G2_GPIO_Port, D4_LED_G2_Pin), Output_Push_Pull, Speed_Low);
  
  
  /*Configure GPIO pin : Detect_SDIO_Pin */
  	/* SDIO_CD: input gpio, card detect */
  #ifdef HAS_SD_DETECT
 //   gpio_mode(IO(Detect_SDIO_GPIO_Port, Detect_SDIO_Pin), Input_Floating);
  #endif  
  
  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/**
  * @brief SPI GPIO Initialization Function
  * @param None
  * @retval None
*/
void SPI_GPIOConfig(void){
  
   /*Configure GPIO pins : SCK */
  gpio_wr(  IO(SD_SCK_GPIO_Port, SD_SCK_Pin), 0);
  gpio_mode(IO(SD_SCK_GPIO_Port, SD_SCK_Pin), Output_Push_Pull, Speed_High);
  
  /*Configure GPIO pins : MOSI */
  gpio_wr(  IO(SD_MOSI_GPIO_Port, SD_MOSI_Pin), 0);
  gpio_mode(IO(SD_MOSI_GPIO_Port, SD_MOSI_Pin), Output_Push_Pull, Speed_High); 
  
  /*Configure GPIO pins : SDSS */
  gpio_wr(  IO(SDSS_GPIO_Port, SDSS_Pin), 0);
  gpio_mode(IO(SDSS_GPIO_Port, SDSS_Pin), Output_Push_Pull, Speed_Low); 
  
  /*Configure GPIO pins : MISO  */
  gpio_mode(IO(SD_MISO_GPIO_Port, SD_MISO_Pin), Input_Floating, 0);
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
  
  //report_WP_ConfigProtection();  
  
  kprint("\nPower up, Boot started.\n");
  
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
  
  kprint("Entering Bootloader...\n");
  Bootloader_Init();
  uint8_t temp_stat = Enter_Bootloader();
  if((temp_stat == ERR_FLASH) || (temp_stat == ERR_VERIFY)) Error_Handler();
  
  /* Check if there is application in user flash area */
  if(Bootloader_CheckForApplication() == BL_OK)
  {
    #if(USE_CHECKSUM)
      /* Verify application checksum */
      if(Bootloader_VerifyChecksum() != BL_OK)
      {
        kprint("Checksum Error.\n");
        Error_Handler();
      }
      else
      {
        kprint("Checksum OK.\n");
      }
    #endif
    
    kprint("Launching Application.\n");
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
  kprint("No application in flash.\n");
  while(1)
  {
    Error_Handler();
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

  
  int fr;
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
    kprint("SD card cannot be mounted.\n");
    //        kprint("FatFs error code: %u\n", fr);
    //      //print(msg);
    return ERR_SD_MOUNT;
  }
  kprint("SD mounted.\n");
  
  /* Open file for programming */
  fr = f_open(&SDFile, CONF_FILENAME, FA_READ);
  if(fr != FR_OK)
  {
    /* f_open failed */
    kprint("File cannot be opened.\n");
    //        kprint("FatFs error code: %u\n", fr);
    //      //print(msg);
    
    SD_Eject();
    kprint("SD ejected.\n");
    return ERR_SD_FILE;
  }
  kprint("Software found on SD.\n");
  
  /* Check size of application found on SD card */
  if(Bootloader_CheckSize(f_size(&SDFile)) != BL_OK)
  {
    kprint("Error: app on SD card is too large.\n");
    
    f_close(&SDFile);
    SD_Eject();
    kprint("SD ejected.\n");
    return ERR_APP_LARGE;
  }
  kprint("App size OK.\n");
  
  /* Step 1: Init Bootloader and Flash */
  
      // debug/test routines used to test sector write protect logic
      //
      // using debugger, set init_1 to a non-zero number to protect a sector
      // in the app and in the bootloader.
      //
      // Ozone debugger removed all sector write protection when downloading
      // a new image or when starting a debug session. 
      
      //if(init_1 ) {
      //  Write_Prot_Old = WRITE_Prot_Old_Flag = Magic_Location = 0;
      //  init_2 = 1;
      //  init_1 = 0;
      //  Bootloader_ConfigProtection(0xFFFFFFFFUL, 0xFFFFFFFFUL, WP_SAVE);
      //}
      //
      //if(init_2 ) {
      //  init_2 = 0;
      //  Write_Prot_Old = WRITE_Prot_Old_Flag = Magic_Location = 0;
      //  Bootloader_ConfigProtection(0x7FFFFFF7UL, 0xFFFFFFFFUL, WP_DONT_SAVE);
      //}  
  
  /* Check for flash write protection of application area*/
  if(~Bootloader_GetProtectionStatus() & WRITE_protection & APP_sector_mask) {  // F407 high bit says sector is protected
    kprint("Application space in flash is write protected.\n");
    if (IGNORE_WRITE_PROTECTION) {                              
      //        kprint("Press button to disable flash write protection...\n");
      //        LED_ALL_ON();
      //        for(i = 0; i < 100; ++i)
      //        {
      //            LED_ALL_TG();
      //            k_delay(50);
      //            if(IS_BTN_PRESSED())
      //            {
      //                kprint("Disabling write protection and generating system "
      //                      "reset...\n");
      //                Bootloader_ConfigProtection(BL_PROTECTION_NONE);
      //            }
      //        }
      //        LED_ALL_OFF();
      //        kprint("Button was not pressed, write protection is still active.\n");
      if (!(WRITE_Prot_Old_Flag == WRITE_Prot_Old_Flag_Restored_flag)) {   // already restored original protection so don't initiate the process again                             
      kprint("Disabling write protection and generating system reset...\n"); 
      /* Eject SD card */
      SD_Eject();
      Magic_Location = Magic_BootLoader;  // flag that we should load the bootloader
                                          // after the next reset
      save_WRP_state();  // save WRP state and set flag so can be restored later                                                                                  
      if (Bootloader_ConfigProtection(WRITE_protection, APP_sector_mask, WP_SAVE) != BL_OK)   // sends system though reset - no more code executed unless there's an error 
        {
          kprint("Failed to set write protection.\n");
          kprint("Exiting Bootloader.\n");
          return ERR_OK;
        }

        kprint("write protection removed\n");

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
  kprint("Erasing flash...\n");
  LED_G2_ON();
  Bootloader_Erase();
  LED_G2_OFF();
  kprint("Flash erase finished.\n");
  
  /* If BTN is pressed, then skip programming */
  //   if(IS_BTN_PRESSED())
  //   {
  //       kprint("Programming skipped.\n");
  //
  //       f_close(&SDFile);
  //       SD_Eject();
  //       kprint("SD ejected.");
  //       return ERR_OK;
  //   }
  
  /* Step 3: Programming */
  kprint("Starting programming...\n");
  LED_G2_ON();
  cntr = 0;
  Bootloader_FlashBegin();
  do
  {
    data = 0xFFFFFFFFFFFFFFFF;
    fr   = f_read(&SDFile, &data, 8, &num);
    //fr   = f_read(&SDFile, &data, 4, &num);
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
        char cmd[64];
        sprintf(cmd, "  offset in file (byte):   %08lX\n", (cntr * 4));
        kprint(cmd);
        
        f_close(&SDFile);
        SD_Eject();
        kprint("SD ejected.\n");
        
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
  kprint("Programming finished.\n");
  sprintf(msg, "Flashed: %ld bytes.\n", (cntr * 4));
  kprint(msg);
  
  
#if 0  // adds 25-26 seconds but doesn't add any value (verify during programming only costs 60mS)  
  /* Step 5: Verify Flash Content */  
  /* Open file for verification */
  fr = f_open(&SDFile, CONF_FILENAME, FA_READ);
  if(fr != FR_OK)
  {
    /* f_open failed */
    kprint("File cannot be opened.\n");
    kprint("FatFs error code: %u\n", fr);
  //print(msg);
    
    SD_Eject();
    kprint("SD ejected.");
    return ERR_SD_FILE;
  }
  
   /* Step 5: Verify Flash Content */
#if 0  // adds 25-26 seconds but doesn't add any value (verify during programming only costs 60mS)
  kprint("Verifying ...\n");
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
        kprint("Verification error at: %lu byte.\n", (cntr * 4));
      //print(msg);
        
        f_close(&SDFile);
        SD_Eject();
        kprint("SD ejected.\n");
        
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

  kprint("Verification passed.\n");
  f_close(&SDFile);
#endif  
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
      if (fr != FR_OK)                                                                 if (fr != FR_OK)  
      {
        /* f_open failed */
        kprint("File cannot be renamed.\n");
        kprint("FatFs error code: %u\n", fr);
        //print(msg);
        
        //SD_Eject();               // allow loading application even if can't rename
        Magic_Location = Magic_Application;  // flag that we should load application
                                             // after the next reset
      }
    }
    else {
    /* f_open failed */
      kprint("removing .CUR failed.\n");
      kprint("FatFs error code: %u\n", fr);

      // allow loading application even if can't rename
      Magic_Location = Magic_Application;  // flag that we should load application
                                           // after the next reset
    }
  #endif

 
  /* Eject SD card */
  SD_Eject();
  kprint("SD ejected.\n");
  
  /* Enable flash write protection on application area */
#if(USE_WRITE_PROTECTION && !RESTORE_WRITE_PROTECTION)
    kprint("Enabling flash write protection and generating system reset...\n");
    if(Bootloader_ConfigProtection(BL_PROTECTION_WRP, APP_sector_mask, WP_DONT_SAVE) != BL_OK)   // sends system though reset - no more code executed unless there's an error
    {
      kprint("Failed to enable write protection.\n");

    }
  #endif

  /* Restore flash write protection */
  #if(!USE_WRITE_PROTECTION && RESTORE_WRITE_PROTECTION && IGNORE_WRITE_PROTECTION)
    if (WRITE_Prot_Old_Flag == WRITE_Prot_Original_flag) {
      WRITE_Prot_Old_Flag = WRITE_Prot_Old_Flag_Restored_flag;  // indicate we've restored the protection
      kprint("Restoring flash write protection and generating system reset...\n");
      //print("  May require power cycle to recover.\n");

      if (Bootloader_ConfigProtection(Write_Prot_Old, APP_sector_mask, WP_DONT_SAVE) != BL_OK)  // sends system though reset - no more code executed unless there's an error
      {
        kprint("Failed to restore write protection.\n");
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
//void kprint(const char* str)
//{
//  kprint(str);
//}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
*/
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  //__disable_irq();   //  k_delay doesn't work if IRQs are disabled            
  while (1)
  {
    LED_G1_ON();
    k_delay(250);
    LED_G1_OFF();
    LED_G2_ON();
    k_delay(250);
    LED_G2_OFF();
  }           
  /* USER CODE END Error_Handler_Debug */
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
