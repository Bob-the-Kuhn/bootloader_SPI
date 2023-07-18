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
void SPIConfig(void);
void SPI_Enable(void);
void report_WP_ConfigProtection(void);

void SPI_Transmit (BYTE *data, UINT size);

void spiBegin(void);

extern uint32_t app_size;

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
  //__disable_irq();
  NVIC_EnableIRQ(SysTick_IRQn);  // enable Systick irq
  
  /* USER CODE END Init */
  
  /* Configure the system clock */
  
  /* USER CODE BEGIN SysInit */
  
  /* USER CODE END SysInit */
  
  /* Initialize all configured peripherals */
  
  
  GPIO_Init();
  
  LED_G1_ON();
  LED_G2_OFF(); 
  
  #ifdef SOFT_SPI
    spiBegin();  //softSPI
    //kprint("SOFT SPI\n");
  #else
    SPI_GPIOConfig();   // hard spi
    //kprint("HARD SPI\n");
    SPIConfig();        // hard spi
    SPI_Enable();       // hard spi
  #endif
  
  
  uint32_t          HAL_RCC_GetSysClockFreq(void);
  uint32_t          HAL_RCC_GetHCLKFreq(void);
  sprintf(msg, "\nSYSCLK_Frequency %08lu\n", HAL_RCC_GetSysClockFreq());
  kprint(msg);
  sprintf(msg, "HCLK_Frequency   %08lu\n", HAL_RCC_GetHCLKFreq());
  kprint(msg);
  
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
   
   
  /*Configure GPIO pin Output Level */
  
//PA13 SWDIO
//PA14 SWCLK 
//  GPIO_CONFIG_ALT(PORTA, 13, 0, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);
//  GPIO_CONFIG_ALT(PORTA, 14, 0, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_HIGH_SPEED);
 
 
   /*Configure GPIO pins : LED_D2_Pin */
  // D2_LED_G1_Pin PA5
  gpio_wr(D2_LED_G1_GPIO_Port, D2_LED_G1_Pin, 0);
  GPIO_CONFIG_OUTPUT(D2_LED_G1_GPIO_Port, D2_LED_G1_Pin, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_LOW_SPEED)
  
  /*Configure GPIO pins : LED_D4_Pin */
  // D4_LED_G2_Pin PA7 
  gpio_wr(D4_LED_G2_GPIO_Port, D4_LED_G2_Pin, 0);
  GPIO_CONFIG_OUTPUT(D4_LED_G2_GPIO_Port, D4_LED_G2_Pin, GPIO_NO_PULL_UP_DOWN, GPIO_OUTPUT_PUSH_PULL, GPIO_OUTPUT_VERY_LOW_SPEED)
  
  
  /*Configure GPIO pin : Detect_SDIO_Pin */
  	/* SDIO_CD: input gpio, card detect */
  #ifdef HAS_SD_DETECT
    GPIO_CONFIG_INPUT(Detect_SDIO_GPIO_Port, Detect_SDIO_Pin, GPIO_NO_PULL_UP_DOWN)
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
  
  
  
  kprint("\nPower up, Boot started.\n");
  
  /* Check system reset flags */
  if(RCC->CSR & (RCC_CSR_PINRSTF | RCC_CSR_BORRSTF) )
  {
    //print("POR/PDR reset flag is active.\n");
    #if(CLEAR_RESET_FLAGS)
      /* Clear system reset flags */
    RCC->CSR |= RCC_CSR_RMVF;
      //print("Reset flags cleared.\n");
    #endif
  }
  
  //kprint("Entering Bootloader...\n");
  Bootloader_Init();
  
  //report_WP_ConfigProtection();  
  
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
  app_size = f_size(&SDFile);
  
  sprintf(msg, "Application size: %8lX\nApplication end:  %8lX\n", app_size, app_size + APP_ADDRESS);
  kprint(msg);
  
  if(Bootloader_CheckSize(app_size) != BL_OK)
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
      //  uint32_t temp_wrp[4] = { (1 | (2 << FLASH_WRP1AR_WRP1A_END_Pos)),  (3 | (4 << FLASH_WRP1BR_WRP1B_END_Pos)), 
      //                           (5 | (6 << FLASH_WRP2AR_WRP2A_END_Pos)),  (7 | (8 << FLASH_WRP2BR_WRP2B_END_Pos))};
      //  Bootloader_ConfigProtection_Set(temp_wrp);
      //  kprint("init_1: shouldn't see this\n");
      //}
      //
      //if(init_2 ) {
      //  init_2 = 0;
      //  Write_Prot_Old = WRITE_Prot_Old_Flag = Magic_Location = 0;
      //  uint32_t temp_wrp[4] = { (1 | (0x7F << FLASH_WRP1AR_WRP1A_END_Pos)),  (0x20 | (0x40 << FLASH_WRP1BR_WRP1B_END_Pos)), 
      //                           (5 | (6 << FLASH_WRP2AR_WRP2A_END_Pos)),  (7 | (8 << FLASH_WRP2BR_WRP2B_END_Pos))};
      //  Bootloader_ConfigProtection_Set(temp_wrp);
      //  kprint("init_2: shouldn't see this\n");
      //}  
  
  /* Check for flash write protection of application area*/
  if(Bootloader_GetProtectionStatus()) {  
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
      WRITE_Prot_Old_Flag = WRITE_Prot_Original_flag;  // flag that protection was saved                                    
      save_WRP_state();  // save WRP state and set flag so can be restored later                                                                                  
      if (Bootloader_ConfigProtection_Keep_Boot() != HAL_OK)   // sends system though reset - no more code executed unless there's an error 
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
        sprintf(cmd, "  offset in file (byte):   %08lX\n", (cntr * 8));
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
  sprintf(msg, "Flashed: %ld bytes.\n", (cntr * 8));
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
#if 0
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
#endif

  /* Eject SD card */
  SD_Eject();
  kprint("SD ejected.\n");
  
  /* Enable flash write protection on application area */
  #if(USE_WRITE_PROTECTION && !RESTORE_WRITE_PROTECTION)
    //kprint("Enabling flash write protection and generating system reset...\n");
    //if(Bootloader_ConfigProtection(temp_wrp) != BL_OK)   // sends system though reset - no more code executed unless there's an error
    //{
    //  kprint("Failed to enable write protection.\n");
    //
    //}
  #endif

  /* Restore flash write protection */
  #if(!USE_WRITE_PROTECTION && RESTORE_WRITE_PROTECTION && IGNORE_WRITE_PROTECTION)
    if (WRITE_Prot_Old_Flag == WRITE_Prot_Original_flag) {
      WRITE_Prot_Old_Flag = WRITE_Prot_Old_Flag_Restored_flag;  // indicate we've restored the protection
      kprint("Restoring flash write protection and generating system reset...\n");
      //print("  May require power cycle to recover.\n");

      if (Bootloader_ConfigProtection_Set(wrp_old) != BL_OK)  // sends system though reset - no more code executed unless there's an error
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
