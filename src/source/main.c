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
#include "ffconf.h"
#include <ctype.h>
#include "gpio.h"

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
void print(const char* str);   // debug
void k_delay(const uint32_t ms);

void SPI_GPIOConfig(void);
void SPIConfig(void);
void SPI_Enable(void);

void SPI_Transmit (BYTE *data, UINT size);

//#include <lpc17xx_clkpwr.h>
//void SystemCoreClockUpdate (void); 
//uint32_t CLKPWR_GetPCLK (uint32_t ClkType);

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
  
  SPI_GPIOConfig();
  SPIConfig();
  SPI_Enable();
  
  SystemCoreClockUpdate();  // get core clock
  sprintf(msg, "\nSystemCoreClock: %08lu\n", SystemCoreClock);
  print(msg);
  sprintf(msg, "\nSPI clock:       %08lu\n", CLKPWR_GetPCLK (CLKPWR_PCLKSEL_SSP1));
  print(msg);
  sprintf(msg, "\nUART0 clock:     %08lu\n", CLKPWR_GetPCLK (CLKPWR_PCLKSEL_UART0));
  print(msg);


  
  LED_G1_OFF();
  LED_G2_OFF();             

  print("debug test\n");  

  LED_G1_ON();
  LED_G2_OFF();  
  
  /* USER CODE BEGIN 2 */
  
  main_boot();             
  /* USER CODE END 2 */
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
 // uint16_t counter = 0;
  while (1)
  {
//    counter++;
//    putchar(counter % 255);
//    if(counter == 1024) {
//      LED_G1_TG();
//      LED_G2_TG();
//      counter = 0;
//      }
    
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
 
  
 
  /*Configure GPIO pins : LED_G1 */
  gpio_wr(  IO(LED_G1_Port, LED_G1_Pin), 0);
  gpio_func(IO(LED_G1_Port, LED_G1_Pin), 0);
  gpio_dir( IO(LED_G1_Port, LED_G1_Pin), GPIO_OUTPUT);
  gpio_mode(IO(LED_G1_Port, LED_G1_Pin), PULL_NO);
  
    /*Configure GPIO pins : LED_G2 */
  gpio_wr(  IO(LED_G2_Port, LED_G2_Pin), 0);
  gpio_func(IO(LED_G2_Port, LED_G2_Pin), 0);
  gpio_dir( IO(LED_G2_Port, LED_G2_Pin), GPIO_OUTPUT);
  gpio_mode(IO(LED_G2_Port, LED_G2_Pin), PULL_NO);

  

  /*Configure GPIO pins : B5 */
  //gpio_wr(  IO(PORTB, 5), 0);
  //gpio_func(IO(PORTB, 5), 0);
  //gpio_dir( IO(PORTB, 5), GPIO_OUTPUT);
  //gpio_mode(IO(PORTB, 5), PULL_NO);
  
  /*Configure GPIO pin : Detect_SDIO_Pin */
  	/* SDIO_CD: input gpio, card detect */
  #ifdef HAS_SD_DETECT
    gpio_func(IO(PORTB, 11), 0);
    gpio_dir( IO(PORTB, 11), GPIO_INPUT);
    gpio_mode(IO(PORTB, 11), PULL_NO);
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
  if(*LPC_SC_RSID & _BV(0))
  {
    //print("POR/PDR reset flag is active.\n");
    #if(CLEAR_RESET_FLAGS)
      /* Clear system reset flags */
      *LPC_SC_RSID = 0;
      //print("Reset flags cleared.\n");
    #endif
  }
  
  print("Entering Bootloader...\n");
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
        print("Checksum Error.\n");
        Error_Handler();
      }
      else
      {
        print("Checksum OK.\n");
      }
    #endif
    
    print("Launching Application.\n");
    LED_G1_ON();
    LED_G2_ON();
    
   
    
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
  uint8_t data[data_length];
  uint32_t cntr;
  //uint32_t addr;
  char SDPath[4] = {0x00};   /* SD logical drive path */
  
  /* Mount SD card */
  fr = f_mount(&SDFatFS, (TCHAR const*)SDPath, 1);
  if(fr != FR_OK)
  {
    /* f_mount failed */
    print("SD card cannot be mounted.\n");
    //        kprint("FatFs error code: %u\n", fr);
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
  do {
    fr   = f_read(&SDFile, &data, data_length, &num);
    if(num)
    { 
      status = Bootloader_FlashNext(data, num);
      if(status == BL_OK)
      {
        cntr += data_length;
      }
      else
      {
        //                kprint("Programming error at: %lu byte\n", (cntr * 8));
        char cmd[64];
        sprintf(cmd, "  offset in file (byte):   %08lX\n", (cntr * data_length));
        kprint(cmd);
        
        f_close(&SDFile);
        SD_Eject();
        print("SD ejected.\n");
        
        LED_ALL_OFF();
        return ERR_FLASH;
      }
    }
    if(cntr % (data_length * 4) == 0)
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
  sprintf(msg, "Flashed: %ld bytes.\n", (cntr));
  print(msg);
  
    
                 
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
        kprint("FatFs error code: %u\n", fr);
        //print(msg);
        
        //SD_Eject();               // allow loading application even if can't rename
        Magic_Location = Magic_Application;  // flag that we should load application
                                             // after the next reset
      }
    }
    else {
    /* f_open failed */
      print("removing .CUR failed.\n");
      kprint("FatFs error code: %u\n", fr);

      // allow loading application even if can't rename
      Magic_Location = Magic_Application;  // flag that we should load application
                                           // after the next reset
    }
  #endif
 
  /* Eject SD card */
  SD_Eject();
  print("SD ejected.\n");
  

  
  return ERR_OK;
}

/**
  * @brief  Debug over UART1 -> ST-LINK -> USB Virtual Com Port
  * @param  str: string to be written to UART2
  * @retval None
*/
void print(const char* str)
{
  kprint(str);
}


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
