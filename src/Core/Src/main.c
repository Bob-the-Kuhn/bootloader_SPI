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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include "bootloader.h"
#include "main.h"
#include "ffconf.h"
#include <ctype.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

static void main_boot(void);
uint8_t Enter_Bootloader(void);
void SD_Eject(void) {};
void UART3_Init(void);
void UART3_DeInit(void);
void Error_Handler(void);
void print(const char* str);

#define PGM_READ_WORD(x) *(x)

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
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
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    print("HAL_RCC_OscConfig\n");
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    print("HAL_RCC_ClockConfig\n");
    Error_Handler();
  }
  HAL_Delay(1);  // some devices take time to powerup/init after clocks are applied
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    print("HAL_SPI_Init\n");
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    print("huart3.Init\n");
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
        HAL_Delay(200);
        LED_G1_OFF();
        LED_G2_ON();
        HAL_Delay(200);
        LED_G2_OFF();
        HAL_Delay(1000);

        /* De-initialize bootloader hardware & peripherals */
//        SD_DeInit();
//        GPIO_DeInit();
#if(USE_VCP)
//        UART3_DeInit();
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
    FRESULT fr;
    UINT num;
//    uint8_t i;
    uint8_t status;
    uint64_t data;
    uint32_t cntr;
    uint32_t addr;
    char msg[40] = {0x00};

    /* Initialize SD card */
    if(FATFS_Init())
    {
        /* SD init failed */
        print("SD card cannot be initialized.\n");
        return ERR_SD_INIT;
    }

    /* Mount SD card */
    fr = f_mount(&USERFatFS, (TCHAR const*)USERPath, 1);
    if(fr != FR_OK)
    {
        /* f_mount failed */
        print("SD card cannot be mounted.\n");
        sprintf(msg, "FatFs error code: %u\n", fr);
        print(msg);
        return ERR_SD_MOUNT;
    }
    print("SD mounted.\n");

    /* Open file for programming */
    fr = f_open(&USERFile, CONF_FILENAME, FA_READ);
    if(fr != FR_OK)
    {
        /* f_open failed */
        print("File cannot be opened.\n");
        sprintf(msg, "FatFs error code: %u\n", fr);
        print(msg);

        SD_Eject();
        print("SD ejected.\n");
        return ERR_SD_FILE;
    }
    print("Software found on SD.\n");

    /* Check size of application found on SD card */
    if(Bootloader_CheckSize(f_size(&USERFile)) != BL_OK)
    {
        print("Error: app on SD card is too large.\n");

        f_close(&USERFile);
        SD_Eject();
        print("SD ejected.\n");
        return ERR_APP_LARGE;
    }
    print("App size OK.\n");

    /* Step 1: Init Bootloader and Flash */

   /* Check for flash write protection of application area*/
   if(~Bootloader_GetProtectionStatus() & APP_sector_mask) {
        print("Application space in flash is write protected.\n");
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
//        print("Disabling write protection and generating system reset...\n");  // apparently not on a STM32F407
        Bootloader_ConfigProtection(APP_sector_mask);
//        print("Exiting Bootloader.\n");
        print("write protection removed\n");
//        return ERR_WRP_ACTIVE;
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
 //       f_close(&USERFile);
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
  //      fr   = f_read(&USERFile, &data, 8, &num);
        fr   = f_read(&USERFile, &data, 4, &num);
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
                sprintf(msg, "Programming error at: %lu byte\n", (cntr * 4));
                print(msg);

                f_close(&USERFile);
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
    f_close(&USERFile);
    LED_ALL_OFF();
    print("Programming finished.\n");
    sprintf(msg, "Flashed: %lu bytes.\n", (cntr * 4));
    print(msg);

    /* Open file for verification */
    fr = f_open(&USERFile, CONF_FILENAME, FA_READ);
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
    print("Verifying ...\n");
    addr = APP_ADDRESS;
    cntr = 0;
    do
    {
        data = 0xFFFFFFFFFFFFFFFF;
        fr   = f_read(&USERFile, &data, 4, &num);
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

                f_close(&USERFile);
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
f_close(&USERFile);
    print("Verification passed.\n");
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

    fr = f_rename(CONF_FILENAME, new_filename);  // rename file to .CUR
    if(fr != FR_OK)
    {
        /* f_open failed */
        print("File cannot be renamed.\n");
        sprintf(msg, "FatFs error code: %u\n", fr);
        print(msg);

      //SD_Eject();               // allow loading application even if can't rename
      //print("SD ejected.\n");
      //return ERR_SD_FILE;
    }
#endif

    /* Eject SD card */
    SD_Eject();
    print("SD ejected.\n");

    /* Enable flash write protection */
#if(USE_WRITE_PROTECTION)
    print("Enabling flash write protection and generating system reset...\n");
    if(Bootloader_ConfigProtection(BL_PROTECTION_WRP) != BL_OK)
    {
        print("Failed to enable write protection.\n");
        print("Exiting Bootloader.\n");
    }
#endif

    return ERR_OK;
}


/**
 * @brief  UART3 initialization function. UART3 is used for debugging. The
 *         data sent over UART2 is forwarded to the USB virtual com port by the
 *         ST-LINK located on the discovery board.
 * @param  None
 * @retval None
 */
void UART3_Init(void)
{
  //MX_USART3_UART_Init();
}

/**
 * @brief  UART3 de-initialization function.
 * @param  None
 * @retval None
 */
void UART3_DeInit(void)
{
    HAL_UART_DeInit(&huart3);
    __HAL_RCC_USART3_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_6);

    __HAL_RCC_USART3_FORCE_RESET();
    __HAL_RCC_USART3_RELEASE_RESET();
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
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
