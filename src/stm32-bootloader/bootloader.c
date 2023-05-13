/**
 *******************************************************************************
 * STM32 Bootloader Source
 *******************************************************************************
 * @author Akos Pasztor
 * @file   bootloader.c
 * @brief  This file contains the functions of the bootloader. The bootloader
 *	       implementation uses the official HAL library of ST.
 *
 * @see    Please refer to README for detailed information.
 *******************************************************************************
 * @copyright (c) 2020 Akos Pasztor.                    https://akospasztor.com
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "bootloader.h"
#include "main.h"
#include <string.h>  // debug
#include <stdio.h>   // debug
#include <inttypes.h>  // debug

void print(const char* str);   // debug
void k_delay(const uint32_t ms);

/* Private defines -----------------------------------------------------------*/
#define BOOTLOADER_VERSION_MAJOR 1 /*!< Major version */
#define BOOTLOADER_VERSION_MINOR 1 /*!< Minor version */
#define BOOTLOADER_VERSION_PATCH 3 /*!< Patch version */
#define BOOTLOADER_VERSION_RC    0 /*!< Release candidate version */

/* Private typedef -----------------------------------------------------------*/
typedef void (*pFunction)(void); /*!< Function pointer definition */

/* Private variables ---------------------------------------------------------*/
/** Private variable for tracking flashing progress */
static uint32_t flash_ptr = APP_ADDRESS;

uint32_t APP_first_sector;  // first FLASH sector an application can be loaded into
uint32_t APP_first_addr;    // beginning address of first FLASH sector an application can be loaded into
uint32_t APP_sector_mask;   // mask used to determine if any application sectors are write protected
                            // F407 mask is actually the first 12 bits in the upper word
uint32_t WRITE_protection = 0xFFFFFFFF;  // default to removing write protection from all pages 
// force the following unintialized variables into a seperate section so they don't get overwritten
// when the reset routine zeroes out the bss section       
uint32_t __attribute__((section("no_init"))) WRITE_Prot_Old_Flag;  // flag if protection was removed (in case need to restore write protection)
uint32_t __attribute__((section("no_init"))) Write_Prot_Old;
// back to normal                 
uint32_t Magic_Location = Magic_BootLoader;  // flag to tell if to boot into bootloader or the application
// provide method for assembly file to access #define values
uint32_t MagicBootLoader = Magic_BootLoader;
uint32_t MagicApplication = Magic_Application;
uint32_t APP_ADDR = APP_ADDRESS;

char msg[64];             

void NVIC_System_Reset(void);
/**
 * @brief  This function initializes bootloader and flash.
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK is returned in every case
 */
uint8_t Bootloader_Init(void)
{
    extern uint32_t _siccmram[];
    // Read and use the `_siccmram` linkerscript variable
    uint32_t siccmram = (uint32_t)_siccmram;
    #define BOOT_LOADER_END siccmram

    /* Clear flash flags */
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    HAL_FLASH_Lock();

    APP_first_sector = 0;
    APP_first_addr = 0;
   
    // STM32F407 has different length FLASH sectors.
    //   Sector 0 to Sector 3 being 16 KB each
    //   Sector 4 is 64 KB
    //   Sector 5–11 are 128 KB each
    
    if (BOOT_LOADER_END <= 0xE0000 + FLASH_BASE) {APP_first_sector = FLASH_SECTOR_11;  APP_first_addr = 0xE0000 + FLASH_BASE;}
    if (BOOT_LOADER_END <= 0xC0000 + FLASH_BASE) {APP_first_sector = FLASH_SECTOR_10;  APP_first_addr = 0xC0000 + FLASH_BASE;}
    if (BOOT_LOADER_END <= 0xA0000 + FLASH_BASE) {APP_first_sector = FLASH_SECTOR_9;   APP_first_addr = 0xA0000 + FLASH_BASE;}
    if (BOOT_LOADER_END <= 0x80000 + FLASH_BASE) {APP_first_sector = FLASH_SECTOR_8;   APP_first_addr = 0x80000 + FLASH_BASE;}
    if (BOOT_LOADER_END <= 0x60000 + FLASH_BASE) {APP_first_sector = FLASH_SECTOR_7;   APP_first_addr = 0x60000 + FLASH_BASE;}
    if (BOOT_LOADER_END <= 0x40000 + FLASH_BASE) {APP_first_sector = FLASH_SECTOR_6;   APP_first_addr = 0x40000 + FLASH_BASE;}
    if (BOOT_LOADER_END <= 0x20000 + FLASH_BASE) {APP_first_sector = FLASH_SECTOR_5;   APP_first_addr = 0x20000 + FLASH_BASE;}
    if (BOOT_LOADER_END <= 0x10000 + FLASH_BASE) {APP_first_sector = FLASH_SECTOR_4;   APP_first_addr = 0x10000 + FLASH_BASE;}
    if (BOOT_LOADER_END <= 0x0C000 + FLASH_BASE) {APP_first_sector = FLASH_SECTOR_3;   APP_first_addr = 0x0C000 + FLASH_BASE;}
    if (BOOT_LOADER_END <= 0x08000 + FLASH_BASE) {APP_first_sector = FLASH_SECTOR_2;   APP_first_addr = 0x08000 + FLASH_BASE;}
    if (BOOT_LOADER_END <= 0x04000 + FLASH_BASE) {APP_first_sector = FLASH_SECTOR_1;   APP_first_addr = 0x04000 + FLASH_BASE;}
    
    
    sprintf(msg, "\nBOOT_LOADER_END %08lX\n", BOOT_LOADER_END);
    print(msg);
    sprintf(msg, "Lowest possible APP_ADDRESS is %08lX\n", APP_first_addr);
    print(msg);
    /* check APP_ADDRESS */
    if (APP_ADDRESS & 0x1ff) {
      print("ERROR - application address not on 512 byte boundary\n");
      Error_Handler();
    }
    if (APP_ADDRESS < APP_first_addr) {
      print("ERROR - application address within same sector as boot loader\n");
      Error_Handler();
    } 
    
    if (APP_OFFSET == 0) return BL_ERASE_ERROR;   // start of boot program
    if (APP_first_sector == 0) return BL_ERASE_ERROR;   // application is within same sector as bootloader


    APP_sector_mask = 0;
    for (uint8_t i = APP_first_sector; i <= LAST_SECTOR; i++) {  // generate mask of sectors we do NOT want write protected
      APP_sector_mask |= 1 << i;
    }
    
    //sprintf(msg, "APP_sector_mask: %08lX\n", APP_sector_mask);
    //print(msg);
    
    return BL_OK;
}

/**
 * @brief  This function erases the user application area in flash
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: upon success
 * @retval BL_ERR: upon failure
 */
uint8_t Bootloader_Erase(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
    HAL_FLASH_Unlock();  
    for (uint32_t i =  APP_first_sector; i <= LAST_SECTOR; i++) {
      kprint(" Erasing sector: %d\n",(uint16_t)i);
//      __disable_irq();
      FLASH_Erase_Sector(i, VOLTAGE_RANGE_3);
      while(FLASH->SR & FLASH_FLAG_BSY){};   // wait for completion
 //     __enable_irq();
      if (FLASH->SR) {
        kprint(" FLASH status register: : %08lX\n",FLASH->SR);
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
      }
//      k_delay(100);
      /* Toggle green LED during erasing */
      LED_G1_TG();
    }

    HAL_FLASH_Lock();
    return (status == HAL_OK) ? BL_OK : BL_ERASE_ERROR;
}

/**
 * @brief  Begin flash programming: this function unlocks the flash and sets
 *         the data pointer to the start of application flash area.
 * @see    README for futher information
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK is returned in every case
 */
uint8_t Bootloader_FlashBegin(void)
{
    /* Reset flash destination address */
    flash_ptr = APP_ADDRESS;

    /* Unlock flash */
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );

    return BL_OK;
}

/**
 * @brief  Program 64bit data into flash: this function writes an 8byte (64bit)
 *         data chunk into the flash and increments the data pointer.
 * @see    README for futher information
 * @param  data: 64bit data chunk to be written into flash
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: upon success
 * @retval BL_WRITE_ERROR: upon failure
 */
uint8_t Bootloader_FlashNext(uint64_t data)
{
    char msg[64]; //debug
    uint64_t read_data;
    HAL_StatusTypeDef status = HAL_OK; //debug
    if(!(flash_ptr <= (FLASH_BASE + FLASH_SIZE - 8)) ||
       (flash_ptr < APP_ADDRESS))
    {
        HAL_FLASH_Lock();
        return BL_WRITE_ERROR;
    }

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_ptr, data);
    if(status == HAL_OK)
    {
        /* Check the written value */
        read_data = *(uint64_t*)flash_ptr; 
        if(read_data != data)
        {
            /* Flash content doesn't match source content */
            HAL_FLASH_Lock();
            kprint("Programming error\n");
            sprintf(msg, "  expected data (64 bit): %08lX %08lX\n", (uint32_t) (data >> 32) ,(uint32_t) data);
            kprint(msg);
            sprintf(msg, "  actual data (64 bit)  : %08lX %08lX\n", (uint32_t) (read_data >> 32) ,(uint32_t) read_data);
            kprint(msg);
            sprintf(msg, "  absolute address (byte): %08lX\n", flash_ptr);
            kprint(msg);
                     
            
            
            return BL_WRITE_ERROR;
        }
        /* Increment Flash destination address */
//        flash_ptr += 8;
        flash_ptr += 4;
    }
    else
    {
        /* Error occurred while writing data into Flash */ 
        HAL_FLASH_Lock();
        return BL_WRITE_ERROR;
    }

    return BL_OK;
}

/**
 * @brief  Finish flash programming: this function finalizes the flash
 *         programming by locking the flash.
 * @see    README for futher information
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK is returned in every case
 */
uint8_t Bootloader_FlashEnd(void)
{
    /* Lock flash */
    HAL_FLASH_Lock();

    return BL_OK;
}

/**
 * @brief  This function returns the protection status of flash.
 * @return Flash protection status ::eFlashProtectionTypes
 */
uint32_t Bootloader_GetProtectionStatus(void)
  {
    FLASH_OBProgramInitTypeDef OBStruct = {0};
    uint32_t protection                  = BL_PROTECTION_NONE;

    HAL_FLASH_Unlock();

    HAL_FLASHEx_OBGetConfig(&OBStruct);
    return OBStruct.WRPSector;

    /* RDP */
    if(OBStruct.RDPLevel != OB_RDP_LEVEL_0)
    {
        protection |= BL_PROTECTION_RDP;
    }

    HAL_FLASH_Lock();
    return protection;
}

// debug helper routine
const char *byte_to_binary (uint32_t x)
{
    static char b[33];
    b[0] = '\0';

    uint32_t z;
    for (z = 1 << 31; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}


/**
 * @brief  This function configures the write protection of flash.
 * @param  protection: protection type ::eFlashProtectionTypes
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: upon success
 * @retval BL_OBP_ERROR: upon failure
 *
 * Setting the protection is a five step process
 *   1) Determine final proection
 *   2) Disable protection on desired sectors
 *   3) Enable protection on all other sectors
 *   4) Invoke HAL_FLASH_OB_Launch()
 *   5) Send the system through reset so that the new settings take effect
 * 
 */
uint8_t Bootloader_ConfigProtection(uint32_t protection, uint32_t mask, uint8_t save) {  
  FLASH_OBProgramInitTypeDef OBStruct = {0};
  HAL_StatusTypeDef status            = HAL_ERROR;

  status = HAL_FLASH_Unlock();
  status |= HAL_FLASH_OB_Unlock();
  
  HAL_FLASHEx_OBGetConfig(&OBStruct);  // get current FLASH config
  
  uint32_t WRPSector_save = OBStruct.WRPSector;
  if (save) Write_Prot_Old = WRPSector_save;   // save current FLASH protect incase we do a restore later
    
    uint32_t final_protection = (protection & mask) | (WRPSector_save & ~mask); // keep protection of bootloader area
    
    //sprintf(msg,"\nsave flag: %0u\n", save);
    //print(msg);
    //sprintf(msg,"requested protection:  %08lX\n", protection);
    //print(msg);
    //
    //sprintf(msg,"mask:                  %08lX\n", mask);
    //print(msg);
    //
    //sprintf(msg,"final protection:      %08lX\n", final_protection);
    //print(msg);
    //
    //sprintf(msg,"reported protection:   %08lX\n", WRPSector_save);
    //print(msg);
    
    
    if (save) {  // only removing write protection
    
    OBStruct.WRPState = OB_WRPSTATE_DISABLE;    //  disable write protection
    OBStruct.WRPSector = final_protection;            // select affected sectors
    status = HAL_FLASHEx_OBProgram(&OBStruct);  // write 
    
    //HAL_FLASHEx_OBGetConfig(&OBStruct);  // get current FLASH config
    //sprintf(msg,"after disable:         %08lX\n", OBStruct.WRPSector);
    //print(msg);
    
  }
  else {

    OBStruct.WRPState = OB_WRPSTATE_ENABLE;      //  enable write protection
    OBStruct.WRPSector = ~final_protection;       // select affected sectors
    status |= HAL_FLASHEx_OBProgram(&OBStruct);  // write 
    
    //HAL_FLASHEx_OBGetConfig(&OBStruct);  // get current FLASH config
    //sprintf(msg,"after enable:          %08lX\n", OBStruct.WRPSector);
    //print(msg);
    
  }
  if(status == HAL_OK)
  {
    if (save) {
      print("write protection removed\n");
      WRITE_Prot_Old_Flag = WRITE_Prot_Original_flag;  // flag that protection was removed so can 
    }  
    else {
      print("write protection restored\n");
      WRITE_Prot_Old_Flag = WRITE_Prot_Old_Flag_Restored_flag;  // flag that protection was restored so won't 
                                                                // try to save write protection after next reset)
    }                                       
      /* Loading Flash Option Bytes - this generates a system reset. */    // apparently not on a STM32F407
      status |= HAL_FLASH_OB_Launch();        //  this is needed plus still need to go through reset  
      
      //HAL_FLASHEx_OBGetConfig(&OBStruct);  // get current FLASH config
      //sprintf(msg,"after OB_Launch:       %08lX\n", OBStruct.WRPSector);
      //print(msg);
      
      
      NVIC_System_Reset();                  // send the system through reset so Flash Option Bytes get loaded
  }

  status |= HAL_FLASH_OB_Lock();
  status |= HAL_FLASH_Lock();

  return (status == HAL_OK) ? BL_OK : BL_OBP_ERROR;
}

/**
 * @brief  This function checks whether the new application fits into flash.
 * @param  appsize: size of application
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: if application fits into flash
 * @retval BL_SIZE_ERROR: if application does not fit into flash
 */
uint8_t Bootloader_CheckSize(uint32_t appsize)
{
    return ((FLASH_BASE + FLASH_SIZE - APP_ADDRESS) >= appsize) ? BL_OK
                                                                : BL_SIZE_ERROR;
}

/**
 * @brief  This function verifies the checksum of application located in flash.
 *         If ::USE_CHECKSUM configuration parameter is disabled then the
 *         function always returns an error code.
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: if calculated checksum matches the application checksum
 * @retval BL_CHKS_ERROR: upon checksum mismatch or when ::USE_CHECKSUM is
 *         disabled
 */
uint8_t Bootloader_VerifyChecksum(void)
{
#if(USE_CHECKSUM)
    CRC_HandleTypeDef CrcHandle;
    volatile uint32_t calculatedCrc = 0;

    __HAL_RCC_CRC_CLK_ENABLE();
    CrcHandle.Instance                     = CRC;
    CrcHandle.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;
    CrcHandle.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;
    CrcHandle.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;
    CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    CrcHandle.InputDataFormat              = CRC_INPUTDATA_FORMAT_WORDS;
    if(HAL_CRC_Init(&CrcHandle) != HAL_OK)
    {
        return BL_CHKS_ERROR;
    }

    calculatedCrc =
        HAL_CRC_Calculate(&CrcHandle, (uint32_t*)APP_ADDRESS, APP_SIZE);

    __HAL_RCC_CRC_FORCE_RESET();
    __HAL_RCC_CRC_RELEASE_RESET();

    if((*(uint32_t*)CRC_ADDRESS) == calculatedCrc)
    {
        return BL_OK;
    }
#endif
    return BL_CHKS_ERROR;
}

/**
 * @brief  This function checks whether a valid application exists in flash.
 *         The check is performed by checking the very first uint32_t (4 bytes) of
 *         the application firmware. In case of a valid application, this uint32_t
 *         must represent the initialization location of stack pointer - which
 *         must be within the boundaries of RAM.
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: if first uint32_t represents a valid stack pointer location
 * @retval BL_NO_APP: first uint32_t value is out of RAM boundaries
 */
uint8_t Bootloader_CheckForApplication(void)
{
    return (((*(uint32_t*)APP_ADDRESS) - RAM_BASE) <= RAM_SIZE) ? BL_OK
                                                                : BL_NO_APP;
}

/**
 * @brief  This function performs the jump to the user application in flash.
 * @details The function carries out the following operations:
 *  - De-initialize the clock and peripheral configuration
 *  - Stop the systick
 *  - Set the vector table location (if ::SET_VECTOR_TABLE is enabled)
 *  - Sets the stack pointer location
 *  - Perform the jump
 */
void Bootloader_JumpToApplication(void)
{
  
  Magic_Location = Magic_Application;  // flag that we should load application 
                                       // after the next reset
  NVIC_System_Reset();                  // send the system through reset
  
//    uint32_t JumpAddress = *(__IO uint32_t*)(APP_ADDRESS + 4);
//    pFunction Jump       = (pFunction)JumpAddress;
//    
//    //char msg[64];
//    //print("JumpToApplication\n");
//    //sprintf(msg, "PC  : %08lX\n", *(__IO uint32_t*)(APP_ADDRESS + 4));
//    //print(msg);
//    //sprintf(msg, "SP  : %08lX\n", *(__IO uint32_t*)APP_ADDRESS);
//    //print(msg);
//    //sprintf(msg, "VTOR: %08lX\n", APP_ADDRESS);
//    //print(msg);
//    //k_delay(500);
//    
//    HAL_RCC_DeInit();
//    HAL_DeInit();
//
//    SysTick->CTRL = 0;
//    SysTick->LOAD = 0;
//    SysTick->VAL  = 0;
//
//#if(SET_VECTOR_TABLE)
//    SCB->VTOR = APP_ADDRESS;
//#endif
//
//    __set_MSP(*(__IO uint32_t*)APP_ADDRESS);
//    Jump();
}

/**
 * @brief  This function performs the jump to the MCU System Memory (ST
 *         Bootloader).
 * @details The function carries out the following operations:
 *  - De-initialize the clock and peripheral configuration
 *  - Stop the systick
 *  - Remap the system flash memory
 *  - Perform the jump
 */
void Bootloader_JumpToSysMem(void)
{
  Magic_Location = Magic_Application;  // flag that we should load application 
                                       // after the next reset
  NVIC_System_Reset();                  // send the system through reset

  
  
  //uint32_t JumpAddress = *(__IO uint32_t*)(SYSMEM_ADDRESS + 4);
  //pFunction Jump       = (pFunction)JumpAddress;
  //
  //HAL_RCC_DeInit();
  //HAL_DeInit();
  //
  //SysTick->CTRL = 0;
  //SysTick->LOAD = 0;
  //SysTick->VAL  = 0;
  //
  //__HAL_RCC_SYSCFG_CLK_ENABLE();
  //__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
  //
  //__set_MSP(*(__IO uint32_t*)SYSMEM_ADDRESS);
  //Jump();

  //while(1)
  //    ;
}

/**
 * @brief  This function returns the version number of the bootloader library.
 *         Semantic versioning is used for numbering.
 * @see    Semantic versioning: https://semver.org
 * @return Bootloader version number combined into an uint32_t:
 *          - [31:24] Major version
 *          - [23:16] Minor version
 *          - [15:8]  Patch version
 *          - [7:0]   Release candidate version
 */
uint32_t Bootloader_GetVersion(void)
{
    return ((BOOTLOADER_VERSION_MAJOR << 24) |
            (BOOTLOADER_VERSION_MINOR << 16) | (BOOTLOADER_VERSION_PATCH << 8) |
            (BOOTLOADER_VERSION_RC));
}


/**
  \brief   System Reset
  \details Initiates a system reset request to reset the MCU.
 */
void NVIC_System_Reset(void)
{
  #define SCB_AIRCR_VECTKEY_Pos 16U   /*!< SCB AIRCR: VECTKEY Position */
  #define SCB_AIRCR_SYSRESETREQ_Pos 2U   /*!< SCB AIRCR: VECTKEY Position */
  volatile uint32_t* SCB_AIRCR = (uint32_t*)0xE000ED0CUL;  

*SCB_AIRCR = (uint32_t)(((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | (1 << SCB_AIRCR_SYSRESETREQ_Pos)));

  for(;;)                                                           /* wait until reset */
  {
  }
}