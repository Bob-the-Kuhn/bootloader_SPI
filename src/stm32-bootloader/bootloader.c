/**
 *******************************************************************************
 * STM32 Bootloader Source
 *******************************************************************************
 * @author Akos Pasztor
 * @file   bootloader.c
 * @brief  This file contains the functions of the bootloader. The bootloader
 *         implementation uses the official HAL library of ST.
 *
 * @see    Please refer to README for detailed information.
 *******************************************************************************
 * @copyright (c) 2020 Akos Pasztor.                    https://akospasztor.com
 *******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "bootloader.h"
#include "main.h"
#include "main_boot.h"

#include <string.h>  // debug
#include <stdio.h>   // debug
#include <inttypes.h>  // debug

void print(const char* str);   // debug


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

void Error_Handler_boot(void);

char msg[64];             

void NVIC_System_Reset(void);
/**
 * @brief  This function initializes bootloader and flash.
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK is returned in every case
 */
uint8_t Bootloader_Init(void)
{
    // Use linkerscript variables to find end of boot image
    extern uint32_t _sidata[];
    uint32_t sidata = (uint32_t)_sidata;
    extern uint32_t _sdata[];
    uint32_t sdata = (uint32_t)_sdata;
    extern uint32_t _sbss[];
    uint32_t sbss = (uint32_t)_sbss;
    #define BOOT_LOADER_END (sidata + (sbss - sdata))

    /* Clear flash flags */
    HAL_FLASH_Unlock();
    FLASH->CCR1 = 0xFFFFFFFF;  // clear all FLASH flags bank 1
    FLASH->CCR2 = 0xFFFFFFFF;  // clear all FLASH flags bank 2
    HAL_FLASH_Lock();

    APP_first_sector = 0;
    APP_first_addr = 0;
   
    // STM32H753 has 8 or 16 equal length FLASH sectors.
    //   All sectors are 32k bytes.

    
    uint32_t mask = 1;  // mask for clearing write protect bits
    for (uint16_t counter = 0; counter < LAST_SECTOR; counter++) {   // find 
      
      APP_first_addr = ((counter * FLASH_SECTOR_SIZE) + FLASH_BASE);
      if (BOOT_LOADER_END <= APP_first_addr) {
        APP_first_sector = counter; 
        break;
      }
      else {
        APP_sector_mask |= mask;   // remove write protection on this sector 
        mask = mask << 1; // move mask to left every sector
      }
    }
    
    APP_sector_mask = ~APP_sector_mask;  // don't touch write protection on sectors with bootloader in it
    
    //sprintf(msg, "\nAPP_sector_mask %08lX\n", APP_sector_mask);
    //print(msg);
    

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
    
    //sprintf(msg, "BOOT_LOADER_END: %08lX\n", BOOT_LOADER_END);
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
    FLASH->CCR1 = 0xFFFFFFFF;  // clear all FLASH flags bank 1
    FLASH->CCR2 = 0xFFFFFFFF;  // clear all FLASH flags bank 2
    uint32_t Sector_num;
    uint32_t Bank_num;
    HAL_FLASH_Unlock();  
    for (uint32_t i =  APP_first_sector; i <= LAST_SECTOR; i++) {
      if ((App_Size + APP_ADDRESS) < ((i * FLASH_SECTOR_SIZE) + FLASH_BASE)) break;  // quit if sector doesn't contain the application
      sprintf(msg, " Erasing sector: %d\n",(uint16_t)i);
      print(msg);
      if (i < 8) {
        Sector_num = i;
        Bank_num = FLASH_BANK_1;
      }
      else {
        Sector_num = i - 8;
        Bank_num = FLASH_BANK_2;
      }
    
      __disable_irq();
      FLASH_Erase_Sector(Sector_num, Bank_num, FLASH_VOLTAGE_RANGE_4);
      while(FLASH->SR1 & FLASH_FLAG_BSY){};   // wait for completion
      while(FLASH->SR2 & FLASH_FLAG_BSY){};   // wait for completion
      __enable_irq();
      if (Bank_num == FLASH_BANK_1) {
        if (FLASH->SR1 & (FLASH_SR_OPERR | FLASH_SR_INCERR | FLASH_SR_PGSERR | FLASH_SR_WRPERR)) {
          sprintf(msg, " FLASH status register 1: : %08lX\n",FLASH->SR1);
          print(msg);
          FLASH->CCR1 = (FLASH_CCR_CLR_OPERR | FLASH_CCR_CLR_INCERR | FLASH_CCR_CLR_PGSERR | FLASH_CCR_CLR_WRPERR);
          status = HAL_ERROR;
        }
        else {
          if (FLASH->SR2 & (FLASH_SR_OPERR | FLASH_SR_INCERR | FLASH_SR_PGSERR | FLASH_SR_WRPERR)) {
            sprintf(msg, " FLASH status register 2: : %08lX\n",FLASH->SR2);
            print(msg);
            FLASH->CCR2 = (FLASH_CCR_CLR_OPERR | FLASH_CCR_CLR_INCERR | FLASH_CCR_CLR_PGSERR | FLASH_CCR_CLR_WRPERR);
            status = HAL_ERROR;
          }
        } 
    }
      HAL_Delay(100);
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

    FLASH->CCR1 = 0xFFFFFFFF;  // clear all FLASH flags bank 1
    FLASH->CCR2 = 0xFFFFFFFF;  // clear all FLASH flags bank 2
    
    /* Unlock flash */
    HAL_FLASH_Unlock();
  

    return BL_OK;
}


/**
  * @brief  Program a flash word at a specified address
  * @param  TypeProgram Indicate the way to program at a specified address.
  *         This parameter can be a value of @ref FLASH_Type_Program
  * @param  FlashAddress specifies the address to be programmed.
  *         This parameter shall be aligned to the Flash word:
  *          - 256 bits for STM32H74x/5X devices (8x 32bits words)
  *          - 128 bits for STM32H7Ax/BX devices (4x 32bits words)
  *          - 256 bits for STM32H72x/3X devices (8x 32bits words)
  * @param  DataAddress specifies the address of data to be programmed.
  *         This parameter shall be 32-bit aligned
  *
  * @retval HAL_StatusTypeDef HAL Status
  */
HAL_StatusTypeDef HAL_FLASH_Program_local(uint32_t TypeProgram, uint32_t FlashAddress, uint32_t DataAddress)
{
  HAL_StatusTypeDef status;
  __IO uint32_t *dest_addr = (__IO uint32_t *)FlashAddress;
  __IO uint32_t *src_addr = (__IO uint32_t*)DataAddress;
  
  //uint32_t *dest_addr;
  //dest_addr = FlashAddress;
  //uint32_t *src_addr;
  //src_addr = DataAddress;
  uint32_t bank;
  uint8_t row_index = FLASH_NB_32BITWORD_IN_FLASHWORD;

  /* Check the parameters */
  assert_param(IS_FLASH_TYPEPROGRAM(TypeProgram));
  assert_param(IS_FLASH_PROGRAM_ADDRESS(FlashAddress));

  /* Process Locked */
  __HAL_LOCK(&pFlash);

  if(IS_FLASH_PROGRAM_ADDRESS_BANK1(FlashAddress))
  {
    bank = FLASH_BANK_1;
  }
  else if(IS_FLASH_PROGRAM_ADDRESS_BANK2(FlashAddress))
  {
    bank = FLASH_BANK_2;
  }
  else
  {
    return HAL_ERROR;
  }

  /* Reset error code */
  pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;

  /* Wait for last operation to be completed */
  //status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE, bank);
  status = FLASH_WaitForLastOperation((uint32_t)50000, bank);

  if(status == HAL_OK)
  {
    if(bank == FLASH_BANK_1)
    {
      {
        /* Set PG bit */
        SET_BIT(FLASH->CR1, FLASH_CR_PG);
      }
    }
    else
    {
      /* Set PG bit */
      SET_BIT(FLASH->CR2, FLASH_CR_PG);
    }

    __ISB();
    __DSB();

    {
      /* Program the flash word */
      do
      {
        // *dest_addr = *src_addr;
        uint32_t temp = *src_addr;
        src_addr++;
        *dest_addr = temp;
        dest_addr++;
        row_index--;
     } while (row_index != 0U);
    }

    __ISB();
    __DSB();

    /* Wait for last operation to be completed */
    //status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE, bank);
    status = FLASH_WaitForLastOperation((uint32_t)50000, bank);


    {
      if(bank == FLASH_BANK_1)
      {
        /* If the program operation is completed, disable the PG */
        CLEAR_BIT(FLASH->CR1, FLASH_CR_PG);
      }
      else
      {
        /* If the program operation is completed, disable the PG */
        CLEAR_BIT(FLASH->CR2, FLASH_CR_PG);
      }
    }


  }

  /* Process Unlocked */
  __HAL_UNLOCK(&pFlash);

  return status;
}


//volatile uint32_t pointer_to_flash_ptr;

/**
 * @brief  Program 256 bit data into flash: this function writes an 32 byte
 *         data chunk into the flash and increments the data pointer.
 * @see    README for futher information
 * @param  data_address: address of 256 bit data chunk to be written into flash
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: upon success
 * @retval BL_WRITE_ERROR: upon failure
 */
uint8_t Bootloader_FlashNext(uint8_t *data)
{
    uint8_t read_data[NUM_BYTES_PER_PASS];
    HAL_StatusTypeDef status = HAL_OK; //debug
    if(!(flash_ptr <= (FLASH_BASE + FLASH_SIZE - NUM_BYTES_PER_PASS)) ||
       (flash_ptr < APP_ADDRESS))
    {
        HAL_FLASH_Lock();
        return BL_WRITE_ERROR;
    }
 
    //pointer_to_flash_ptr = (uint32_t)flash_ptr;  // ?? HAL wants a pointer to a pointer to the FLASH destination address??
    status = HAL_FLASH_Program_local(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t)flash_ptr, (uint32_t)data);  
    if(status == HAL_OK)
    {
        /* Check the written value */
        uint8_t count;
        for (count = 0; count < NUM_BYTES_PER_PASS; count++) {
          read_data[count]  = *(uint8_t*)(flash_ptr + count); 
          if(read_data[count] != data[count])
          {
              /* Flash content doesn't match source content */
              HAL_FLASH_Lock();
              print("Programming error\n");
              sprintf(msg, "  expected data (8 bit): %02X\n", data[count]);
              print(msg);
              sprintf(msg, "  actual data (8 bit)  : %02X\n", read_data[count]);
              print(msg);
              sprintf(msg, "  absolute address (byte): %08lX\n", flash_ptr + count);
              print(msg);
              return BL_WRITE_ERROR;
          }
        }
        /* Increment Flash destination address */
        //flash_ptr += 8;
        flash_ptr += NUM_BYTES_PER_PASS;
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
  //  FLASH_OBProgramInitTypeDef OBStruct = {0};

 //   OBStruct.Banks = FLASH_BANK_BOTH;
    
    HAL_FLASH_Unlock();
 
    //HAL_FLASHEx_OBGetConfig(&OBStruct);
    
    uint32_t WP_bank_1 = FLASH->WPSN_CUR1;
    uint32_t WP_bank_2 = FLASH->WPSN_CUR2;
    
    HAL_FLASH_Lock();
//    return (OBStruct.WRPSector >> 16);  // Write protect bits are in bits 16-23 on F746
//     return (OBStruct.WRPSector); 
    return (WP_bank_1 | (WP_bank_2 << 8) | 0xFFFF0000UL);  
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
 * @param  protection: desired protection - 1: not protected, 0: protected
           mask ; which sectors to ignore - 1: change this sector, 0: keep current sector protection
           save: 
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: upon success
 * @retval BL_OBP_ERROR: upon failure
 *
 * Setting the protection is a four step process
 *   1) enable access to OPTCR register
 *   2) Write protection bits to WPSN_PRGx registers
 *   3) Write 1 to OPTSTART bit in OPTCR register to start programming sequence 
 *         H7 CPUs don't need to go through reset to program option bytes
 *   4) Wait for OPT_BUSY bit to go inactive in OPTSR_CUR register
 *
 * Protection bits 0-7 are bank 1 sectors 0-7
 * Protection bits 8-15 are bank 2 sectors 0-7
 * 
 */
uint8_t Bootloader_ConfigProtection(uint32_t protection, uint32_t mask, uint8_t save) {  
  //FLASH_OBProgramInitTypeDef OBStruct = {0};
  HAL_StatusTypeDef status            = HAL_ERROR;


  //sprintf(msg,"current protection:    %08lX\n", Bootloader_GetProtectionStatus());
  //print(msg);
  //sprintf(msg,"requested protection:  %08lX\n", protection);
  //print(msg);
  //sprintf(msg,"mask:                  %08lX\n", mask);
  //print(msg);
  //sprintf(msg,"save:                  %0X\n", save);
  //print(msg);


  status = HAL_FLASH_Unlock();
  status |= HAL_FLASH_OB_Unlock();
  
  uint32_t WRPSector_save = Bootloader_GetProtectionStatus();
  if (save) Write_Prot_Old = WRPSector_save;   // save current FLASH protect incase we do a restore later
  
  uint32_t final_protection = (protection & mask) | (WRPSector_save & ~mask); // keep protection of bootloader area
 
  FLASH->WPSN_PRG1 = final_protection & 0x000000ffUL;
  FLASH->WPSN_PRG2 = (final_protection >> 8) & 0x000000ffUL;
  FLASH->OPTCR |= 0x02UL;  //set OPTSTART bit to 1 to start programming sequence
  while(FLASH->OPTSR_CUR & 0x01UL);  // wait for OPT_BUSY to go inactive (programming sequence to finish)

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
      /* Loading Flash Option Bytes. */    
      status |= HAL_FLASH_OB_Launch();        //  H753 -  changes option bytes, reset not needed.
      
      
     // NVIC_System_Reset();                  // send the system through reset so Flash Option Bytes get loaded
  }

  //sprintf(msg,"new protection:        %08lX\n", Bootloader_GetProtectionStatus());
  //print(msg);

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
  
  //sprintf(msg,"application size:      %08lX\n", appsize);
  //print(msg);
  //sprintf(msg,"end of application:    %08lX\n", appsize + APP_ADDRESS);
  //print(msg);
  
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
  Error_Handler_boot();
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
//    //HAL_Delay(500);
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