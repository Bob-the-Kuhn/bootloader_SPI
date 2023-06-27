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
#if(USE_CHECKSUM)
  #include "stm32f1xx_hal_crc.h"
#endif     
#include <string.h>  // debug
#include <stdio.h>   // debug
#include <inttypes.h>  // debug

//void kprint(const char* str);   // debug
void k_delay(const uint32_t ms);

void NVIC_System_Reset(void);

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

uint16_t APP_first_sector = 0; // first FLASH sector an application can be loaded into
uint32_t APP_first_addr = 0;  // beginning address of first FLASH sector an application can be loaded into
uint32_t APP_sector_mask = 0xFFFFFFFF;  // mask to not affect bootloader sectors,  defaults to removing write protection from all pages 
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

/**
 * @brief  This function initializes bootloader and flash.
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK is returned in every case
 */
uint8_t Bootloader_Init(void)
{
    extern uint32_t _sidata[];  // end of program, start of data to be copied to RAM
    extern uint32_t _sdata[];   // start of RAM area for data
    extern uint32_t _edata[];   // end of RAM area for data
    // Read and use the linkerscript variables to determine end of bootloader in FLASH
    uint32_t boot_loader_end = (uint32_t)_sidata + (uint32_t)_edata - (uint32_t)_sdata ; 
    #define BOOT_LOADER_END boot_loader_end 

    /* Clear flash flags */
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    HAL_FLASH_Lock();

    // STM32F103ZE has 256 pages of 2K bytes each
    // pages 0-61 are write protected as pairs
    // pages 62-251 are protected as a block
    
    // The protection is not affected if a page pair/block only has the bootloader in it.
    // The protection is cleared if a page pair/block has only the application space in it
    // The protection is cleared if a page pair/block has both the bootloader the application space in it
 
    
    uint32_t mask = 1;  // mask for clearing write protect bits
    for (uint16_t counter = 1; counter < FLASH_SIZE/FLASH_SECTOR_SIZE; counter++) {   // find 
      if (!(counter % 2) && (counter <= 62)) mask = mask << 1; // move mask to left every 2 pages until past 62
      APP_first_addr = ((counter * FLASH_SECTOR_SIZE) + FLASH_BASE);
      if (BOOT_LOADER_END <= APP_first_addr) {
        APP_first_sector = counter; 
        APP_sector_mask |= mask;   // remove write protection on this page pair 
        break;
      }
      else {
        APP_sector_mask &= ~mask;  // don't touch write protection on pages with bootloader in it
      }
    }
    
    //sprintf(msg, "\nAPP_sector_mask %08lX\n", APP_sector_mask);
    //kprint(msg);
    // 
    sprintf(msg, "\nBOOT_LOADER_END %08lX\n", BOOT_LOADER_END);
    kprint(msg);
    //sprintf(msg, "Lowest possible APP_ADDRESS is %08lX\n", APP_first_addr);
    //kprint(msg);
 
    /* check APP_ADDRESS */
    if (APP_ADDRESS & 0x1ff) {
      kprint("ERROR - application address not on 512 byte boundary\n");
      Error_Handler();
    }
    if (APP_ADDRESS < APP_first_addr) {
      kprint("ERROR - application address within same sector as boot loader\n");
      Error_Handler();
    } 
    
    if (APP_OFFSET == 0) return BL_ERASE_ERROR;   // start of boot program
    if (APP_first_sector == 0) return BL_ERASE_ERROR;   // application is within same sector as bootloader

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
    uint32_t PageError;
    FLASH_EraseInitTypeDef pEraseInit;
    pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;  // erase pages mode
    pEraseInit.Banks = 0;                          // don't care in erase pages mode    
//    pEraseInit.PageAddress = APP_first_addr;       // address within first page to erase
//    pEraseInit.NbPages =  FLASH_SIZE/FLASH_SECTOR_SIZE - APP_first_sector + 1;

    HAL_FLASH_Unlock();  
    
    LED_G1_ON(); 
    LED_G2_OFF();
    #define NBPAGES (FLASH_SIZE/FLASH_SECTOR_SIZE - APP_first_sector)
    #define FLASH_INCR 25  // number of pages to erase before reporting status
    for (uint16_t count = 0; count < NBPAGES; count += FLASH_INCR) {
      pEraseInit.PageAddress = APP_first_addr + count * FLASH_SECTOR_SIZE;       // address within first page to erase
      pEraseInit.NbPages = ((count + FLASH_INCR) < NBPAGES) ? 25 : (NBPAGES - count); // number pages to erase this loop
      sprintf(msg, "Erasing page: %u\n", APP_first_sector + count);
      LED_ALL_TG();
      kprint(msg);
      HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
      if ((status != HAL_OK) | (PageError != 0xFFFFFFFF)) {
        sprintf(msg, "ERROR: status:    %u\n", status);
        kprint(msg);
        sprintf(msg, "ERROR: PageError: %08lX\n", PageError);
        kprint(msg);
      }
    }
    
    HAL_FLASH_Lock();                 
    return (PageError == 0xFFFFFFFF) ? BL_OK : BL_ERASE_ERROR;
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
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);
    
    return BL_OK;
}

/**
 * @brief  Program 16 bit data into flash: this function writes an 2 byte (16 bit)
 *         data chunk into the flash and increments the data pointer.
 * @see    README for futher information
 * @param  data: pointer to buffer with data to be written into flash
                 buffer needs to be 16 bit word aligned to match FLASH writes
           count: number of bytes in the buffer
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: upon success
 * @retval BL_WRITE_ERROR: upon failure
 */
uint8_t Bootloader_FlashNext_Buf(uint8_t *data, UINT count)
{
    if(!(flash_ptr <= (FLASH_BASE + FLASH_SIZE - count)) ||
       (flash_ptr < APP_ADDRESS))
    {
        HAL_FLASH_Lock();
        return BL_WRITE_ERROR;
    }

    uint16_t read_data;
    uint16_t data_16;
    uint16_t i;
    for (i = 0; i < count/2; i++) {
      data_16 = (uint16_t) (*data  + (*(data+1) << 8));  // assemble two bytes
      SET_BIT(FLASH->CR, FLASH_CR_PG);  // tell FLASH controller we'll be writing to FLASH
      *(__IO uint16_t*)flash_ptr = data_16;
      read_data = *(uint16_t*)flash_ptr;
      if (read_data != data_16)            
      {
      /* Flash content doesn't match source content */
        HAL_FLASH_Lock();
        kprint("Programming error\n");
        sprintf(msg, "expected data (16 bit): %04X\n", data_16);
        kprint(msg);   
        sprintf(msg, "actual data (16 bit)  : %04X\n", read_data);
        kprint(msg);   
        sprintf(msg, "absolute address (byte): %08lX\n", flash_ptr);
        kprint(msg);
    
        HAL_FLASH_Lock();
        return BL_WRITE_ERROR;
      }
      flash_ptr += 2;
      data += 2;
    } 
    
    if (count%2) {  // have an odd number of bytes - need to pad it out & write it - this will only happen at the end of the file
      data_16 = ((*data << 8) + *(data+1));  // assemble two bytes into a
      SET_BIT(FLASH->CR, FLASH_CR_PG);  // tell FLASH controller we'll be writing to FLASH
      *(__IO uint16_t*)flash_ptr = data_16;
      read_data = *(uint16_t*)flash_ptr;
      if(read_data != data_16)            
      {
      /* Flash content doesn't match source content */
        HAL_FLASH_Lock();
        kprint("Programming error\n");
        sprintf(msg, "expected data (16 bit): %04X\n", data_16);
        kprint(msg);   
        sprintf(msg, "actual data (16 bit)  : %04X\n", read_data);
        kprint(msg);   
        sprintf(msg, "absolute address (byte): %08lX\n", flash_ptr);
        kprint(msg);
        
        HAL_FLASH_Lock();
        return BL_WRITE_ERROR;
      }
    }
     
    if (FLASH->CR & (FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR)) return BL_WRITE_ERROR;

    HAL_FLASH_Lock();
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
    uint64_t read_data;
    HAL_StatusTypeDef status = HAL_OK; //debug
    if(!(flash_ptr <= (FLASH_BASE + FLASH_SIZE - 8)) ||
       (flash_ptr < APP_ADDRESS))
    {
        HAL_FLASH_Lock();
        return BL_WRITE_ERROR;
    }

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flash_ptr, data);
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
        flash_ptr += 8;
//        flash_ptr += 4;
    }
    else
    {
        /* Error occurred while writing data into Flash */ 
        HAL_FLASH_Lock();
        
        sprintf(msg, "programming error - status: %04X\n", status);
        kprint(msg);
        
        sprintf(msg, "absolute address (byte): %08lX\n", flash_ptr);
        kprint(msg);
        
        sprintf(msg, "sector: %08lX\n", ((flash_ptr - 0x08000000)/FLASH_SECTOR_SIZE));
        kprint(msg);
        
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
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

    return BL_OK;
}

/**
 * @brief  This function returns the protection status of flash.
 * @return Flash protection status ::eFlashProtectionTypes
 */
uint32_t Bootloader_GetProtectionStatus(void)
  {
    FLASH_OBProgramInitTypeDef OBStruct = {0};

    HAL_FLASH_Unlock();

    HAL_FLASHEx_OBGetConfig(&OBStruct);
    HAL_FLASH_Lock();
    return OBStruct.WRPPage;  // 1 means NOT write protected
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


/********************************************************************
 * The following code is based on ST's STM32 library.
 * 
 * It has been modified so that setting and clearing the write protect
 * option bits is done in a single pass.
 * 
 * Process overview:
 *   1) save existing DATA and USER bytes
 *   2) erase option bytes
 *        This routine saves & restores the RDP byte.
 *   3) restore the DATA and USER bytes
 *   4) program the WRP bytes
 * 
 *  It does NOT save the compliments of the bytes.
 * 
 */


static HAL_StatusTypeDef FLASH_OB_ProgramOB_byte(uint32_t Address, uint8_t Data)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
  
  if(status == HAL_OK)
  {
    /* Clean the error context */
    //pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;

    /* Enables the Option Bytes Programming operation */
    SET_BIT(FLASH->CR, FLASH_CR_OPTPG); 
    *(__IO uint16_t*)Address = Data;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
    
    /* If the program operation is completed, disable the OPTPG Bit */
    CLEAR_BIT(FLASH->CR, FLASH_CR_OPTPG);
  }
  /* Return the Option Byte Data Program Status */
  return status;
}

 
HAL_StatusTypeDef _HAL_FLASHEx_OBProgram_WRP(uint32_t WRP_bits) { 
  
//  sprintf(msg,"WRP_bits               %08lX\n", WRP_bits);
                                                                    
//  kprint(msg);
  
  
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_FLASH_Unlock();
  status |= HAL_FLASH_OB_Unlock();  // unlock option bytes
                                                         
  
  if (status != HAL_OK) {kprint("unlock error\n"); goto QUIT;}
                                                                      
                 
  
  volatile uint8_t* OB_BLOCK = (uint8_t*)0x1FFFF800UL; // pointer to option byte block
        
  
  // save option bytes we won't be messing with
  //uint8_t RDP   = *OB_BLOCK;  // don't mess with RDP - erase routine saves & stores it
  uint8_t USER  = *(OB_BLOCK + 2);
  uint8_t Data0 = *(OB_BLOCK + 4);
  uint8_t Data1 = *(OB_BLOCK + 6);
  
  // breakup WRP into bytes
  uint8_t WRP0 = (uint8_t) WRP_bits;
  uint8_t WRP1 = (uint8_t) (WRP_bits >> 8);
  uint8_t WRP2 = (uint8_t) (WRP_bits >> 16);
  uint8_t WRP3 = (uint8_t) (WRP_bits >> 24);

  status |= FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
  
  if (status != HAL_OK) {
    kprint("FLASH_TIMEOUT error\n"); 
    // goto QUIT;  // try to restore the OB block no matter what
  }
  
 
  if(FLASH->CR & 1) {
    FLASH->CR &= 0xFFFFFFFE;  // clear out the PE bit
                                                                                                  
  }
  
  status |= HAL_FLASHEx_OBErase();  // erase OB block
  
  if (status != HAL_OK) {
    kprint("erase OB block error\n"); 
//    FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
//    status = HAL_FLASHEx_OBErase();  // erase OB block
//    if (status != HAL_OK) {
//      kprint("erase OB block error 2\n");
//    }
//    else {
//      status = HAL_OK;
//    }
//    
    // goto QUIT;  // try to restore the OB block no matter what
  }
  
  status |= FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
  
  
  if (status != HAL_OK) {
    kprint("WaitForLastOperation error\n"); 
    sprintf(msg,"erase error - HAL status:          %02X\n", status);
    kprint(msg);
    sprintf(msg,"FLASH status register:          %08lX\n", FLASH->SR);
    kprint(msg);
    // goto QUIT;  // try to restore the OB block no matter what
  }   
  
  uint32_t timeout = FLASH_TIMEOUT_VALUE;

  status |= FLASH_OB_ProgramOB_byte(0x1FFFF802, USER);
  status |= FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
  k_delay(10);

  status |= FLASH_OB_ProgramOB_byte(0x1FFFF804, Data0);
  status |= FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
  k_delay(10);

  status |= FLASH_OB_ProgramOB_byte(0x1FFFF806, Data1);
  status |= FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
  k_delay(10);

  status |= FLASH_OB_ProgramOB_byte(0x1FFFF808, WRP0);
  status |= FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
  k_delay(10);

  status |= FLASH_OB_ProgramOB_byte(0x1FFFF80A, WRP1);
  status |= FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
  k_delay(10);

  status |= FLASH_OB_ProgramOB_byte(0x1FFFF80C, WRP2);
  status |= FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
  k_delay(10);

  status |= FLASH_OB_ProgramOB_byte(0x1FFFF80E, WRP3);
  status |= FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);


  if (HAL_GetTick() >= timeout) {kprint("timeout - WRP3\n"); goto QUIT;}
  
  if (status != HAL_OK) kprint("program byte error\n"); 
  
  status |= HAL_FLASH_OB_Lock();
  status |= HAL_FLASH_Lock();
  
  if (status != HAL_OK) kprint("lock error\n");
  
  ////uint8_t _RDP   = *OB_BLOCK;
  //uint8_t _USER  = *(OB_BLOCK + 2);
  //uint8_t _Data0 = *(OB_BLOCK + 4);
  //uint8_t _Data1 = *(OB_BLOCK + 6);
  //uint8_t _WRP0  = *(OB_BLOCK + 8);
  //uint8_t _WRP1  = *(OB_BLOCK + 10);
  //uint8_t _WRP2  = *(OB_BLOCK + 12);
  //uint8_t _WRP3  = *(OB_BLOCK + 14);
  
  //sprintf(msg,"WRP read back          %02X%02X%02X%02X\n", _WRP3, _WRP2, _WRP1, _WRP0);
  //kprint(msg);
  
  //////if (RDP   != _RDP)   while(1){};
  //if (USER  != _USER)  FLASH_OB_ProgramOB_byte(0x1FFFF802, USER);
  //if (Data0 != _Data0) FLASH_OB_ProgramOB_byte(0x1FFFF804, Data0);
  //if (Data1 != _Data1) FLASH_OB_ProgramOB_byte(0x1FFFF806, Data1);
  //if (WRP0  != _WRP0)  FLASH_OB_ProgramOB_byte(0x1FFFF808, WRP0);
  //if (WRP1  != _WRP1)  FLASH_OB_ProgramOB_byte(0x1FFFF80A, WRP1);
  //if (WRP2  != _WRP2)  FLASH_OB_ProgramOB_byte(0x1FFFF80C, WRP2);
  //if (WRP3  != _WRP3)  FLASH_OB_ProgramOB_byte(0x1FFFF80E, WRP3);
  //
  //_WRP0  = *(OB_BLOCK + 8);
  //_WRP1  = *(OB_BLOCK + 10);
  //_WRP2  = *(OB_BLOCK + 12);
  //_WRP3  = *(OB_BLOCK + 14);
  //
  //
  //sprintf(msg,"WRP read back          %02X%02X%02X%02X\n", _WRP3, _WRP2, _WRP1, _WRP0);
  //kprint(msg);
  
 QUIT: 
  status |= HAL_FLASH_OB_Lock();
  status |= HAL_FLASH_Lock();
  
  return status;
}
    
  
  
/**
 * @brief  This function configures the write protection of flash.
 * @param  protection: desired protection: '1' - no protection, '0' write protect
 *         mask: '1' - program per protection , '0' don't touch this bit
 *         set: unused
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: upon success
 * @retval BL_OBP_ERROR: upon failure
 *
 * 
 * Setting the protection is a three step process
 *   1) Determine final protection
 *   2) Invoke _HAL_FLASH_OB_Launch()
 *   3) Send the system through reset so that the new settings take effect
 * 
 */

uint8_t Bootloader_ConfigProtection(uint32_t protection, uint32_t mask, uint8_t set)
{
  FLASH_OBProgramInitTypeDef OBStruct = {0};
  HAL_StatusTypeDef status = HAL_ERROR;

  status = HAL_FLASH_Unlock();
  status |= HAL_FLASH_OB_Unlock();
  
  //sprintf(msg,"requested protection:  %08lX\n", protection);
  //kprint(msg);
  //
  //sprintf(msg,"mask:                  %08lX\n", mask);
  //kprint(msg);
  
  HAL_FLASHEx_OBGetConfig(&OBStruct);  // get current FLASH config
  uint32_t WRPSector_save = OBStruct.WRPPage;
  uint32_t final_protection = (protection & mask) | (WRPSector_save & ~mask);
  
  //sprintf(msg,"final_protection:      %08lX\n", final_protection);
  //kprint(msg);
  //
  //
  //sprintf(msg,"reported protection:   %08lX\n", WRPSector_save);
  //kprint(msg);
  
  status |= HAL_FLASH_OB_Lock();
  status |= HAL_FLASH_Lock();
  
  status = _HAL_FLASHEx_OBProgram_WRP(final_protection );  // set protection of affected sectors
  

  HAL_FLASH_OB_Launch();      // sends system through reset which loads the OB bytes into the registers  

  return (status == HAL_OK) ? BL_OK : BL_OBP_ERROR;
}

#if 0   //  standard library

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
  
  uint32_t WRPSector_save = OBStruct.WRPPage;
  OBStruct.OptionType = OPTIONBYTE_WRP;       //  enable changing write protection
  if (save) Write_Prot_Old = WRPSector_save;   // save current FLASH protect incase we do a restore later
    
    uint32_t final_protection = (protection & mask) | (WRPSector_save & ~mask); // keep protection of bootloader area
    
    //sprintf(msg,"\nsave flag: %0u\n", save);
    //kprint(msg);
    //sprintf(msg,"requested protection:  %08lX\n", protection);
    //kprint(msg);
    //
    //sprintf(msg,"mask:                  %08lX\n", mask);
    //kprint(msg);
    //
    //sprintf(msg,"final protection:      %08lX\n", final_protection);
    //kprint(msg);
    //
    //sprintf(msg,"reported protection:   %08lX\n", WRPSector_save);
    //kprint(msg);
    
    
    if (save) {  // only removing write protection
    
    OBStruct.WRPState = OB_WRPSTATE_DISABLE;    // clear/disable write protection
    OBStruct.WRPPage = final_protection;            // select affected sectors
    status = HAL_FLASHEx_OBProgram(&OBStruct);  // write 
    
    //HAL_FLASHEx_OBGetConfig(&OBStruct);  // get current FLASH config
    //sprintf(msg,"after disable:         %08lX\n", OBStruct.WRPPage);
    //kprint(msg);
    
  }
  else {

    OBStruct.WRPState = OB_WRPSTATE_ENABLE;     // set/activate write protection;
    OBStruct.WRPPage = ~final_protection;       // select affected sectors
    status |= HAL_FLASHEx_OBProgram(&OBStruct);  // write 
    
    //HAL_FLASHEx_OBGetConfig(&OBStruct);  // get current FLASH config
    //sprintf(msg,"after enable:          %08lX\n", OBStruct.WRPPage);
    //kprint(msg);
    
  }
  if(status == HAL_OK)
  {
    if (save) {
      kprint("write protection removed\n");
      WRITE_Prot_Old_Flag = WRITE_Prot_Original_flag;  // flag that protection was removed so can 
    }  
    else {
      kprint("write protection restored\n");
      WRITE_Prot_Old_Flag = WRITE_Prot_Old_Flag_Restored_flag;  // flag that protection was restored so won't 
                                                                // try to save write protection after next reset)
    }                                       

      HAL_FLASH_OB_Launch();        //  this is needed plus still need to go through reset  
      
      //HAL_FLASHEx_OBGetConfig(&OBStruct);  // get current FLASH config
      //sprintf(msg,"after OB_Launch:       %08lX\n", OBStruct.WRPPage);
      //kprint(msg);
      
      
      //NVIC_System_Reset();                  // send the system through reset so Flash Option Bytes get loaded
      HAL_FLASH_OB_Launch();      // sends system through reset which loads the OB bytes into the registers 
  }

  status |= HAL_FLASH_OB_Lock();
  status |= HAL_FLASH_Lock();

  return (status == HAL_OK) ? BL_OK : BL_OBP_ERROR;
}
#endif  // standard library

void save_WRP_state(void) {
  FLASH_OBProgramInitTypeDef OBStruct = {0};
  HAL_FLASHEx_OBGetConfig(&OBStruct);  // get current FLASH config
  Write_Prot_Old = OBStruct.WRPPage;
  WRITE_Prot_Old_Flag = WRITE_Prot_Original_flag;  // flag that protection was saved
}

void report_WP_ConfigProtection(void)
{
  FLASH_OBProgramInitTypeDef OBStruct = {0};
  HAL_StatusTypeDef status = HAL_ERROR;

  status = HAL_FLASH_Unlock();
  status |= HAL_FLASH_OB_Unlock();
 
  HAL_FLASHEx_OBGetConfig(&OBStruct);  // get current FLASH config
  uint32_t temp = OBStruct.WRPPage;
  
  sprintf(msg,"\nWRPPage:               %08lX\n", temp);
  kprint(msg);
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
 *         The check is performed by checking the very first DWORD (4 bytes) of
 *         the application firmware. In case of a valid application, this DWORD
 *         must represent the initialization location of stack pointer - which
 *         must be within the boundaries of RAM.
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: if first DWORD represents a valid stack pointer location
 * @retval BL_NO_APP: first DWORD value is out of RAM boundaries
 */
uint8_t Bootloader_CheckForApplication(void)
{
  return ((((*(uint32_t*)APP_ADDRESS) - RAM_BASE) <= RAM_SIZE) ? BL_OK : BL_NO_APP);
                                                                            
}


/**
 * @brief  This function performs the jump to the user application in flash.
 * @details The function carries out the following operations:
 *  - Sets the flag that we should load application after the next reset
 *  - Sends the card through reset
 *  
 *  The reset handler checks this flag and will either continue to start the
 *  bootloader or it will start the application.
 */

void Bootloader_JumpToApplication(void) {
 
  Magic_Location = Magic_Application;  // flag that we should load application 
                                       // after the next reset
  NVIC_System_Reset();                  // send the system through reset
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
//uint32_t Bootloader_GetVersion(void)
//{
//    return ((BOOTLOADER_VERSION_MAJOR << 24) |
//            (BOOTLOADER_VERSION_MINOR << 16) | (BOOTLOADER_VERSION_PATCH << 8) |
//            (BOOTLOADER_VERSION_RC));
//}


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
