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
  #include "stm32g0xx_hal_crc.h"
#endif     
#include <string.h>  // debug
#include <stdio.h>   // debug
#include <inttypes.h>  // debug

//void kprint(const char* str);   // debug
void k_delay(const uint32_t ms);
uint32_t k_ticks(void);

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
uint32_t __attribute__((section("no_init"))) wrp_old[4];  // array to hold original write protect settings
uint8_t __attribute__((section("no_init"))) wrp1a_strt;
uint8_t __attribute__((section("no_init"))) wrp1a_end;
uint8_t __attribute__((section("no_init"))) wrp2a_strt;
uint8_t __attribute__((section("no_init"))) wrp2a_end; 
uint8_t __attribute__((section("no_init"))) wrp1b_strt;
uint8_t __attribute__((section("no_init"))) wrp1b_end;
uint8_t __attribute__((section("no_init"))) wrp2b_strt;
uint8_t __attribute__((section("no_init"))) wrp2b_end; 
uint8_t __attribute__((section("no_init"))) wrp1a_strt_original;
uint8_t __attribute__((section("no_init"))) wrp1a_end_original;
uint8_t __attribute__((section("no_init"))) wrp2a_strt_original;
uint8_t __attribute__((section("no_init"))) wrp2a_end_original; 
uint8_t __attribute__((section("no_init"))) wrp1b_strt_original;
uint8_t __attribute__((section("no_init"))) wrp1b_end_original;
uint8_t __attribute__((section("no_init"))) wrp2b_strt_original;
uint8_t __attribute__((section("no_init"))) wrp2b_end_original; 
// back to normal     

uint32_t Magic_Location = Magic_BootLoader;  // flag to tell if to boot into bootloader or the application
// provide method for assembly file to access #define values
uint32_t MagicBootLoader = Magic_BootLoader;
uint32_t MagicApplication = Magic_Application;
uint32_t APP_ADDR = APP_ADDRESS;

uint8_t last_boot_page;
uint32_t app_size;



uint32_t Bank1a_prot_start;
uint32_t Bank1a_prot_end;
uint32_t Bank2a_prot_start;
uint32_t Bank2a_prot_end;
uint32_t Bank1b_prot_start;
uint32_t Bank1b_prot_end;
uint32_t Bank2b_prot_start;
uint32_t Bank2b_prot_end;

//uint32_t temp_wrp[4] = { (1 | (2 << FLASH_WRP1AR_WRP1A_END_Pos)),  (3 | (4 << FLASH_WRP1BR_WRP1B_END_Pos)), 
//                         (5 | (6 << FLASH_WRP2AR_WRP2A_END_Pos)),  (7 | (8 << FLASH_WRP2BR_WRP2B_END_Pos))};
//
uint32_t wrp_save[4]; 

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
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_SR_ERRORS);
  HAL_FLASH_Lock();
  
  // STM32G0B1RE has 256 pages of 2K bytes each
  
  // 512K bytes of FLASH in two banks
  // 256 sectors/pages of 2K bytes each
  // FLASH start 0800 0000
  // bank1 - 0800 0000 - 0803 FFFF (sectors   0-127)
  // bank2 - 0804 0000 - 0807 FFFF (sectors 128-255)
  
  // Three sector/page lists:
  //   boot: 0 through last_boot_page
  //   wrp_bank1
  //   wrp_bank2
  
  
  for (uint16_t counter = 0; counter < FLASH_SIZE/FLASH_SECTOR_SIZE; counter++) {   // find 
    APP_first_addr = ((counter * FLASH_SECTOR_SIZE) + FLASH_BASE);
    if (BOOT_LOADER_END <= APP_first_addr) {
      APP_first_sector = counter; 
      break;
    }
    else {
      last_boot_page = counter;
    }
  }
  
  //sprintf(msg, "\nAPP_sector_mask %08lX\n", APP_sector_mask);
  //kprint(msg);
  // 
  sprintf(msg, "\nBOOT_LOADER_END %08lX\n", BOOT_LOADER_END);
  kprint(msg);
  sprintf(msg, "Lowest possible APP_ADDRESS is %08lX\n", APP_first_addr);
  kprint(msg);
  
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
  
  wrp1a_strt = (uint8_t)(rd32(R_FLASH_WRP1AR) & 0x07f);
  wrp1a_end  = (uint8_t)((rd32(R_FLASH_WRP1AR) >> 16) & 0x07f);
  wrp2a_strt = (uint8_t)(rd32(R_FLASH_WRP2AR) & 0x07f);
  wrp2a_end  = (uint8_t)((rd32(R_FLASH_WRP2AR) >> 16) & 0x07f);
  wrp1b_strt = (uint8_t)(rd32(R_FLASH_WRP1BR) & 0x07f);
  wrp1b_end  = (uint8_t)((rd32(R_FLASH_WRP1BR) >> 16) & 0x07f);
  wrp2b_strt = (uint8_t)(rd32(R_FLASH_WRP2BR) & 0x07f);
  wrp2b_end  = (uint8_t)((rd32(R_FLASH_WRP2BR) >> 16) & 0x07f);
  
  if (wrp1a_end < wrp1a_strt) {
    Bank1a_prot_start = 0;
    Bank1a_prot_end   = 0;
  }
  else {
    Bank1a_prot_start = FLASH_BASE + (wrp1a_strt * FLASH_SECTOR_SIZE) ;
    Bank1a_prot_end   =  FLASH_BASE + ((wrp1a_end + 1) * FLASH_SECTOR_SIZE) -1; 
  }
  
  if (wrp2a_end < wrp2a_strt) {
    Bank2a_prot_start = 0;
    Bank2a_prot_end   = 0;
  }
  else {
    Bank2a_prot_start = FLASH_BASE + ((wrp2a_strt + 128) * FLASH_SECTOR_SIZE);
    Bank2a_prot_end   = FLASH_BASE + ((wrp2a_end + 128 + 1) * FLASH_SECTOR_SIZE) -1 ;
  }  
  
  
  if (wrp1b_end < wrp1b_strt) {
    Bank1b_prot_start = 0;
    Bank1b_prot_end   = 0;
  }
  else {
    Bank1b_prot_start = FLASH_BASE + (wrp1b_strt * FLASH_SECTOR_SIZE) ;
    Bank1b_prot_end   =  FLASH_BASE + ((wrp1b_end + 1) * FLASH_SECTOR_SIZE) -1; 
  }
  
  if (wrp2b_end < wrp2b_strt) {
    Bank2b_prot_start = 0;
    Bank2b_prot_end   = 0;
  }
  else {
    Bank2b_prot_start = FLASH_BASE + ((wrp2b_strt + 128) * FLASH_SECTOR_SIZE);
    Bank2b_prot_end   = FLASH_BASE + ((wrp2b_end + 128 + 1) * FLASH_SECTOR_SIZE) -1 ;
  } 
  
  return BL_OK;
}


/**
  * @brief  Erase the specified FLASH memory page.
  * @param  Banks: Banks to be erased
  *         This parameter can one of the following values:
  *            @arg FLASH_BANK_1: Bank1 to be erased
  *            @arg FLASH_BANK_2: Bank2 to be erased*
  * @param  Page FLASH page to erase
  *         This parameter must be a value between 0 and (max number of pages in Flash - 1)
  * @note (*) availability depends on devices
  * @retval None
  */
void FLASH_PageErase_(uint32_t Banks, uint32_t Page)
{
  uint32_t tmp;

  /* Get configuration register, then clear page number */
  tmp = (FLASH->CR & ~FLASH_CR_PNB);

  /* Check if page has to be erased in bank 1 or 2 */
  if (Banks != FLASH_BANK_1)
  {
    tmp |= FLASH_CR_BKER;
  }
  else
  {
    tmp &= ~FLASH_CR_BKER;
  }


  /* Set page number, Page Erase bit & Start bit */
  FLASH->CR = (tmp | (FLASH_CR_STRT | (Page <<  FLASH_CR_PNB_Pos) | FLASH_CR_PER));
}

/**
  * @brief  Perform a mass erase or erase the specified FLASH memory pages.
  * @param[in]  pEraseInit Pointer to an @ref FLASH_EraseInitTypeDef structure that
  *         contains the configuration information for the erasing.
  * @param[out]  PageError Pointer to variable that contains the configuration
  *         information on faulty page in case of error (0xFFFFFFFF means that all
  *         the pages have been correctly erased)
  * @retval HAL Status
  */
HAL_StatusTypeDef HAL_FLASHEx_Erase_(uint32_t bank, uint8_t start_page, uint8_t num_to_erase, uint32_t *PageError)
{
  HAL_StatusTypeDef status;
  uint32_t index;


  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

  if (status == HAL_OK)
  {

    /*Initialization of PageError variable*/
    *PageError = 0xFFFFFFFFU;

    for (index = start_page; index < (start_page + num_to_erase); index++)
    {
      /* Start erase page */
      FLASH_PageErase_(bank, index);

      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);

      if (status != HAL_OK)
      {
        /* In case of error, stop erase procedure and return the faulty address */
        *PageError = index;
        break;
      }
    }

    /* If operation is completed or interrupted, disable the Page Erase Bit */
    CLEAR_BIT(FLASH->CR, FLASH_CR_PER);

  }


  /* return status */
  return status;
}

/**
  * @brief  This function erases the user application area in flash
  * @return Bootloader error code ::eBootloaderErrorCodes
  * @retval BL_OK: upon success
  * @retval BL_ERR: upon failure
  *
  * Each bank is erased seperately.
  * 
*/
uint8_t Bootloader_Erase(void)
{
  uint32_t PageError;
  
  uint8_t start_page = (APP_ADDRESS - FLASH_BASE)/FLASH_SECTOR_SIZE;
  uint8_t nbpages = app_size/FLASH_SECTOR_SIZE; // number full pages occupied by application
  if (app_size % FLASH_SECTOR_SIZE) nbpages++; // + 1 if have a partial page used
  
  //sprintf(msg, "start_page       : %2X\n", start_page);
  //kprint(msg);
  //sprintf(msg, "nbpages          : %2X\n", nbpages);
  //kprint(msg);
  
  uint8_t start_page_bank_1;
  uint8_t start_page_bank_2;
  uint8_t nbpages_bank_1;
  uint8_t nbpages_bank_2;
  
  if (start_page < 128) {
    start_page_bank_1 = start_page;
    if ((start_page + nbpages) <128) {  // application entirely within bank 1
      start_page_bank_2 = 0;
      nbpages_bank_1 = nbpages;
      nbpages_bank_2 = 0;
    }
    else {                              // application in both banks 1 & 2
      start_page_bank_2 = 0;
      nbpages_bank_1 = 128 - start_page;
      nbpages_bank_2 = nbpages - nbpages_bank_1 ;
    }
  }
  else {                                // application only in bank 2
    start_page_bank_1 = 0;
    nbpages_bank_1 = 0;
    start_page_bank_2 = start_page - 128;
    nbpages_bank_2 = nbpages;
  }
  
  //sprintf(msg, "start_page_bank_1: %2X\nstart_page_bank_2: %2X\n", start_page_bank_1, start_page_bank_2);
  //kprint(msg);
  //sprintf(msg, "nbpages_bank_1:    %2X\nnbpages_bank_2:    %2X\n", nbpages_bank_1, nbpages_bank_2);
  //kprint(msg);
  
  HAL_FLASH_Unlock();  
  
  LED_G1_ON(); 
  LED_G2_OFF();
  
  #define FLASH_INCR 25  // number of pages to erase before reporting status
  uint8_t num_to_erase;
  
  if (nbpages_bank_1) {
    for (uint16_t count = 0; count < nbpages_bank_1; count += FLASH_INCR) {
      num_to_erase = ((count + FLASH_INCR) < nbpages_bank_1) ? FLASH_INCR : (nbpages_bank_1 - count); // number pages to erase this loop
      sprintf(msg, "Erasing bank 1, page: %02X  num to erase: %02X\n", start_page_bank_1 + count, num_to_erase);
      LED_ALL_TG();
      kprint(msg);
      HAL_StatusTypeDef status = HAL_FLASHEx_Erase_(FLASH_BANK_1, start_page_bank_1 + count, num_to_erase, &PageError);
      //uint32_t* flash_addr = (uint32_t *)(((start_page_bank_1 + count) *FLASH_SECTOR_SIZE) + FLASH_BASE);
      //uint32_t flash_data = *flash_addr; // read first word of just erased page
      //sprintf(msg, "count: %02X  flash_addr: %08lX  flash_data: %08lX\n", count, flash_addr, flash_data);
      //kprint(msg);
      //sprintf(msg, " num pages erased:   %02X\n",  num_to_erase);
      //kprint(msg);
      
      if ((status != HAL_OK) | (PageError != 0xFFFFFFFF)) {
        sprintf(msg, "ERROR: status:    %u\n", status);
        kprint(msg);
        sprintf(msg, "ERROR: PageError: %08lX\n", PageError);
        kprint(msg);
      }
    }
  }
  
  if (nbpages_bank_2) {
    for (uint16_t count = 0; count < nbpages_bank_2; count += FLASH_INCR) {
      num_to_erase = ((count + FLASH_INCR) < nbpages_bank_2) ? FLASH_INCR : (nbpages_bank_2 - count); // number pages to erase this loop
      sprintf(msg, "Erasing bank 2, page: %02X  num to erase: %02X\n", start_page_bank_2 + count, num_to_erase);
      LED_ALL_TG();
      kprint(msg);
      HAL_StatusTypeDef status = HAL_FLASHEx_Erase_(FLASH_BANK_2, start_page_bank_2 + count, num_to_erase, &PageError);
      if ((status != HAL_OK) | (PageError != 0xFFFFFFFF)) {
        sprintf(msg, "ERROR: status:    %u\n", status);
        kprint(msg);
        sprintf(msg, "ERROR: PageError: %08lX\n", PageError);
        kprint(msg);
      }
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
  __HAL_FLASH_CLEAR_FLAG(FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PROGERR);
  
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
  
  if (FLASH->SR & (FLASH_SR_WRPERR | FLASH_SR_PROGERR)) return BL_WRITE_ERROR;
  
  HAL_FLASH_Lock();
  return BL_OK;
}


/**
  * @brief  Wait for a FLASH operation to complete.
  * @param  Timeout maximum flash operation timeout
  * @retval HAL_StatusTypeDef HAL Status
  */
HAL_StatusTypeDef FLASH_WaitForLastOperation_(uint32_t Timeout)
{
  uint32_t error;
  /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH operation fails, the BUSY flag will be reset and an error
     flag will be set */
  uint32_t timeout = k_ticks() + Timeout;

  /* Wait if any operation is ongoing */
#if defined(FLASH_DBANK_SUPPORT)
  error = (FLASH_SR_BSY1 | FLASH_SR_BSY2);
#else
  error = FLASH_SR_BSY1;
#endif /* FLASH_DBANK_SUPPORT */

  while ((FLASH->SR & error) != 0x00U)
  {
    if (k_ticks() >= timeout)
    {
      return HAL_TIMEOUT;
    }
  }

  /* check flash errors */
  error = (FLASH->SR & FLASH_SR_ERRORS);

  /* Clear SR register */
  FLASH->SR = FLASH_SR_CLEAR;

  if (error != 0x00U)
  {
    return HAL_ERROR;
  }

  /* Wait for control register to be written */
  timeout = k_ticks() + Timeout;

  while ((FLASH->SR & FLASH_SR_CFGBSY) != 0x00U)
  {
    if (k_ticks() >= timeout)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

/**
  * @brief  Program double-word (64-bit) at a specified address.
  * @param  Address Specifies the address to be programmed.
  * @param  Data Specifies the data to be programmed.
  * @retval None
  */
static void FLASH_Program_DoubleWord_(uint32_t Address, uint64_t Data)
{
  /* Set PG bit */
  SET_BIT(FLASH->CR, FLASH_CR_PG);

  /* Program first word */
  *(uint32_t *)Address = (uint32_t)Data;

  /* Barrier to ensure programming is performed in 2 steps, in right order
    (independently of compiler optimization behavior) */
  __ISB();

  /* Program second word */
  *(uint32_t *)(Address + 4U) = (uint32_t)(Data >> 32U);
}


/**
  * @brief  Program double word or fast program of a row at a specified address.
  * @param  TypeProgram Indicate the way to program at a specified address.
  *                      This parameter can be a value of @ref FLASH_Type_Program
  * @param  Address Specifies the address to be programmed.
  * @param  Data Specifies the data to be programmed
  *               This parameter is the data for the double word program and the address where
  *               are stored the data for the row fast program depending on the TypeProgram:
  *               TypeProgram = FLASH_TYPEPROGRAM_DOUBLEWORD (64-bit)
  *               TypeProgram = FLASH_TYPEPROGRAM_FAST (32-bit).
  *
  * @retval HAL_StatusTypeDef HAL Status
  */
HAL_StatusTypeDef HAL_FLASH_Program_Double(uint32_t Address, uint64_t Data)
{
  HAL_StatusTypeDef status;

 
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation_(FLASH_TIMEOUT_VALUE);

  if (status == HAL_OK)
  {
    /* Program double-word (64-bit) at a specified address */
    FLASH_Program_DoubleWord_(Address, Data);

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation_(FLASH_TIMEOUT_VALUE);

    /* If the program operation is completed, disable the PG */
    CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
  }

  /* Process Unlocked */
  __HAL_UNLOCK(&pFlash);

  /* return status */
  return status;
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
  
  status = HAL_FLASH_Program_Double(flash_ptr, data);
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
  __HAL_FLASH_CLEAR_FLAG(FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PROGERR);
  
  return BL_OK;
}

/**
  * @brief  This function returns the protection status of the application area flash.
  * @return true if application area is protected
*/
uint8_t Bootloader_GetProtectionStatus(void)
{
  #define WITHIN(N,L,H)       ((N) >= (L) && (N) <= (H))
  
  if (app_size) {
    uint8_t first_app_page = (APP_ADDRESS - FLASH_BASE)/FLASH_SECTOR_SIZE;
    uint8_t last_app_page = ((app_size/FLASH_SECTOR_SIZE) + first_app_page);
    if (app_size % FLASH_SECTOR_SIZE) last_app_page++;  // include partial page
    
    sprintf(msg, "first_app_page : %2X   last_app_page: %2X\n", first_app_page, last_app_page);
    kprint(msg);

    if (wrp1a_end >= wrp1a_strt) {
      // CASE A - wrp1a starts in app area
      if (WITHIN(wrp1a_strt, first_app_page, last_app_page)) {
        return 1;
      }
      // CASE B - wrp1a ends in application area
      if (WITHIN(wrp1a_end, first_app_page, last_app_page)) {
        return 1;
      }
      // CASE C - application entirely within wrp1a
      if (WITHIN(last_app_page, wrp1a_strt, wrp1a_end)) {
        return 1;
      }
    }  // end wrp1a
    
    
    if (wrp1b_end >= wrp1b_strt) {
      // CASE A - wrp1b starts in app area
      if (WITHIN(wrp1b_strt, first_app_page, last_app_page)) {
        return 1;
      }
      // CASE B - wrp1b ends in application area
      if (WITHIN(wrp1b_end, first_app_page, last_app_page)) {
        return 1;
      }
      // CASE C - application entirely within wrp1b
      if (WITHIN(last_app_page, wrp1b_strt, wrp1b_end)) {
        return 1;
      }
    }  // end wrp1b
    
    
    if (wrp2a_end >= wrp2a_strt) {
      // CASE A - wrp2a starts in app area
      if (WITHIN(wrp2a_strt + 128, first_app_page, last_app_page)) {
        return 1;
      }
      // CASE B - wrp2a ends in application area
      if (WITHIN(wrp2a_end + 128, first_app_page, last_app_page)) {
        return 1;
      }
      // CASE C - application entirely within wrp2a
      if (WITHIN(last_app_page, wrp2a_strt + 128, wrp2a_end + 128)) {
        return 1;
      }
    }  // end wrp2a
    
    
    if (wrp2b_end >= wrp2b_strt) {
      // CASE A - wrp2b starts in app area
      if (WITHIN(wrp2b_strt + 128, first_app_page, last_app_page)) {
        return 1;
      }
      // CASE B - wrp2b ends in application area
      if (WITHIN(wrp2b_end + 128, first_app_page, last_app_page)) {
        return 1;
      }
      // CASE C - application entirely within wrp2b
      if (WITHIN(last_app_page, wrp2b_strt + 128, wrp2b_end + 128)) {
        return 1;
      }
    }  // end wrp2b
    
  }  // app size
  return 0;
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
  * @retval BL_OK: 
  
  *
  * Setting the protection is a five step process
  *   1) Determine final proection
  *   2) Disable protection on desired sectors
  *   3) Enable protection on all other sectors
  *   4) Invoke HAL_FLASH_OB_Launch()
  *   5) Send the system through reset so that the new settings take effect
  * 
*/

uint8_t Bootloader_ConfigProtection_Set(uint32_t *data) {  
  /*           @arg @ref OB_WRPAREA_ZONE_A Flash Zone A
    *           @arg @ref OB_WRPAREA_ZONE_B Flash Zone B
    *           @arg @ref OB_WRPAREA_ZONE2_A Flash Bank 2 Zone A (*)
    *           @arg @ref OB_WRPAREA_ZONE2_B Flash Bank 2 Zone B (*)
    * @note  (*) availability depends on devices
    * @param  WRPStartOffset  Specifies the start page of the write protected area
    *         This parameter can be page number between 0 and (max number of pages in the Flash Bank - 1)
    * @param  WRDPEndOffset  Specifies the end page of the write protected area
    *         This parameter can be page number between WRPStartOffset and (max number of pages in the Flash Bank - 1)
    * @retval None
  */
  
  HAL_FLASH_Unlock();
  HAL_FLASH_OB_Unlock();
  while( rd32(R_FLASH_SR) & FLASH_SR_BSY1);  // wait until BSY1 is inactive before programming
  
  uint8_t* wrp1ar_strt_ptr = (uint8_t*)0x4002202C;
  uint8_t* wrp1ar_end_ptr  = (uint8_t*)0x4002202E;
  uint8_t* wrp1br_strt_ptr = (uint8_t*)0x40022030;
  uint8_t* wrp1br_end_ptr  = (uint8_t*)0x40022032;
  uint8_t* wrp2ar_strt_ptr = (uint8_t*)0x4002204C;
  uint8_t* wrp2ar_end_ptr  = (uint8_t*)0x4002204E;
  uint8_t* wrp2br_strt_ptr = (uint8_t*)0x40022050;
  uint8_t* wrp2br_end_ptr  = (uint8_t*)0x40022052;
  
  #define FLASH_WRP_MSK 0x0000007F   
  
  *wrp1ar_strt_ptr = (uint8_t)(data[0] & FLASH_WRP_MSK); 
  *wrp1ar_end_ptr  = (uint8_t)((data[0] >> 16) & FLASH_WRP_MSK); 
  *wrp1br_strt_ptr = (uint8_t)(data[1] & FLASH_WRP_MSK); 
  *wrp1br_end_ptr  = (uint8_t)((data[1] >> 16) & FLASH_WRP_MSK);
  *wrp2ar_strt_ptr = (uint8_t)(data[2] & FLASH_WRP_MSK); 
  *wrp2ar_end_ptr  = (uint8_t)((data[2] >> 16) & FLASH_WRP_MSK);
  *wrp2br_strt_ptr = (uint8_t)(data[3] & FLASH_WRP_MSK); 
  *wrp2br_end_ptr  = (uint8_t)((data[3] >> 16) & FLASH_WRP_MSK);
  
  
  //FLASH_OBProgramInitTypeDef pOBInit ={0};
  //HAL_FLASHEx_OBGetConfig(&pOBInit);
  //
  //pOBInit.OptionType = OPTIONBYTE_WRP;
  //pOBInit.WRPArea = OB_WRPAREA_ZONE_A; // OB_WRPAREA_ZONE_A, OB_WRPAREA_ZONE_B , OB_WRPAREA_ZONE2_A, OB_WRPAREA_ZONE2_B
  //pOBInit.WRPStartOffset = 0x2;  //  zero based sector/page number
  //pOBInit.WRPEndOffset = 0x4; 
  //
  //HAL_FLASHEx_OBProgram(&pOBInit);
  //
  or32(R_FLASH_CR , FLASH_CR_OPTSTRT);   //starts option byte programming
  while( rd32(R_FLASH_SR) & FLASH_SR_BSY1);  // wait until BSY1 is inactive
  or32(R_FLASH_CR , FLASH_CR_OBL_LAUNCH);  //sends system through reset which makes the new option bytes active.
  HAL_FLASH_OB_Lock();   // should never be executed
  HAL_FLASH_Lock();   // should never be executed
  
  return BL_OK;
}

void Bootloader_ConfigProtection_Save(uint32_t *data) {  
  /*           @arg @ref OB_WRPAREA_ZONE_A Flash Zone A
    *           @arg @ref OB_WRPAREA_ZONE_B Flash Zone B
    *           @arg @ref OB_WRPAREA_ZONE2_A Flash Bank 2 Zone A (*)
    *           @arg @ref OB_WRPAREA_ZONE2_B Flash Bank 2 Zone B (*)
    * @note  (*) availability depends on devices
    * @param  WRPStartOffset  Specifies the start page of the write protected area
    *         This parameter can be page number between 0 and (max number of pages in the Flash Bank - 1)
    * @param  WRDPEndOffset  Specifies the end page of the write protected area
    *         This parameter can be page number between WRPStartOffset and (max number of pages in the Flash Bank - 1)
    * @retval None
  */
  
  data[0] = rd32(R_FLASH_WRP1AR);
  data[1] = rd32(R_FLASH_WRP1BR);
  data[2] = rd32(R_FLASH_WRP2AR);
  data[3] = rd32(R_FLASH_WRP2BR);
  
}

void Bootloader_ConfigProtection_Clear(uint32_t *data) {  
  
  data[0] = 0x0000007F; // start > end so no protection
  data[1] = 0x0000007F; // start > end so no protection
  data[2] = 0x0000007F; // start > end so no protection
  data[3] = 0x0000007F; // start > end so no protection
  
}


// remove protection from application area
// 
// To simplify the logic, wrp1a and wrp1b are treated as one large protection area
//   the final wrp1a holds the protected area before the application
//   the final wrp1b holds the protected area after the application
//
// NOTE: this may protect pages not originally protected.  That's not a problem
//       because the "extra" pages don't get written to and the original
//       protection is restored later in the program.
//
// always sends system through reset
uint8_t  Bootloader_ConfigProtection_Keep_Boot(void) {  
  
  #define WITHIN(N,L,H)       ((N) >= (L) && (N) <= (H))
  #define MIN_2(a,b)      ((a)<(b)?(a):(b))
  #define MAX_2(a,b)      ((a)>(b)?(a):(b))
  uint32_t temp_wrp[4] = {0};
  
  if (app_size) {
    
    uint8_t first_app_page = (APP_ADDRESS - FLASH_BASE)/ FLASH_SECTOR_SIZE;
    uint8_t last_app_page = ((app_size/FLASH_SECTOR_SIZE) + first_app_page);
    if (app_size % FLASH_SECTOR_SIZE) last_app_page++;  // include partial page
 
 
// WRP1
 
    uint8_t wrp_strt = MIN_2(wrp1a_strt, wrp1b_strt);  // combine wrp1a and wrp1b into one large virtual area
    uint8_t wrp_end  = MAX_2(wrp1a_end, wrp1b_end);
    sprintf(msg, "wrp_strt 1: %2X\n", wrp_strt);
    kprint(msg);
    sprintf(msg, "wrp_end 1:  %2X\n", wrp_end);
    kprint(msg);
      
      
    temp_wrp[0] = 0x0000007F;         // set default as disabled (set last < start)
    temp_wrp[1] = 0x0000007F;         // set default as disabled (set last < start)

    
    
    if (first_app_page <= 0x7f) {  // app starts in bank 1)
      uint8_t last_app_page_wrp1 = last_app_page;
      if (last_app_page > 0x7f) last_app_page_wrp1 = 0x7f;  //limit last page to bank 1
      if (wrp_end >= wrp_strt) {
        // CASE A - wrp1 starts in app area
        if (WITHIN(wrp_strt, first_app_page, last_app_page_wrp1)) {
          if ( wrp_end <= last_app_page_wrp1) {  
            // CASE A1 - wrp1 entirely within application area
            kprint("CASE: A1\n");
            //temp_wrp[0] = 0x0000007F;         // disable wrp1 (set last < start)
            //temp_wrp[1] = 0x0000007F;         // disable wrp1 (set last < start)
          }
          else {
            // CASE A2 - wrp1 extends beyond application
            kprint("CASE: A2\n");
            temp_wrp[0] = ((last_app_page_wrp1 + 1) | (wrp_end << 16));  // application is partially within wrp
            //temp_wrp[1] = 0x0000007F;                         // disable wrp1b (set last < start)
          }
        }  
        // CASE B - wrp1 starts before application
        else if (WITHIN(wrp_strt, 0, first_app_page -1)) {
          if (WITHIN(wrp_end, 0, first_app_page -1)) {
            // CASE B1 - wrp1 entirely before application area 
            kprint("CASE: B1\n");
            temp_wrp[0] = (wrp1a_strt | (wrp1a_end << 16));   // no change to wrp
            temp_wrp[1] = (wrp1b_strt | (wrp1b_end << 16));   // no change to wrp
          }
          else if (WITHIN(wrp_end, 0, last_app_page_wrp1)) {
              // CASE B2 - wrp1 starts before application area but stops within application area
              kprint("CASE: B2\n");
              temp_wrp[0] = (wrp_strt | ((first_app_page - 1) << 16));  // wrp1 end to end of area before application
              //temp_wrp[1] = 0x0000007F;                               // disable wrp1b (set last < start)
            }
            else {
              // CASE B3 - wrp1 starts before application area but extends beyond application area
              kprint("CASE: B3\n");
              //           this requires trying to use wrp1b to protect the area beyond the application.
               temp_wrp[0] = (wrp_strt | ((first_app_page - 1) << 16));  // wrp1a used for area before application
               temp_wrp[1] = ((last_app_page_wrp1 + 1) | (wrp_end << 16));          // wrp1b used for area after application
            } 
        }
        else {
          // CASE C - wrp1 starts beyond the application so no changes needed 
          kprint("CASE: C\n");
          temp_wrp[0] = (wrp1a_strt | (wrp1a_end << 16));   // no change to wrp
          temp_wrp[1] = (wrp1b_strt | (wrp1b_end << 16));   // no change to wrp
        }
      }  // end wrp1
    }  // end application starts in bank 1
    
   
    sprintf(msg, "temp_wrp[0]: %08lX\n", temp_wrp[0]);
    kprint(msg);
    sprintf(msg, "temp_wrp[1]: %08lX\n", temp_wrp[1]);
    kprint(msg);
 
 // wrp2
    
    wrp_strt = MIN_2(wrp2a_strt, wrp2b_strt) + 128;  // combine wrp2a and wrp2b into one large virtual area
    wrp_end  = MAX_2(wrp2a_end, wrp2b_end) + 128;
    
    sprintf(msg, "wrp_strt 2: %2X\n", wrp_strt);
    kprint(msg);
    sprintf(msg, "wrp_end 2:  %2X\n", wrp_end);
    kprint(msg);
      
    temp_wrp[2] = 0x0000007F;         // set default as disabled (set last < start)
    temp_wrp[3] = 0x0000007F;         // set default as disabled (set last < start)

    uint8_t first_app_page_wrp2 = 0;
    if (first_app_page > 0x7f) first_app_page_wrp2 = first_app_page - 0x7f;  //limit first page to bank 2
    uint8_t last_app_page_wrp2 = last_app_page;
    if (last_app_page > 0x7f) {    // application extends into bank 2
      last_app_page_wrp2 -= 0x7f;       // limit last page to bank 2
 
      if (wrp_end >= wrp_strt) {
        // CASE D - wrp2 starts in app area
        if (WITHIN(wrp_strt, first_app_page_wrp2, last_app_page_wrp2)) {
          if ( wrp_end <= last_app_page_wrp2) {  
            // CASE D1 - wrp2 entirely within application area
            kprint("CASE: D1\n");
            //temp_wrp[2] = 0x0000007F;         // disable wrp2 (set last < start)
            //temp_wrp[3] = 0x0000007F;         // disable wrp2 (set last < start)
          }
          else {
            // CASE D2 - wrp2 extends beyond application
            kprint("CASE: D2\n");
            temp_wrp[2] = ((last_app_page_wrp2 + 1) | (wrp_end << 16));  // application is partially within wrp
            //temp_wrp[3] = 0x0000007F;                         // disable wrp2b (set last < start)
          }
        }  
        // CASE E - wrp2 starts before application
        else if (WITHIN(wrp_strt, 0, first_app_page_wrp2 -1)) {
          if (WITHIN(wrp_end, 0, first_app_page_wrp2 -1)) {
            // CASE E1 - wrp2 entirely before application area  (never executes because Bootloader_GetProtectionStatus
            //                                                  is always call before this routine)
            kprint("CASE: E1\n");
            
            temp_wrp[2] = (wrp2a_strt | (wrp2a_end << 16));   // no change to wrp
            temp_wrp[3] = (wrp2b_strt | (wrp2b_end << 16));   // no change to wrp
          }
          else if (WITHIN(wrp_end, 0, last_app_page_wrp2)) {
              // CASE E2 - wrp2 starts before application area but stops within application area
              kprint("CASE: E2\n");
              temp_wrp[2] = (wrp_strt | ((first_app_page_wrp2 - 1) << 16));  // wrp2 end to end of area before application
              //temp_wrp[3] = 0x0000007F;                               // disable wrp2b (set last < start)
            }
            else {
              // CASE E3 - wrp2 starts before application area but extends beyond application area
              kprint("CASE: E3\n");
              //           this requires trying to use wrp2b to protect the area beyond the application.
               temp_wrp[2] = (wrp_strt | ((first_app_page_wrp2 - 1) << 16));  // wrp2a used for area before application
               temp_wrp[3] = ((last_app_page_wrp2 + 1) | (wrp_end << 16));          // wrp2b used for area after application
            } 
        }
        else {
          // CASE F - wrp2 starts beyond the application so no changes needed  (never executes because Bootloader_GetProtectionStatus
          //                                                                   is always call before this routine)
          kprint("CASE: F\n");
          temp_wrp[2] = (wrp2a_strt | (wrp2a_end << 16));   // no change to wrp
          temp_wrp[3] = (wrp2b_strt | (wrp2b_end << 16));   // no change to wrp
        }
      }  // end wrp2
    }  // end application extends into bank 2
    
    sprintf(msg, "temp_wrp[2]: %08lX\n", temp_wrp[2]);
    kprint(msg);
    sprintf(msg, "temp_wrp[3]: %08lX\n", temp_wrp[3]);
    kprint(msg);
    
    Bootloader_ConfigProtection_Set(temp_wrp);   // update wrp
    
    kprint("Keep_Boot:  should never see this\n");   
    
  }  // app size
  return HAL_OK;
}



void save_WRP_state(void) {
  WRITE_Prot_Old_Flag = WRITE_Prot_Original_flag;  // flag that protection was saved
  Bootloader_ConfigProtection_Save(wrp_old);  // save the current write protection
  //sprintf(msg, "saved wrp_old[0] (wrp1a): %08lX\n", wrp_old[0]);
  //kprint(msg);                    
  //sprintf(msg, "saved wrp_old[1] (wrp1b): %08lX\n", wrp_old[1]);
  //kprint(msg);                    
  //sprintf(msg, "saved wrp_old[2] (wrp2a): %08lX\n", wrp_old[2]);
  //kprint(msg);                    
  //sprintf(msg, "saved wrp_old[3] (wrp2b): %08lX\n", wrp_old[3]);
  //kprint(msg);
}

void report_WP_ConfigProtection(void)
{
  
  if (wrp1a_end < wrp1a_strt) {
    kprint("BANK1 area A protection: none\n");
  }
  else {
    sprintf(msg,"BANK1 area A protection: %08lX", Bank1a_prot_start);
    kprint(msg);
    sprintf(msg," - %08lX\n", Bank1a_prot_end);
    kprint(msg);
  }
  
  if (wrp1b_end < wrp1b_strt) {
    kprint("BANK1 area B protection: none\n");
  }
  else {
    sprintf(msg,"BANK1 area B protection: %08lX", Bank1b_prot_start);
    kprint(msg);
    sprintf(msg," - %08lX\n", Bank1b_prot_end);
    kprint(msg);
  }
  
  if (wrp2a_end < wrp2a_strt) {
    kprint("BANK2 area A protection: none\n");
  }
  else {
    sprintf(msg,"BANK2 area A protection: %08lX", Bank2a_prot_start );
    kprint(msg);
    sprintf(msg," - %08lX\n", Bank2a_prot_end);
    kprint(msg);
  }  
  
  if (wrp2b_end < wrp2b_strt) {
    kprint("BANK2 area B protection: none\n");
  }
  else {
    sprintf(msg,"BANK2 area B protection: %08lX", Bank2b_prot_start );
    kprint(msg);
    sprintf(msg," - %08lX\n", Bank2b_prot_end);
    kprint(msg);
  }  
  sprintf(msg, "wrp1a: %08lX\n", rd32(R_FLASH_WRP1AR) );
  kprint(msg); 
  sprintf(msg, "wrp1b: %08lX\n", rd32(R_FLASH_WRP1BR) );
  kprint(msg);                   
  sprintf(msg, "wrp2a: %08lX\n", rd32(R_FLASH_WRP2AR) );
  kprint(msg);
  sprintf(msg, "wrp2b: %08lX\n", rd32(R_FLASH_WRP2BR) );
  kprint(msg);
  
  sprintf(msg, "      wrp_old[0] (wrp1a): %08lX\n", wrp_old[0]);
  kprint(msg);       
  sprintf(msg, "      wrp_old[1] (wrp1b): %08lX\n", wrp_old[1]);
  kprint(msg);       
  sprintf(msg, "      wrp_old[2] (wrp2a): %08lX\n", wrp_old[2]);
  kprint(msg);       
  sprintf(msg, "      wrp_old[3] (wrp2b): %08lX\n", wrp_old[3]);
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
  
 //sprintf(msg, "APP size: %8lX\n", appsize);
 //kprint(msg);
  
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
