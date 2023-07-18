/**
 *******************************************************************************
 * STM32 Bootloader Header
 *******************************************************************************
 * @author Akos Pasztor
 * @file   bootloader.h
 * @brief  This file contains the bootloader configuration parameters,
 *	       function prototypes and other required macros and definitions.
 *
 * @see    Please refer to README for detailed information.
 *******************************************************************************
 * @copyright (c) 2020 Akos Pasztor.                    https://akospasztor.com
 *******************************************************************************
 */

#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H
#include "ff.h"
#include "intdefs.h"
#include <stm32g4xx_hal_def.h>


/** Bootloader Configuration
 * @defgroup Bootloader_Configuration Bootloader Configuration
 * @{
 */


/** Check application checksum on startup */
#define USE_CHECKSUM 0

/** Enable write protection after performing in-app-programming */
#define USE_WRITE_PROTECTION 0

/** Ignore write protection of application area (clear protection and write to FLASH) */
#define IGNORE_WRITE_PROTECTION 1

/** Restore write protection after performing in-app-programming */
#define RESTORE_WRITE_PROTECTION 1
/** Automatically set vector table location before launching application */
#define SET_VECTOR_TABLE 1

/** Clear reset flags
 *  - If enabled: bootloader clears reset flags. (This occurs only when OBL RST
 * flag is active.)
 *  - If disabled: bootloader does not clear reset flags, not even when OBL RST
 * is active.
 */
#define CLEAR_RESET_FLAGS 1

/** Start address of application space in flash */
#define APP_ADDRESS (uint32_t)0x08008000
//#define APP_ADDRESS (uint32_t)0x0800A000
//#define APP_ADDRESS (uint32_t)0x08020000  // first available for EXFAT
/** End address of application space (address of last byte) */
#define END_ADDRESS (uint32_t)0x080FFFFB

/** Start address of application checksum in flash */
#define CRC_ADDRESS (uint32_t)0x080FFFFC

/** Address of System Memory (ST Bootloader) */
#define SYSMEM_ADDRESS (uint32_t)0x1FFF0000
/** @} */
/* End of configuration ------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/


/* Defines -------------------------------------------------------------------*/
/** Size of application in DWORD (32bits or 4bytes) */
#define APP_SIZE (uint32_t)(((END_ADDRESS - APP_ADDRESS) + 3) / 4)

/** Number of sectors per bank in flash */
//uint32_t APP_first_sector;  // first FLASH sector an application can be loaded into
//uint32_t APP_first_addr;    // beginning address of first FLASH sector an application can be loaded into
//uint32_t APP_sector_mask;   // mask used to determine if any application sectors are write protected
#define APP_OFFSET (APP_ADDRESS - FLASH_BASE)  // how far from start of FLASH the APP starts
//#define FLASH_SIZE            ((uint32_t)0x100000)  // 1024K bytes
#define FLASH_SIZE            ((uint32_t)0x80000)  // 512K bytes 
//#define FLASH_SIZE            ((uint32_t)0x40000)  // 256K bytes
//#define LAST_SECTOR           11  // 1024K bytes STM32F407 has FLASH sectors 0-11
#define LAST_SECTOR            7  // 512K bytes STM32F407VE has FLASH sectors 0-7
#define FLASH_SECTOR_NBPERBANK  (1)
#define FLASH_SECTOR_SIZE       ((uint32_t)0x800)  // 2K bytes
//#define FLASH_SECTOR_SIZE       ((uint32_t)0x4000)  // 16K bytes
//#define FLASH_BASE            ((uint32_t)0x08000000) // FLASH(up to 1 MB) base address in the alias region
//#define SRAM1_BASE            ((uint32_t)0x20000000)  // SRAM1 base address in the alias region (provided by G474 platform)
//#define SRAM1_SIZE_MAX        ((uint32_t)0x1BFFF)     // SRAM1 length (112 KB)
//#define SRAM1_SIZE_MAX        ((uint32_t)0xFFFF)     // SRAM1 length (64 KB)
//#define SRAM1_SIZE_MAX        ((uint32_t)0x1FFFF)     // SRAM1 length (128 KB)
//#define SRAM1_SIZE_MAX        ((uint32_t)0x23FFF)     // SRAM1 length (144 KB)
//#define SRAM1_SIZE_MAX        ((uint32_t)0x13FFF)     // SRAM1 length (80 KB)  (provided by G474 platform)
//#define SRAM2_BASE            ((uint32_t)0x2001C000) // SRAM2(16 KB) base address in the alias region
//#define SRAM2_SIZE_MAX        ((uint32_t)0x03FFF)
//#define PERIPH_BASE           ((uint32_t)0x40000000) // Peripheral base address in the alias region


#define FLASH_FLAG_ALL_SR_ERRORS (FLASH_SR_OPTVERR  | FLASH_SR_RDERR   | FLASH_SR_PROGERR |  \
                                  FLASH_SR_WRPERR   | FLASH_SR_PGAERR  | FLASH_SR_SIZERR  |  \
                                  FLASH_SR_MISERR   | FLASH_SR_OPERR   | FLASH_SR_PGSERR  |  \
                                  FLASH_SR_FASTERR)
    
                         

/* MCU RAM information (to check whether flash contains valid application) */
//#define RAM_BASE SRAM1_BASE     /*!< Start address of RAM */  //defined by STM32G474 platform
#ifdef SRAM2_SIZE_MAX
  #define RAM_SIZE (SRAM1_SIZE_MAX + SRAM2_SIZE_MAX + 2) /*!< RAM size in bytes */
#else
  #define RAM_SIZE SRAM1_SIZE_MAX +1 /*!< RAM size in bytes */
#endif

/* Enumerations --------------------------------------------------------------*/
/** Bootloader error codes */
enum eBootloaderErrorCodes
{
    BL_OK = 0,      /*!< No error */
    BL_NO_APP,      /*!< No application found in flash */
    BL_SIZE_ERROR,  /*!< New application is too large for flash */
    BL_CHKS_ERROR,  /*!< Application checksum error */
    BL_ERASE_ERROR, /*!< Flash erase error */
    BL_WRITE_ERROR, /*!< Flash write error */
    BL_OBP_ERROR    /*!< Flash option bytes programming error */
};

/* Functions -----------------------------------------------------------------*/
HAL_StatusTypeDef  HAL_FLASH_Unlock(void);
HAL_StatusTypeDef  HAL_FLASH_Lock(void);
HAL_StatusTypeDef  HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef  HAL_FLASH_OB_Lock(void);
HAL_StatusTypeDef  HAL_FLASH_OB_Launch(void);
#define FLASH_BANK_1              0x00000001U              /*!< Bank 1   */
#define FLASH_BANK_2              0x00000002U              /*!< Bank 2   */
#define FLASH_TIMEOUT_VALUE             1000U   /* 1 s  */
#define __HAL_RCC_GET_FLAG(__FLAG__) (((((((__FLAG__) >> 5U) == 1U) ? RCC->CR :                     \
                                        ((((__FLAG__) >> 5U) == 4U) ? RCC->CRRCR :                  \
                                        ((((__FLAG__) >> 5U) == 2U) ? RCC->BDCR :                   \
                                        ((((__FLAG__) >> 5U) == 3U) ? RCC->CSR : RCC->CIFR)))) &    \
                                          ((uint32_t)1U << ((__FLAG__) & RCC_FLAG_MASK))) != 0U) \
                                            ? 1U : 0U)

uint8_t Bootloader_Init(void);
uint8_t Bootloader_Erase(void);

uint8_t Bootloader_FlashBegin(void);
uint8_t Bootloader_FlashNext_Buf(uint8_t *data, UINT count);
uint8_t Bootloader_FlashNext(uint64_t data);
uint8_t Bootloader_FlashEnd(void);

uint8_t Bootloader_ConfigProtection_Keep_Boot(void);
uint8_t Bootloader_ConfigProtection_Set(uint32_t *data);
uint8_t Bootloader_GetProtectionStatus(void);
uint8_t Bootloader_CheckSize(uint32_t appsize);
uint8_t Bootloader_VerifyChecksum(void);
uint8_t Bootloader_CheckForApplication(void);
void Bootloader_JumpToApplication(void);
void Bootloader_JumpToSysMem(void);

uint32_t Bootloader_GetVersion(void);

extern uint32_t Magic_Location;
#define Magic_BootLoader 0xB00720AD   //  semi random pattern to flag that next
                                      //  reset should load the bootloader code
#define Magic_Application 0xB0B1B0B2  //  semi random pattern to flag that next
                                      //  reset should load the APPLICATION code
extern char msg[64];
extern uint32_t WRITE_protection;  
extern uint32_t APP_sector_mask;   // mask used to determine which sectors have the bootloader in them
#define WRITE_PROTECT_DEFAULT 0xFFFFFFFF  //  Which sectors to affect when enabling/disabling write protection                                
extern uint32_t WRITE_Prot_Old_Flag;             // flag if protection was removed (in case need to restore write protection)
#define WRITE_Prot_Original_flag 0xB0B3B0B4
#define WRITE_Prot_Old_Flag_Restored_flag 0xB0B5B0B6   // flag if protection was restored (to break an endless loop))
extern uint32_t Write_Prot_Old;
extern uint32_t app_size;
extern uint32_t wrp_old[2];


void save_WRP_state(void);        // save current WRP state and set WRITE_Prot_Old_Flag to WRITE_Prot_Original_flag
// WRITE_Prot_Old_Flag has three states:
//   WRITE_Prot_Original_flag: original WRP state has been saved
//   WRITE_Prot_Old_Flag_Restored_flag: original WRP state has been restored
//   anything else: original WRP state has not been saved

#define WP_DONT_SAVE 0
#define WP_SAVE 1  


#endif /* __BOOTLOADER_H */
