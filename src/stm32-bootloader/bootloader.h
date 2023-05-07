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

/** Bootloader Configuration
 * @defgroup Bootloader_Configuration Bootloader Configuration
 * @{
 */

/** Select target MCU family: please define the target MCU family type below.
 * Currently supported MCU families:
 *  - STM32L4
 *  - STM32F1
 */
//#define STM32F1

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
/* Include the appropriate header file */
//#if defined(STM32F1)
  #include "stm32f1xx.h"
//#else
//  #error "Target MCU header file is not defined or unsupported."
//#endif

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
#define SRAM1_BASE            ((uint32_t)0x20000000)  // SRAM1 base address in the alias region
//#define SRAM1_SIZE_MAX        ((uint32_t)0x1BFFF)     // SRAM1 length (112 KB)
#define SRAM1_SIZE_MAX        ((uint32_t)0xFFFF)     // SRAM1 length (64 KB)
//#define SRAM2_BASE            ((uint32_t)0x2001C000) // SRAM2(16 KB) base address in the alias region  
//#define SRAM2_SIZE_MAX        ((uint32_t)0x03FFF)
//#define PERIPH_BASE           ((uint32_t)0x40000000) // Peripheral base address in the alias region    


#if defined(FLASH_BANK2_END)
  #define FLASH_FLAG_ALL_ERRORS   (FLASH_FLAG_BSY          | FLASH_FLAG_PGERR       | \
                                   FLASH_FLAG_WRPERR       | FLASH_FLAG_EOP         | \
                                   FLASH_FLAG_BSY_BANK1    | FLASH_FLAG_PGERR_BANK1 | \
                                   FLASH_FLAG_WRPERR_BANK1 | FLASH_FLAG_EOP_BANK1   | \
                                   FLASH_FLAG_BSY_BANK2    | FLASH_FLAG_PGERR_BANK2 | \
                                   FLASH_FLAG_WRPERR_BANK2 | FLASH_FLAG_EOP_BANK2   | \
                                   FLASH_FLAG_OPTVERR)
#else  
  #define FLASH_FLAG_ALL_ERRORS   (FLASH_FLAG_BSY          | FLASH_FLAG_PGERR       | \
                                   FLASH_FLAG_WRPERR       | FLASH_FLAG_EOP         | \
                                   FLASH_FLAG_OPTVERR)       
#endif

/* MCU RAM information (to check whether flash contains valid application) */
#define RAM_BASE SRAM1_BASE     /*!< Start address of RAM */
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
uint8_t Bootloader_Init(void);
uint8_t Bootloader_Erase(void);

uint8_t Bootloader_FlashBegin(void);
uint8_t Bootloader_FlashNext_Buf(uint8_t *data, UINT count);
uint8_t Bootloader_FlashNext(uint64_t data);
uint8_t Bootloader_FlashEnd(void);

uint32_t Bootloader_GetProtectionStatus(void);
uint8_t Bootloader_ConfigProtection(uint32_t protection, uint8_t set);

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

extern uint32_t WRITE_protection;
extern uint32_t WRITE_Prot_Old_Flag;             // flag if protection was removed (in case need to restore write protection)
#define WRITE_Prot_Original_flag 0xB0B3B0B4
#define WRITE_Prot_Old_Flag_Restored_flag 0xB0B5B0B6   // flag if protection was restored (to break an endless loop))
extern uint32_t Write_Prot_Old;


#define WP_CLEAR 0
#define WP_SET 1


#endif /* __BOOTLOADER_H */
