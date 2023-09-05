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
 * IAP FLASH command/interface setup
 *
 * All FLASH commands take the form of:
 *   iap_entry (command, output);
*/
#define IAP_LOCATION 0x1FFF1FF1
typedef void (*IAP)(unsigned int [],unsigned int[]);
IAP iap_entry;

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
    extern uint32_t _edata[];
    uint32_t edata = (uint32_t)_edata;
    extern uint32_t _sdata[];
    uint32_t sdata = (uint32_t)_sdata;
    //extern uint32_t _ebss[];
    //uint32_t ebss = (uint32_t)_ebss;
    //extern uint32_t _sbss[];
    //uint32_t sbss = (uint32_t)_sbss;
  
    #define BOOT_LOADER_END (sidata + (edata - sdata) + 8)
    
    //sprintf(msg, "\nsidata %08lX\n", sidata);
    //print(msg);
    //sprintf(msg, "\nedata  %08lX\n", edata);
    //print(msg);
    //sprintf(msg, "\nsdata  %08lX\n", sdata);
    //print(msg);
    //sprintf(msg, "\nebss   %08lX\n", ebss);
    //print(msg);
    //sprintf(msg, "\nsbss   %08lX\n", ebss);
    //print(msg);
    
    /* Clear flash flags */
 //   HAL_FLASH_Unlock();
 //   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
 //   HAL_FLASH_Lock();

    APP_first_sector = 0;
    APP_first_addr = 0;
   
    // LPC1769 4K length FLASH sectors 0-29
    
    for (uint16_t counter = 0; counter <= LAST_SECTOR; counter++) {   
      APP_first_addr = ((counter * FLASH_SECTOR_SIZE) + FLASH_BASE);
      if (BOOT_LOADER_END <= APP_first_addr) {
        APP_first_sector = counter; 
        break;
      }
    }

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
     
    return BL_OK;
}


void print_IAP_error_code(uint16_t error_code) {
  
  kprint("FLASH ERROR CODE: ");
  switch(error_code) {
    case 0  : kprint("0  - CMD_SUCCESS Command is executed successfully.\n"); break;
    case 1  : kprint("1  - INVALID_COMMAND Invalid command.\n"); break;
    case 2  : kprint("2  - SRC_ADDR_ERROR Source address is not on word boundary.\n"); break;
    case 3  : kprint("3  - DST_ADDR_ERROR Destination address is not on a correct boundary.\n"); break;
    case 4  : kprint("4  - SRC_ADDR_NOT_MAPPED Source address is not mapped in the memory map.\n"); break;
    case 5  : kprint("5  - DST_ADDR_NOT_MAPPED Destination address is not mapped in the memory map.\n"); break;
    case 6  : kprint("6  - COUNT_ERROR Byte count is not multiple of 4 or is not a permitted value.\n"); break;
    case 7  : kprint("7  - INVALID_SECTOR Sector number is invalid or end sector number is greater than start sector number.\n"); break;
    case 8  : kprint("8  - SECTOR_NOT_BLANK Sector is not blank.\n"); break;
    case 9  : kprint("9  - SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION.\n"); break;
    case 10 : kprint("10 - COMPARE_ERROR Source and destination data not equal.\n"); break;
    case 11 : kprint("11 - BUSY Flash programming hardware interface is busy.\n"); break;
    case 12 : kprint("12 - PARAM_ERROR Insufficient number of parameters or invalid parameter.\n"); break;
    case 13 : kprint("13 - ADDR_ERROR Address is not on word boundary.\n"); break;
    case 14 : kprint("14 - ADDR_NOT_MAPPED Address is not mapped in the memory map.\n"); break;
    case 15 : kprint("15 - CMD_LOCKED Command is locked.\n"); break;
    case 16 : kprint("16 - INVALID_CODE Unlock code is invalid.\n"); break;
    case 17 : kprint("17 - INVALID_BAUD_RATE Invalid baud rate setting.\n"); break;
    case 18 : kprint("18 - INVALID_STOP_BIT Invalid stop bit setting.\n"); break;
    case 19 : kprint("19 - CODE_READ_PROTECTION_ENABLED.\n"); break;
    default : kprint("%2d - UNKNOWN ERROR CODE.\n", error_code); break;
  }
   
}
  

void FLASH_UNLOCK(void) {
  __disable_irq();
  iap_entry=(IAP)IAP_LOCATION;
	unsigned int command[5]={0,0,0,0,0};
	unsigned int result[5]={0,0,0,0,0};
	command[0]= 50;
	command[1]= APP_first_sector;
	command[2]= LAST_SECTOR;
	iap_entry(command,result);
  __enable_irq();
  if(result[0]) print_IAP_error_code(result[0]);
  
  
}

/**
 * @brief  This function erases the user application area in flash
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: upon success
 * @retval BL_ERR: upon failure
 */
uint8_t Bootloader_Erase(void)
{
  FLASH_UNLOCK();
  __disable_irq();
  iap_entry=(IAP)IAP_LOCATION;
  unsigned int command[5]={0,0,0,0,0};
	unsigned int result[5]={0,0,0,0,0};
  command[0] = 52; // command: Erase sector(s)
  command[1] = APP_first_sector; 
  command[2] = LAST_SECTOR;
  command[3] = 120000; // CPU Clock Frequency (CCLK) in kHz.
  command[4] = 0;
  
  iap_entry(command, result);  // execute IAP FLASH command
  __enable_irq();
  if(result[0]) print_IAP_error_code(result[0]);

  return (result[0] ? BL_OK : BL_ERASE_ERROR);
}

/**
 * @brief  this function erases the flash and sets
 *         the data pointer to the start of application flash area.
 * @return Bootloader error code ::eBootloaderErrorCodes
*/
uint8_t Bootloader_FlashBegin(void)
{
  /* Reset flash destination address */
  flash_ptr = APP_ADDRESS;
  return Bootloader_Erase();
}

/**
 * @brief  Program a max 4096 byte chunck to flash: This function writes a 
 *         data chunk into the flash and increments the data pointer.
 * @see    README for futher information
 * @param  data: max 4096 byte data chunk to be written into flash
 * @param  flash_ptr: pointer to start of FLASH programming area.  512 byte aligned earlier
 *         so don't need to worry about 256 byte alignment requirement
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: upon success
 * @retval BL_WRITE_ERROR: upon failure
 */
uint8_t Bootloader_FlashNext(uint8_t *data, uint16_t length)
{
  
  uint16_t padding = length % data_length;  //  need to padd buffer if ran out of data on SD card
  if (padding) for (uint16_t i = padding; i < data_length; i++) data[i] = 0xFF;
  
  
  FLASH_UNLOCK();

  
  __disable_irq();
  iap_entry=(IAP)IAP_LOCATION;
	unsigned int command[5]={0,0,0,0,0};
	unsigned int result[5]={0,0,0,0,0};
  command[0] = 51; // command:Copy RAM to Flash
  command[1] = flash_ptr;   // destination pointer
  command[2] = (uint32_t)data;  // source pointer
  command[3] = data_length; // # bytes to program
  command[4] = 120000;      // CPU Clock Frequency (CCLK) in kHz.
  
  iap_entry(command, result);  // execute IAP FLASH command
  
  __enable_irq(); 
  if(result[0]) {
    print_IAP_error_code(result[0]);;
    return BL_WRITE_ERROR;
  }

  /* Check the written values */
  for (uint16_t i = 0; i <data_length; i++) {

    uint8_t read_data = *(uint8_t*)(flash_ptr + i); 
    if(read_data != data[i])
    {
        /* Flash content doesn't match source content */
        kprint("Programming error\n");
        sprintf(msg, "  expected data (8 bit): %02X\n", data[i]);
        kprint(msg);
        sprintf(msg, "  actual data (8 bit)  : %02X\n", read_data);
        kprint(msg);
        sprintf(msg, "  absolute address (byte): %08lX\n", (flash_ptr + i));
        kprint(msg);
                 
        return BL_WRITE_ERROR;
    }
  }
  flash_ptr += data_length;
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
    return BL_OK;
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
 * @brief  This function checks whether the new application fits into flash.
 * @param  appsize: size of application
 * @return Bootloader error code ::eBootloaderErrorCodes
 * @retval BL_OK: if application fits into flash
 * @retval BL_SIZE_ERROR: if application does not fit into flash
 */
uint8_t Bootloader_CheckSize(uint32_t appsize)
{
 //   return ((FLASH_BASE + FLASH_SIZE - APP_ADDRESS) >= appsize) ? BL_OK
   return BL_OK ;
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
  \brief   System Reset
  \details Initiates a system reset request to reset the MCU.
 */
void NVIC_System_Reset(void)
{
    
  *SCB_AIRCR = (uint32_t)(((SCB_AIRCR_VECTKEY << SCB_AIRCR_VECTKEY_Pos) | (1 << SCB_AIRCR_SYSRESETREQ_Pos)));
  
  for(;;)                                                           /* wait until reset */
  {
  }
}