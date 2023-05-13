/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.h
  * @brief   This file contains the common defines and functions prototypes for
  *          the user_diskio driver.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_DISKIO_H
#define __USER_DISKIO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* USER CODE BEGIN 0 */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#include "integer.h"
#include <intdefs.h>
#include <stdint.h>
#include <stddef.h>
#include "ffconf.h"


/* Status of Disk Functions */
typedef BYTE	DSTATUS;

/* Results of Disk Functions */
typedef enum {
	RES_OK = 0,		/* 0: Successful */
	RES_ERROR,		/* 1: R/W Error */
	RES_WRPRT,		/* 2: Write Protected */
	RES_NOTRDY,		/* 3: Not Ready */
	RES_PARERR		/* 4: Invalid Parameter */
} DRESULT;

/**
  * @brief  Disk IO Driver structure definition
  */
typedef struct
{
  DSTATUS (*disk_initialize) (BYTE);                     /*!< Initialize Disk Drive                     */
  DSTATUS (*disk_status)     (BYTE);                     /*!< Get Disk Status                           */
  DRESULT (*disk_read)       (BYTE, BYTE*, DWORD, UINT);       /*!< Read Sector(s)                            */
  DRESULT (*disk_write)      (BYTE, const BYTE*, DWORD, UINT); /*!< Write Sector(s) when _USE_WRITE = 0       */
  DRESULT (*disk_ioctl)      (BYTE, BYTE, void*);              /*!< I/O control operation when _USE_IOCTL = 1 */
}Diskio_drvTypeDef;

/**
  * @brief  Global Disk IO Drivers structure definition
  */
typedef struct
{
  uint8_t                 is_initialized[FF_VOLUMES];
  const Diskio_drvTypeDef *drv[FF_VOLUMES];
  uint8_t                 lun[FF_VOLUMES];
  volatile uint8_t        nbr;

}Disk_drvTypeDef;


/*---------------------------------------*/
/* Prototypes for disk control functions */

extern Diskio_drvTypeDef  USER_Driver;

DSTATUS USER_SPI_initialize (BYTE pdrv);
DSTATUS USER_SPI_status (BYTE pdrv);
DRESULT USER_SPI_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT USER_SPI_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
DRESULT USER_SPI_ioctl (BYTE pdrv, BYTE cmd, void *buff);

DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);

#define disk_initialize  USER_initialize
#define disk_status      USER_status
#define disk_read        USER_read
#define disk_write       USER_write
#define disk_ioctl       USER_ioctl

DSTATUS disk_initialize (BYTE pdrv);
DSTATUS disk_status (BYTE pdrv);
DRESULT disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void* buff);

/* Disk Status Bits (DSTATUS) */

#define STA_NOINIT		0x01	/* Drive not initialized */
#define STA_NODISK		0x02	/* No medium in the drive */
#define STA_PROTECT		0x04	/* Write protected */

/* Command code for disk_ioctrl fucntion */

/* Generic command (Used by FatFs) */
#define CTRL_SYNC			0	/* Complete pending write process (needed at _FS_READONLY == 0) */
#define GET_SECTOR_COUNT	1	/* Get media size (needed at _USE_MKFS == 1) */
#define GET_SECTOR_SIZE		2	/* Get sector size (needed at _MAX_SS != _MIN_SS) */
#define GET_BLOCK_SIZE		3	/* Get erase block size (needed at _USE_MKFS == 1) */
#define CTRL_TRIM			4	/* Inform device that the data on the block of sectors is no longer used (needed at _USE_TRIM == 1) */

/* Generic command (Not used by FatFs) */
#define CTRL_POWER			5	/* Get/Set power status */
#define CTRL_LOCK			6	/* Lock/Unlock media removal */
#define CTRL_EJECT			7	/* Eject media */
#define CTRL_FORMAT			8	/* Create physical format on the media */

/* MMC/SDC specific ioctl command */
#define MMC_GET_TYPE		10	/* Get card type */
#define MMC_GET_CSD			11	/* Get CSD */
#define MMC_GET_CID			12	/* Get CID */
#define MMC_GET_OCR			13	/* Get OCR */
#define MMC_GET_SDSTAT		14	/* Get SD status */
#define ISDIO_READ			55	/* Read data form SD iSDIO register */
#define ISDIO_WRITE			56	/* Write data to SD iSDIO register */
#define ISDIO_MRITE			57	/* Masked write data to SD iSDIO register */

/* ATA/CF specific ioctl command */
#define ATA_GET_REV			20	/* Get F/W revision */
#define ATA_GET_MODEL		21	/* Get model name */
#define ATA_GET_SN			22	/* Get serial number */

/* USER CODE END 0 */

#ifdef __cplusplus
}
#endif

#endif /* __USER_DISKIO_H */
