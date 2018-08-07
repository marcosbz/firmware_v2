/*
 * @brief Glue functions to merge Chan FATFS with CIAA Firmware
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "fsusb_cfg.h"
#include "board.h"
#include "chip.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Disk Status */
static volatile DSTATUS Stat = STA_NOINIT;

/* 100Hz decrement timer stopped at zero (disk_timerproc()) */
static volatile WORD Timer2;

static DISK_HANDLE_T *hDisk;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

extern Device *fat_glue_dev;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Initialize Disk Drive */
DSTATUS disk_initialize(BYTE drv)
{
   if (drv) {
      return STA_NOINIT;            /* Supports only single drive */
   }
   /*   if (Stat & STA_NODISK) return Stat;   *//* No card in the socket */

   if (Stat != STA_NOINIT) {
      return Stat;               /* card is already enumerated */

   }

   Stat &= ~STA_NOINIT;
   return Stat;

}

/* Disk Drive miscellaneous Functions */
DRESULT disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
   DRESULT res;
   BlockDevice bdev;
   blockDevInfo_t blockInfo;

   if (NULL == fat_glue_dev) {
      return RES_PARERR;
   }

   bdev = ooc_get_interface((Object)*fat_glue_dev, BlockDevice);
   if(bdev == NULL)
   {
      return RES_ERROR;
   }

   /*
   typedef enum
   {
      BLKDEV_UNINIT,
      BLKDEV_STOP,
      BLKDEV_ACTIVE,
      BLKDEV_CONNECTING,
      BLKDEV_DISCONNECTING,
      BLKDEV_READY,
      BLKDEV_READING,
      BLKDEV_WRITING,
      BLKDEV_SYNCING
   } blockDevState_t;
   */
   if( 0 > bdev->getState((Object)*fat_glue_dev, &blockDevState) );
   {
      return RES_ERROR;
   }
   if(BLKDEV_UNINIT == blockDevState)
   {
      return RES_NOTRDY;
   }

   res = RES_ERROR;

   switch (ctrl) {
   case CTRL_SYNC:   /* Make sure that no pending write process */
      if (FSUSB_DiskReadyWait(hDisk, 50)) {
         res = RES_OK;
      }
      for(i=0; i<0xFFFFFF; i++)
      {
         if( 0 > bdev->getState((Object)*fat_glue_dev, &blockDevState) );
         {
            return RES_ERROR;
         }
         if( BLKDEV_READY == blockDevState)
         {
            res = RES_OK;
            break;
         }
      }
      break;

   case GET_SECTOR_COUNT:   /* Get number of sectors on the disk (DWORD) */
      if( 0 > bdev->ioctl((Object)*fat_glue_dev, IOCTL_BLOCK_GETINFO, &blockInfo) )
      {
         return RES_ERROR;
      }
      *(DWORD *) buff = blockInfo.blockInfo->num;
      res = RES_OK;
      break;

   case GET_SECTOR_SIZE:   /* Get R/W sector size (WORD) */
      if( 0 > bdev->ioctl((Object)*fat_glue_dev, IOCTL_BLOCK_GETINFO, &blockInfo) )
      {
         return RES_ERROR;
      }
      *(DWORD *) buff = blockInfo.blockInfo->size;
      res = RES_OK;
      break;

   case GET_BLOCK_SIZE:/* Get erase block size in unit of sector (DWORD) */
      *(DWORD *) buff = 1;
      res = RES_OK;
      break;

   default:
      res = RES_PARERR;
      break;
   }

   return res;
}

/* Read Sector(s) */
DRESULT disk_read(BYTE drv, BYTE *buff, DWORD sector, BYTE count)
{
   if (drv || !count) {
      return RES_PARERR;
   }
   if (Stat & STA_NOINIT) {
      return RES_NOTRDY;
   }

   if (FSUSB_DiskReadSectors(hDisk, buff, sector, count)) {
      return RES_OK;
   }

   return RES_ERROR;

   ssize_t ret;
   BlockDevice bdev = ooc_get_interface((Object)*fat_glue_dev, BlockDevice);

   if(!bdev)
   {
      return -1;
   }

   ret = bdev->lseek(((Object)*fat_glue_dev, offset, SEEK_SET);
   if(ret!=offset)
   {
      return -1;
   }
   ret = bdev->read((Object)*fat_glue_dev, buf, nbyte);
   if(ret!=nbyte)
   {
      return -1;
   }

   return 0;
}

/* Get Disk Status */
DSTATUS disk_status(BYTE drv)
{
   if (drv) {
      return STA_NOINIT;   /* Supports only single drive */
   }
   return Stat;
}

/* Write Sector(s) */
DRESULT disk_write(BYTE drv, const BYTE *buff, DWORD sector, BYTE count)
{

   if (drv || !count) {
      return RES_PARERR;
   }
   if (Stat & STA_NOINIT) {
      return RES_NOTRDY;
   }

   if (FSUSB_DiskWriteSectors(hDisk, (void *) buff, sector, count)) {
      return RES_OK;
   }

   return RES_ERROR;
}
