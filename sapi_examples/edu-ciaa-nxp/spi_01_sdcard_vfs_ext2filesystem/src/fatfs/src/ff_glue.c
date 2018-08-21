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

#include "board.h"
#include "chip.h"
#include "ff.h"
#include "diskio.h"
#include "device.h"
#include "blockDevice.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/


/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

//extern Device *fat_glue_dev;
extern Device *fat_device_association_list[10];

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Como manejo el tema de fmount? Todos los devices van a tener el mismo numero drv
   Guardar numero drive en fsinfo
   Mas adelante podria tener una lista que relaciona drvnum con  device.
   1) Pass argument as global variable
   2) Use a list to associate devnumber and device, then pass device number allocated in fsinfo
*/
/* Initialize Disk Drive */
DSTATUS disk_initialize(BYTE drv)
{
   DRESULT res;
   BlockDevice bdev;
   blockDevInfo_t blockInfo;
   blockDevState_t blockDevState;

   if (NULL == fat_device_association_list[drv]) {
      return RES_PARERR;
   }

   bdev = ooc_get_interface((Object)*(fat_device_association_list[drv]), BlockDevice);
   if(bdev == NULL)
   {
      return RES_ERROR;
   }
   if( 0 > bdev->getState((Object)*(fat_device_association_list[drv]), &blockDevState) )
   {
      return RES_ERROR;
   }
   if(BLKDEV_READY != blockDevState)
   {
      return STA_NOINIT;
   }

   return 0;
}

/* Disk Drive miscellaneous Functions */
DRESULT disk_ioctl(BYTE drv, BYTE ctrl, void *buff)
{
   DRESULT res;
   BlockDevice bdev;
   blockDevInfo_t blockInfo;
   blockDevState_t blockDevState;
   uint32_t i;

   if (NULL == fat_device_association_list[drv]) {
      return RES_PARERR;
   }

   bdev = ooc_get_interface((Object)*(fat_device_association_list[drv]), BlockDevice);
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
   if( 0 > bdev->getState((Object)*(fat_device_association_list[drv]), &blockDevState) )
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
      for(i=0; i<0xFFFFFF; i++)
      {
         if( 0 > bdev->getState((Object)*(fat_device_association_list[drv]), &blockDevState) )
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
      if( 0 > bdev->ioctl((Object)*(fat_device_association_list[drv]), IOCTL_BLOCK_GETINFO, &blockInfo) )
      {
         return RES_ERROR;
      }
      *(DWORD *) buff = blockInfo.num;
      res = RES_OK;
      break;

   case GET_SECTOR_SIZE:   /* Get R/W sector size (WORD) */
      if( 0 > bdev->ioctl((Object)*(fat_device_association_list[drv]), IOCTL_BLOCK_GETINFO, &blockInfo) )
      {
         return RES_ERROR;
      }
      *(DWORD *) buff = blockInfo.size;
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
   DRESULT res;
   BlockDevice bdev;
   blockDevInfo_t blockInfo;
   blockDevState_t blockDevState;

   if (NULL == fat_device_association_list[drv]) {
      return RES_PARERR;
   }

   bdev = ooc_get_interface((Object)*(fat_device_association_list[drv]), BlockDevice);
   if(bdev == NULL)
   {
      return RES_ERROR;
   }

   if( 0 > bdev->getState((Object)*(fat_device_association_list[drv]), &blockDevState) )
   {
      return RES_ERROR;
   }
   if( BLKDEV_READY != blockDevState)
   {
      return RES_NOTRDY;
   }

   if( 0 > bdev->ioctl((Object)*(fat_device_association_list[drv]), IOCTL_BLOCK_GETINFO, &blockInfo) )
   {
      return RES_ERROR;
   }

   if( sector*blockInfo.size != bdev->lseek( (Object)*(fat_device_association_list[drv]), sector*(blockInfo.size), SEEK_SET) )
   {
      return RES_ERROR;
   }
   if( count*(blockInfo.size) != bdev->read((Object)*(fat_device_association_list[drv]), buff, count*(blockInfo.size)) )
   {
      return RES_ERROR;
   }

   return RES_OK;
}

/* Get Disk Status */
DSTATUS disk_status(BYTE drv)
{
   BlockDevice bdev;
   blockDevState_t blockDevState;

   if (NULL == fat_device_association_list[drv]) {
      return RES_PARERR;
   }

   bdev = ooc_get_interface((Object)*(fat_device_association_list[drv]), BlockDevice);
   if(bdev == NULL)
   {
      return RES_ERROR;
   }

   if( 0 > bdev->getState((Object)*(fat_device_association_list[drv]), &blockDevState) )
   {
      return STA_NOINIT;
   }
   if( BLKDEV_READY == blockDevState)
   {
      return 0;
   }
   return STA_NOINIT;
}

/* Write Sector(s) */
DRESULT disk_write(BYTE drv, const BYTE *buff, DWORD sector, BYTE count)
{
   DRESULT res;
   BlockDevice bdev;
   blockDevInfo_t blockInfo;
   blockDevState_t blockDevState;

   if (NULL == fat_device_association_list[drv]) {
      return RES_PARERR;
   }

   bdev = ooc_get_interface((Object)*(fat_device_association_list[drv]), BlockDevice);
   if(bdev == NULL)
   {
      return RES_ERROR;
   }

   if( 0 > bdev->getState((Object)*(fat_device_association_list[drv]), &blockDevState) )
   {
      return RES_ERROR;
   }
   if( BLKDEV_READY != blockDevState)
   {
      return RES_NOTRDY;
   }

   if( 0 > bdev->ioctl((Object)*(fat_device_association_list[drv]), IOCTL_BLOCK_GETINFO, &blockInfo) )
   {
      return RES_ERROR;
   }

   if( sector*blockInfo.size != bdev->lseek( (Object)*(fat_device_association_list[drv]), sector*blockInfo.size, SEEK_SET) )
   {
      return RES_ERROR;
   }
   if( count*(blockInfo.size) != bdev->write((Object)*(fat_device_association_list[drv]), buff, count*(blockInfo.size)) )
   {
      return RES_ERROR;
   }

   return RES_OK;
}
