/*-----------------------------------------------------------------------
/  Low level disk interface modlue include file   (C)ChaN, 2012
/-----------------------------------------------------------------------*/

#ifndef _MMC_DISKIO_DEFINED
#define _MMC_DISKIO_DEFINED

#ifdef __cplusplus
extern "C" {
#endif

#include "integer.h"
#include "diskio.h"

/*---------------------------------------*/
/* Prototypes for disk control functions */


DSTATUS mmc_disk_initialize (BYTE);
DSTATUS mmc_disk_status (BYTE);
DRESULT mmc_disk_read (BYTE, BYTE*, DWORD, UINT);
DRESULT mmc_disk_write (BYTE, const BYTE*, DWORD, UINT);
DRESULT mmc_disk_ioctl (BYTE, BYTE, void*);

#ifdef __cplusplus
}
#endif

#endif
