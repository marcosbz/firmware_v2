/* Copyright 2014, ACSE & CADIEEL
 *    ACSE   : http://www.sase.com.ar/asociacion-civil-sistemas-embebidos/ciaa/
 *    CADIEEL: http://www.cadieel.org.ar
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef FAT_H
#define FAT_H

/** \brief FAT driver header file
 **
 ** This is the header file of the fat file system implementation
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup MTests CIAA Firmware Module Tests
 ** @{ */
/** \addtogroup Filesystem EXT2
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * MZ         Marcos Ziegler
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160101 v0.0.1 MZ Initial version
 */

/*==================[inclusions]=============================================*/

#include <stdint.h>
#include "vfs.h"

/*==================[macros]=================================================*/

/** \brief Maximum quantity of simultaneous mounts */
#define FAT_MAX_MOUNTS 10
/** \brief Size of array fat_buffer defined in fat.c. Its used to read and write portions of clusters */
#define FAT_BUFFER_SIZE 1024
#define FAT_NAME_BUFFER_SIZE 256
/** \brief fat_buffer */
#define FAT_PATH_MAX 50

#define   FAT_MAXNAMELEN   255

/* Bitmap handle macros */
/* Set the bit i from array */
#define setbit(array,i)   ((array)[(i)>>3] |= 1<<((i)&0x07))
/* Clear the bit i from array */
#define clrbit(array,i)   ((array)[(i)>>3] &= ~(1<<((i)&0x07)))

/* Macros for counting and rounding. */
/* Count how many y bytes size items are needed to hold x bytes */
#define howmany(x, y) (((x)+((y)-1))/(y))
/* Roundup x to the nearest y multiple */
#define roundup(x, y)  ((((x)+((y)-1))/(y))*(y))
/* Rounddown x to the nearest y multiple */
#define rounddown(x,y) (((x)/(y))*(y))

/* Misc macros */
/* Check if power of 2 */
#define is_powerof2(x) (( (x) != 0 ) && !( (x) & ( (x) - 1)))



/*==================[typedef]================================================*/

/** \brief fat file system
 **
 ** fat file system information
 **
 **/
typedef struct fat_fs_info
{
   uint32_t   s_block_size;         /* Block size in bytes. */
   uint32_t   s_inodes_per_block;   /* Number of inodes per block */
   uint32_t   s_itb_per_group;      /* Number of inode table blocks per group */
   uint32_t   s_ginfodb_count;      /* Number of group descriptor blocks */
   uint32_t   s_desc_per_block;     /* Number of group descriptors per block */
   uint32_t   s_groups_count;       /* Number of groups in the fs */
   uint8_t    sectors_in_block;     /* Sector is 512 bytes long */
   uint16_t   s_buff_size;          /* Size of the block chunks to be read in buffer */
   uint8_t    s_buff_per_block;     /* How much chunks per block */
} fat_fs_info_t;

/** \brief fat file info
 **
 ** fat file information
 **/
typedef struct fat_file_info
{
   FIL fatfs_fp;
   FILINFO fno;
   uint32_t            f_size;
} fat_file_info_t;

typedef struct fat_format_param
{
   uint32_t partition_size;
   uint16_t block_size; /* Valid block_size values are 1024, 2048 and 4096 bytes per block */
   uint8_t block_node_factor; /* blocks/nodes. Default is 4 */
} fat_format_param_t;
/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

#endif /* FAT_H */
