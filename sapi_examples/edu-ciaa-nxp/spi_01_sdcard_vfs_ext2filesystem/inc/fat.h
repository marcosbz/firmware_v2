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
/** \addtogroup Filesystem FAT
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
#define FAT_MAX_MOUNTS 32
/** \brief Size of array ext2_block_buffer defined in ext2.c. Its used to read and write portions of blocks */
#define FAT_BLOCK_BUFFER_SIZE 1024
/** \brief fat_block_buffer */
#define FAT_PATH_MAX 50


/** \brief Special inode numbers */
//#define   EXT2_RESERVED_INODES   10
//#define   EXT2_BADBLKINO         1
//...

#define   FAT_MAXNAMELEN   255

/** \brief fat file types */
//#define EXT2_FT_UNKNOWN         0   /* Unknown File Type */
//...

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

/** \brief ext2 superblock
 **
 ** Superblock of the ext2 file system
 **
 ** TODO: Skip suffix padding to economize memory
 **/
typedef struct ext2sb
{                                      /* offset : description */
   uint32_t   s_inodes_count;          /* 0: Inodes count */
   uint32_t   s_blocks_count;          /* 4: Blocks count */
   uint32_t   s_r_blocks_count;        /* 8: Reserved blocks count */
   uint32_t   s_free_blocks_count;     /* 12: Free blocks count */
   uint32_t   s_free_inodes_count;     /* 16: Free inodes count */
   uint32_t   s_first_data_block;      /* 20: First Data Block */
   uint32_t   s_log_block_size;        /* 24: Block size */
   uint32_t   s_log_frag_size;         /* 28: Fragment size */
   uint32_t   s_blocks_per_group;      /* 32: # Blocks per group */
   uint32_t   s_frags_per_group;       /* 36: # Fragments per group */
   uint32_t   s_inodes_per_group;      /* 40: # Inodes per group */
   uint32_t   s_mtime;                 /* 44: Mount time */
   uint32_t   s_wtime;                 /* 48: Write time */
   uint16_t   s_mnt_count;             /* 52: Mount count */
   uint16_t   s_max_mnt_count;         /* 54: Maximal mount count */
   uint16_t   s_magic;                 /* 56: Number that identifies the system as ext2. Fixed as 0xEF53 */
   uint16_t   s_state;                 /* 58: File system state. Set to EXT2_ERROR_FS when mounted.
                                        * Set to EXT2_VALID_FS when correctly unmounted.
                                        */
   uint16_t   s_errors;                /* 60: What should the fs do when error detected
                                        * EXT2_ERRORS_CONTINUE: No special action
                                        * EXT2_ERRORS_RO: Remount read-only
                                        * EXT2_ERRORS_PANIC: Kernel panic
                                        */
   uint16_t   s_minor_rev_level;       /* 62: minor revision level */
   uint32_t   s_lastcheck;             /* 64: Unix time of the last file system check */
   uint32_t   s_checkinterval;         /* 68: Maximum Unix interval allowed between checks */
   uint32_t   s_creator_os;            /* 72: Identifier of the OS that created the fs
                                        * EXT2_OS_LINUX
                                        * EXT2_OS_HURD
                                        * EXT2_OS_MASIX
                                        * EXT2_OS_FREEBSD
                                        * EXT2_OS_LITES
                                        */
   uint32_t   s_rev_level;             /* 76: Revision level
                                        * EXT2_GOOD_OLD_REV: Old basic version
                                        * EXT2_DYNAMIC_REV: variable node size, extended attributes, etc.
                                        */
   uint16_t   s_def_resuid;            /* 80: Default user id for reserved blocks */
   uint16_t   s_def_resgid;            /* 82: Default group id for reserved blocks */
   uint32_t   s_first_ino;             /* 84: First non-reserved inode */
   uint16_t   s_inode_size;            /* 88: size of inode structure. 128 in revision 0. */
   uint16_t   s_block_group_nr;        /* 90: Number of block group containing this superblock */
   uint32_t   s_feature_compat;        /* 92: compatible features set */
   uint32_t   s_feature_incompat;      /* 96: incompatible feature set */
   uint32_t   s_feature_ro_compat;     /* 100: readonly-compatible feature set */
   uint8_t    s_uuid[16];              /* 104: 128-bit uuid for volume */
   char       s_volume_name[16];       /* 120: volume name string */
   //uint8_t    s_last_mounted[64];      /* 136: path of directory where last mounted */
   //uint32_t   s_algo_bitmap;           /* 200: For data compression. Unused feature in this implementation */
   //uint8_t    s_prealloc_blocks;       /* 204: # of blocks to preallocate when creating regular file */
   //uint8_t    s_prealloc_dir_blocks;   /* 205: # of blocks to preallocate when creating directory */
   //uint8_t    s_alignment[2];          /* 206: 4 byte alignment */
   //uint8_t    padding[816];            /* 208: Not used fields from here on. Padding */
} ext2_superblock_t;

/** \brief ext2 group descriptor
 **
 ** ext2 file system block group descriptor
 **
 **/
typedef struct ext2_gd
{
    uint32_t  block_bitmap;        /* Blocks bitmap block */
    uint32_t  inode_bitmap;        /* Inodes bitmap block */
    uint32_t  inode_table;         /* Inodes table block */
    uint16_t  free_blocks_count;   /* Free blocks count */
    uint16_t  free_inodes_count;   /* Free inodes count */
    uint16_t  used_dirs_count;     /* Directories count */
    uint16_t  pad;
    uint8_t  reserved[12];
} ext2_gd_t;

/** \brief ext2 directory information
 **
 ** ext2 file system file entry in directory
 **
 **/
typedef struct ext2_direntry
{
   uint32_t inode;                   /* Inode number of entry */
   uint16_t rec_len;                 /* Length of this record */
   uint8_t name_len;                 /* Length of string in d_name */
   uint8_t file_type;                /* File type */
   char    name[EXT2_MAXNAMELEN];   /* Name with length<=EXT2_MAXNAMELEN */
} ext2_direntry_t;

/** \brief ext2 node structure
 **
 ** ext2 node structure
 **
 **/
typedef struct ext2_inode
{                              /* offset : description */
   uint16_t   i_mode;          /* 0: Format of file and access rights */
   uint16_t   i_uid;           /* 2: User id */
   uint32_t   i_size;          /* 4: Size in bytes */
   uint32_t   i_atime;         /* 8: Unix time of last acces */
   uint32_t   i_ctime;         /* 12: Unix time of creation */
   uint32_t   i_mtime;         /* 16: Unix time of last modification */
   uint32_t   i_dtime;         /* 20: Unix time of last deletion */
   uint16_t   i_gid;           /* 24: id of group which has access to this file */
   uint16_t   i_links_count;   /* 26: Counts how many times this inode is referred to */
   uint32_t   i_blocks;        /* 28: 512-byte data block count, independently of whether they are used or not */
   uint32_t   i_flags;         /* 32: How should the OS behave when accessing the data of this node */
   uint32_t   i_osd1;          /* 36: OS dependent value */
   uint32_t   i_block[N_DIRECT_BLOCKS + N_INDIRECT_BLOCKS];   /* 40: disk blocks */
   uint32_t   i_gen;           /* 100: generation number. Used in NFS */
   uint32_t   i_file_acl;      /* 104: file ACL (not implemented). Always 0 */
   uint32_t   i_dir_acl;       /* 108: dir ACL (not implemented). Always 0 */
   uint32_t   i_faddr;         /* 112: fragment address. Obsolete, always 0 */
   uint32_t   i_osd2[3];       /* 116: OS dependent value */
} ext2_inode_t;

/** \brief fat file system
 **
 ** fat file system information
 **
 **/

//typedef struct ext2_fs_info
//{
//   ext2_superblock_t e2sb;
//   uint32_t   s_block_size;         /* Block size in bytes. */
//   uint32_t   s_inodes_per_block;   /* Number of inodes per block */
//   uint32_t   s_itb_per_group;      /* Number of inode table blocks per group */
//   uint32_t   s_ginfodb_count;      /* Number of group descriptor blocks */
//   uint32_t   s_desc_per_block;     /* Number of group descriptors per block */
//   uint32_t   s_groups_count;       /* Number of groups in the fs */
//   uint8_t    sectors_in_block;     /* Sector is 512 bytes long */
//   uint16_t   s_buff_size;          /* Size of the block chunks to be read in buffer */
//   uint8_t    s_buff_per_block;     /* How much chunks per block */
//} ext2_fs_info_t;

typedef struct fat_fs_info
{
  off_t    fs_hwsectorsize;        /* HW: Sector size reported by block driver*/
  off_t    fs_hwnsectors;          /* HW: The number of sectors reported by the hardware */
  off_t    fs_fatbase;             /* Logical block of start of filesystem (past resd sectors) */
  off_t    fs_rootbase;            /* MBR: Cluster no. of 1st cluster of root dir */
  off_t    fs_database;            /* Logical block of start data sectors */
  off_t    fs_fsinfo;              /* MBR: Sector number of FSINFO sector */
  off_t    fs_currentsector;       /* The sector number buffered in fs_buffer */
  uint32_t fs_nclusters;           /* Maximum number of data clusters */
  uint32_t fs_nfatsects;           /* MBR: Count of sectors occupied by one fat */
  uint32_t fs_fattotsec;           /* MBR: Total count of sectors on the volume */
  uint32_t fs_fsifreecount;        /* FSI: Last free cluster count on volume */
  uint32_t fs_fsinextfree;         /* FSI: Cluster number of 1st free cluster */
  uint16_t fs_fatresvdseccount;    /* MBR: The total number of reserved sectors */
  uint16_t fs_rootentcnt;          /* MBR: Count of 32-bit root directory entries */
  bool     fs_mounted;             /* true: The file system is ready */
  bool     fs_dirty;               /* true: fs_buffer is dirty */
  bool     fs_fsidirty;            /* true: FSINFO sector must be written to disk */
  uint8_t  fs_type;                /* FSTYPE_FAT12, FSTYPE_FAT16, or FSTYPE_FAT32 */
  uint8_t  fs_fatnumfats;          /* MBR: Number of FATs (probably 2) */
  uint8_t  fs_fatsecperclus;       /* MBR: Sectors per allocation unit: 2**n, n=0..7 */
  uint8_t *fs_buffer;              /* This is an allocated buffer to hold one sector
                                    * from the device */
} fat_fs_info_t;;

/** \brief ext2 file info
 **
 ** ext2 file information
 ** TODO: Is it necessary to keep f_pointer? f_group (can be calculated from f_inumber)? f_size (already in upper layer)?
 **/
//typedef struct ext2_file_info
//{
//   //ext2_inode_t        *pinode;        /* Copy of on-disk inode */
//   //uint16_t            f_group;     /* Block group number in which the node resides */
//   uint32_t            f_pointer;   /* Local seek pointer */
//   uint32_t            f_inumber;   /* Inode number */
//   uint32_t            f_size;
//} ext2_file_info_t;

typedef struct fat_file_info
{
  struct fat_file_s *ff_next;      /* Retained in a singly linked list */
  uint8_t  ff_bflags;              /* The file buffer flags */
  uint8_t  ff_oflags;              /* Flags provided when file was opened */
  uint8_t  ff_sectorsincluster;    /* Sectors remaining in cluster */
  uint16_t ff_dirindex;            /* Index into ff_dirsector to directory entry */
  uint32_t ff_currentcluster;      /* Current cluster being accessed */
  off_t    ff_dirsector;           /* Sector containing the directory entry */
  off_t    ff_size;                /* Size of the file in bytes */
  off_t    ff_startcluster;        /* Start cluster of file on media */
  off_t    ff_currentsector;       /* Current sector being operated on */
  off_t    ff_cachesector;         /* Current sector in the file buffer */
  uint8_t *ff_buffer;              /* File buffer (for partial sector accesses) */
} fat_file_info_t;


//typedef struct ext2_format_param
//{
//   uint32_t partition_size;
//   uint16_t block_size; /* Valid block_size values are 1024, 2048 and 4096 bytes per block */
//   uint8_t block_node_factor; /* blocks/nodes. Default is 4 */
//} ext2_format_param_t;
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

#endif /* EXT2_H */
