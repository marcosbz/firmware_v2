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

/** \brief DOS 3.31 BIOS Parameter Block structure (FAT12/16) 25 bytes
 **
 ** 
 **/
struct bpb {
	uint8_t bytepersec_l;	/* logical bytes per sector LSB (0x00) */
	uint8_t bytepersec_h;	/* logical bytes per sector MSB (0x02) */
	uint8_t	secperclus;		/* logical sectors per cluster (1,2,4,8,16,32,64,128 are valid) */
	uint8_t reserved_l;		/* logical reserved sectors LSB */
	uint8_t reserved_h;		/* logical reserved sectors MSB */
	uint8_t numfats;		/* number of FATs */
	uint8_t rootentries_l;	/* root dir entries LSB */
	uint8_t rootentries_h;	/* root dir entries MSB */
	uint8_t sectors_s_l;	/* total logical sectors LSB */
	uint8_t sectors_s_h;	/* total logical sectors MSB */
	uint8_t mediatype;	/* media descriptor */
	uint8_t secperfat_l;	/* logical sectors per FAT LSB */
	uint8_t secperfat_h;	/* logical sectors per FAT MSB */
	uint8_t secpertrk_l;	/* physical sectors per track LSB */
	uint8_t secpertrk_h;	/* physical sectors per track MSB */
	uint8_t heads_l;		/* number of heads LSB */
	uint8_t heads_h;		/* number of heads MSB */
	uint8_t hidden_0;		/* hidden sectors low byte */
	uint8_t hidden_1;		/* (note - this is the number of MEDIA sectors before */
	uint8_t hidden_2;		/* first sector of VOLUME - we rely on the MBR instead) */
	uint8_t hidden_3;		/* hidden sectors high byte */
	uint8_t sectors_l_0;	/* large total sectors low byte */
	uint8_t sectors_l_1;	/* */
	uint8_t sectors_l_2;	/* */
	uint8_t sectors_l_3;	/* large total sectors high byte */
};

/** \brief format of DOS 3.34 and OS/2 1.0-1.1 extended BPB (FAT12/16/16B) 32 bytes
 **
 ** 
 **/
struct ebpb {
	uint8_t unit;			/* int 13h drive# */
	uint8_t head;			/* archaic, used by Windows NT-class OSes for flags */
	uint8_t signature;		/* 0x28 or 0x29 */
	uint8_t serial_0;		/* serial# */
	uint8_t serial_1;		/* serial# */
	uint8_t serial_2;		/* serial# */
	uint8_t serial_3;		/* serial# */
	uint8_t label[11];		/* volume label */
	uint8_t system[8];		/* filesystem ID */
};

/** \brief format of full DOS 7.1 extended BIOS parameter block for FAT32 79 bytes
 **
 ** 
 **/
struct ebpb32 {
	uint8_t fatsize_0;		/* logical sectors per FAT LSB */
	uint8_t fatsize_1;		/* */
	uint8_t fatsize_2;		/* */
	uint8_t fatsize_3;		/* logical sectors per FAT MSB */
	uint8_t extflags_l;		/* extended flags LSB */
	uint8_t extflags_h;		/* extended flags MSB */
	uint8_t fsver_l;		/* filesystem version LSB */
	uint8_t fsver_h;		/* filesystem version MSB */
	uint8_t root_0;			/* Root directory cluster, LSB */
	uint8_t root_1;			/* */
	uint8_t root_2;			/* */
	uint8_t root_3;			/* Root directory cluster, LSB */
	uint8_t fsinfo_l;		/* sector pointer to FSINFO within reserved area, low byte (2) */
	uint8_t fsinfo_h;		/* sector pointer to FSINFO within reserved area, high byte (0) */
	uint8_t bkboot_l;		/* sector pointer to backup boot sector within reserved area, low byte (6) */
	uint8_t bkboot_h;		/* sector pointer to backup boot sector within reserved area, high byte (0) */
	uint8_t reserved[12];	/* reserved, should be 0 */
	uint8_t unit;			/* int 13h drive# */
	uint8_t head;			/* archaic, used by Windows NT-class OSes for flags */
	uint8_t signature;		/* 0x28 or 0x29 */
	uint8_t serial_0;		/* serial# */
	uint8_t serial_1;		/* serial# */
	uint8_t serial_2;		/* serial# */
	uint8_t serial_3;		/* serial# */
	uint8_t label[11];		/* volume label */
	uint8_t system[8];		/* filesystem ID */
};

/** \brief logical boot record
 **
 ** 
 **/
typedef struct lbr {
	uint8_t jump[3];		/* JMP instruction */
	uint8_t oemid[8];		/* OEM ID, space-padded */
	struct bpb bpb;				/* BIOS Parameter Block */
	union {
		struct ebpb ebpb;		/* FAT12/16 Extended BIOS Parameter Block */
		struct ebpb32 ebpb32;	/* FAT32 Extended BIOS Parameter Block */
	} ebpb;
	uint8_t code[420];		/* boot sector code */
	uint8_t sig_55;			/* 0x55 signature byte */
	uint8_t sig_aa;			/* 0xaa signature byte */
} fat_bootsector_t;

/** \brief Partition table entry
 **
 ** 
 **/
struct pt_info {
	uint8_t	active; /* Bootable flag (partition "active"). 0x80 if partition active */
	uint8_t	start_h;		/* Starting CHS head */
	uint8_t	start_cs_l;		/* Starting CHS sector/cylinder (LSB) */
	uint8_t	start_cs_h;		/* Starting CHS sector/cylinder (MSB) */
	uint8_t	type;			/* Partition type */
	uint8_t	end_h;			/* Ending CHS head */
	uint8_t	end_cs_l;		/* Ending CHS sector/cylinder (LSB) */
	uint8_t	end_cs_h;		/* Ending CHS sector/cylinder (MSB) */
	uint8_t	start_0;		/* starting sector offset (LSB) */
	uint8_t	start_1;		/* */
	uint8_t	start_2;		/* */
	uint8_t	start_3;		/* starting sector offset (MSB) */
	uint8_t	size_0;			/* Length of partition in sectors (LSB) */
	uint8_t	size_1;			/* */
	uint8_t	size_2;			/* */
	uint8_t	size_3;			/* Length of partition in sectors (MSB) */
};

/*
 *	Master Boot Record structure
 */
typedef struct mbr {
	uint8_t bootcode[0x1be];	/* boot sector */
	struct pt_info ptable[4];		/* four partition table structures */
	uint8_t sig_55;				/* 0x55 signature byte */
	uint8_t sig_aa;				/* 0xaa signature byte */
} fat_masterbootrecord_t;
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

typedef struct fat_direntry {
	uint8_t name[11];		/* filename */
	uint8_t attr;			/* attributes (see ATTR_* constant definitions) */
	uint8_t reserved;		/* reserved, must be 0 */
	uint8_t crttimetenth;	/* create time, 10ths of a second (0-199 are valid) */
	uint8_t crttime_l;		/* creation time low byte */
	uint8_t crttime_h;		/* creation time high byte */
	uint8_t crtdate_l;		/* creation date low byte */
	uint8_t crtdate_h;		/* creation date high byte */
	uint8_t lstaccdate_l;	/* last access date low byte */
	uint8_t lstaccdate_h;	/* last access date high byte */
	uint8_t startclus_h_l;	/* high word of first cluster, low byte (FAT32) */
	uint8_t startclus_h_h;	/* high word of first cluster, high byte (FAT32) */
	uint8_t wrttime_l;		/* last write time low byte */
	uint8_t wrttime_h;		/* last write time high byte */
	uint8_t wrtdate_l;		/* last write date low byte */
	uint8_t wrtdate_h;		/* last write date high byte */
	uint8_t startclus_l_l;	/* low word of first cluster, low byte */
	uint8_t startclus_l_h;	/* low word of first cluster, high byte */
	uint8_t filesize_0;		/* file size, low byte */
	uint8_t filesize_1;		/* */
	uint8_t filesize_2;		/* */
	uint8_t filesize_3;		/* file size, high byte */
} fat_direntry_t;

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

//struct fat_fs_info {
//	struct volinfo vi;
//	struct block_dev *bdev;
//	struct node *root;
//};

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
} fat_fs_info_t;

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
   uint32_t diroffset;         /* byte offset of dir entry in disk */
   uint32_t firstcluster;     /* first cluster of file */
   uint32_t cluster;          /* current cluster */
   uint16_t cluster_size;
   uint8_t log_cluster_size;
   uint32_t f_size;           /* Byte size of file */
} fat_file_info_t;

//typedef struct fat_file_info
//{
//  uint8_t  ff_bflags;              /* The file buffer flags */
//  uint8_t  ff_oflags;              /* Flags provided when file was opened */
//  uint8_t  ff_sectorsincluster;    /* Sectors remaining in cluster */
//  uint16_t ff_dirindex;            /* Index into ff_dirsector to directory entry */
//  uint32_t ff_currentcluster;      /* Current cluster being accessed */
//  off_t    ff_dirsector;           /* Sector containing the directory entry */
//  off_t    ff_size;                /* Size of the file in bytes */
//  off_t    ff_startcluster;        /* Start cluster of file on media */
//  off_t    ff_currentsector;       /* Current sector being operated on */
//  off_t    ff_cachesector;         /* Current sector in the file buffer */
//  uint8_t *ff_buffer;              /* File buffer (for partial sector accesses) */
//} fat_file_info_t;


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
