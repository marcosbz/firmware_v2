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

/** \brief Fat filesystem implementation
 **
 ** Lower fs layer. These functions are managed by the vfs, with the fsdriver_operations structure
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Template Template to start a new module
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * MZ      Marcos Ziegler
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20160101 v0.0.1 MZ initial version
 */

/*
TODO:
-Try to implement everything with static memory. Exceptions can occur
-Optimize algorithms in speed and memory usage
-Add new fields to finfo and fsinfo, as they help to avoid calculating every time a function is called
-Implement error codes. By now when an error is found, -1 is returned. No matter which error was produced
-Fix "addtogroup" description from doxygen
-Add more comments
-Optimize space by allocating minimal memory for non open files, and allocating structures only for open files.
For example, make f_di field in fat_file_info a pointer, and alloc structure only for open files.
-Economize memory. Start with physical inode buffer.
1)physical node buffer for every node. That consumes a lot of memory
2)buffer only for open files.
3)One buffer for all operations.

With the buffer, the number of reads reduces, but the number of writes remain the same in most cases.
A block cache for every thread could solve these problems. No big scalability then needed in fs implementation

Thinking in the diference between 2) and 3).
2) is more scalable than 3).
3) is the most efficient in memory

Is there a problem when multiple threads use the same file? There would be redundant reads. Again, that is solved with a block cache. I think there is no need to have an inode cache if there is a block cache.

FIXME:
-Try to call return only once for every function. Use an auxiliary variable
-Review code entirely to see if the variable types are declared according to requirements
-For example, dont use uint32_t when only uint16_t is needed.
-Put variables in order for good alignment. First uint32_t, then uint16_t, etc.
-Block alloc does not delete block contents. Set them to 0
-In fat_format(), if format_parameters.partition_size == 16K then inodes_per_block == 0 then error
*/

/*==================[inclusions]=============================================*/

#include <string.h>
#include <stdbool.h>
#include "fat.h"
#include "vfs.h"
#include "tlsf.h"

/*==================[macros and definitions]=================================*/


/*==================[internal data declaration]==============================*/
/*==================[internal functions declaration]=========================*/

/** \brief Adds an entry in the directory with given parameters
 **
 **
 **
 ** \param[in] node directory node in which to add entry
 ** \param[in] ino the new entry will have this inode number
 ** \param[in] name the new entry will have this name
 ** \param[in] namlen the new entry will have this namlen
 ** \param[in] type the new entry will have this file type
 **    FAT_FT_UNKNOWN   -Unknown File Type
 **    FAT_FT_REG_FILE  -Regular File
 **    FAT_FT_DIR       -Directory File
 **    FAT_FT_CHRDEV    -Character Device
 **    FAT_FT_BLKDEV    -Block Device
 **    FAT_FT_FIFO      -Buffer File
 **    FAT_FT_SOCK      -Socket File
 **    FAT_FT_SYMLINK   -Symbolic Link
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_dir_add_entry(vnode_t *node, uint32_t ino, char *name, uint16_t namlen, uint8_t type);

/** \brief Delete an entry in the given directory
 **
 **
 **
 ** \param[in] node directory node in which to delete entry
 ** \param[in] name name of the entry to be deleted
 ** \param[in] namlen length of the name of the entry to be deleted
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_dir_delete_entry(vnode_t *node, char *name, uint16_t namlen);

/** \brief read file data to a buffer
 **
 ** \param[in] dest_node node from which to read data
 ** \param[in] buf buffer to which copy data read
 ** \param[in] size size of the data to read, in bytes
 ** \param[out] total_read_size_p pointer to the variable indicating how many bytes were read
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_buf_read_file(vnode_t *dest_node, uint8_t *buf, uint32_t size, uint32_t *total_read_size_p);

/** \brief Read superblock data from formatted device
 **
 ** \param[in] device device from which to read the superblock
 ** \param[in] sb_p pointer to the structure containing the superblock raw data
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_get_superblock(Device dev, fat_superblock_t * sb_p);

/** \brief Write the on-memory superblock and group descriptors to disk
 **
 ** \param[in] node node whose filesystem information is to be sync with disk data
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_fsinfo_flush(vnode_t *node);

/** \brief Load all directories and files starting from root recursively to the vfs tree
 **
 ** \param[in] dir_node directory to be the root of the new mount
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_mount_load(vnode_t *dir_node);

/** \brief auxiliary function for fat_mount_load()
 **
 ** \param[in] dir_node subtree directory to mount recursively
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_mount_load_rec(vnode_t *dir_node);

static int fat_search_directory(vnode_t *node, char *name, uint16_t namlen, uint32_t *inumber_p);


/** \brief create a new FAT_S_IFREG regular file
 **
 ** \param[in] parent_node parent of the new node to be created
 ** \param[in] node node to be created in the file system
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_create_regular_file(vnode_t *parent_node, vnode_t *node);

/** \brief create a new FAT_S_IFDIR directory file
 **
 ** \param[in] parent_node parent of the new node to be created
 ** \param[in] node node to be created in the file system
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_create_directory_file(vnode_t *parent_node, vnode_t *node);

/** \brief delete the FAT_S_IFDIR directory file given
 **
 ** \param[in] parent_node parent of the node to be deleted
 ** \param[in] node node to be removed from the file system
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_delete_directory_file(vnode_t *parent_node, vnode_t *node);

/** \brief delete the FAT_S_IFREG regular file given
 **
 ** \param[in] parent_node parent of the node to be deleted
 ** \param[in] node node to be removed from the file syst
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_delete_regular_file(vnode_t *parent_node, vnode_t *node);

/** \brief open the given file
 **
 ** \param[in] file file to be opened
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_file_open(file_desc_t *file);

/** \brief close the given file
 **
 ** \param[in] file file to be closed
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_file_close(file_desc_t *file);

/** \brief read the file data
 **
 ** \param[in] file file from which to read data
 ** \param[in] buf buffer whom to fill with data
 ** \param[in] size size of data to be read in bytes
 ** \return -1 if failed, a non negative integer representing the count of
 **         read bytes if success
 **/
static size_t fat_file_read(file_desc_t *file, void *buf, size_t size);

/** \brief write data to the file
 **
 ** \param[in] file file from which to read data
 ** \param[in] buf buffer whom to fill with data
 ** \param[in] size size of data to be read in bytes
 ** \return -1 if failed, a non negative integer representing the count of
 **         read bytes if success
 **/
static size_t fat_file_write(file_desc_t *file, void *buf, size_t size);

/** \brief throw a command to the file
 **
 ** \param[in] file file to be controlled
 ** \param[in] request command to be executed to the file
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_file_ioctl(file_desc_t *file, int request);

/** \brief init the file system
 ** TODO
 ** \param[in]
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_init(void *par);

/** \brief format a device with fat format
 **
 ** \param[in] dev_node node of the device to be formatted
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_format(filesystem_info_t *fs, void *param);

/** \brief mount a disk to a directory node
 **
 ** \param[in] dev_node node of the device to be mounted
 ** \param[in] dest_node rooot directory of the new mount
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_mount(filesystem_info_t *fs, vnode_t *dest_node);

/** \brief create a new file
 **
 ** \param[in] parent_node parent of the new node
 ** \param[in] child_node node to be created in disk
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_create_node(vnode_t *parent_node, vnode_t *child_node);

/** \brief delete given file
 **
 ** \param[in] parent_node parent of the node to be deleted
 ** \param[in] child_node node to be erased in disk
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_delete_node(vnode_t *parent_node, vnode_t *child_node);

/** \brief truncate given file data
 ** TODO
 ** \param[in] node
 ** \param[in] length
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_truncate(file_desc_t *desc, size_t length);

/** \brief detach filesystem from directory. Also eliminate subtree from file tree and remove on-mem fs info
 **
 ** \param[in] node root node of the filesystem
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_umount(vnode_t *node);

/** \brief remove subtree. For every node remove associated lower layer on-memory info.
 ** \remarks recursive function
 ** \param[in] root root node of the subtree
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_umount_subtree_rec(vnode_t *root);

/** \brief Read data from an offset in a block device
 **
 ** read nbyte from the block device device in offset offset from the beginning
 **
 ** \param[in]  device  device to be read
 ** \param[in]  buf     buffer to store the read data
 ** \param[in]  offset  offset from the beginning of the device
 ** \param[in]  nbyte   count of bytes to be read
 ** \return     the count of bytes read
 **/
static ssize_t fat_device_buf_read(Device dev, uint8_t * const buf, off_t const offset,
                                 size_t const nbyte);

/** \brief Writes data to an offset of a block device
 **
 ** Writes nbyte to the file descriptor fildes from the buffer buf
 **
 ** \param[in]  device  device to be written
 ** \param[in]  buf     buffer with the data to be written
 ** \param[in]  offset  offset from the beginning of the device
 ** \param[in]  nbyte   count of bytes to be written
 ** \return     the count of bytes written
 **/
static ssize_t fat_device_buf_write(Device dev, uint8_t * const buf, off_t const offset,
                                 size_t const nbyte);

/** \brief Writes c into each of the first nbyte bytes starting at offset bytes of device
 **
 ** \param[in]  device  device to be written
 ** \param[in]  c     buffer with the data to be written
 ** \param[in]  offset  offset from the beginning of the device
 ** \param[in]  nbyte   count of bytes to be written
 ** \return -1 if an error occurs, in other case 0
 **/
static int fat_device_buf_memset(Device dev, uint8_t c, off_t const offset,
                                 size_t nbyte);

/*==================[internal data definition]===============================*/

/** \brief fat driver operations
 *
 */
fsdriver_operations_t fat_driver_operations =
{
   fat_file_open,
   fat_file_close,
   fat_file_read,
   fat_file_write,
   fat_file_ioctl,

   fat_init,
   fat_format,
   fat_mount,
   fat_create_node,
   fat_delete_node,
   fat_truncate,
   fat_umount
};

/** \brief Buffer used to read and write portions of blocks
 *
 */
uint8_t fat_buffer[FAT_BUFFER_SIZE];

/*==================[external data definition]===============================*/

/** \brief fat driver structure
 *
 * It will be declared as "extern" in vfs.c, so its visible to function vfs_get_driver()
 */
filesystem_driver_t fat_driver =
{
   "FAT",
   &fat_driver_operations
};

/*==================[internal functions definition]==========================*/

/* Objetivo: Reemplazar todos los malloc con un buffer unico de 1KB.
 * Problema: Si por ejemplo el bloque es de 2KB, los entries no estan alineados a 1KB, sino a 2KB
 * Es decir, pueden aparecer entries que ocupen el final de un bloque de 1KB y el principio del otro
 * Con un bloque de 1KB no puedo abarcar el entry entero. Tengo que buscar una solucion a eso.
 */
static int fat_search_directory(vnode_t *node, char *name, uint16_t namlen, uint32_t *inumber_p)
{
   uint32_t data_block, block_offset, file_pointer;
   uint16_t entry_pointer;
   int ret;
   uint8_t flag_entry_found = 0;
   //fat_direntry_t fat_dir_entry_buffer;

   Device dev;
   fat_file_info_t *finfo;
   fat_fs_info_t *fsinfo;

   dev = node->fs_info->device;
   finfo = (fat_file_info_t *)node->f_info.down_layer_info;
   fsinfo = (fat_fs_info_t *)node->fs_info->down_layer_info;

   if(NULL == inumber_p)
      *inumber_p = 0;
   /* size must be multiple of block size. Dont consider special cases */
   /* jump from block to block in every iteration */
   for(file_pointer = 0; file_pointer < finfo->f_size; file_pointer += fsinfo->s_block_size)
   {
      /* Map file block to disk block. Result in data_block */
      ret = fat_block_map(node, file_pointer>>(10+fsinfo->e2sb.s_log_block_size), &data_block);
      if(ret)
      {
         return -1;
      }
      /* block_offset points to the first byte address of the current block */
      block_offset = data_block<<(10+fsinfo->e2sb.s_log_block_size);
      /* entry pointer iterates sequentially over the entries until it reaches the end of the block */
      for(entry_pointer = 0; entry_pointer < fsinfo->s_block_size;
            entry_pointer += fat_dir_entry_buffer.rec_len)
      {
         /* Read entry fields, except for the name. 8 bytes in total */
         ret = fat_device_buf_read(dev, (uint8_t *)&fat_dir_entry_buffer, block_offset + entry_pointer, 8);
         if(ret)
         {
            return -1;
         }
         if (0 != fat_dir_entry_buffer.inode)
         {
            if(fat_dir_entry_buffer.name_len == namlen)
            {
               /* Read the name of the entry */
               ret = fat_device_buf_read(dev, (uint8_t *)((uint8_t *)&fat_dir_entry_buffer + 8), block_offset + entry_pointer + 8,
                                          fat_dir_entry_buffer.name_len);
               if(ret)
               {
                  return -1;
               }
               if(!strncmp(fat_dir_entry_buffer.name, name, fat_dir_entry_buffer.name_len))
               {
                  flag_entry_found = 1;
                  *inumber_p = fat_dir_entry_buffer.inode;
                  break;
               }
            }
         }
      }
      if(flag_entry_found)
      {
         /* entry_pointer keeps the address of the entry to be deleted
          * block_offset keeps the address of the block to be modified
          */
         break;
      }
   }
   if (!flag_entry_found)
   {
      return -1;
   }
   return 0;
}

static int fat_file_open(file_desc_t *file)
{
   fat_file_info_t *finfo;
   //int ret;
   finfo = file->node->f_info.down_layer_info;
   /*
   finfo->pinode = (fat_inode_t *) tlsf_malloc(fs_mem_handle, sizeof(fat_inode_t));
   if(NULL == finfo->pinode)
   {
      return -1;
   }
   ret = fat_get_inode(&(file->node->fs_info), finfo->f_inumber, finfo->pinode);
   if(0 > ret)
   {
      return -1;
   }
   */
   finfo->f_pointer = 0;
   /* reset seek pointer */

   return 0;
}

static int fat_file_close(file_desc_t *file)
{
   //fat_file_info_t *finfo;
   /*
   finfo = file->node->f_info.down_layer_info;
   tlsf_free(fs_mem_handle, (void *)finfo->pinode);
   finfo->pinode = NULL;
   */

   return 0;
}

static size_t fat_file_read(file_desc_t *file, void *buf, size_t size)
{
   fat_file_info_t *finfo;
   uint32_t read_size;
   int ret;

   /*TODO: Validate file->cursor */
   finfo = file->node->f_info.down_layer_info;
   finfo->f_pointer = file->cursor;
   ret = fat_buf_read_file(file->node, (uint8_t *)buf, size, &read_size);

   if(ret)
   {
      return 0;
   }
   file->cursor += read_size;
   return read_size;
}

static int fat_file_ioctl(file_desc_t *file, int request)
{
   return 0;
}

static int fat_init(void *par)
{
   return 0;
}

static int fat_format(filesystem_info_t *fs, void *param)
{
   int ret;
   Device dev;
   BlockDevice bdev;
   blockDevInfo_t blockInfo;
   fat_format_param_t format_parameters;

   fat_superblock_t superblock;
   fat_gd_t gd_buffer;
   fat_inode_t node;

   uint16_t i, aux,group_index;
   uint16_t ngroups, nblocks_gd, inodes_per_group, inodeblocks_per_group,
            minmetablocks_per_group,  nblocks_last_group, nblocks_group_overhead;
   uint32_t block_offset, free_blocks_count, free_inodes_count, write_offset;
   uint32_t group_offset;
   uint8_t   aux_byte;

   dev = fs->device;
   bdev = ooc_get_interface((Object)dev, BlockDevice);

   if(bdev == NULL)
   {
      return -1;
   }
   ret = bdev->ioctl((Object) dev, IOCTL_BLOCK_GETINFO, &blockInfo);

   if(ret < 0)
   {
      return -1;
   }
   if(param == NULL)
   {
      /* Default format setup: 1KB Block and whole device */
      format_parameters.block_size = 1024;
      format_parameters.partition_size = blockInfo.num * blockInfo.size / format_parameters.block_size;
      format_parameters.block_node_factor = FAT_DEFAULT_BLOCKNODE_FACTOR;
   }
   else
   {
      memcpy((void *)&format_parameters, (void *)param, sizeof(fat_format_param_t));
      if(format_parameters.block_size < FAT_MIN_BLOCKSIZE)
         format_parameters.block_size = FAT_MIN_BLOCKSIZE;
      if(format_parameters.block_size > FAT_MAX_BLOCKSIZE)
         format_parameters.block_size = FAT_MAX_BLOCKSIZE;
      if(!(is_powerof2(format_parameters.block_size)))
         return -1;
      if(!(format_parameters.block_node_factor >= 2 && format_parameters.block_node_factor <= 10))
         format_parameters.block_node_factor = FAT_DEFAULT_BLOCKNODE_FACTOR;
      /* Partition size must not exceed device size */
      if(format_parameters.partition_size * format_parameters.block_size >= blockInfo.num * blockInfo.size)
         return -1;
   }

   /* Initialize default fat superblock contents */
   memset((uint8_t *)&superblock, 0, sizeof(fat_superblock_t));

   for(i = 0, aux = 1024; aux < format_parameters.block_size; i++, aux = 1024 << i);   /* blocksize log2 */
   superblock.s_log_block_size = i;
   superblock.s_log_frag_size = superblock.s_log_block_size;
   superblock.s_first_data_block = (format_parameters.block_size > FAT_SBOFF) ? 0 : 1;  /* First Data Block */
   superblock.s_blocks_per_group = format_parameters.block_size*8;  /* Number of blocks per group */
   superblock.s_frags_per_group = superblock.s_blocks_per_group;   /* Number of fragments per group */
   superblock.s_wtime = 0;             /* Last write time */
   superblock.s_mnt_count = 0;            /* Current mount count */
   superblock.s_max_mnt_count = -1;        /* Max mount count */
   superblock.s_magic = FAT_MAGIC;                /* Magic number */
   superblock.s_state = 1;                /* File system state. FAT_VALID_FS */
   superblock.s_errors = 1;               /* Behaviour when detecting errors. FAT_ERRORS_CONTINUE */
   superblock.s_lastcheck = 0;         /* time of last check */
   superblock.s_rev_level = 0;         /* Good old rev */
   superblock.s_first_ino = 11;         /* First non-reserved inode. FAT_GOOD_OLD_FIRST_INO */
   superblock.s_inode_size = 128;           /* size of inode structure. FAT_GOOD_OLD_INODE_SIZE */
   superblock.s_block_group_nr = 0;       /* block group # of this superblock */
   superblock.s_feature_compat = 0x00;    /* compatible feature set */
   superblock.s_feature_incompat = 0x00;  /* incompatible feature set */
   superblock.s_feature_ro_compat = 0x00;  /* readonly-compatible feature set */
   superblock.s_uuid[0] = 0;   /* 128-bit uuid for volume */
   superblock.s_uuid[1] = 0;
   superblock.s_uuid[2] = 0;
   superblock.s_uuid[3] = 0;
   superblock.s_uuid[4] = 0;
   superblock.s_uuid[5] = 0;
   superblock.s_uuid[6] = 0;
   superblock.s_uuid[7] = 0;
   superblock.s_uuid[8] = 0;
   superblock.s_uuid[9] = 0;
   superblock.s_uuid[10] = 0;
   superblock.s_uuid[11] = 0;
   superblock.s_uuid[12] = 0;
   superblock.s_uuid[13] = 0;
   superblock.s_uuid[14] = 0;
   superblock.s_uuid[15] = 0;
   strcpy((char *) superblock.s_volume_name, "fat");      /* volume name */

   /* Total blocks */
   superblock.s_blocks_count = format_parameters.partition_size;
   /* (block/node) factor = 4 */
   superblock.s_inodes_count = superblock.s_blocks_count / format_parameters.block_node_factor;
   /* Reserved blocks count */
   superblock.s_r_blocks_count = 0;
   /* Number of block groups */
   ngroups = howmany(superblock.s_blocks_count - superblock.s_first_data_block,
                     superblock.s_blocks_per_group);
   /* Number of blocks reserved for the group descriptors in every group */
   nblocks_gd = howmany(sizeof(fat_gd_t)*ngroups, format_parameters.block_size);
   /* Number of nodes which fit in a single block */
   inodes_per_group = superblock.s_inodes_count / ngroups;
   /* Number of blocks reserved for physical node allocations */
   inodeblocks_per_group = howmany(superblock.s_inode_size * inodes_per_group, format_parameters.block_size);
   /* Number of total metadata blocks per group */
   minmetablocks_per_group = 1 /*Superblock*/ + nblocks_gd + 1 /*Block bitmap*/ +
                     1 /*Inode bimap*/ + inodeblocks_per_group + 1 /*At least 1 data block*/;
   if(format_parameters.partition_size <= minmetablocks_per_group)
   {
      /* Device size is not enough to hold minimum file system data */
      return -1;
   }
   /* The last group contains generally less blocks than the others. It contains the remaining blocks */
   nblocks_last_group = superblock.s_blocks_count - superblock.s_first_data_block -
                        superblock.s_blocks_per_group * (ngroups - 1);

   /* If the last group contains less blocks than the minimum allowed, eliminate the group */
   if(nblocks_last_group < minmetablocks_per_group)
   {
      superblock.s_blocks_count -= nblocks_last_group;
      ngroups--;
      nblocks_last_group = superblock.s_blocks_per_group;
      nblocks_gd = howmany(sizeof(fat_gd_t)*ngroups, format_parameters.block_size);
      inodes_per_group = superblock.s_inodes_count / ngroups;
   }

   /* The number of nodes in a group must be multiple of 8 */
   superblock.s_inodes_per_group = rounddown(inodes_per_group, 8);
   /* Total node count in disk */
   superblock.s_inodes_count = inodes_per_group * ngroups;
   /* Recalculate number of nodes which fit in a single block */
   inodeblocks_per_group = howmany(superblock.s_inode_size * inodes_per_group, format_parameters.block_size);
   /* Overhead blocks per group */
   nblocks_group_overhead = 1/*Superblock*/ + nblocks_gd + 1 /*Blockbitmap*/+ 1 /*Inodebitmap*/+
                              inodeblocks_per_group;

   free_inodes_count=free_blocks_count=0;
   for(group_index=0; group_index<ngroups; group_index++)
   {
      /*** INODE GROUPS DESCRIPTORS ***/
      block_offset = superblock.s_first_data_block + superblock.s_blocks_per_group*group_index;
      block_offset += 1 /*Superblock*/+ nblocks_gd;
      gd_buffer.block_bitmap = block_offset;
      block_offset += 1; /*Block bitmap*/
      gd_buffer.inode_bitmap = block_offset;
      block_offset += 1; /*Inode table*/
      gd_buffer.inode_table = block_offset;
      if(group_index == ngroups-1)
         gd_buffer.free_blocks_count = nblocks_last_group -
                                                      nblocks_group_overhead /*Overhead*/;
      else if(group_index == 0)
         /* Reserve a block for root entries */
         gd_buffer.free_blocks_count = superblock.s_blocks_per_group - nblocks_group_overhead - 1;
      else
         gd_buffer.free_blocks_count = superblock.s_blocks_per_group - nblocks_group_overhead /*Overhead*/;
      free_blocks_count += gd_buffer.free_blocks_count;
      if(group_index == 0)
         gd_buffer.free_inodes_count = superblock.s_inodes_per_group - FAT_RESERVED_INODES;
      else
         gd_buffer.free_inodes_count = superblock.s_inodes_per_group;

      free_inodes_count += gd_buffer.free_inodes_count;
      gd_buffer.used_dirs_count = (group_index == 0) ? 1 : 0; /* Consider the root dir */
      /* Write data to every group in the partition */
      for(i=0, group_offset = superblock.s_first_data_block*format_parameters.block_size; i<ngroups;
            i++, group_offset += superblock.s_blocks_per_group*format_parameters.block_size)
      {
         ret = fat_device_buf_write(dev, (uint8_t *)&gd_buffer,
                                       group_offset + format_parameters.block_size +
                                       group_index*sizeof(fat_gd_t), sizeof(fat_gd_t));
         if(ret)
         {
            return -1;
         }
      }

      /*** BLOCK BITMAP ***/
      if(group_index == 0)
      {
         nblocks_group_overhead += 1; /* Consider the block containing root dir entries */
      }
      block_offset = gd_buffer.block_bitmap * format_parameters.block_size;
      /* Must set non existent blocks as reserved
       * Must set metadata blocks as reserved
       */
      /* Set the first bits representing busy blocks */
      ret = fat_device_buf_memset(dev, 0xFF, block_offset, nblocks_group_overhead/8);
      if(ret < 0)
      {
         return -1;
      }
      /* Clear the last bits representing free blocks */
      ret = fat_device_buf_memset(dev, 0x00, block_offset + nblocks_group_overhead/8,
                                    format_parameters.block_size - nblocks_group_overhead/8);
      if(ret < 0)
      {
         return -1;
      }
      /* Before this, whole bytes were set or cleared. There is a byte which could not be 0x00 or 0xFF.
       * Set the bits of this byte individually.
       */
      aux_byte = 0;
      for(i=0; i<(nblocks_group_overhead%8);i++)
      {
         setbit(&aux_byte,i);
      }
      ret = fat_device_buf_write(dev, (uint8_t *)&aux_byte, block_offset + nblocks_group_overhead/8, 1);
      if(ret < 0)
      {
         return -1;
      }
      if(group_index == 0)
      {
         nblocks_group_overhead -= 1; /* Consider the block containing root dir entries */
      }


      /*** INODE BITMAP ***/

      /* The root inode is already considered in the reserved inodes */
      block_offset = gd_buffer.inode_bitmap * format_parameters.block_size;
      /* Inodes are all free at the beginning. Alloc special nodes later */
      /* Clear the first bits corresponding to free inodes */
      ret = fat_device_buf_memset(dev, 0x00, block_offset, superblock.s_inodes_per_group/8);
      if(ret < 0)
      {
         return -1;
      }
      /* Set the last bits representing non-existent inodes */
      //printf("fat_format(): INODE BITMAP: Set bits offset: %d quant: %d\n",
                        //block_offset + superblock.s_inodes_per_group/8,
                        //format_parameters.block_size - superblock.s_inodes_per_group/8);
      ret = fat_device_buf_memset(dev, 0xFF, block_offset + superblock.s_inodes_per_group/8,
                                    format_parameters.block_size - superblock.s_inodes_per_group/8);
      if(ret < 0)
      {
         return -1;
      }
      /* Before this, whole bytes were set or cleared. There is a byte which could not be 0x00 or 0xFF.
       * Set the bits of this byte individually.
       */
      aux_byte = 0xFF;
      for(i=0; i<(superblock.s_inodes_per_group%8);i++)
      {
         clrbit(&aux_byte,i);
      }
      ret = fat_device_buf_write(dev, (uint8_t *)&aux_byte, block_offset + superblock.s_inodes_per_group/8, 1);
      if(ret)
      {
         return -1;
      }
      if (group_index == 0)
      {
         /* mark reserved inodes in first group */
         ret = fat_device_buf_memset(dev, 0xFF, block_offset, FAT_RESERVED_INODES/8);
         if(ret)
         {
            return -1;
         }
         /* Before this, whole bytes were set or cleared. There is a byte which could not be 0x00 or 0xFF.
          * Set the bits of this byte individually.
          */
         aux_byte = 0;
         for(i=0; i<(FAT_RESERVED_INODES%8);i++)
         {
            setbit(&aux_byte,i);
         }
         ret = fat_device_buf_write(dev, (uint8_t *)&aux_byte, block_offset + FAT_RESERVED_INODES/8, 1);
         if(ret)
         {
            return -1;
         }
      }
   }
   /* These values change as new files are created */
   superblock.s_free_inodes_count = free_inodes_count;
   superblock.s_free_blocks_count = free_blocks_count;
   /* write on-memory superblock to all groups */
   for(group_index=0; group_index<ngroups; group_index++)
   {
      group_offset = (superblock.s_first_data_block + superblock.s_blocks_per_group) * format_parameters.block_size * group_index;
      /* If the block size is 1024, block 0 is boot block, block 1 is superblock. Blocks 1 to 8192 are group 1 blocks
       * Boot block dont count as group 1 block
       */
      /* If the block size is 2048, block 0 contains boot block and superblock.
       */
      write_offset = (group_index == 0) ? FAT_SBOFF : group_offset;
      /* Clear all superblock bytes */
      ret = fat_device_buf_memset(dev, 0, write_offset, FAT_SBSIZE);
      if(ret)
      {
         return -1;
      }
      superblock.s_block_group_nr = group_index;
      /* Copy superblock to disk */
      ret = fat_device_buf_write(dev, (uint8_t *)&superblock, write_offset, sizeof(fat_superblock_t));
      if(ret)
      {
         return -1;
      }
   }

   /* Read first group descriptor */
   ret = fat_device_buf_read(dev, (uint8_t *)&gd_buffer, (superblock.s_first_data_block + 1)*format_parameters.block_size,
                               sizeof(fat_gd_t));
   if(ret)
   {
      return -1;
   }
   /* Clean all reserved inodes */
   ret = fat_device_buf_memset(dev, 0, gd_buffer.inode_table * format_parameters.block_size,
                                 superblock.s_inode_size*FAT_RESERVED_INODES);
   if(ret)
   {
      return -1;
   }
   /* Create root directory */
   memset(&node, 0, sizeof(fat_inode_t));
   node.i_mode = 040755;
   node.i_uid = node.i_gid = 1000;
   node.i_size = format_parameters.block_size;
   node.i_atime = node.i_ctime = node.i_mtime = 0;
   node.i_dtime = 0;
   node.i_links_count = 2;
   node.i_blocks = format_parameters.block_size / 512;

   /* Reserve free block in first group for root directory data */
   /* The block was already reserved when setting block bitmap bits. Its the first block after overhead blocks */
   /* This block will be assigned to node.i_block[0] */
   node.i_block[0] = nblocks_group_overhead;

   /* Fill root inode */
   ret = fat_device_buf_write(dev, (uint8_t *)&node, gd_buffer.inode_table * format_parameters.block_size +
                               superblock.s_inode_size, sizeof(fat_inode_t)); /* Second inode in table. ROOT INODE */
   if(ret)
   {
      return -1;
   }

   /* Create root entry */

   /* Self entry */
   fat_dir_entry_buffer.inode = 2;
   fat_dir_entry_buffer.rec_len = 12;
   fat_dir_entry_buffer.name_len = 1;
   fat_dir_entry_buffer.file_type = 2;
   strcpy(fat_dir_entry_buffer.name, ".");

   ret = fat_device_buf_write(dev, (uint8_t *)&fat_dir_entry_buffer, node.i_block[0]*format_parameters.block_size, sizeof(fat_direntry_t)); /* First entry, "." */
   if(ret)
   {
      return -1;
   }

   /* Parent entry */
   fat_dir_entry_buffer.inode = 2;
   fat_dir_entry_buffer.rec_len = format_parameters.block_size - 12;
   fat_dir_entry_buffer.name_len = 2;
   fat_dir_entry_buffer.file_type = 2;
   strcpy(fat_dir_entry_buffer.name, "..");

   ret = fat_device_buf_write(dev, (uint8_t *)&fat_dir_entry_buffer, node.i_block[0]*format_parameters.block_size+12, sizeof(fat_direntry_t)); /* Second entry, ".." */
   if(ret)
   {
      return -1;
   }

   return 0;
}

/* Set root info in mountpoint */
/* Preconditions: dest_node is allocated in vfs */
static int fat_mount(filesystem_info_t *fs, vnode_t *dest_node)
{
   int ret;
   Device dev;
   fat_file_info_t *finfo;
   fat_fs_info_t *fsinfo;

   /* FIXME: This implementation needs the mount directory to be non existant.
    *  Does not overwrite directory correctly
    */
   /* TODO: Root inode is a special case? */
   finfo=dest_node->f_info.down_layer_info = (fat_file_info_t *) tlsf_malloc(fs_mem_handle, sizeof(fat_file_info_t));
   if(NULL == finfo)
   {
      return -1;
   }
   fsinfo=dest_node->fs_info->down_layer_info = (fat_fs_info_t *) tlsf_malloc(fs_mem_handle, sizeof(fat_fs_info_t));
   if(NULL == fsinfo)
   {
      return -1;
   }
   /* Load file system information to memory */
   dev = dest_node->fs_info->device;
   ret = fat_get_superblock(dev, &(fsinfo->e2sb));
   if(ret)
   {
      return -1;
   }
   /* Calculate the number of group descriptors from the superblock info */
   fsinfo->s_groups_count = (fsinfo->e2sb.s_inodes_count)/(fsinfo->e2sb.s_inodes_per_group);
   fsinfo->s_block_size = 1024 << fsinfo->e2sb.s_log_block_size;
   fsinfo->sectors_in_block = fsinfo->s_block_size >> 9;   /* s_block_size/512 */
   fsinfo->s_inodes_per_block = (fsinfo->s_block_size)/(fsinfo->e2sb.s_inode_size);
   /* Size of the block chunks to be read in buffer */
   fsinfo->s_buff_size = (FAT_BLOCK_BUFFER_SIZE < fsinfo->s_block_size) ? FAT_BLOCK_BUFFER_SIZE : fsinfo->s_block_size;
   fsinfo->s_buff_per_block = fsinfo->s_block_size / fsinfo->s_buff_size;      /* How much chunks per block */

   ret = fat_mount_load(dest_node);
   return ret;
}

static int fat_create_node(vnode_t *parent_node, vnode_t *child_node)
{
   int ret;

   /* Node creation depends on the file type. A directory needs to have 2 default directories after creation */
   if(VFS_FTDIR == child_node->f_info.type)
   {
      ret = fat_create_directory_file(parent_node, child_node);
   }
   else if(VFS_FTREG == child_node->f_info.type)
   {
      ret = fat_create_regular_file(parent_node,child_node);
   }
   else
   {
      ret = -1;
   }

   return ret;
}


static int fat_delete_node(vnode_t *parent_node, vnode_t *child_node)
{
   int ret;

   if(VFS_FTDIR == child_node->f_info.type)
   {
      ret = fat_delete_directory_file(parent_node, child_node);
   }
   else if(VFS_FTREG == child_node->f_info.type)
   {
      ret = fat_delete_regular_file(parent_node,child_node);
   }
   else
   {
      ret = -1;
   }

   return ret;
}

static int fat_truncate(file_desc_t *desc, size_t length)
{
   /* TODO */
   return 0;
}

/* FIXME: Mount and umount shouldnt eliminate mount root node. This implementation does. Review */
/* FIXME: Ignoring mounts inside another mount. Should consider this case */
/* TODO: set s_state in superblock when mounting and unmounting. */
static int fat_umount(vnode_t *root)
{
   int ret;
   vnode_t *parent;

   if(NULL == root)
   {
      return -1;
   }
   parent = root->parent_node;
   if(NULL == parent)
   {
      /* Cant delete root */
      return -1;
   }
   if(VFS_FTDIR == root->f_info.type)
   {
      /* Recursively delete all subtrees under this subroot */
      ret = fat_umount_subtree_rec(root->child_node);
      if(ret)
      {
         return -1;
      }
   }
   if(NULL != root->f_info.down_layer_info)
   {
      tlsf_free(fs_mem_handle, (void *)root->f_info.down_layer_info);
      root->f_info.down_layer_info = NULL;
   }
   if(NULL != root->fs_info->down_layer_info)
   {
      tlsf_free(fs_mem_handle, (void *)root->fs_info->down_layer_info);
      root->fs_info->down_layer_info = NULL;
   }
   /* Unlink subroot and delete it */
   ret = vfs_delete_child(root);
   if(ret)
   {
      return -1;
   }
   return 0;
}

static int fat_umount_subtree_rec(vnode_t *root)
{
   vnode_t *snode;
   int ret;

   if(NULL == root)
   {
      return -1;
   }
   snode = root->sibling_node;
   if(VFS_FTDIR == root->f_info.type)
   {
      ret = fat_umount_subtree_rec(root->child_node);
      if(ret)
      {
         return -1;
      }
   }

   if(NULL != root->f_info.down_layer_info)
   {
      tlsf_free(fs_mem_handle, (void *)root->f_info.down_layer_info);
      root->f_info.down_layer_info = NULL;
   }
   ret = vfs_node_free(root);
   if(ret)
   {
      return -1;
   }
   ret = fat_umount_subtree_rec(snode);
   if(ret)
   {
      return -1;
   }
   return 0;
}

static size_t fat_file_write(file_desc_t *desc, void *buf, size_t size)
{
   int ret;

   vnode_t *node;
   Device dev;
   fat_inode_t *pinode;
   fat_file_info_t *finfo;
   fat_fs_info_t *fsinfo;

   node = desc->node;
   dev = node->fs_info->device;
   finfo = (fat_file_info_t *)node->f_info.down_layer_info;
   fsinfo = (fat_fs_info_t *)node->fs_info->down_layer_info;

   uint32_t buf_pos, remaining_write_size, device_block;
   uint32_t write_size, write_offset;

   if(NULL == finfo || NULL == fsinfo)
   {
      return 0;
   }

   finfo->f_pointer = desc->cursor;
   buf_pos = 0;
   remaining_write_size = size;
   while (remaining_write_size > 0) {
      /* Position inside current block */
      write_offset = finfo->f_pointer % fsinfo->s_block_size;
      /* #bytes to write to current block */
      write_size = fsinfo->s_block_size - write_offset;
      /* remaining bytes to write less than block size */
      if (write_size > remaining_write_size)
         write_size = remaining_write_size;
      /* map file block to disk block */
      ret = fat_file_map_alloc(node, finfo->f_pointer >> (10+fsinfo->e2sb.s_log_block_size), &device_block);
      if(ret)
      {
         return 0;
      }
      /* write to current block */
      ret = fat_device_buf_write(dev, (uint8_t *)buf + buf_pos,
                                  (device_block<<(10+fsinfo->e2sb.s_log_block_size))+write_offset, write_size);
      if(ret)
      {
         return -1;
      }
      /* update iterators */
      finfo->f_pointer += write_size;
      buf_pos += write_size;
      remaining_write_size -= write_size;
   }
   /* all bytes written. Update pointers */
   desc->cursor = finfo->f_pointer;
   if(finfo->f_pointer > finfo->f_size)
      finfo->f_size = finfo->f_pointer;
   /* finfo->pinode->i_blocks was refreshed everytime fat_file_map_alloc() was called.
    * Write inode in its position in disk
    * flush memory info to disk
    */
   /* Refresh inode in disk */
   pinode = &fat_node_buffer;
   ret = fat_get_inode(node->fs_info, finfo->f_inumber, pinode);
   if(0 > ret)
   {
      return -1;
   }
   pinode->i_size = finfo->f_size;
   ret = fat_set_inode(node->fs_info, finfo->f_inumber, pinode);
   if(0 > ret)
   {
      return -1;
   }
   /* Write to disk the changes made to superblock and block descriptors */
   ret = fat_fsinfo_flush(node);
   if(ret)
   {
      return 0;
   }

   return buf_pos;
}



static int fat_buf_read_file(vnode_t * dest_node, uint8_t *buf, uint32_t size, uint32_t *total_read_size_p)
{
   /* FIXME: If size>finfo->f_size error or truncate? */
   int ret;
   uint32_t file_block, block_offset;
   uint32_t device_block, device_offset;
   uint16_t block_size, block_shift, block_mask;
   uint32_t total_remainder_size, block_remainder, read_size, buf_pos;
   fat_file_info_t *finfo;
   fat_fs_info_t *fsinfo;
   Device dev;

   fsinfo = dest_node->fs_info->down_layer_info;
   finfo = dest_node->f_info.down_layer_info;
   dev = dest_node->fs_info->device;

   block_size = fsinfo->s_block_size;
   block_shift = (uint16_t)fsinfo->e2sb.s_log_block_size + 10;
   block_mask = (uint32_t)block_size-1;

   file_block = finfo->f_pointer >> block_shift;
   block_offset = finfo->f_pointer & block_mask;

   if(NULL != total_read_size_p)
      *total_read_size_p = 0;

   if(finfo->f_pointer > finfo->f_size)
      finfo->f_pointer = finfo->f_size;
   /*Truncate read size*/
   if(finfo->f_pointer + size > finfo->f_size)
      size = finfo->f_size - finfo->f_pointer;

   /* Must read total_remainder_size bytes from file.
    * Cant read all in a row. It must be read block by block.
    * First the disk block must be retrieved, which matches the actual file pointer.
    * Start reading this block in position (offset%block_size).
    * Must read at most to the end of this block.
    * Copy chunk of read data to buffer.
    * Subtract read bytes from total_remainder_size. Add read bytes to fpointer.
    * Check loop condition and repeat iteration if valid
    */

   /* Init: total_remainder_size = size.
    * Condition: total_remainder_size>0
    */
   for(   total_remainder_size = size, buf_pos = 0;
      total_remainder_size>0;)
   {
      file_block = finfo->f_pointer >> block_shift;
      block_offset = finfo->f_pointer & block_mask;
      ret = fat_block_map(dest_node, file_block, &device_block);
      if(ret)
      {
         if(NULL != total_read_size_p)
            *total_read_size_p = buf_pos;
         return ret;
      }
      device_offset = (device_block << block_shift) + block_offset;

      block_remainder = block_size - block_offset;
      if(total_remainder_size > block_remainder)
         read_size = block_remainder;
      else
         read_size = total_remainder_size;

      ret = fat_device_buf_read(dev, (uint8_t *)(buf+buf_pos), device_offset, read_size);
      if(ret)
      {
         if(NULL != total_read_size_p)
            *total_read_size_p = buf_pos;
         return -1;
      }
      buf_pos += read_size;
      finfo->f_pointer += read_size;
      total_remainder_size -= read_size;
   }

   if(NULL != total_read_size_p)
      *total_read_size_p = buf_pos;
   if(buf_pos != size)
      return -1;

   return 0;
}

static int fat_mount_load(vnode_t *dir_node)
{
   int ret;

   /* FIXME: How to mount on an existing directory with previous data? How to overwrite? How to recover previous subtree? */
   /* Read root inode */
   ret = fat_read_inode(dir_node, FAT_ROOTINO);   //Estoy sobreescribiendo algo de lo que tenia antes?
   if(ret)
   {
      return -1;
   }
   ret = fat_mount_load_rec(dir_node);
   if(ret)
   {
      return -1;
   }

   return 0;
}


/* Precondition: mount root inode must be initialized */
static int fat_mount_load_rec(vnode_t *dir_node)
{
   vnode_t *child_node;
   fat_direntry_t fat_dir_entry_buffer;
   uint32_t data_block, block_offset, file_pointer;
   uint16_t entry_pointer;
   int ret, mode;

   Device dev;
   fat_file_info_t *finfo;
   fat_fs_info_t *fsinfo;

   dev = dir_node->fs_info->device;
   finfo = (fat_file_info_t *)dir_node->f_info.down_layer_info;
   fsinfo = (fat_fs_info_t *)dir_node->fs_info->down_layer_info;

   /* size must be multiple of block size. Dont consider special cases */
   /* jump from block to block in every iteration */
   for(file_pointer = 0; file_pointer < finfo->f_size; file_pointer += fsinfo->s_block_size)
   {
      /* Map file block to disk block. Result in data_block */
      ret = fat_block_map(dir_node, file_pointer>>(10+fsinfo->e2sb.s_log_block_size), &data_block);
      if(ret)
      {
         return -1;
      }
      /* block_offset points to the first byte address of the current block */
      block_offset = data_block<<(10+fsinfo->e2sb.s_log_block_size);
      /* entry pointer iterates sequentially over the entries until it reaches the end of the block */
      for(entry_pointer = 0; entry_pointer < fsinfo->s_block_size;
            entry_pointer += fat_dir_entry_buffer.rec_len)
      {
         /* Read entry fields, except for the name. 8 bytes in total */

         ret = fat_device_buf_read(dev, (uint8_t *)&fat_dir_entry_buffer, block_offset + entry_pointer, 8);
         if(ret)
         {
            return -1;
         }
         if(8 >= fat_dir_entry_buffer.rec_len)
         {
            break;
         }
         if(0 == fat_dir_entry_buffer.inode)
         {
            continue;
         }
         if(((fat_dir_entry_buffer.rec_len - 8) < fat_dir_entry_buffer.name_len) ||
            (FAT_MAXNAMELEN < fat_dir_entry_buffer.name_len))
         {
            continue; /* Directory corrupt */
         }
         /* Read the name of the entry */
         ret = fat_device_buf_read(dev, (uint8_t *)fat_dir_entry_buffer.name, block_offset + entry_pointer + 8,
                                    fat_dir_entry_buffer.name_len);
         if(ret)
         {
            return -1;
         }
         ////printf("fat_mount_load_rec():Current entry: %.*s\n", fat_dir_entry_buffer.name_len,
         //                  fat_dir_entry_buffer.name);
         if(!strncmp(fat_dir_entry_buffer.name, ".", 1) || !strncmp(fat_dir_entry_buffer.name, "..", 2))
         {
            continue;
         }
         mode=0;
         child_node = vfs_create_child(dir_node, fat_dir_entry_buffer.name, fat_dir_entry_buffer.name_len, mode);
         if(NULL == child_node)
         {
            return -1;
         }
         /* Alloc and initialize the file low level information */
         child_node->f_info.down_layer_info = (fat_file_info_t *) tlsf_malloc(fs_mem_handle, sizeof(fat_file_info_t));
         if(NULL == child_node->f_info.down_layer_info || NULL == child_node->fs_info->down_layer_info)   //FIXME
         {
            return -1;
         }
         /* Read node data from disk */
         ret = fat_read_inode(child_node, fat_dir_entry_buffer.inode);
         if(ret)
         {
            return -1;
         }
         if(FAT_FT_DIR == fat_dir_entry_buffer.file_type)
         {
            ret = fat_mount_load_rec(child_node);
            if(ret)
            {
               return -1;
            }
         }
      }
   }
   return 0;
}

static int fat_create_regular_file(vnode_t *parent_node, vnode_t *node)
{
/* 1.Alloc space for node->f_info.down_layer_info
 * 2.Reserve a bit from inode_bitmap, with current allocation policies (ex.: Same gd as parent)
 * 3.Fill f_info fileds of the new child and its parent directory
     finfo->f_num the new node inumber
     pinode->i_size
     pinode->i_links_count
     pinode->i_mode
 * 4.Alloc a data block for the new node
 * 5.Add new entry in parents dir data
 * 6.Refresh bookkeeping info in superblock and group descriptor
 */

   int ret;
   uint32_t new_inumber, aux;
   fat_inode_t *pinode=NULL;
   fat_file_info_t *finfo;

   finfo = (fat_file_info_t *) tlsf_malloc(fs_mem_handle, sizeof(fat_file_info_t));
   if(NULL == finfo)
   {
      return -1;
   }
   memset((void *)finfo, 0, sizeof(fat_file_info_t));
   node->f_info.down_layer_info = (void *)finfo;
   /* Reserve a inode bit. This function modifies bookkeeping info on memory */
   ret = fat_alloc_inode_bit(node, &new_inumber);
   if(ret)
   {
      return -1;
   }
   /* Fill node fields */
   pinode = &fat_node_buffer;
   memset((void *)pinode, 0, sizeof(fat_inode_t));
   pinode->i_mode = 0x81B6;
   pinode->i_uid = 0;
   pinode->i_atime = 0;
   pinode->i_ctime = 0;
   pinode->i_flags = 0;
   /* The new node is referred by the father. Increment link count */
   pinode->i_links_count = 1;
   /* Write inode in its disk position */
   ret = fat_set_inode(node->fs_info, new_inumber, pinode);
   if(0 > ret)
   {
      return -1;
   }
   /* Reserve a data block for the new node. This function modifies node on memory */
   ret = fat_file_map_alloc(node, 0, &aux);
   if(ret)
   {
      return -1;
   }

   /* Add new entry in parent dir data.  This function modifies node, gd & sb if new block reserved */
   ret = fat_dir_add_entry(parent_node, new_inumber, node->f_info.file_name, node->f_info.file_namlen, FAT_FT_REG_FILE);
   if(ret)
   {
      return -1;
   }

   /* Already modified parent node in disk */
   /* Write to disk the modified sb and gd */
   ret = fat_fsinfo_flush(node);
   if(ret)
   {
      return -1;
   }
   return 0;
}

static int fat_delete_regular_file(vnode_t *parent_node, vnode_t *node)
{
   int ret;
   uint32_t file_blocks; /* File blocks count */
   uint32_t fblock, device_block;
   fat_inode_t *pinode=NULL;
   fat_file_info_t *finfo, *parent_finfo;
   fat_fs_info_t *fsinfo;

   finfo = (fat_file_info_t *)node->f_info.down_layer_info;
   parent_finfo = (fat_file_info_t *)parent_node->f_info.down_layer_info;
   fsinfo = (fat_fs_info_t *)node->fs_info->down_layer_info;

   file_blocks = finfo->f_size >> (10+fsinfo->e2sb.s_log_block_size);
   /* dealloc all data block of file */
   for(fblock = 0; fblock <= file_blocks; fblock++)
   {
      ret = fat_block_map(node, fblock, &device_block);
      if(ret)
      {
         return -1;
      }
      ret = fat_dealloc_block_bit(node, device_block);
      if(ret)
      {
         return -1;
      }
   }
   /* clear inode in inode table */
   ret = fat_dir_delete_entry(parent_node, node->f_info.file_name, node->f_info.file_namlen);
   if(ret)
   {
      return -1;
   }
   /* dealloc bit in inode bitmap */
   ret = fat_dealloc_inode_bit(node); /* FIXME: Sets finfo->f_inumber to 0 */
   if(ret)
   {
      return -1;
   }
   /* Modify parent inode fields */
   pinode = &fat_node_buffer;
   ret = fat_get_inode(parent_node->fs_info, parent_finfo->f_inumber, pinode);
   if(0 > ret)
   {
      return -1;
   }
   pinode->i_links_count--;
   ret = fat_set_inode(parent_node->fs_info, parent_finfo->f_inumber, pinode);
   if(0 > ret)
   {
      return -1;
   }
   /* Erase on-disk node, which means, clear all bits in its memory space */
   memset(pinode, 0, sizeof(fat_inode_t));
   ret = fat_set_inode(node->fs_info, finfo->f_inumber, pinode); /* FIXME: invalid data */
   if(0 > ret)
   {
      return -1;
   }
   /* Write modified sb & gd */
   ret = fat_fsinfo_flush(node); /* FIXME: Causes file0 entry to get corrupted (bad type file) */
   if(ret)
   {
      return -1;
   }
   /* Free low level data from the deleted file */
   if(NULL != node->f_info.down_layer_info)
   {
      tlsf_free(fs_mem_handle, (void *)node->f_info.down_layer_info);
      node->f_info.down_layer_info = NULL;
   }
   return 0;
}

static int fat_create_directory_file(vnode_t *parent_node, vnode_t *node)
{
/* 1.Alloc space for node->f_info.down_layer_info
 * 2.Reserve a bit from inode_bitmap, with current allocation policies (ex.: Same gd as parent)
 * 3.Fill f_info fileds of the new child and its parent directory
     finfo->f_num the new node inumber
     pinode->i_size
     pinode->i_links_count
     pinode->i_mode
 * 4.Alloc a data block for the new node
 * 5.Add default entries for the new node. Refresh bookkeeping info
 * 6.Add new entry in parents dir data
 * 7.Refresh bookkeeping info in superblock and group descriptor
 */
   fat_gd_t gd;
   int ret;
   uint32_t new_inumber, aux;
   uint16_t inode_group;
   fat_inode_t *pinode=NULL;
   fat_file_info_t *finfo, *parent_finfo;
   fat_fs_info_t *fsinfo;

   finfo = (fat_file_info_t *)node->f_info.down_layer_info;
   parent_finfo = (fat_file_info_t *)parent_node->f_info.down_layer_info;

   finfo = (fat_file_info_t *) tlsf_malloc(fs_mem_handle, sizeof(fat_file_info_t));
   if(NULL == finfo)
   {
      return -1;
   }
   memset((void *)finfo, 0, sizeof(fat_file_info_t));
   node->f_info.down_layer_info = (void *)finfo;
   fsinfo = (fat_fs_info_t *) parent_node->fs_info->down_layer_info;
   ret = fat_alloc_inode_bit(node, &new_inumber);
   if(ret)
   {
      while(1);
      return -1;
   }
   /* mkdir creates dirs with 0755 access */
   /* Fill the new inode fields */
   pinode = &fat_node_buffer;
   memset((void *)pinode, 0, sizeof(fat_inode_t));
   pinode->i_mode = 0x41FD;
   pinode->i_uid = 0;
   pinode->i_atime = 0;
   pinode->i_ctime = 0;
   pinode->i_flags = 0;
   pinode->i_links_count = 2;
   ret = fat_set_inode(node->fs_info, new_inumber, pinode);
   if(0 > ret)
   {
      return -1;
   }
   ret = fat_file_map_alloc(node, 0, &aux);
   if(ret)
   {
      return -1;
   }
   /* Add the default entries: . and .. */
   ret = fat_dir_add_entry(node, new_inumber, ".", 1, FAT_FT_DIR);
   if(ret)
   {
      return -1;
   }
   ret = fat_dir_add_entry(node, parent_finfo->f_inumber, "..", 2, FAT_FT_DIR);
   if(ret)
   {
      return -1;
   }
   /* Add new entry to parent directory data */
   ret = fat_dir_add_entry(parent_node, new_inumber, node->f_info.file_name, node->f_info.file_namlen, FAT_FT_DIR);
   if(ret)
   {
      return -1;
   }

   /* Modify the parent inode fields */
   ret = fat_get_inode(node->fs_info, parent_finfo->f_inumber, pinode);
   if(0 > ret)
   {
      return -1;
   }
   pinode->i_links_count++;
   ret = fat_set_inode(node->fs_info, parent_finfo->f_inumber, pinode);
   if(0 > ret)
   {
      return -1;
   }
   /* Calculate the inode group */
   inode_group = (finfo->f_inumber-1)/fsinfo->e2sb.s_inodes_per_group;
   /* Refresh used dir count in on-memory group descriptor */
   ret = fat_get_groupdesc(parent_node->fs_info, inode_group, &gd);
   if(ret)
   {
      return -1;
   }
   gd.used_dirs_count++;
   ret = fat_set_groupdesc(parent_node->fs_info, inode_group, &gd);
   if(ret)
   {
      return -1;
   }
   //while(1);
   /* Write sb and gd changes to disk */
   ret = fat_fsinfo_flush(node);
   if(ret)
   {
      return -1;
   }
   return 0;
}

static int fat_delete_directory_file(vnode_t *parent_node, vnode_t *node)
{
   int ret;
   uint32_t file_blocks; /* File blocks count */
   uint32_t fblock, device_block;
   fat_inode_t *pinode=NULL;
   fat_file_info_t *finfo, *parent_finfo;
   fat_fs_info_t *fsinfo;

   parent_finfo = (fat_file_info_t *)parent_node->f_info.down_layer_info;
   finfo = (fat_file_info_t *)node->f_info.down_layer_info;
   fsinfo = (fat_fs_info_t *)node->fs_info->down_layer_info;

   ret = fat_dir_delete_entry(node, ".", 1);
   if(ret)
   {
      return -1;
   }
   ret = fat_dir_delete_entry(node, "..", 2);
   if(ret)
   {
      return -1;
   }
   file_blocks = finfo->f_size >> (10+fsinfo->e2sb.s_log_block_size);
   /* dealloc all data block of file */
   for(fblock = 0; fblock <= file_blocks; fblock++)
   {
      ret = fat_block_map(node, fblock, &device_block);
      if(ret)
      {
         return -1;
      }
      ret = fat_dealloc_block_bit(node, device_block);
      if(ret)
      {
         return -1;
      }
   }
   /* clear entry in directory */
   ret = fat_dir_delete_entry(parent_node, node->f_info.file_name, node->f_info.file_namlen);
   if(ret)
   {
      return -1;
   }
   /* dealloc bit in inode bitmap */
   ret = fat_dealloc_inode_bit(node);
   if(ret)
   {
      return -1;
   }

   /* Modify the parent inode fields */
   pinode = &fat_node_buffer;
   ret = fat_get_inode(node->fs_info, parent_finfo->f_inumber, pinode);
   if(0 > ret)
   {
      return -1;
   }
   pinode->i_links_count--;
   ret = fat_set_inode(node->fs_info, parent_finfo->f_inumber, pinode);
   if(0 > ret)
   {
      return -1;
   }
   /* Erase node by writing 0 in its space */
   memset(pinode, 0, sizeof(fat_inode_t));   //FIXME: Should erase 128 bits, size that appears in superblock, not struct
   ret = fat_set_inode(node->fs_info, finfo->f_inumber, pinode);
   if(0 > ret)
   {
      return -1;
   }
   /* Write sb and gd changes to disk */
   ret = fat_fsinfo_flush(node);
   if(ret)
   {
      return -1;
   }
   /* Free low level info */
   if(NULL != node->f_info.down_layer_info)
   {
      tlsf_free(fs_mem_handle, (void *)node->f_info.down_layer_info);
      node->f_info.down_layer_info = NULL;
   }
   return 0;
}

static int fat_dir_add_entry(vnode_t *node, uint32_t ino, char *name, uint16_t namlen, uint8_t type)
{
   uint32_t data_block, block_offset, file_pointer;
   uint16_t new_entry_size, entry_pointer;
   int ret;
   uint8_t flag_entry_found = 0;
   //fat_direntry_t fat_dir_entry_buffer;
   fat_direntry_t *fat_dir_entry_buffer_p;

   Device dev;
   fat_file_info_t *finfo;
   fat_fs_info_t *fsinfo;

   dev = node->fs_info->device;
   finfo = (fat_file_info_t *)node->f_info.down_layer_info;
   fsinfo = (fat_fs_info_t *)node->fs_info->down_layer_info;
   fat_dir_entry_buffer_p = &fat_dir_entry_buffer;

   new_entry_size = DENTRY_MIN_SIZE + namlen;
   /* Align required_space to 4 bytes. Directory entry rule */
   new_entry_size += (new_entry_size & 0x03) ? (4 - (new_entry_size & 0x03)) : 0;
   /* size must be multiple of block size. Dont consider special cases */
   /* jump from block to block in every iteration */
   for(file_pointer = 0; file_pointer < finfo->f_size; file_pointer += fsinfo->s_block_size)
   {
      /* Map file block to disk block. Result in data_block */
      ret = fat_block_map(node, file_pointer>>(10+fsinfo->e2sb.s_log_block_size), &data_block);
      if(ret)
      {
         return -1;
      }
      /* block_offset points to the first byte address of the current block */
      block_offset = data_block<<(10+fsinfo->e2sb.s_log_block_size);
      /* entry pointer iterates sequentially over the entries until it reaches the end of the block */
      for(entry_pointer = 0; entry_pointer < fsinfo->s_block_size; entry_pointer += fat_dir_entry_buffer.rec_len)
      {
         /* Read entry fields, except for the name. 8 bytes in total */
         ret = fat_device_buf_read(dev, (uint8_t *)&fat_dir_entry_buffer, block_offset + entry_pointer, 8);
         if(ret)
         {
            return -1;
         }
         /* Consider case for empty block */
         if(0 != fat_dir_entry_buffer.inode)
         {
            /* Check if this is a corrupt entry */
            if(8 >= fat_dir_entry_buffer.rec_len)
            {
               return -1;
            }
            if(((fat_dir_entry_buffer.rec_len - 8) < fat_dir_entry_buffer.name_len) ||
               (FAT_MAXNAMELEN < fat_dir_entry_buffer.name_len))
            {
               return -1; /* Directory corrupt */
            }
         }
         if (0 == fat_dir_entry_buffer.inode)
         {
            /* Free entry found. Check if it has space for the new entry */
            if (new_entry_size <= fat_dir_entry_buffer.rec_len)
            {
               flag_entry_found = 1;
               break;
            }
         }
         /* See if there is enough space in padding to add new entry */
         if ( new_entry_size <= (fat_dir_entry_buffer.rec_len - DENTRY_TOTAL_SIZE(fat_dir_entry_buffer_p)) )
         {
            /* Enough space to alloc new entry
             * Must shrink the previous entry before adding the new
             */
            /* Calculate the padding size, this will be the rec_len of the new entry */
            new_entry_size = fat_dir_entry_buffer.rec_len - DENTRY_TOTAL_SIZE(fat_dir_entry_buffer_p);
            /* Update current entry so that rec_len points to the start of the new entry */
            fat_dir_entry_buffer.rec_len = DENTRY_TOTAL_SIZE(fat_dir_entry_buffer_p);
            ret = fat_device_buf_write(dev, (uint8_t *)&fat_dir_entry_buffer, block_offset + entry_pointer, 8);
            /* Point to the next entry */
            entry_pointer += fat_dir_entry_buffer.rec_len;
            fat_dir_entry_buffer.rec_len = new_entry_size;
            /* Flag to tell that an entry has been found */
            flag_entry_found = 1;
            break;
         }

      }
      if(flag_entry_found)
      {
         /* entry_pointer keeps the address of the new entry
          * block_offset keeps the address of the block to be modified
          */
         break;
      }
   }
   /* No free or adequate entry found. Must extend the directory */
   if (!flag_entry_found)
   {
      /* directory is full and no room left in last block */
      ret = fat_file_map_alloc(node, (finfo->f_size)>>(10+fsinfo->e2sb.s_log_block_size), &data_block);
      if(ret)
      {
         return -1;
      }
      /* New directory size adds 1 block */
      ret = fat_get_inode(node->fs_info, finfo->f_inumber, &fat_node_buffer);
      if(ret)
      {
         return -1;
      }
      fat_node_buffer.i_size += fsinfo->s_block_size;
      finfo->f_size += fat_node_buffer.i_size;
      ret = fat_set_inode(node->fs_info, finfo->f_inumber, &fat_node_buffer);
      if(ret)
      {
         return -1;
      }
      /* block_offset points to the first byte address of the current block  */
      block_offset = data_block<<(10+fsinfo->e2sb.s_log_block_size);
      entry_pointer = 0;
      fat_dir_entry_buffer.rec_len = fsinfo->s_block_size;
   }
   /* dpointer now points to a new valid entry. block_buffer contains the modified block with the new entry */
   fat_dir_entry_buffer.inode = ino;
   fat_dir_entry_buffer.file_type = type;
   fat_dir_entry_buffer.name_len = namlen;
   /* Write the new entry to disk */
   ret = fat_device_buf_write(dev, (uint8_t *)&fat_dir_entry_buffer, block_offset + entry_pointer, 8);
   if(ret)
   {
      return -1;
   }
   /* write the entry name to disk*/
   ret = fat_device_buf_write(dev, (uint8_t *)name, block_offset + entry_pointer + 8, fat_dir_entry_buffer.name_len);
   if(ret)
   {
      return -1;
   }

  return 0;
}

static int fat_dir_delete_entry(vnode_t *node, char *name, uint16_t namlen)
{
   uint32_t data_block, block_offset, file_pointer, aux;
   uint16_t entry_pointer, prev_entry_pointer;
   int ret;
   uint8_t flag_entry_found = 0;
   //fat_direntry_t fat_dir_entry_buffer;

   Device dev;
   fat_file_info_t *finfo;
   fat_fs_info_t *fsinfo;

   dev = node->fs_info->device;
   finfo = (fat_file_info_t *)node->f_info.down_layer_info;
   fsinfo = (fat_fs_info_t *)node->fs_info->down_layer_info;

   /* size must be multiple of block size. Dont consider special cases */
   /* jump from block to block in every iteration */
   for(file_pointer = 0; file_pointer < finfo->f_size; file_pointer += fsinfo->s_block_size)
   {
      /* Map file block to disk block. Result in data_block */
      ret = fat_block_map(node, file_pointer>>(10+fsinfo->e2sb.s_log_block_size), &data_block);
      if(ret)
      {
         return -1;
      }
      /* block_offset points to the first byte address of the current block */
      block_offset = data_block<<(10+fsinfo->e2sb.s_log_block_size);
      /* entry pointer iterates sequentially over the entries until it reaches the end of the block */
      for(entry_pointer = 0, prev_entry_pointer = 0; entry_pointer < fsinfo->s_block_size;
            entry_pointer += fat_dir_entry_buffer.rec_len)
      {
         /* Read entry fields, except for the name. 8 bytes in total */
         ret = fat_device_buf_read(dev, (uint8_t *)&fat_dir_entry_buffer, block_offset + entry_pointer, 8);
         if(ret)
         {
            return -1;
         }
         if (0 != fat_dir_entry_buffer.inode)
         {
            if(fat_dir_entry_buffer.name_len == namlen)
            {
               /* Read the name of the entry */
               ret = fat_device_buf_read(dev, (uint8_t *)((uint8_t *)&fat_dir_entry_buffer + 8), block_offset + entry_pointer + 8,
                                          fat_dir_entry_buffer.name_len);
               if(ret)
               {
                  return -1;
               }
               if(!strncmp(fat_dir_entry_buffer.name, name, fat_dir_entry_buffer.name_len))
               {
                  flag_entry_found = 1;
                  break;
               }
            }
         }
         prev_entry_pointer = entry_pointer;
      }
      if(flag_entry_found)
      {
         /* entry_pointer keeps the address of the entry to be deleted
          * block_offset keeps the address of the block to be modified
          */
         break;
      }
   }
   if (flag_entry_found)
   {
      /* Requested entry found. Delete it */
      fat_dir_entry_buffer.inode = 0;
      ret = fat_device_buf_write(dev, (uint8_t *)&fat_dir_entry_buffer, block_offset + entry_pointer, 8);
      if(ret)
      {
         return -1;
      }
      /* Now we have cleared dentry, if it's not the first one,
       * merge it with previous one.  Since we assume, that
       * existing dentry must be correct, there is no way to
       * spann a data block.
       */
      if (entry_pointer)
      {
         /* entry_pointer does not point to the first entry
          * Previous entry now spans deleted entry
          * Deleted entry is now padding of previous entry
          */
         aux = fat_dir_entry_buffer.rec_len;
         ret = fat_device_buf_read(dev, (uint8_t *)&fat_dir_entry_buffer, block_offset + prev_entry_pointer, 8);
         if(ret)
         {
            return -1;
         }
         fat_dir_entry_buffer.rec_len += aux;
         ret = fat_device_buf_write(dev, (uint8_t *)&fat_dir_entry_buffer, block_offset + prev_entry_pointer, 8);
         if(ret)
         {
            return -1;
         }
      }
   }
   else
   {
      return -1;
   }
   return 0;
}

/* Writes the on-memory sb and gd to disk */
static int fat_fsinfo_flush(vnode_t *node)
{
   int ret;
   uint32_t group_offset;
   uint16_t group_index;

   Device dev;
   fat_fs_info_t *fsinfo;

   dev = node->fs_info->device;
   fsinfo = (fat_fs_info_t *)node->fs_info->down_layer_info;

   for(group_index = 0; group_index<(fsinfo->s_groups_count); group_index++)
   {
      group_offset = fsinfo->e2sb.s_blocks_per_group * fsinfo->s_block_size * group_index;
      /* If the block size is 1024, block 0 is boot block, block 1 is superblock. Blocks 1 to 8192 are group 1 blocks
       * Boot block dont count as group 1 block
       */
      if(1024 != fsinfo->s_block_size && 0 == group_index)
      {
         group_offset = FAT_SBOFF;
      }
      ret = fat_device_buf_write(dev, (uint8_t *)&(fsinfo->e2sb), group_offset, sizeof(fat_superblock_t));
      if(ret)
      {
         return -1;
      }
   }
   return 0;
}

static ssize_t fat_device_buf_read(Device dev, uint8_t * const buf, off_t const offset,
                                 size_t const nbyte)
{
   ssize_t ret;
   BlockDevice bdev = ooc_get_interface((Object) dev, BlockDevice);

   if(!bdev)
   {
      return -1;
   }

   ret = bdev->lseek((Object)dev, offset, SEEK_SET);
   if(ret!=offset)
   {
      return -1;
   }
   ret = bdev->read((Object)dev, buf, nbyte);
   if(ret!=nbyte)
   {
      return -1;
   }

   return 0;
}

static ssize_t fat_device_buf_write(Device dev, uint8_t * const buf, off_t const offset,
                                 size_t const nbyte)
{
   ssize_t ret;
   BlockDevice bdev = ooc_get_interface((Object) dev, BlockDevice);

   if(!bdev)
   {
      return -1;
   }

   ret = bdev->lseek((Object)dev, offset, SEEK_SET);
   if(ret!=offset)
   {
      return -1;
   }
   ret = bdev->write((Object)dev, buf, nbyte);
   if(ret!=nbyte)
   {
      return -1;
   }
   return 0;
}

/* Writes to fat_block_buffer */
static int fat_device_buf_memset(Device dev, uint8_t c, off_t const offset,
                                 size_t nbyte)
{
   uint32_t write_offset, write_size;
   ssize_t ret;

   memset(fat_block_buffer, c, FAT_BLOCK_BUFFER_SIZE);
   write_offset = offset;
   while(nbyte)
   {
      write_size = nbyte <= FAT_BLOCK_BUFFER_SIZE ? nbyte : FAT_BLOCK_BUFFER_SIZE;
      ret = fat_device_buf_write(dev, (uint8_t *)fat_block_buffer, write_offset, write_size);
      if(ret)
      {
         return -1;
      }
      nbyte -= write_size;
      write_offset += write_size;
   }

   return 0;
}

/*==================[external functions definition]==========================*/

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
