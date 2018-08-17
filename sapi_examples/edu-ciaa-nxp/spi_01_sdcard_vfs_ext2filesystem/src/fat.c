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
uint8_t fat_path_buffer[FAT_PATH_BUFFER_SIZE];

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
   int ret = -1;
   fat_file_info_t *finfo;

   finfo = file->node->f_info.down_layer_info;

   /* Call Chan FatFS */
   if( FR_OK == f_lseek (finfo->fatfs_fp, file->cursor) )
   {
      ret = 0;
   }
   else /* Error in f_seek */
   {

   }


   return ret;
}

static int fat_file_close(file_desc_t *file)
{
   return 0;
}

/* TODO: Ver el caso de EOF */
static size_t fat_file_read(file_desc_t *file, void *buf, size_t size)
{
   fat_file_info_t *finfo;
   fat_fs_info_t *fsinfo;
   Device dev;
   uint32_t read_size;
   int ret;

   fsinfo = file->node->fs_info->down_layer_info;
   finfo = file->node->f_info.down_layer_info;
   dev = file->node->fs_info->device;

   if(file->cursor > finfo->f_size)
      file->cursor = finfo->f_size;
   /*Truncate read size*/
   if(file->cursor + size > finfo->f_size)
      size = finfo->f_size - file->cursor;

   /* Call Chan FatFS */
   if( FR_OK == f_lseek (finfo->fatfs_fp, file->cursor) )
   {
      if( FR_OK == f_read(finfo->fatfs_fp, buf, size, &read_size) )
      {
         file->cursor += read_size;
      }
      else /* Error in f_read */
      {

      }
   }
   else /* Error in f_seek */
   {

   }
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
   int ret = -1;
   Device dev;
   BlockDevice bdev;
   blockDevInfo_t blockInfo;
   fat_format_param_t format_parameters;
   uint8_t fat_mkfs_workingbuffer[512];

   uint16_t i, aux,group_index;

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
      format_parameters.partition_size = 0;
   }
   else
   {
     format_parameters.partition_size = ((fat_format_param_t *)param)->partition_size;
   }

   dev = dest_node->fs_info->device;
   /* Asocio dev a devnum 0, que es especial */
   if( 0 <= fat_fatfs_associate_dev(&dev, 0) )
   {
      /* TODO: armar fat_path_buffer para mount. Utilizar devnum 0. */
      if( FR_OK == f_mkfs("0:", FM_ANY, format_parameters.partition_size, fat_mkfs_workingbuffer, 512) )
      {
         fat_fatfs_desassociate_dev(0);
         ret = 0;
      }
   }

   return ret;
}

#if 0
FRESULT f_mkfs (
  const TCHAR* path,  /* [IN] Logical drive number */
  BYTE  opt,          /* [IN] Format options */
  DWORD au,           /* [IN] Size of the allocation unit */
  void* work,         /* [-]  Working buffer */
  UINT len            /* [IN] Size of working buffer */
);
#endif

/* Set root info in mountpoint */
/* Preconditions: dest_node is allocated in vfs */
static int fat_mount(filesystem_info_t *fs, vnode_t *dest_node)
{
   int ret = -1;
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

   dev = dest_node->fs_info->device;
   if( 0 <= fat_fatfs_associate_dev(&dev, &(fsinfo->fatfs_devnum)) )
   {
      /* TODO: armar fat_path_buffer para mount. Utilizar devnum. 0:path, 1:path, etc. */
      fat_path_buffer[0] = fsinfo->fatfs_devnum + '0';
      fat_path_buffer[1] = ':'; fat_path_buffer[2] = '\0';
      if( FR_OK == f_mount(fsinfo->fatfs_devnum, fat_path_buffer, fsinfo->fatfs_mounthandle) )
      {
         if( FR_OK == scan_files(dest_node) )
         {
            ret = 0
         }
         else /* Error in scan_files() */
         {
         }
      }
      else /* Error in f_mount() */
      {
         fsinfo->fatfs_mounthandle = NULL;
         tlsf_free(fs_mem_handle, (void *)dest_node->fs_info->down_layer_info);
         tlsf_free(fs_mem_handle, (void *)dest_node->f_info.down_layer_info);
      }
   }
   else /* Failed to associate device with devnumber */
   {

   }

   return ret;
}

#if 0
int fat_fatfs_associate_dev(Device *dev, uint8_t devnum)
{
   int ret = -1;
   uint8_t i;

   if(devnum < 10)
   {
      for(i=1; i<10; i++) /* devnum 0 es especial para format */
      {
         if(NULL == fat_device_association_list[i]) /* Found free devnum */
         {
            fat_device_association_list[i] = dev;
         }
      }
      if(10 > i) /* a devnum was available */
      {
         ret = 0;
      }
   }
   return ret;
}
#endif

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
   fat_file_info_t *finfo;
   fat_fs_info_t *fsinfo;
   Device dev;
   uint32_t write_size;
   int ret;

   fsinfo = file->node->fs_info->down_layer_info;
   finfo = file->node->f_info.down_layer_info;
   dev = file->node->fs_info->device;

   if(file->cursor > finfo->f_size)
      file->cursor = finfo->f_size;

   /* Call Chan FatFS */
   if( FR_OK == f_lseek(finfo->fatfs_fp, file->cursor) )
   {
      if( FR_OK == f_write(finfo->fatfs_fp, buf, size, &read_size) )
      {
         if(file->cursor + size > finfo->f_size)
            finfo->f_size = file->cursor + size;
         file->cursor += read_size;
      }
      else /* Error in f_write */
      {

      }
   }
   else /* Error in f_seek */
   {

   }
   return read_size;
}

/* Uses fat_path_buffer[] */
FRESULT scan_files(vnode_t dir_node)
{
   FRESULT res;
   DIR dir;
   uint32_t i;
   static FILINFO fno;

   fat_file_info_t *finfo;
   fat_fs_info_t *fsinfo;

   res = f_opendir(&dir, fat_path_buffer);
   if( FR_OK == res )
   {
      while(1)
      {
         res = f_readdir(&dir, &fno);
         if(FR_OK != res || 0 == fno.fname[0]) break; /* error or EOF */
         /* fno.name includes \0. No danger, so use FS_NAME_MAX */
         child_node = vfs_create_child(dir_node, fno.name, FS_NAME_MAX, 0);
         if(NULL == child_node)
         {
            res = FR_NOT_ENOUGH_CORE;
            break;
         }
         /* Alloc and initialize the file low level information */
         child_node->f_info.down_layer_info = (fat_file_info_t *) tlsf_malloc(fs_mem_handle, sizeof(fat_file_info_t));
         if(NULL == child_node->f_info.down_layer_info || NULL == child_node->fs_info->down_layer_info)   //FIXME
         {
            res = FR_NOT_ENOUGH_CORE;
            break;
         }
         /* Only read data if this is a regular file. Dont read directory data? */
         finfo = child_node->f_info.down_layer_info;
         memcpy((void *)&(finfo->fno), (void *)&fno, sizeof(FILINFO));
         if(fno.fattrib & AM_DIR)
         {
            i = strlen(fat_path_buffer);
            if( &fat_path_buffer[FAT_PATH_BUFFER_SIZE] == strncpy(&fat_path_buffer[i],
                fno.name, FAT_PATH_BUFFER_SIZE - i) )
            {
               /* End of path reached */
               return -1;
            }
            res = scan_files(fat_path_buffer, &child_node);
            if(FR_OK != res) break;
            fat_path_buffer[i] = 0;
         }
         else
         {
            res = f_open(&(finfo->fatfs_fp), fat_path_buffer, FA_READ | FA_WRITE);
            if( FR_OK == res )
            {

            }
            else /* Open failed */
            {
               break;
            }
         }
      }
      f_closedir(&dir);
   }
   else
   {

   }
   return res;
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
