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
#include "ff.h"

/*==================[macros and definitions]=================================*/


/*==================[internal data declaration]==============================*/
/*==================[internal functions declaration]=========================*/

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

/** \brief Bind a device driver to a free device number, to call fatfs functions
 **
 ** \param[in]  dev  device to be associated to a devnum
 ** \param[out] devnum devnum associated with given device
 ** \return -1 if an error occurs, in other case 0
 **/
int fat_fatfs_associate_dev(Device *dev, uint8_t *devnum);

/** \brief Bind a device driver to a specified device number, to call fatfs functions
 **
 ** \param[in]  dev  device to be associated to a devnum
 ** \param[out] devnum devnum associated with given device
 ** \return -1 if an error occurs, in other case 0
 **/

int fat_fatfs_associate_dev_select(Device *dev, uint8_t devnum);

/** \brief Delete bind between a device and a specified devnum
 **
 ** \param[in]  devnum  devnum to be desassociated
 ** \return -1 if an error occurs, in other case 0
 **/
int fat_fatfs_desassociate_dev(uint8_t devnum);


/** \brief Recursively add files and directories to tree
 **
 ** \param[in]  dir_node  root node of the subtree to add
 ** \return FR_OK if success
 **/
FRESULT scan_files(vnode_t *dir_node);

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

/** \brief Buffer used to write absolute paths and file names
 *
 */
char fat_path_buffer[FAT_PATH_BUFFER_SIZE];

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

/* Used in ff_glue.c */
Device fat_device_association_list[10] = {0};

/*==================[internal functions definition]==========================*/

static int fat_file_open(file_desc_t *file)
{
   int ret = -1;
   fat_file_info_t *finfo;

   finfo = file->node->f_info.down_layer_info;

   /* Call Chan FatFS */
   if( FR_OK == f_lseek (&(finfo->fatfs_fp), file->cursor) )
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

static size_t fat_file_read(file_desc_t *file, void *buf, size_t size)
{
   fat_file_info_t *finfo;
   uint32_t read_size;

   finfo = file->node->f_info.down_layer_info;

   if(file->cursor > f_size(&(finfo->fatfs_fp)) )
      file->cursor = f_size(&(finfo->fatfs_fp));
   /*Truncate read size*/
   if(file->cursor + size > f_size(&(finfo->fatfs_fp)))
      size = f_size(&(finfo->fatfs_fp)) - file->cursor;

   /* Call Chan FatFS */
   if( FR_OK == f_lseek (&(finfo->fatfs_fp), file->cursor) )
   {
      if( FR_OK == f_read(&(finfo->fatfs_fp), buf, size, (UINT*)&read_size) )
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

   dev = fs->device;
   /* Asocio dev a devnum 0, que es especial */
   if( 0 <= fat_fatfs_associate_dev_select(&dev, 0) )
   {
      /* TODO: armar fat_path_buffer para mount. Utilizar devnum 0. */
      if( FR_OK == f_mkfs("0:", FM_ANY, 0, fat_mkfs_workingbuffer, 512) )
      {
         fat_fatfs_desassociate_dev(0);
         ret = 0;
      }
   }

   return ret;
}

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
      /* TODO: assert fsinfo->fatfs_mounthandle not mounted, so not to overwrite */
      if( FR_OK == f_mount(&(fsinfo->fatfs_mounthandle), (const TCHAR*)fat_path_buffer, 1) )
      {
         if( FR_OK == scan_files(dest_node) )
         {
            ret = 0;
         }
         else /* Error in scan_files() */
         {
         }
      }
      else /* Error in f_mount() */
      {
         tlsf_free(fs_mem_handle, (void *)dest_node->fs_info->down_layer_info);
         tlsf_free(fs_mem_handle, (void *)dest_node->f_info.down_layer_info);
      }
   }
   else /* Failed to associate device with devnumber */
   {

   }

   return ret;
}

int fat_fatfs_associate_dev(Device *dev, uint8_t *devnum)
{
   int ret = -1;
   uint8_t i;

   for(i=1; i<10; i++) /* devnum 0 es especial para format */
   {
      if(NULL == fat_device_association_list[i]) /* Found free devnum */
      {
         fat_device_association_list[i] = *dev;
         break;
      }
   }
   if(10 > i) /* a devnum was available */
   {
      *devnum = i;
      ret = 0;
   }

   return ret;
}

int fat_fatfs_associate_dev_select(Device *dev, uint8_t devnum)
{
   int ret = -1;

   if(devnum < 10)
   {
      if(NULL == fat_device_association_list[devnum])
      {
         fat_device_association_list[devnum] = *dev;
         ret = 0;
      }
   }

   return ret;
}

int fat_fatfs_desassociate_dev(uint8_t devnum)
{
   int ret = -1;

   if(devnum < 10)
   {
      fat_device_association_list[devnum] = NULL;
      ret = 0;
   }

   return ret;
}

/*
if( 0 == print_path(node, fat_path_buffer, FAT_PATH_BUFFER_SIZE) )
{

}
*/

static int fat_create_node(vnode_t *parent_node, vnode_t *child_node)
{
   int ret = -1;
   fat_fs_info_t *fsinfo;
   fat_file_info_t *finfo;

   fsinfo = child_node->fs_info->down_layer_info;
   finfo = child_node->f_info.down_layer_info;

   child_node->f_info.down_layer_info = tlsf_malloc(fs_mem_handle, sizeof(fat_file_info_t));
   if(NULL != child_node->f_info.down_layer_info)
   {
      finfo = (fat_file_info_t *) child_node->f_info.down_layer_info;
      memset((void *)finfo, 0, sizeof(fat_file_info_t));

      fat_path_buffer[0] = fsinfo->fatfs_devnum + '0';
      fat_path_buffer[1] = ':'; fat_path_buffer[2] = '\0';

      if(VFS_FTDIR == child_node->f_info.type)
      {
         if( 0 == print_relative_path(child_node, fat_path_buffer+2, FAT_PATH_BUFFER_SIZE-2) )
         {
            if( FR_OK == f_mkdir((TCHAR*)fat_path_buffer) )
            {
               if( FR_OK == f_stat((TCHAR*)fat_path_buffer, &(finfo->fno)) )
               {
                  ret = 0;
               }
            }
         }
      }
      else if(VFS_FTREG == child_node->f_info.type)
      {
         if( 0 == print_relative_path(child_node, fat_path_buffer+2, FAT_PATH_BUFFER_SIZE-2) )
         {
            if( FR_OK == f_open(&(finfo->fatfs_fp), (TCHAR*)fat_path_buffer, FA_CREATE_ALWAYS | FA_READ | FA_WRITE) )
            {
               if( FR_OK == f_stat((TCHAR*)fat_path_buffer, &(finfo->fno)) )
               {
                  ret = 0;
               }
            }
         }
      }
      else
      {

      }
   }
   else /* tlsf_malloc() failed */
   {

   }
   if(0 > ret)
   {
      tlsf_free(fs_mem_handle, (void *)child_node->f_info.down_layer_info);
      child_node->f_info.down_layer_info = NULL;
   }
   return ret;
}


static int fat_delete_node(vnode_t *parent_node, vnode_t *child_node)
{
   int ret = -1;
   fat_fs_info_t *fsinfo;
   fat_file_info_t *finfo;

   fsinfo = child_node->fs_info->down_layer_info;
   finfo = child_node->f_info.down_layer_info;

   fat_path_buffer[0] = fsinfo->fatfs_devnum + '0';
   fat_path_buffer[1] = ':'; fat_path_buffer[2] = '\0';

   if( 0 == print_relative_path(child_node, fat_path_buffer+2, FAT_PATH_BUFFER_SIZE-2) )
   {
      if( VFS_FTREG == child_node->f_info.type ) /* File is regular so it was open */
      {
         if( FR_OK == f_close(&(finfo->fatfs_fp)) ) /* Close file before removal */
         {
            if( FR_OK == f_unlink((TCHAR*)fat_path_buffer) )
            {
               ret = 0;
            }
         }
      }
      else /* File is dir so it was not open */
      {
         if( FR_OK == f_unlink((TCHAR*)fat_path_buffer) ) /* Just remove dir */
         {
            ret = 0;
         }
      }
      if(0 == ret)
      {
         tlsf_free(fs_mem_handle, (void *)child_node->f_info.down_layer_info);
         child_node->f_info.down_layer_info = NULL;
      }
   }
   else /* print_relative_path() failed */
   {

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
   uint32_t write_size;

   finfo = desc->node->f_info.down_layer_info;

   if(desc->cursor > f_size(&(finfo->fatfs_fp)))
      desc->cursor = f_size(&(finfo->fatfs_fp));

   /* Call Chan FatFS */
   if( FR_OK == f_lseek(&(finfo->fatfs_fp), desc->cursor) )
   {
      if( FR_OK == f_write(&(finfo->fatfs_fp), buf, size, (UINT*)&write_size) )
      {
         desc->cursor += write_size;
      }
      else /* Error in f_write */
      {

      }
   }
   else /* Error in f_seek */
   {

   }
   return write_size;
}

/* Uses fat_path_buffer[] */
FRESULT scan_files(vnode_t *dir_node)
{
   FRESULT res;
   DIR dir;
   uint32_t i;
   static FILINFO fno;
   vnode_t *child_node;
   fat_file_info_t *finfo;

   res = f_opendir(&dir, (const TCHAR*)fat_path_buffer);
   if( FR_OK == res )
   {
      while(1)
      {
         res = f_readdir(&dir, &fno);
         if(FR_OK != res || 0 == fno.fname[0]) break; /* error or EOF */
         /* fno.fname includes \0. No danger, so use FS_NAME_MAX */
         child_node = vfs_create_child(dir_node, fno.fname, FS_NAME_MAX, 0);
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
                fno.fname, FAT_PATH_BUFFER_SIZE - i) )
            {
               /* End of path reached */
               return -1;
            }
            res = scan_files(child_node);
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

/*==================[external functions definition]==========================*/

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
