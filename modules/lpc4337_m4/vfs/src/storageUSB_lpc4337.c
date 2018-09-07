/* Copyright 2014, ACSE & CADIEEL
 *    ACSE   : http://www.sase.com.ar/asociacion-civil-sistemas-embebidos/ciaa/
 *    CADIEEL: http://www.cadieel.org.ar
 * All rights reserved.
 *
 *    or
 *
 * Copyright 2014, Your Name <youremail@domain.com>
 * All rights reserved.
 *
 *    or
 *
 * Copyright 2014, ACSE & CADIEEL & Your Name <youremail@domain.com
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

/** \brief StorageUSB class implementation file
 **
 ** Class of usb storage device
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Template Template to start a new module
 ** @{ */

/*==================[inclusions]=============================================*/
#include <string.h>
#include "board.h"
#include "chip.h"
#include "USB.h"
#include "storageUSB.h"
#include "implement/storageUSB_impl_lpc4337.h"
#include "device.h"
#include "implement/device_impl_lpc4337.h"
/*==================[macros and definitions]=================================*/
#define STORAGE_USB_BLOCKSIZE 512

/*==================[internal data definition]===============================*/

/** LPCUSBlib Mass Storage Class driver interface configuration and state information. This structure is
 *  passed to all Mass Storage Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
static USB_ClassInfo_MS_Host_t FlashDisk_MS_Interface0 = {
	.Config = {
		.DataINPipeNumber       = 1,
		.DataINPipeDoubleBank   = false,

		.DataOUTPipeNumber      = 2,
		.DataOUTPipeDoubleBank  = false,
		.PortNumber = 0,
	},
};

/*==================[internal functions declaration]=========================*/
/* BlockDevice interface implementation */
static ssize_t storageUSB_read(StorageUSB self, uint8_t * const buf, size_t const nbyte);
static ssize_t storageUSB_write(StorageUSB self, uint8_t const * const buf, size_t const nbyte);
static ssize_t storageUSB_lseek(StorageUSB self, off_t const offset, uint8_t const whence);
static int storageUSB_ioctl(StorageUSB self, int32_t request, void* param);
static int storageUSB_connect(StorageUSB self);
static int storageUSB_disconnect(StorageUSB self);
static int storageUSB_getState(StorageUSB self, blockDevState_t *state);
static int storageUSB_getInfo(StorageUSB self, blockDevInfo_t *info);

/*==================[internal data definition]===============================*/
/*==================[external data definition]===============================*/
/** \brief Allocating the class description table and the vtable
 */
InterfaceRegister(StorageUSB)
{
	AddInterface(StorageUSB, BlockDevice)
};

AllocateClassWithInterface(StorageUSB, Device);

/** \brief Class virtual function prototypes
 */

/*==================[internal functions definition]==========================*/

/** \brief Class initializing
 */

static void StorageUSB_initialize( Class this )
{
   /* Init vtable and override/assign virtual functions */
   StorageUSBVtable vtab = & StorageUSBVtableInstance;

   vtab->BlockDevice.read = (ssize_t (*)(Object, uint8_t * const, size_t const))storageUSB_read;
   vtab->BlockDevice.write = (ssize_t (*)(Object, uint8_t const * const buf, size_t const))storageUSB_write;
   vtab->BlockDevice.lseek = (ssize_t (*)(Object, off_t const, uint8_t const))storageUSB_lseek;
   vtab->BlockDevice.ioctl = (int (*)(Object, int32_t, void*))storageUSB_ioctl;
   vtab->BlockDevice.connect = (int (*)(Object))storageUSB_connect;
   vtab->BlockDevice.disconnect = (int (*)(Object))storageUSB_disconnect;
   vtab->BlockDevice.getState = (int (*)(Object, blockDevState_t *))storageUSB_getState;
   vtab->BlockDevice.getInfo = (int (*)(Object, blockDevInfo_t *))storageUSB_getInfo;
   /* Allocate global resources here */
}

/** \brief Class finalizing
 */

#ifndef OOC_NO_FINALIZE
static void StorageUSB_finalize( Class this )
{
   /* Release global resources! */
}
#endif

/** \brief Constructor */
static void StorageUSB_constructor( StorageUSB self, const void *params )
{
   storageUSB_constructor_params_t *storageusb_params;
   assert( ooc_isInitialized( StorageUSB ) );
   chain_constructor( StorageUSB, self, NULL );
   storageusb_params = (storageUSB_constructor_params_t *)params;
   if(storageusb_params != NULL)
   {
      /* TODO */
   }
   else
   {
      /* TODO */
   }
   self->FlashDisk_MS_Interface = &FlashDisk_MS_Interface0;
   self->position = 0;
   self->status = USB_STATUS_UNINIT;
}

/** \brief Destructor */
static void StorageUSB_destructor( StorageUSB self, StorageUSBVtable vtab )
{
   /* Nothing allocated, no resources to free. Do nothing */
}

/** \brief Copy constructor */
static int StorageUSB_copy( StorageUSB self, const StorageUSB from )
{
   /* Prevent object duplication */
   return OOC_NO_COPY;
}

/*==================[external functions definition]==========================*/
/** \brief Class member functions */


StorageUSB storageUSB_new(void)
{
   storageUSB_constructor_params_t storageusb_params;
   return (StorageUSB) ooc_new(StorageUSB, (void *)&storageusb_params);
}

usb_status_t storageUSB_getStatus(StorageUSB self)
{
   return self->status;
}

int storageUSB_init(StorageUSB self)
{
   int ret = 1;
   uint8_t n;

#if (defined(CHIP_LPC43XX) || defined(CHIP_LPC18XX))
   if (self->FlashDisk_MS_Interface->Config.PortNumber == 0)
   {
      Chip_USB0_Init();
   }
   else
   {
      Chip_USB1_Init();
   }
#endif
   USB_Init(self->FlashDisk_MS_Interface->Config.PortNumber, USB_MODE_Host);
   /* Hardware Initialization */
   Board_Debug_Init();

   /* Reset */
   self->status = USB_STATUS_UNINIT;

   /* Wait for card to be inserted */
   while (USB_HostState[self->FlashDisk_MS_Interface->Config.PortNumber] != HOST_STATE_Configured) {
      MS_Host_USBTask(self->FlashDisk_MS_Interface);
      USB_USBTask(self->FlashDisk_MS_Interface->Config.PortNumber, USB_MODE_Host);
   }

   DEBUGOUT("Waiting for ready...");
   for (;; )
   {
      uint8_t ErrorCode = MS_Host_TestUnitReady(self->FlashDisk_MS_Interface, 0);

      if (!(ErrorCode)) {
         break;
      }

      /* Check if an error other than a logical command error (device busy) received */
      if (ErrorCode != MS_ERROR_LOGICAL_CMD_FAILED) {
         DEBUGOUT("Failed\r\n");
         USB_Host_SetDeviceConfiguration(self->FlashDisk_MS_Interface->Config.PortNumber, 0);
         ret = -1;
         break;
      }
   }
   if(1 == ret)
   {
      DEBUGOUT("Done.\r\n");

      if (0 == MS_Host_ReadDeviceCapacity(self->FlashDisk_MS_Interface, 0, &(self->DiskCapacity)))
      {
         DEBUGOUT(("%lu blocks of %lu bytes.\r\n"), self->DiskCapacity.Blocks, self->DiskCapacity.BlockSize);
         self->status = USB_STATUS_READY;
         ret = 0;
      }
      else
      {
         DEBUGOUT("Error retrieving device capacity.\r\n");
         USB_Host_SetDeviceConfiguration(self->FlashDisk_MS_Interface->Config.PortNumber, 0);
         ret = -1;
      }
   }
   return ret;
}

int storageUSB_isInserted(StorageUSB self)
{
   return (HOST_STATE_Unattached == USB_HostState[self->FlashDisk_MS_Interface->Config.PortNumber]) ? 0 : 1;
}

int storageUSB_singleBlockRead(StorageUSB self, uint8_t *readBlock, uint32_t sector)
{
   int ret = -1;

   if(USB_STATUS_READY == self->status && sector < self->DiskCapacity.Blocks)
   {
      if(0 == MS_Host_ReadDeviceBlocks(self->FlashDisk_MS_Interface, 0, sector, 1,
         self->DiskCapacity.BlockSize, (void *)readBlock))
      {
         ret = 0;
      }
      else
      {
         DEBUGOUT("Error reading device block.\r\n");
         /*USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface.Config.PortNumber, 0);*/
      }
   }

   return ret;
}

int storageUSB_singleBlockWrite(StorageUSB self, const uint8_t *writeBlock, uint32_t sector)
{
   int ret = -1;

   if(USB_STATUS_READY == self->status && sector < self->DiskCapacity.Blocks)
   {
      if(0 == MS_Host_WriteDeviceBlocks(self->FlashDisk_MS_Interface, 0, sector, 1,
         self->DiskCapacity.BlockSize, (void *)writeBlock))
      {
         ret = 0;
      }
      else
      {
         DEBUGOUT("Error writing device block.\r\n");
         /*USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface.Config.PortNumber, 0);*/
      }
   }

   return ret;
}

int storageUSB_blockErase(StorageUSB self, uint32_t start, uint32_t end)
{
   int ret = -1;
   uint32_t i;
   static const uint8_t zeroBlock[STORAGE_USB_BLOCKSIZE] = {0};

   if(start <= end && end < self->DiskCapacity.Blocks)
   {
      for(i = start; i <= end; i++)
      {
         if(MS_Host_WriteDeviceBlocks(self->FlashDisk_MS_Interface, 0, i, 1,
            self->DiskCapacity.BlockSize, (void *)zeroBlock))
         {
            break;
         }
      }
      if(i == end+1)
      {
         ret = 0;
      }
   }
   return ret;
}

uint32_t storageUSB_getSize(StorageUSB self)
{
   return self->DiskCapacity.Blocks;
}

/* BlockDevice interface implementation */
static ssize_t storageUSB_read(StorageUSB self, uint8_t * const buf, size_t const nbyte)
{
   ssize_t ret = -1;
   size_t bytes_left, bytes_read, i, sector, position, bytes_offset;
   assert(ooc_isInstanceOf(self, StorageUSB));

   i=0; bytes_left = nbyte; sector = self->position / self->DiskCapacity.BlockSize; position = self->position;
   while(bytes_left)
   {
      bytes_offset = position % self->DiskCapacity.BlockSize;
      bytes_read = (bytes_left > (self->DiskCapacity.BlockSize - bytes_offset)) ? (self->DiskCapacity.BlockSize - bytes_offset) : bytes_left;
      if(storageUSB_singleBlockRead(self, self->block_buf, sector) == 0)
      {
         memcpy(buf + i, self->block_buf + bytes_offset, bytes_read);
         bytes_left -= bytes_read;
         i += bytes_read;
         position += bytes_read;
         sector++;
      }
      else
      {
         break;
      }
   }
   if(0 == bytes_left)
   {
      ret = i;
      self->position += i;
   }
   else
   {

   }

   return ret;
}

static ssize_t storageUSB_write(StorageUSB self, uint8_t const * const buf, size_t const nbyte)
{
   ssize_t ret = -1;
   size_t bytes_left, bytes_write, i, sector, position, bytes_offset;
   assert(ooc_isInstanceOf(self, StorageUSB));

   i=0; bytes_left = nbyte; sector = self->position / self->DiskCapacity.BlockSize; position = self->position;
   while(bytes_left)
   {
      bytes_offset = position % self->DiskCapacity.BlockSize;
      bytes_write = bytes_left > (self->DiskCapacity.BlockSize - bytes_offset) ? (self->DiskCapacity.BlockSize - bytes_offset) : bytes_left;
      //printf("storageUSB_write(): bytes_left: %d bytes_write: %d sector: %d\n",
      //                  bytes_left, bytes_write, sector);
      if(storageUSB_singleBlockRead(self, self->block_buf, sector) == 0)
      {
         //printf("storageUSB_write(): Block readed, now copy new data\n");
         memcpy(self->block_buf + bytes_offset, buf + i, bytes_write);
         //printf("storageUSB_write(): Data copied. Now write back\n");
         if(storageUSB_singleBlockWrite(self, self->block_buf, sector) == 0)
         {
            //printf("storageUSB_write(): write back succesfull. Next iteration\n");
            bytes_left -= bytes_write;
            i += bytes_write;
            sector++;
         }
         else
         {

         }
      }
   }
   if(0 == bytes_left)
   {

      ret = i;
      self->position += i;
   }
   else
   {

   }

   return ret;
}

static ssize_t storageUSB_lseek(StorageUSB self, off_t const offset, uint8_t const whence)
{
   assert(ooc_isInstanceOf(self, StorageUSB));
   off_t destination = -1;
   size_t partition_size = self->DiskCapacity.BlockSize * self->DiskCapacity.Blocks;

   switch(whence)
   {
      case SEEK_END:
         destination = partition_size + offset;
         break;
      case SEEK_CUR:
         destination = self->position + offset;
         break;
      default:
         destination = offset;
         break;
   }

   if ((destination >= 0) && (destination < partition_size))
   {
      self->position = destination;
   }

   return destination;
}

static int storageUSB_ioctl(StorageUSB self, int32_t request, void* param)
{
   assert(ooc_isInstanceOf(self, StorageUSB));

   int32_t ret = -1;

   blockDevInfo_t * blockInfo = (blockDevInfo_t *)param;

   switch(request)
   {
      case IOCTL_BLOCK_GETINFO:
         blockInfo->size = self->DiskCapacity.BlockSize;
         blockInfo->num = self->DiskCapacity.Blocks;
         ret = 1;
         break;
      default:
         break;
   }

   return ret;
}

static int storageUSB_connect(StorageUSB self)
{
   assert(ooc_isInstanceOf(self, StorageUSB));
   return 0;
}
static int storageUSB_disconnect(StorageUSB self)
{
   assert(ooc_isInstanceOf(self, StorageUSB));
   return 0;
}

static int storageUSB_getState(StorageUSB self, blockDevState_t *state)
{
   assert(ooc_isInstanceOf(self, StorageUSB));
	 /* TODO: Should specify specific state, but quick fix now */
	 *state = BLKDEV_UNINIT;
	 if(USB_STATUS_READY == self->status)
	    *state = BLKDEV_READY;
   return 0;
}

static int storageUSB_getInfo(StorageUSB self, blockDevInfo_t *info)
{
   assert(ooc_isInstanceOf(self, StorageUSB));
   return 0;
}

/** Event handler for the USB_DeviceAttached event. This indicates that a device has been attached to the host, and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Host_DeviceAttached(const uint8_t corenum)
{
	DEBUGOUT(("Device Attached on port %d\r\n"), corenum);
}

/** Event handler for the USB_DeviceUnattached event. This indicates that a device has been removed from the host, and
 *  stops the library USB task management process.
 */
void EVENT_USB_Host_DeviceUnattached(const uint8_t corenum)
{
	DEBUGOUT(("\r\nDevice Unattached on port %d\r\n"), corenum);
}

/** Event handler for the USB_DeviceEnumerationComplete event. This indicates that a device has been successfully
 *  enumerated by the host and is now ready to be used by the application.
 */
void EVENT_USB_Host_DeviceEnumerationComplete(const uint8_t corenum)
{
	uint16_t ConfigDescriptorSize;
	uint8_t  ConfigDescriptorData[512];

	if (USB_Host_GetDeviceConfigDescriptor(corenum, 1, &ConfigDescriptorSize, ConfigDescriptorData,
										   sizeof(ConfigDescriptorData)) != HOST_GETCONFIG_Successful) {
		DEBUGOUT("Error Retrieving Configuration Descriptor.\r\n");
		return;
	}

	FlashDisk_MS_Interface0.Config.PortNumber = corenum;
	if (MS_Host_ConfigurePipes(&FlashDisk_MS_Interface0,
							   ConfigDescriptorSize, ConfigDescriptorData) != MS_ENUMERROR_NoError) {
		DEBUGOUT("Attached Device Not a Valid Mass Storage Device.\r\n");
		return;
	}

	if (USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface0.Config.PortNumber, 1) != HOST_SENDCONTROL_Successful) {
		DEBUGOUT("Error Setting Device Configuration.\r\n");
		return;
	}

	uint8_t MaxLUNIndex;
	if (MS_Host_GetMaxLUN(&FlashDisk_MS_Interface0, &MaxLUNIndex)) {
		DEBUGOUT("Error retrieving max LUN index.\r\n");
		USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface0.Config.PortNumber, 0);
		return;
	}

	DEBUGOUT(("Total LUNs: %d - Using first LUN in device.\r\n"), (MaxLUNIndex + 1));

	if (MS_Host_ResetMSInterface(&FlashDisk_MS_Interface0)) {
		DEBUGOUT("Error resetting Mass Storage interface.\r\n");
		USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface0.Config.PortNumber, 0);
		return;
	}

	SCSI_Request_Sense_Response_t SenseData;
	if (MS_Host_RequestSense(&FlashDisk_MS_Interface0, 0, &SenseData) != 0) {
		DEBUGOUT("Error retrieving device sense.\r\n");
		USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface0.Config.PortNumber, 0);
		return;
	}

	SCSI_Inquiry_Response_t InquiryData;
	if (MS_Host_GetInquiryData(&FlashDisk_MS_Interface0, 0, &InquiryData)) {
		DEBUGOUT("Error retrieving device Inquiry data.\r\n");
		USB_Host_SetDeviceConfiguration(FlashDisk_MS_Interface0.Config.PortNumber, 0);
		return;
	}

	DEBUGOUT("Mass Storage Device Enumerated.\r\n");
}

/** Event handler for the USB_HostError event. This indicates that a hardware error occurred while in host mode. */
void EVENT_USB_Host_HostError(const uint8_t corenum, const uint8_t ErrorCode)
{
	USB_Disable(corenum, USB_MODE_Host);

	DEBUGOUT(("Host Mode Error\r\n"
			  " -- Error port %d\r\n"
			  " -- Error Code %d\r\n" ), corenum, ErrorCode);

	for (;; ) {}
}

/** Event handler for the USB_DeviceEnumerationFailed event. This indicates that a problem occurred while
 *  enumerating an attached USB device.
 */
void EVENT_USB_Host_DeviceEnumerationFailed(const uint8_t corenum,
											const uint8_t ErrorCode,
											const uint8_t SubErrorCode)
{
	DEBUGOUT(("Dev Enum Error\r\n"
			  " -- Error port %d\r\n"
			  " -- Error Code %d\r\n"
			  " -- Sub Error Code %d\r\n"
			  " -- In State %d\r\n" ),
			 corenum, ErrorCode, SubErrorCode, USB_HostState[corenum]);

}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
