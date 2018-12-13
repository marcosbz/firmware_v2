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

/** \brief Nbd-Client class implementation file
 **
 ** Class of nbd client block device
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
#include "storageUSB.h"
#include "implement/storageUSB_impl_lpc4337.h"
#include "device.h"
#include "implement/device_impl_lpc4337.h"

/*==================[macros and definitions]=================================*/
#define NBD_BLOCKSIZE 512

/*==================[internal data definition]===============================*/
static const char * const string_nbd_nbdmagic = "NBDMAGIC";
static const char * const string_nbd_ihaveopt = "IHAVEOPT";
static const char * const string_nbd_flag_NBD_OPT_EXPORT_NAME = ""; /* Default export */
static const uint32_t string_nbd_simple_reply_magic = 0x67446698;
static const uint32_t string_nbd_request_magic = 0x25609513;
static const uint16_t string_nbd_request_read = 0;
static const uint16_t string_nbd_request_write = 1;
static const uint16_t string_nbd_request_disconnect = 2;
static const uint16_t string_nbd_request_flush = 3;

/*==================[internal functions declaration]=========================*/
/* BlockDevice interface implementation */
static ssize_t nbd_read(Nbd self, uint8_t * const buf, size_t const nbyte);
static ssize_t nbd_write(Nbd self, uint8_t const * const buf, size_t const nbyte);
static ssize_t nbd_lseek(Nbd self, off_t const offset, uint8_t const whence);
static int nbd_ioctl(Nbd self, int32_t request, void* param);
static int nbd_connect(Nbd self);
static int nbd_disconnect(Nbd self);
static int nbd_getState(Nbd self, blockDevState_t *state);
static int nbd_getInfo(Nbd self, blockDevInfo_t *info);

int nbd_request_write(struct netconn *conn, const uint8_t *data, uint32_t length, uint64_t offset);
int nbd_request_read(struct netconn *conn, uint8_t *data, uint32_t length, uint64_t offset);
int build_header(uint8_t *header, uint16_t request_type, uint64_t offset, uint32_t length);
int parse_reply(struct netconn *conn, uint8_t *reply_data, uint32_t data_length, uint32_t *reply_errno);
uint64_t htonll(uint64_t n);
uint64_t ntohll(uint64_t n);

/*==================[external data definition]===============================*/
/** \brief Allocating the class description table and the vtable
*/

InterfaceRegister(Nbd)
{
   AddInterface(Nbd, BlockDevice)
};

AllocateClassWithInterface(Nbd, Device);
/** \brief Class virtual function prototypes
*/
/*==================[internal functions definition]==========================*/
/** \brief Class initializing
*/
static void Nbd_initialize( Class this )
{
   /* Init vtable and override/assign virtual functions */
   NbdVtable vtab = & NbdVtableInstance;
   vtab->BlockDevice.read = (ssize_t (*)(Object, uint8_t * const, size_t const))nbd_read;
   vtab->BlockDevice.write = (ssize_t (*)(Object, uint8_t const * const buf, size_t const))nbd_write;
   vtab->BlockDevice.lseek = (ssize_t (*)(Object, off_t const, uint8_t const))nbd_lseek;
   vtab->BlockDevice.ioctl = (int (*)(Object, int32_t, void*))nbd_ioctl;
   vtab->BlockDevice.connect = (int (*)(Object))nbd_connect;
   vtab->BlockDevice.disconnect = (int (*)(Object))nbd_disconnect;
   vtab->BlockDevice.getState = (int (*)(Object, blockDevState_t *))nbd_getState;
   vtab->BlockDevice.getInfo = (int (*)(Object, blockDevInfo_t *))nbd_getInfo;
   /* Allocate global resources here */
}

/** \brief Class finalizing
*/
#ifndef OOC_NO_FINALIZE
static void Nbd_finalize( Class this )
{
   /* Release global resources! */
}
#endif
/** \brief Constructor */
static void Nbd_constructor( Nbd self, const void *params )
{
   nbd_constructor_params_t *nbd_params;
   assert( ooc_isInitialized( Nbd ) );
   chain_constructor( Nbd, self, NULL );
   nbd_params = (nbd_constructor_params_t *)params;
   if(nbd_params != NULL)
   {
      /* TODO */
   }
   else
   {
      /* TODO */
   }
   //self->FlashDisk_MS_Interface = &FlashDisk_MS_Interface0;
   self->conn = NULL;
   IP4_ADDR(&(self->remote_ip),192,168,0,2);
   self->server_export_size = 0;
   self->server_transmission_flags = 0;
   self->position = 0;
   self->status = NBD_STATUS_UNINIT;
}
/** \brief Destructor */
static void Nbd_destructor( Nbd self, NbdVtable vtab )
{
   /* Nothing allocated, no resources to free. Do nothing */
}
/** \brief Copy constructor */
static int Nbd_copy( Nbd self, const Nbd from )
{
   /* Prevent object duplication */
   return OOC_NO_COPY;
}
/*==================[external functions definition]==========================*/
/** \brief Class member functions */
Nbd nbd_new(void)
{
   nbd_constructor_params_t nbd_params;
   return (Nbd) ooc_new(Nbd, (void *)&nbd_params);
}
nbd_status_t nbd_getStatus(Nbd self)
{
   return self->status;
}
int nbd_init(Nbd self)
{
   int ret = -1;
   uint8_t n;

   self->status = NBD_STATUS_UNINIT;
   self->DiskCapacity.BlockSize = 512;
   xTaskCreate(vSetupIFTask, (signed char *) "SetupIFx",
            configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
            (xTaskHandle *) NULL);

   if( 0 == nbd_connect(self) )
   {
      self->status = NBD_STATUS_READY;
      ret = 0;
   }

   return ret;
}

int nbd_isInserted(Nbd self)
{
   return (NBD_STATUS_READY == self->status) ? 0 : -1;
}

int nbd_singleBlockRead(Nbd self, uint8_t *readBlock, uint32_t sector)
{
   int ret = -1;
   if(NBD_STATUS_READY == self->status && sector < self->DiskCapacity.Blocks)
   {
      if(0 == nbd_request_read(readBlock, self->DiskCapacity.BlockSize, sector * self->DiskCapacity.BlockSize))
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

int nbd_singleBlockWrite(Nbd self, const uint8_t *writeBlock, uint32_t sector)
{
   int ret = -1;
   if(NBD_STATUS_READY == self->status && sector < self->DiskCapacity.Blocks)
   {
      if(0 == nbd_request_write(writeBlock, self->DiskCapacity.BlockSize, sector * self->DiskCapacity.BlockSize))
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

int nbd_blockErase(Nbd self, uint32_t start, uint32_t end)
{
   int ret = -1;
   uint32_t i;
   static const uint8_t zeroBlock[NBD_BLOCKSIZE] = {0};
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

uint32_t nbd_getSize(Nbd self)
{
   return self->DiskCapacity.Blocks;
}

/* BlockDevice interface implementation */
static ssize_t nbd_read(Nbd self, uint8_t * const buf, size_t const nbyte)
{
   ssize_t ret = -1;
   size_t bytes_left, bytes_read, i, sector, position, bytes_offset;
   assert(ooc_isInstanceOf(self, Nbd));
   i=0; bytes_left = nbyte; sector = self->position / self->DiskCapacity.BlockSize; position = self->position;
   while(bytes_left)
   {
      bytes_offset = position % self->DiskCapacity.BlockSize;
      bytes_read = (bytes_left > (self->DiskCapacity.BlockSize - bytes_offset)) ? (self->DiskCapacity.BlockSize - bytes_offset) : bytes_left;
      if(nbd_singleBlockRead(self, self->block_buf, sector) == 0)
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
static ssize_t nbd_write(Nbd self, uint8_t const * const buf, size_t const nbyte)
{
   ssize_t ret = -1;
   size_t bytes_left, bytes_write, i, sector, position, bytes_offset;
   assert(ooc_isInstanceOf(self, Nbd));
   i=0; bytes_left = nbyte; sector = self->position / self->DiskCapacity.BlockSize; position = self->position;
   while(bytes_left)
   {
      bytes_offset = position % self->DiskCapacity.BlockSize;
      bytes_write = bytes_left > (self->DiskCapacity.BlockSize - bytes_offset) ? (self->DiskCapacity.BlockSize - bytes_offset) : bytes_left;
      //printf("nbd_write(): bytes_left: %d bytes_write: %d sector: %d\n",
      // bytes_left, bytes_write, sector);
      if(nbd_singleBlockRead(self, self->block_buf, sector) == 0)
      {
         //printf("nbd_write(): Block readed, now copy new data\n");
         memcpy(self->block_buf + bytes_offset, buf + i, bytes_write);
         //printf("nbd_write(): Data copied. Now write back\n");
         if(nbd_singleBlockWrite(self, self->block_buf, sector) == 0)
         {
            //printf("nbd_write(): write back succesfull. Next iteration\n");
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

static ssize_t nbd_lseek(Nbd self, off_t const offset, uint8_t const whence)
{
   assert(ooc_isInstanceOf(self, Nbd));
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

static int nbd_ioctl(Nbd self, int32_t request, void* param)
{
   assert(ooc_isInstanceOf(self, Nbd));
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



/*
S: 64 bits, 0x4e42444d41474943 (ASCII 'NBDMAGIC') (as in the old style handshake)
S: 64 bits, 0x49484156454F5054 (ASCII 'IHAVEOPT') (note different magic number)
S: 16 bits, handshake flags
C: 32 bits, client flags

C: 64 bits, 0x49484156454F5054 (ASCII 'IHAVEOPT') (note same newstyle handshake's magic number)
C: 32 bits, option
C: 32 bits, length of option data (unsigned)
C: any data needed for the chosen option, of length as specified above.

S: 64 bits, size of the export in bytes (unsigned)
S: 16 bits, transmission flags
S: 124 bytes, zeroes (reserved) (unless NBD_FLAG_C_NO_ZEROES was negotiated by the client)
*/

static int nbd_connect(Nbd self)
{
   int ret = -1;

   struct netbuf *inbuf;
   uint32_t buflen;
   uint8_t *buf;

   uint32_t string_nbd_flag_offset;

   assert(ooc_isInstanceOf(self, Nbd));

   /* Create a new TCP connection handle */
   self->conn = netconn_new(NETCONN_TCP);
   if(NULL != conn)
   {
      /* Bind to port 80 (HTTP) with default IP address */
      if(ERR_OK == netconn_bind(self->conn, NULL, 10809))
      {
         if(ERR_OK == netconn_connect(self->conn, self->remote_ip, 10809))
         {
            /*** ALREADY CONNECTED. BEGIN PROTOCOL ***/
            /* Read the data from the port, blocking if nothing yet there. */
            /* We assume the request is in one netbuf. */
            netbuf_delete(inbuf);
            if (ERR_OK == netconn_recv(conn, &inbuf))
            {
            /* Read data from netbuf to the provided buffer. */
               netbuf_data(inbuf, (void**)&buf, &buflen);
               string_nbd_flag_offset = strlen(string_nbd_nbdmagic) + strlen(string_nbd_ihaveopt);
               /*
               S: 64 bits, 0x4e42444d41474943 (ASCII 'NBDMAGIC') (as in the old style handshake)
               S: 64 bits, 0x49484156454F5054 (ASCII 'IHAVEOPT') (note different magic number)
               */
               if(string_nbd_flag_offset + 2 <= buflen)
               {
                  self->server_flags = ntohs(*((uint16_t *)&buf[string_nbd_flag_offset]));
                  /*
                  S: 16 bits, handshake flags
                  */
                  if(self->server_flags) /* Check handshake flags */
                  {
                     /* Send 32 bits 0s */
                     /*
                     C: 32 bits, client flags
                     */
                     if(ERR_OK == netconn_write(self->conn, (void *)htonl(self->client_flags), 4, NETCONN_NOCOPY))
                     {
                        /*
                        C: 64 bits, 0x49484156454F5054 (ASCII 'IHAVEOPT') (note same newstyle handshake's magic number)
                        */
                        if(ERR_OK == netconn_write(self->conn, (void *)string_nbd_ihaveopt, strlen(string_nbd_ihaveopt), NETCONN_NOCOPY))
                        {
                           memset(aux_string, '\0', 4); /* export_name_length = 0 */
                           /*
                           C: 32 bits, option
                           */
                           if(ERR_OK == netconn_write(self->conn, (void *)aux_string, 4, NETCONN_NOCOPY))
                           {
                              /*
                              C: 32 bits, length of option data (unsigned)
                              */
                              if(ERR_OK == netconn_write(self->conn, (void *)aux_string, 4, NETCONN_NOCOPY))
                              {
                                 /*
                                 C: any data needed for the chosen option, of length as specified above.
                                 */
                                 if(ERR_OK == netconn_write(self->conn, (void *)string_nbd_flag_NBD_OPT_EXPORT_NAME, 0, NETCONN_NOCOPY))
                                 {
                                    netbuf_delete(inbuf);
                                    if (ERR_OK == netconn_recv(conn, &inbuf))
                                    {
                                       netbuf_data(inbuf, (void**)&buf, &buflen);
                                       if(8 + 2 + 124 <= buflen) /* Size of the export 64bits + transmission flags 16 bits + 124bytes zeroes */
                                       {
                                          self->server_export_size = ntohll(*((uint64_t *)buf));
                                          self->server_transmission_flags = ntohs(*((uint16_t *)(buf + 8)));
                                          self->DiskCapacity.Blocks = self->server_export_size / self->DiskCapacity.BlockSize;
                                          /* Handshake finished */
                                          self->status = NBD_STATUS_READY;
                                          ret = 0
                                       }
                                    }
                                 }
                              }
                           }
                        }
                     }
                  }
               }
            }
         }
      }
   }
   netbuf_delete(inbuf);
   return ret;
}
static int nbd_disconnect(Nbd self)
{
   assert(ooc_isInstanceOf(self, Nbd));

   /* TODO: Disconnect from nbd server. Disconnect request */
   /* Free netconn socket. */
   netconn_close(self->conn);
   netconn_delete(self->conn);

   return 0;
}
static int nbd_getState(Nbd self, blockDevState_t *state)
{
   assert(ooc_isInstanceOf(self, Nbd));
   /* TODO: Should specify specific state, but quick fix now */
   *state = BLKDEV_UNINIT;
   if(NBD_STATUS_READY == self->status)
      *state = BLKDEV_READY;
   return 0;
}
static int nbd_getInfo(Nbd self, blockDevInfo_t *info)
{
   assert(ooc_isInstanceOf(self, Nbd));
   return 0;
}

/* aux functions */

/*
/* There are three message types in the transmission phase: the request, the simple reply, and the structured reply chunk. */

int nbd_request_write(struct netconn *conn, const uint8_t *data, uint32_t length, uint64_t offset)
{
   int ret = -1;
   uint8_t nbd_header_buf[28];
   uint32_t reply_errno;

   build_header(nbd_header_buf, string_nbd_request_write, offset, length);
   if(ERR_OK == netconn_write(conn, (void *)nbd_header_buf, 28, NETCONN_NOCOPY))
   {
      if(ERR_OK == netconn_write(conn, (void *)data, length, NETCONN_NOCOPY))
      {
         if(0 <= parse_reply(conn, NULL, 0, &reply_errno))
         {
            if(0 == reply_errno)
            {
               ret = 0;
            }
         }
      }
   }
   return ret;
}

int nbd_request_read(struct netconn *conn, uint8_t *data, uint32_t length, uint64_t offset)
{
   int ret = -1;
   uint8_t nbd_header_buf[28];

   build_header(nbd_packet_buf, string_nbd_request_read, offset, length);
   if(ERR_OK == netconn_write(conn, (void *)nbd_packet_buf, 28, NETCONN_NOCOPY))
   {
      if(0 <= parse_reply(conn, data, length, &reply_errno))
      {
         if(0 == reply_errno)
         {
            ret = 0;
         }
      }
   }
   return ret;
}

int build_header(uint8_t *header, uint16_t request_type, uint64_t offset, uint32_t length)
{
   int ret = -1;
   uint16_t command_flags = 0;
   uint32_t nbd_request_magic;
   uint64_t handle = 0;

   nbd_request_magic = htonl(string_nbd_request_magic);
   command_flags = htons(command_flags);
   request_type = htons(request_type);
   handle = htonll(handle);
   offset = htonll(offset);
   length = htonl(length);
   memcpy((void *)header, (void *)&nbd_request_magic, 4);
   memcpy((void *)(header + 4), (void *)&command_flags, 2);
   memcpy((void *)(header + 4 + 2), (void *)&request_type, 2);
   memcpy((void *)(header + 4 + 2 + 2), (void *)&handle, 8);
   memcpy((void *)(header + 4 + 2 + 2 + 8), (void *)&offset, 8);
   memcpy((void *)(header + 4 + 2 + 2 + 8 + 8), (void *)&length, 4);
   ret = 0;
   return ret;
}

int parse_reply(struct netconn *conn, uint8_t *reply_data, uint32_t data_length, uint32_t *reply_errno)
{
   int ret = -1;
   uint32_t reply_len, len, length_payload_remain, size_aux_copy;
   uint8_t *data;
   uint32_t reply_magic;
   uint64_t reply_handle;
   struct netbuf *inbuf;

   if (NULL != (inbuf = netconn_recv(conn)))
   {
      netbuf_data(inbuf, &data, &len);
      if(len >= 16)
      {
         memcpy((void *)&reply_magic, (void *)data, 4);
         memcpy((void *)reply_errno, (void *)(data + 4), 4);
         memcpy((void *)&reply_handle, (void *)(data + 8), 8);
         reply_magic = ntohl(reply_magic);
         *reply_errno = ntohl(*reply_errno);
         reply_handle = ntohll(reply_handle);

         if(string_nbd_simple_reply_magic == reply_magic)
         {
            if(NULL != reply_data)
            {
               /* reply_data is valid pointer, so data reply expected. Copy it to buffer. */
               length_payload_remain = data_length;
               if(length_payload_remain < (len - 16))
                  size_aux_copy = length_payload_remain;
               else
                  size_aux_copy = len - 16;
               memcpy((void *)reply_data, (void *)(data + 16), size_aux_copy);
               reply_len = len - 16;
               length_payload_remain -= size_aux_copy;
               while(length_payload_remain && netbuf_next(inbuf) >= 0)
               {
                  netbuf_data(inbuf, &data, &len);
                  if(length_payload_remain < len)
                     size_aux_copy = length_payload_remain;
                  else
                     size_aux_copy = len;
                  memcpy((void *)(reply_data + reply_len), (void *)data, size_aux_copy);
                  reply_len += len;
                  length_payload_remain -= size_aux_copy;
               }
               if(0 == length_payload_remain)
               {
                  /* All requested data copied, does not matter if more data is available */
                  ret = 0;
               }
            }
            else
            {
               /* No payload requested. If header data is ok then everything ok */
               ret = 0;
            }
         }
      }
   }
   /* FIXME: Calling this deallocates netbuf, despite of netbuf_next(inbuf) being previously called? Inbuf should be the original? */
   netbuf_delete(inbuf);
   return ret;
}


uint64_t htonll(uint64_t n)
{
#if __BYTE_ORDER == __BIG_ENDIAN
    return n;
#else
    return (((uint64_t)htonl(n)) << 32) + htonl(n >> 32);
#endif
}

uint64_t ntohll(uint64_t n)
{
#if __BYTE_ORDER == __BIG_ENDIAN
    return n;
#else
    return (((uint64_t)ntohl(n)) << 32) + ntohl(n >> 32);
#endif
}
