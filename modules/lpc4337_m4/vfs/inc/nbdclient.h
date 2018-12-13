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

#ifndef NBDCLIENT_H
#define NBDCLIENT_H
/** \brief Short description of this file
 **
 ** Long description of this file
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Template Template to start a new module
 ** @{ */

/*==================[inclusions]=============================================*/
#include <stdint.h>
#include "device.h"
#include "ooc.h"
#include "blockDevice.h"

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
/*==================[typedef]================================================*/
/** \brief Describe peripheral current state */
typedef enum
{
   NBD_STATUS_UNINIT,
   NBD_STATUS_READY,
   NBD_STATUS_BUSY
} nbd_status_t;

/** \brief NBD device type */
typedef enum
{
} nbd_devType_t;

typedef struct
{

} nbd_constructor_params_t;

/*==================[external data declaration]==============================*/
/** \brief Nbd class declaration. Nbd inherits from Device.
 **
 **/
DeclareClass(Nbd, Device);

/*==================[external functions declaration]=========================*/
/** \brief Wrapper to class constructor with parameters. Use this function for object instantiation.
 **
 ** \return    The newly created Object
 **/
Nbd nbd_new(void);

/** \brief NBD device initialization sequence
 **
 **
 ** \param[in] self NBD device handle
 ** \return    0 success, else error
 **/
int nbd_init(Nbd self);

/** \brief get NBD device status
 **
 **
 ** \param[in] self NBD device handle
 ** \return    Ready: Ready to operate, Busy: Must wait for new operation.
 **/
nbd_status_t nbd_getStatus(Nbd self);

/** \brief Check if NBD device present in slot
 **
 ** \param[in] Nbd NBD device handle
 ** \return    0 if not present, not 0 otherwise
 **/
int nbd_isInserted(Nbd self);

/** \brief Read single block
 **
 ** \param[in] Nbd NBD device handler
 ** \param[out] readBlock buffer in which to read block
 ** \param[in] sector Block offset
 ** \return    -1 for timeout and other errors, 0 if success
 **/
int nbd_singleBlockRead(Nbd self, uint8_t *readBlock, uint32_t sector);

/** \brief Write single block
 **
 ** \param[in] Nbd NBD device handler
 ** \param[in] writeBlock buffer from which to write block
 ** \param[in] sector Block offset
 ** \return    -1 for timeout and other errors, 0 if success
 **/
int nbd_singleBlockWrite(Nbd self, const uint8_t *writeBlock, uint32_t sector);

/** \brief get card size in blocks
 **
 ** \param[in] Nbd NBD device handler
 ** \return    number of blocks in card
 **/
uint32_t nbd_getSize(Nbd self);


/** \brief erase multiple adjacent blocks
 **
 ** \param[in] Nbd NBD device handler
 ** \param[in] start block from where to start erasing
 ** \param[in] end last block to erase
 ** \return    -1 for timeout and other errors, 0 if success
 **/
int nbd_blockErase(Nbd self, uint32_t start, uint32_t end);

//int Nbd_multipleBlockRead(Nbd self, uint8_t *readBuffer, uint32_t sector, uint32_t readCount);
//int Nbd_multipleBlockWrite(Nbd self, uint8_t *writeBuffer, uint32_t sector, uint32_t writeCount);
/* Virtual function definitions */
Virtuals( Nbd, Device )
   Interface(BlockDevice);
EndOfVirtuals;

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif  /* #ifndef NBDCLIENT_H */
