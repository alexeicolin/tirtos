/*
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== USBCDCD_LoggerIdle.h ========
 */

#ifndef USBCDCD_LOGGERIDLE_H_
#define USBCDCD_LOGGERIDLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <xdc/std.h>
#include "USB_config/descriptors.h"

/*
 * This needs to be updated if there are more than one CDC class instances
 * defined in descriptors.h
 */
#define USBCDCD_INTFNUM        CDC0_INTFNUM

/*!
 *  ======== USBCDCD_init ========
 *  Function to initialize the USB serial reference module.
 *
 *  Note: This function is not reentrant safe.
 */
extern Void USBCDCD_init(Void);

/*!
 *  ======== USBCDCD_LoggerIdle_sendData ========
 *  Writes a buffer of data to the USB for transmission.
 *
 *  @param(pStr)    Pointer to a buffer of data
 *
 *  @param(length)  Number of bytes to be sent
 *
 *  @return         Number of bytes sent
 */
Int USBCDCD_LoggerIdle_sendData(UChar *pStr, Int length);

#ifdef __cplusplus
}
#endif

#endif /* USBCDCD_LOGGERIDLE_H_ */
