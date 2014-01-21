/*
 * Copyright (c) 2012, Texas Instruments Incorporated
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
 *  ======== USBMH.h ========
 */

#ifndef USBMH_H_
#define USBMH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <xdc/std.h>

/* Data structure used to specify the current state of USB mouse */
typedef struct {
    Char deltaX;    /*! X position offset */
    Char deltaY;    /*! Y position offset */
    Bool button1;   /*! Button 1 */
    Bool button2;   /*! Button 2 */
    Bool button3;   /*! Button 3 */
} USBMH_State;

/*!
 *  ======== USBMH_init ========
 *  Function to initialize the USB mouse reference module.
 *
 *  Note: This function is not reentrant safe.
 */
extern Void USBMH_init(Void);

/*!
 *  ======== USBMH_getState ========
 *  Function is a NON-blocking function that returns the status of the mouse.
 *
 *  Function updates the MouseState structure with the latest values from the
 *  mouse device.
 *
 *  @return         A bit packed value
 *                  Bit 0: 0 - Disconnected; 1 - Connected
 */
extern Void USBMH_getState(USBMH_State *mouseState);

/*!
 *  ======== USBMH_waitForConnect ========
 *  This function blocks while the USB is not connected
 */
extern Bool USBMH_waitForConnect(UInt timeout);

#ifdef __cplusplus
}
#endif

#endif /* USBMH_H_ */
