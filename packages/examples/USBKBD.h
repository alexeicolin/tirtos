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
 *  ======== USBKBD.h ========
 */

#ifndef USBKBD_H_
#define USBKBD_H_ 

#ifdef __cplusplus
extern "C" {
#endif

#include <xdc/std.h>

/* Data structure used to specify the current state of the LEDs */
typedef struct {
    Bool numLED;
    Bool capsLED;
    Bool scrollLED;
} USBKBD_State;

/*!
 *  ======== USBKBD_getState ========
 *  Function is a NON-blocking function that returns the status of the keyboard.
 *
 *  Function returns the status of the keyboard by returning a bit-packed
 *  variable. See Below.
 * 
 */
extern Void USBKBD_getState(USBKBD_State *keyboardState);

/*!
 *  ======== USBKBD_init ========
 *  Function to initialize the USB keyboard reference module.
 *
 *  Note: This function is not reentrant safe.
 */
extern Void USBKBD_init(Void);

/*!
 *  ======== USBKBD_putChar ========
 *  A blocking function that sends a character to the host.
 *
 *  Function to simulates a keyboard press and release sequence for an
 *  single character to the host.
 *
 *  @param(ch)      The printable character that will be sent to the host
 *
 *  @param(timeout) Number of ticks to wait for the character to be read by the
 *                  host 
 *
 *  @return         If there are no errors, the same character sent will be
 *                  returned. Else it returns 0.
 */
extern Int USBKBD_putChar(Int ch, UInt timeout);

/*!
 *  ======== USBKBD_putString ========
 *  A blocking function that sends a NULL terminated string to the host.
 *
 *  Function to simulates a keyboard press and release sequence for an array of
 *  characters to the host.
 *
 *  @param(chArray) The printable character that will be sent to the host
 *
 *  @param(timeout) Number of ticks to wait for each character in the array to
 *                  be read by the host 
 *
 *  @return         Returns the number of characters that have been sent
 *                  successfully.
 */
extern Int USBKBD_putString(String chArray, UInt length, UInt timeout);

/*!
 *  ======== USBKBD_waitForConnect ========
 *  This function blocks while the USB is not connected
 */
extern Bool USBKBD_waitForConnect(UInt timeout);

#ifdef __cplusplus
}
#endif

#endif /* USBKBD_H_ */
