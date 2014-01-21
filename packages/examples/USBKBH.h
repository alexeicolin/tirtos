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
 *  ======== USBKBH.h ========
 */

#ifndef USBKBH_H_
#define USBKBH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <xdc/std.h>

/* Data structure used to specify the current state of the LEDs */
typedef struct {
    Bool numLED;
    Bool capsLED;
    Bool scrollLED;
} USBKBH_State;

/*!
 *  ======== USBKBH_getChar ========
 *  Function blocks for the specified timeout ticks for a character.
 *
 *  @param(timeout) Parameter corresponds to the number of ticks to wait for a
 *                  character.
 *
 *  @return         Returns the character received from a key press.
 *                  0 is returned if a timeout event has occurred of if the
 *                  keyboard is not connected.
 */
extern Int USBKBH_getChar(UInt timeout);

/*!
 *  ======== USBKBH_getState ========
 *  Function is a NON-blocking function that returns the status of the keyboard.
 *
 *  Function returns the status of the keyboard by returning a bit-packed
 *  variable. See Below.
 *
 *  @return         A bit packed value; each corresponding to a particular LED
 *                  Bit 0: 0 - Disconnected; 1 - Connected
 *                  Bit 1: 0 - NUMLOCKLED clr; 1 - NUMLOCKLED set
 *                  Bit 2: 0 - CAPSLOCKLED clr; 1 - CAPSLOCKLED set
 *                  Bit 3: 0 - SCROLLLOCKLED clr; 1 - SCROLLLOCKLED set
 */
extern Void USBKBH_getState(USBKBH_State *keyboardState);

/*!
 *  ======== USBKBH_putString ========
 *  Function blocks indefinitely until a <LF> (return) key was pressed.
 *
 *  @param(chArray) Pointer to a String array to which data will be stored in
 *                  The caller of this function needs to ensure a large enough
 *                  buffer.
 *                  The buffer will be NULL terminated rather containing a <LF>
 *
 *  @return         Number of characters received.
 */
extern Int USBKBH_getString(String chArray, UInt length, UInt timeout);

/*!
 *  ======== USBKBH_init ========
 *  Function to initialize the USB keyboard reference module.
 *
 *  Note: This function is not reentrant safe.
 */
extern Void USBKBH_init(Void);

/*!
 *  ======== USBKBH_setState ========
 *  Function sends the keyboard a status update turn on the specified LEDs
 *
 *  @param(status)  A bit packed value; that can be determined from _getState()
 *                  Bit 0 Is ignored.
 */
extern Void USBKBH_setState(USBKBH_State *keyboardState);

/*!
 *  ======== USBKBH_waitForConnect ========
 *  This function blocks while the USB is not connected
 */
extern Bool USBKBH_waitForConnect(UInt timeout);

#ifdef __cplusplus
}
#endif

#endif /* USBKBH_H_ */
