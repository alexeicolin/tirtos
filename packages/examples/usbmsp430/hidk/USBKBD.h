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
 *  ======== USBKBD.h ========
 */

#ifndef USBKBD_H_
#define USBKBD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <xdc/std.h>
#include "USB_config/descriptors.h"

/*
 * This needs to be updated if there are more than one HID class instances
 * defined in descriptors.h
 */
#define KB_INTFNUM                HID0_INTFNUM

#define KEY_LEFT_CTRL           0x80
#define KEY_LEFT_SHIFT          0x81
#define KEY_LEFT_ALT            0x82
#define KEY_LEFT_GUI            0x83
#define KEY_RIGHT_CTRL          0x84
#define KEY_RIGHT_SHIFT         0x85
#define KEY_RIGHT_ALT           0x86
#define KEY_RIGHT_GUI           0x87
#define KEY_UP_ARROW            0xDA
#define KEY_DOWN_ARROW          0xD9
#define KEY_LEFT_ARROW          0xD8
#define KEY_RIGHT_ARROW         0xD7
#define KEY_BACKSPACE           0xB2
#define KEY_TAB                 0xB3
#define KEY_RETURN              0xB0
#define KEY_ESC                 0xB1
#define KEY_INSERT              0xD1
#define KEY_DELETE              0xD4
#define KEY_PAGE_UP             0xD3
#define KEY_PAGE_DOWN           0xD6
#define KEY_HOME                0xD2
#define KEY_END                 0xD5
#define KEY_CAPS_LOCK           0xC1
#define KEY_F1                  0xC2
#define KEY_F2                  0xC3
#define KEY_F3                  0xC4
#define KEY_F4                  0xC5
#define KEY_F5                  0xC6
#define KEY_F6                  0xC7
#define KEY_F7                  0xC8
#define KEY_F8                  0xC9
#define KEY_F9                  0xCA
#define KEY_F10                 0xCB
#define KEY_F11                 0xCC
#define KEY_F12                 0xCD

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
