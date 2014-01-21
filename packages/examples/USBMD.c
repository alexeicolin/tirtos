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
 *  ======== USBMD.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/gates/GateMutex.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <stdbool.h>
#include <stdint.h>

/* driverlib Header files */
#include <inc/hw_ints.h>
#include <inc/hw_types.h>

/* usblib Header files */
#include <usblib/usb-ids.h>
#include <usblib/usblib.h>
#include <usblib/usbhid.h>
#include <usblib/device/usbdevice.h>
#include <usblib/device/usbdhid.h>
#include <usblib/device/usbdhidmouse.h>

/* Example/Board Header files */
#include "USBMD.h"

#if defined(TIVAWARE)
/* c99 types required by TivaWare */
#include <stdint.h>
typedef uint32_t                USBMDEventType;
#else
#define eUSBModeForceDevice USB_MODE_FORCE_DEVICE
typedef ULong               USBMDEventType;
#endif

/* Typedefs */
typedef volatile enum {
    USBMD_STATE_IDLE = 0,
    USBMD_STATE_SENDING,
    USBMD_STATE_UNCONFIGURED
} USBMD_USBState;

/* Static variables and handles */
static volatile USBMD_USBState state;
static GateMutex_Handle gateMouse;
static GateMutex_Handle gateUSBWait;
static Semaphore_Handle semMouse;
static Semaphore_Handle semUSBConnected;

/* Function prototypes */
static USBMDEventType cbMouseHandler(Void *cbData, USBMDEventType event,
                                     USBMDEventType eventMsg,
                                     Void *eventMsgPtr);
static Void USBMD_hwiHandler(UArg arg0);
static Int sendState(USBMD_State *mouseState, UInt timeout);
static Bool waitUntilSent(UInt timeout);
Void USBMD_init(Void);
UInt USBMD_setState(USBMD_State *mouseState, UInt timeout);
Bool USBMD_waitForConnect(UInt timeout);

/* The languages supported by this device. */
const UChar langDescriptor[] =
{
    4,
    USB_DTYPE_STRING,
    USBShort(USB_LANG_EN_US)
};

/* The manufacturer string. */
const UChar manufacturerString[] =
{
    (17 + 1) * 2,
    USB_DTYPE_STRING,
    'T', 0, 'e', 0, 'x', 0, 'a', 0, 's', 0, ' ', 0, 'I', 0, 'n', 0, 's', 0,
    't', 0, 'r', 0, 'u', 0, 'm', 0, 'e', 0, 'n', 0, 't', 0, 's', 0,
};

/* The product string. */
const UChar productString[] =
{
    (13 + 1) * 2,
    USB_DTYPE_STRING,
    'M', 0, 'o', 0, 'u', 0, 's', 0, 'e', 0, ' ', 0, 'E', 0, 'x', 0, 'a', 0,
    'm', 0, 'p', 0, 'l', 0, 'e', 0
};

/* The serial number string. */
const UChar serialNumberString[] =
{
    (8 + 1) * 2,
    USB_DTYPE_STRING,
    '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0
};

/* The interface description string. */
const UChar hidInterfaceString[] =
{
    (19 + 1) * 2,
    USB_DTYPE_STRING,
    'H', 0, 'I', 0, 'D', 0, ' ', 0, 'M', 0, 'o', 0, 'u', 0, 's', 0,
    'e', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0,
    'a', 0, 'c', 0, 'e', 0
};

/* The configuration description string. */
const UChar configString[] =
{
    (23 + 1) * 2,
    USB_DTYPE_STRING,
    'H', 0, 'I', 0, 'D', 0, ' ', 0, 'M', 0, 'o', 0, 'u', 0, 's', 0,
    'e', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 'f', 0, 'i', 0, 'g', 0,
    'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0, 'o', 0, 'n', 0
};

/* The descriptor string table. */
const UChar * const stringDescriptors[] =
{
    langDescriptor,
    manufacturerString,
    productString,
    serialNumberString,
    hidInterfaceString,
    configString
};

#define STRINGDESCRIPTORSCOUNT (sizeof(stringDescriptors) / \
                                sizeof(UChar *))

#if defined(TIVAWARE)
static tUSBDHIDMouseDevice mouseDevice =
{
    USB_VID_TI_1CBE,
    USB_PID_MOUSE,
    500,
    USB_CONF_ATTR_SELF_PWR,
    cbMouseHandler,
    NULL,
    stringDescriptors,
    STRINGDESCRIPTORSCOUNT
};
#else  /* MWARE */
static tHIDMouseInstance mouseInstance;
const tUSBDHIDMouseDevice mouseDevice =
{
    USB_VID_TI,
    USB_PID_MOUSE,
    500,
    USB_CONF_ATTR_SELF_PWR,
    cbMouseHandler,
    NULL,
    stringDescriptors,
    STRINGDESCRIPTORSCOUNT,
    &mouseInstance /* Old usblib stores a pointer */
};
#endif

/*
 *  ======== cbMouseHandler ========
 *  Callback handler for the USB stack.
 *
 *  Callback handler call by the USB stack to notify us on what has happened in
 *  regards to the keyboard.
 *
 *  @param(cbData)          A callback pointer provided by the client.
 *
 *  @param(event)           Identifies the event that occurred in regards to
 *                          this device.
 *
 *  @param(eventMsgData)    A data value associated with a particular event.
 *
 *  @param(eventMsgPtr)     A data pointer associated with a particular event.
 *
 */
static USBMDEventType cbMouseHandler (Void *cbData, USBMDEventType event,
                                      USBMDEventType eventMsg,
                                      Void *eventMsgPtr)
{
    /* Determine what event has happened */
    switch (event) {
        case USB_EVENT_CONNECTED:
            state = USBMD_STATE_IDLE;
            Semaphore_post(semUSBConnected);
            break;

        case USB_EVENT_DISCONNECTED:
            if (state == USBMD_STATE_SENDING) {
                state = USBMD_STATE_UNCONFIGURED;
                Semaphore_post(semMouse);
            }
            else {
                state = USBMD_STATE_UNCONFIGURED;
            }
            break;

        case USB_EVENT_TX_COMPLETE:
            state = USBMD_STATE_IDLE;
            Semaphore_post(semMouse);
            break;

        default:
            break;
    }

    return (0);
}

/*
 *  ======== USBMD_hwiHandler ========
 *  This function calls the USB library's device interrupt handler.
 */
static Void USBMD_hwiHandler(UArg arg0)
{
    USB0DeviceIntHandler();
}

/*
 *  ======== sendState ========
 */
static Int sendState(USBMD_State *mouseState, UInt timeout)
{
    UInt key;
    Int retValue;
    UChar buttons = 0;

    /* Set the bit packed button values */
    buttons |= (mouseState->button1) ? HID_MOUSE_BUTTON_1 : 0;
    buttons |= (mouseState->button2) ? HID_MOUSE_BUTTON_2 : 0;
    buttons |= (mouseState->button3) ? HID_MOUSE_BUTTON_3 : 0;

    /* Acquire lock */
    key = GateMutex_enter(gateMouse);

    state = USBMD_STATE_SENDING;
    retValue = USBDHIDMouseStateChange((tUSBDHIDMouseDevice *)&mouseDevice,
                                        mouseState->deltaX,
                                        mouseState->deltaY,
                                        buttons);

    retValue = (retValue) ? retValue : !waitUntilSent(timeout);

    /* Release lock */
    GateMutex_leave(gateMouse, key);

    return (retValue);
}

/*
 *  ======== waitUntilSent ========
 *  Function will determine if the last key press/release was sent to the host
 *
 *  @return             0: Assume that there was an error (likely a disconnect)
 *                      1: Successful
 */
static Bool waitUntilSent(UInt timeout)
{

    while (state == USBMD_STATE_SENDING) {
        if (!Semaphore_pend(semMouse, timeout)) {
            state = USBMD_STATE_UNCONFIGURED;
            return (FALSE);
        }
    }

    return (TRUE);
}

/*
 *  ======== USBMD_init ========
 */
Void USBMD_init(Void)
{
    Hwi_Handle hwi;
    Error_Block eb;
    Semaphore_Params semParams;

    Error_init(&eb);

    /* Install interrupt handler */
    hwi = Hwi_create(INT_USB0, USBMD_hwiHandler, NULL, &eb);
    if (hwi == NULL) {
        System_abort("Can't create USB Hwi");
    }

    /* RTOS primitives */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    semMouse = Semaphore_create(0, &semParams, &eb);
    if (semMouse == NULL) {
        System_abort("Can't create mouse semaphore");
    }

    semUSBConnected = Semaphore_create(0, &semParams, &eb);
    if (semUSBConnected == NULL) {
        System_abort("Can't create USB semaphore");
    }

    gateMouse = GateMutex_create(NULL, &eb);
    if (gateMouse == NULL) {
        System_abort("Can't create mouse gate");
    }

    gateUSBWait = GateMutex_create(NULL, &eb);
    if (gateUSBWait == NULL) {
        System_abort("Could not create USB Wait gate");
    }

    /* State specific variables */
    state = USBMD_STATE_UNCONFIGURED;

    /* Set the USB stack mode to Device mode with VBUS monitoring */
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    /*
     * Pass our device information to the USB HID device class driver,
     * initialize the USB controller and connect the device to the bus.
     */
    if (!USBDHIDMouseInit(0, &mouseDevice)) {
        System_abort("Error initializing the mouse");
    }

}

/*
 *  ======== USBMD_setState ========
 */
UInt USBMD_setState(USBMD_State *mouseState, UInt timeout)
{
    UInt retValue = 0;

    if (state == USBMD_STATE_IDLE) {
        retValue = sendState(mouseState, timeout);
    }

    return (retValue);
}

/*
 *  ======== USBMD_waitForConnect ========
 */
Bool USBMD_waitForConnect(UInt timeout)
{
    Bool ret = TRUE;
    UInt key;

    /* Need exclusive access to prevent a race condition */
    key = GateMutex_enter(gateUSBWait);

    if (state == USBMD_STATE_UNCONFIGURED) {
        if (!Semaphore_pend(semUSBConnected, timeout)) {
            ret = FALSE;
        }
    }

    GateMutex_leave(gateUSBWait, key);

    return (ret);
}

