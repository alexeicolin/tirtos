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
 *  ======== USBMH.c ========
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
#include <ti/sysbios/knl/Task.h>

#include <stdbool.h>
#include <stdint.h>

/* driverlib Header files */
#include <inc/hw_ints.h>
#include <inc/hw_types.h>

/* usblib Header files */
#include <usblib/usb-ids.h>
#include <usblib/usblib.h>
#include <usblib/usbhid.h>
#include <usblib/host/usbhost.h>
#include <usblib/host/usbhhid.h>
#include <usblib/host/usbhhidmouse.h>

/* Example/Board Header files */
#include "USBMH.h"

#if defined(TIVAWARE)
/* c99 types required by TivaWare */
#include <stdint.h>
typedef tUSBHMouse             *USBMHType;
typedef uint32_t                USBMHEventType;
typedef Void                    USBMHHandleType;
#else /* MWARE */
#define g_sUSBHIDClassDriver    g_USBHIDClassDriver
#define ui32Event               ulEvent
#define eUSBModeHost            USB_MODE_HOST
typedef ULong                   USBMHType;
typedef ULong                   USBMHEventType;
typedef ULong                   USBMHHandleType;
#endif

/* Defines */
#define HCDMEMORYPOOLSIZE   128 /* Memory for the Host Class Driver */
#define MMEMORYPOOLSIZE     128 /* Memory for the mouse host driver */

/* Typedefs */
typedef volatile enum {
    USBMH_NO_DEVICE = 0,
    USBMH_INIT,
    USBMH_CONNECTED,
    USBMH_UNKNOWN,
    USBMH_POWER_FAULT
} USBMH_USBState;

/* Static variables and handles */
static volatile USBMH_USBState state;
static UChar            memPoolHCD[HCDMEMORYPOOLSIZE];
static UChar            memPoolM[MMEMORYPOOLSIZE];
static volatile Char    xPosition;
static volatile Char    yPosition;
static volatile UChar   mouseButtons;
static USBMHType        mouseInstance;
static GateMutex_Handle gateUSBWait;
static GateMutex_Handle gateUSBLibAccess;
static Semaphore_Handle semUSBConnected;

/* Function prototypes */
#if defined(TIVAWARE)
static USBMHHandleType cbMouseHandler(USBMHType instance, USBMHEventType event,
                                      USBMHEventType eventMsg, Void *eventMsgPtr);
#else /* MWARE */
static USBMHHandleType cbMouseHandler(Void *instance, USBMHEventType event,
		                              USBMHEventType eventMsg, Void *eventMsgPtr);
#endif
static Void USBMH_hwiHandler(UArg arg0);
static Void serviceUSBHost(UArg arg0, UArg arg1);
static Void usbHCDEvents(Void *cbData);
Void USBMH_getState(USBMH_State *mouseState);
Void USBMH_init(Void);
Bool USBMH_waitForConnect(UInt timeout);

/* MACRO to create a generic USB event host driver */
DECLARE_EVENT_DRIVER(USBMH_eventDriver, 0, 0, usbHCDEvents);

/* A list of available Host Class Drivers */
static tUSBHostClassDriver const * const usbHCDDriverList[] = {
    &g_sUSBHIDClassDriver,
    &USBMH_eventDriver
};

/* Variable containing the number of HCDs in usbHCDDriverList */
const ULong numHostClassDrivers =
    sizeof(usbHCDDriverList) / sizeof(tUSBHostClassDriver *);

/*
 *  ======== cbMouseHandler ========
 *  Callback handler for the USB stack.
 *
 *  Callback handler call by the USB stack to notify us on what has happened in
 *  regards to the mouse.
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
#if defined(TIVAWARE)
static USBMHHandleType cbMouseHandler(USBMHType instance, USBMHEventType event,
		                              USBMHEventType eventMsg, Void *eventMsgPtr)
#else /* MWARE */
static USBMHHandleType cbMouseHandler(Void *instance, USBMHEventType event,
		                              USBMHEventType eventMsg, Void *eventMsgPtr)
#endif
{
    /* Determine what event has happened */
    switch (event) {
        case USB_EVENT_CONNECTED:
            /* Set the mouse state for initialization */
            state = USBMH_INIT;
            Semaphore_post(semUSBConnected);
            break;

        case USB_EVENT_DISCONNECTED:
            /* Set the mouse state as not connected */
            state = USBMH_NO_DEVICE;
            break;

        case USBH_EVENT_HID_MS_PRESS:
            /* Set the button states */
            mouseButtons |= eventMsg;
            break;

        case USBH_EVENT_HID_MS_REL:
            /* Clear the button states */
            mouseButtons &= ~eventMsg;
            break;

        case USBH_EVENT_HID_MS_X:
            /* Update the variable containing the X coordinate */
            xPosition += (UChar)eventMsg;
            break;

        case USBH_EVENT_HID_MS_Y:
            /* Update the variable containing the Y coordinate */
            yPosition += (UChar)eventMsg;
            break;

        default:
            break;
    }
#if defined(MWARE)
    return (0);
#endif
}

/*
 *  ======== USBMH_hwiHandler ========
 *  This function calls the USB library's device interrupt handler.
 */
static Void USBMH_hwiHandler(UArg arg0)
{
    USB0HostIntHandler();
}

/*
 *  ======== serviceUSBHost ========
 *  Task to periodically service the USB Stack
 *
 *  USBHCDMain handles the USB Stack's state machine. For example it handles the
 *  enumeration process when a device connects.
 *  Future USB library improvement goal is to remove this polling requirement..
 */
static Void serviceUSBHost(UArg arg0, UArg arg1)
{
    UInt key;

    while (TRUE) {
        key = GateMutex_enter(gateUSBLibAccess);
        USBHCDMain();
        GateMutex_leave(gateUSBLibAccess, key);

        /* Future enhancement to remove the Task_sleep */
        Task_sleep(10);
    }
}

/*
 *  ======== usbHCDEvents ========
 *  Generic USB Host Class Driver event callback.
 *
 *  This callback is called to notify the application that a unknown device was
 *  connected. (e.g. It wasn't a mouse)
 */
static Void usbHCDEvents(Void *cbData)
{
    tEventInfo *pEventInfo;

    /* Cast this pointer to its actual type. */
    pEventInfo = (tEventInfo *)cbData;

    switch (pEventInfo->ui32Event) {

        case USB_EVENT_UNKNOWN_CONNECTED:
            /* An unknown device was detected. */
            state = USBMH_UNKNOWN;
            break;

        case USB_EVENT_DISCONNECTED:
            /* Unknown device has been removed. */
            state = USBMH_NO_DEVICE;
            break;

        case USB_EVENT_POWER_FAULT:
            /* No power means no device is present. */
            state = USBMH_POWER_FAULT;
            break;

        default:
            break;
    }
}

/*
 *  ======== USBMH_getState ========
 */
Void USBMH_getState(USBMH_State *mouseState)
{
    UInt key;

    switch (state) {
        case USBMH_NO_DEVICE:
            break;

        case USBMH_INIT:
            state = USBMH_CONNECTED;

            /* Acquire lock */
            key = GateMutex_enter(gateUSBLibAccess);

            /* Reset global variables */
            xPosition = 0x00;
            yPosition = 0x00;
            mouseButtons = 0x00;

            USBHMouseInit(mouseInstance);

            /* Release lock */
            GateMutex_leave(gateUSBLibAccess, key);

        default:
            break;
    }

    /* Return current status; regardless of the state */
    key = Hwi_disable();
    mouseState->deltaX = xPosition;
    mouseState->deltaY = yPosition;
    mouseState->button1 = (mouseButtons & HID_MOUSE_BUTTON_1) ? TRUE : FALSE;
    mouseState->button2 = (mouseButtons & HID_MOUSE_BUTTON_2) ? TRUE : FALSE;
    mouseState->button3 = (mouseButtons & HID_MOUSE_BUTTON_3) ? TRUE : FALSE;
    Hwi_restore(key);
}

/*
 *  ======== USBMH_init ========
 */
Void USBMH_init(Void)
{
    Hwi_Handle hwi;
    Error_Block eb;
    Task_Handle task;
    Task_Params taskParams;
    Semaphore_Params semParams;

    Error_init(&eb);

    /* Initialize the USB stack for host mode. */
    USBStackModeSet(0, eUSBModeHost, NULL);

    /* Register host class drivers */
    USBHCDRegisterDrivers(0, usbHCDDriverList, numHostClassDrivers);

    /* Open an instance of the mouse host driver */
    mouseInstance = USBHMouseOpen(cbMouseHandler, memPoolM, MMEMORYPOOLSIZE);
    if(!mouseInstance) {
        System_abort("Error initializing the Mouse Host");
    }

    /* Install interrupt handler */
    hwi = Hwi_create(INT_USB0, USBMH_hwiHandler, NULL, &eb);
    if (hwi == NULL) {
        System_abort("Can't create USB Hwi");
    }

    /* Initialize USB power configuration */
    USBHCDPowerConfigInit(0, USBHCD_VBUS_AUTO_HIGH | USBHCD_VBUS_FILTER);

    /* Enable the USB stack */
    USBHCDInit(0, memPoolHCD, HCDMEMORYPOOLSIZE);

    /* RTOS primitives */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    semUSBConnected = Semaphore_create(0, &semParams, &eb);
    if (semUSBConnected == NULL) {
        System_abort("Could not create USB Connect semaphore");
    }

    gateUSBLibAccess = GateMutex_create(NULL, &eb);
    if (gateUSBLibAccess == NULL) {
        System_abort("Could not create USB Wait gate");
    }

    gateUSBWait = GateMutex_create(NULL, &eb);
    if (gateUSBWait == NULL) {
        System_abort("Could not create USB Wait gate");
    }

    /*
     * Note that serviceUSBHost() should not be run until the USB Stack has been
     * initialized!!
     */
    Task_Params_init(&taskParams);
    taskParams.priority = Task_numPriorities - 1;
    taskParams.stackSize = 768;
    task = Task_create(serviceUSBHost, &taskParams, &eb);
    if (task == NULL) {
        System_abort("Can't create USB service task");
    }

}

/*
 *  ======== USBMH_waitForConnect ========
 */
Bool USBMH_waitForConnect(UInt timeout)
{
    Bool ret = TRUE;
    UInt key;

    /* Need exclusive access to prevent a race condition */
    key = GateMutex_enter(gateUSBWait);

    if (state == USBMH_NO_DEVICE) {
        if (!Semaphore_pend(semUSBConnected, timeout)) {
            ret = FALSE;
        }
    }

    GateMutex_leave(gateUSBWait, key);

    return (ret);
}
