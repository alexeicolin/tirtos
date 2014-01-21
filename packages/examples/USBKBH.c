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
 *  ======== USBKBH.c ========
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
#include <usblib/host/usbhhidkeyboard.h>

/* Example/Board Header files */
#include "USBKBH.h"

#if defined(TIVAWARE)
/* c99 types required by TivaWare */
#include <stdint.h>
typedef tUSBHKeyboard          *USBKBHType;
typedef uint32_t                USBKBHEventType;
typedef Void                    USBKBHHandleType;
#else /* MWARE */
#define g_sUSBHIDClassDriver    g_USBHIDClassDriver
#define ui32Event               ulEvent
#define eUSBModeHost            USB_MODE_HOST
typedef ULong                   USBKBHType;
typedef ULong                   USBKBHEventType;
typedef ULong                   USBKBHHandleType;
#endif

/* Defines */
#define HCDMEMORYPOOLSIZE   128 /* Memory for the Host Class Driver */
#define KBMEMORYPOOLSIZE    128 /* Memory for the keyboard host driver */

/* Typedefs */
typedef volatile enum {
    USBKBH_NO_DEVICE = 0,
    USBKBH_INIT,
    USBKBH_CONNECTED,
    USBKBH_UNKNOWN,
    USBKBH_POWER_FAULT
} USBKBH_USBState;

/* Static variables and handles */
static volatile USBKBH_USBState state;
static UChar            memPoolHCD[HCDMEMORYPOOLSIZE];
static UChar            memPoolKB[KBMEMORYPOOLSIZE];
static volatile Int     kbCh;
static volatile UChar   kbLEDs;
static USBKBHType       keyboardInstance;
static GateMutex_Handle gateUSBLibAccess;
static GateMutex_Handle gateUSBWait;
static Semaphore_Handle semKeyboard;
static Semaphore_Handle semUSBConnected;

/* External Host keyboard map provided with the USB library */
extern const tHIDKeyboardUsageTable g_sUSKeyboardMap;

/* Function prototypes */
#if defined(TIVAWARE)
static USBKBHHandleType cbKeyboardHandler(USBKBHType instance, USBKBHEventType event,
                                          USBKBHEventType eventMsg, Void *eventMsgPtr);
#else /* MWARE */
static USBKBHHandleType cbKeyboardHandler(Void *instance, USBKBHEventType event,
                                          USBKBHEventType eventMsg, Void *eventMsgPtr);
#endif
static Int getC(UInt time);
static Void USBKBH_hwiHandler(UArg arg0);
static Void serviceUSBHost(UArg arg0, UArg arg1);
static Void usbHCDEvents(Void *cbData);
Int USBKBH_getChar(UInt timeout);
Void USBKBH_getState(USBKBH_State *keyboardState);
Int USBKBH_getString(String chArray, UInt length, UInt timeout);
Void USBKBH_init(Void);
Void USBKBH_setState(USBKBH_State *keyboardState);
Bool USBKBH_waitForConnect(UInt timeout);

/* MACRO to create a generic USB event host driver */
DECLARE_EVENT_DRIVER(USBKBH_eventDriver, 0, 0, usbHCDEvents);

/* A list of available Host Class Drivers */
static tUSBHostClassDriver const * const usbHCDDriverList[] = {
    &g_sUSBHIDClassDriver,
    &USBKBH_eventDriver
};

/* Variable containing the number of HCDs in usbHCDDriverList */
const ULong numHostClassDrivers =
    sizeof(usbHCDDriverList) / sizeof(tUSBHostClassDriver *);

/*
 *  ======== cbKeyboardHandler ========
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
#if defined(TIVAWARE)
static USBKBHHandleType cbKeyboardHandler(USBKBHType instance, USBKBHEventType event,
                                          USBKBHEventType eventMsg, Void *eventMsgPtr)
#else /* MWARE */
static USBKBHHandleType cbKeyboardHandler(Void *instance, USBKBHEventType event,
                                          USBKBHEventType eventMsg, Void *eventMsgPtr)
#endif
{
    /* Determine what event has happened */
    switch (event) {
        case USB_EVENT_CONNECTED:
            /* Set the keyboard state for initialization */
            state = USBKBH_INIT;
            Semaphore_post(semUSBConnected);
            break;

        case USB_EVENT_DISCONNECTED:
            /* Set the keyboard state as not connected */
            state = USBKBH_NO_DEVICE;
            break;

        case USBH_EVENT_HID_KB_PRESS:
            /* A key was pressed, determine which one */
            switch (eventMsg) {
                case HID_KEYB_USAGE_CAPSLOCK:
                    /* Toggle CAPLOCK LED */
                    kbLEDs ^= HID_KEYB_CAPS_LOCK;
                    break;

                /* SCROLL LOCK is not defined in the USB library */
                case 0x47:
                    kbLEDs ^= HID_KEYB_SCROLL_LOCK;
                    break;

                /* NUM Lock is not defined in the USB library */
                case 0x53:
                    kbLEDs ^= HID_KEYB_NUM_LOCK;
                    break;

                case HID_KEYB_USAGE_BACKSPACE:
                    kbCh = '\b';
                    Semaphore_post(semKeyboard);
                    break;

                /* Else, it's a character */
                default:
                    kbCh = USBHKeyboardUsageToChar(
                            keyboardInstance,
                            &g_sUSKeyboardMap,
                            (UChar) eventMsg);

                    Semaphore_post(semKeyboard);
                    break;
            }
            break;

        case USBH_EVENT_HID_KB_MOD:
            break;

        case USBH_EVENT_HID_KB_REL:
            break;

        default:
            break;
    }
#if defined(MWARE)
    return (0);
#endif
}

/*
 *  ======== getC ========
 */
static Int getC(UInt time)
{
    /* Wait for the callback handler to post the semaphore */
    if (!Semaphore_pend(semKeyboard, time)) {
        return (0);
    }
    else {
        return (kbCh);
    }
}

/*
 *  ======== USBKBH_hwiHandler ========
 *  This function calls the USB library's device interrupt handler.
 */
static Void USBKBH_hwiHandler(UArg arg0)
{
    USB0HostIntHandler();
}

/*
 *  ======== serviceUSBHost ========
 *  Task to periodically service the USB Stack
 *
 *  USBHCDMain handles the USB Stack's statemachine. For example it handles the
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
 *  connected. (e.g. It wasn't a keyboard)
 */
static Void usbHCDEvents(Void *cbData)
{
    tEventInfo *pEventInfo;

    /* Cast this pointer to its actual type. */
    pEventInfo = (tEventInfo *)cbData;

    switch (pEventInfo->ui32Event) {

        case USB_EVENT_UNKNOWN_CONNECTED:
            /* An unknown device was detected. */
            state = USBKBH_UNKNOWN;
            break;

        case USB_EVENT_DISCONNECTED:
            /* Unknown device has been removed. */
            state = USBKBH_NO_DEVICE;
            break;

        case USB_EVENT_POWER_FAULT:
            /* No power means no device is present. */
            state = USBKBH_POWER_FAULT;
            break;

        default:
            break;
    }
}

/*
 *  ======== USBKBH_getChar ========
 */
Int USBKBH_getChar(UInt timeout)
{
    UInt key;
    Int ch = 0;

    switch (state) {
        case USBKBH_NO_DEVICE:
            USBKBH_waitForConnect(timeout);
            break;

        case USBKBH_INIT:
            key = GateMutex_enter(gateUSBLibAccess);
            USBHKeyboardInit(keyboardInstance);
            GateMutex_leave(gateUSBLibAccess, key);
            ch = getC(timeout);

            state = USBKBH_CONNECTED;

            break;

        case USBKBH_CONNECTED:
            ch = getC(timeout);
            break;

        default:
            break;
    }

    return (ch);
}

/*
 *  ======== USBKBH_getState ========
 */
Void USBKBH_getState(USBKBH_State *keyboardState)
{
    UInt key;

    key = Hwi_disable();
    keyboardState->numLED = (kbLEDs & HID_KEYB_NUM_LOCK) ? TRUE : FALSE;
    keyboardState->capsLED = (kbLEDs & HID_KEYB_CAPS_LOCK) ? TRUE : FALSE;
    keyboardState->scrollLED = (kbLEDs & HID_KEYB_SCROLL_LOCK) ? TRUE : FALSE;
    Hwi_restore(key);
}

/*
 *  ======== USBKBH_getString ========
 */
Int USBKBH_getString(String chArray, UInt length, UInt timeout)
{
    Int ch;
    Int count = 0;

    if (!length) {
        return (0);
    }

    do {
        ch = USBKBH_getChar(timeout);

        if (ch == '\n') {
            ch = 0;
        }
        else if (ch == '\b') {
            if (count) {
                count--;
            }
            continue;
        }
        chArray[count] = ch;
        count++;

    } while ((ch != 0) && ((count - 1) < length));

    return (count - 1);
}

/*
 *  ======== USBKBH_init ========
 */
Void USBKBH_init(Void)
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

    /* Open an instance of the keyboard host driver */
    keyboardInstance = USBHKeyboardOpen(cbKeyboardHandler, memPoolKB, KBMEMORYPOOLSIZE);
    if(!keyboardInstance) {
        System_abort("Error initializing the Keyboard Host");
    }


    /* Install interrupt handler */
    hwi = Hwi_create(INT_USB0, USBKBH_hwiHandler, NULL, &eb);
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
    semKeyboard = Semaphore_create(0, &semParams, &eb);
    if (semKeyboard == NULL) {
        System_abort("Can't create keyboard semaphore");
    }

    semUSBConnected = Semaphore_create(0, &semParams, &eb);
    if (semUSBConnected == NULL) {
        System_abort("Can't create USB semaphore");
    }

    gateUSBLibAccess = GateMutex_create(NULL, &eb);
    if (gateUSBLibAccess == NULL) {
        System_abort("Can't create keyboard gate");
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

    /* State specific variables */
    kbLEDs = 0x00;
    kbCh = 0x00;

}

/*
 *  ======== USBKBH_setState ========
 */
Void USBKBH_setState(USBKBH_State *keyboardState)
{
    UInt key;
    UChar leds = 0;

    if (state == USBKBH_CONNECTED) {
        /* Set the bit packed LED value */
        leds |= (keyboardState->numLED) ? HID_KEYB_NUM_LOCK : 0;
        leds |= (keyboardState->capsLED) ? HID_KEYB_CAPS_LOCK : 0;
        leds |= (keyboardState->scrollLED) ? HID_KEYB_SCROLL_LOCK : 0;

        key = GateMutex_enter(gateUSBLibAccess);
        USBHKeyboardModifierSet(keyboardInstance, leds);
        GateMutex_leave(gateUSBLibAccess, key);
    }
}

/*
 *  ======== USBKBH_waitForConnect ========
 */
Bool USBKBH_waitForConnect(UInt timeout)
{
    Bool ret = TRUE;
    UInt key;

    /* Need exclusive access to prevent a race condition */
    key = GateMutex_enter(gateUSBWait);

    if (state == USBKBH_NO_DEVICE) {
        if (!Semaphore_pend(semUSBConnected, timeout)) {
            ret = FALSE;
        }
    }

    GateMutex_leave(gateUSBWait, key);

    return (ret);
}
