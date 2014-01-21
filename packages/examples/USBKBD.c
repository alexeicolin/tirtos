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
 *  ======== USBKBD.c ========
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
#include <usblib/device/usbdhidkeyb.h>

/* Example/Board Header files */
#include "USBKBD.h"

#if defined(TIVAWARE)
/* c99 types required by TivaWare */
#include <stdint.h>
typedef uint32_t            USBKBDEventType;
#else
#define eUSBModeForceDevice USB_MODE_FORCE_DEVICE
typedef ULong               USBKBDEventType;
#endif

/* Typedefs */
typedef volatile enum {
    USBKBD_STATE_IDLE = 0,
    USBKBD_STATE_SUSPENDED,
    USBKBD_STATE_SENDING,
    USBKBD_STATE_UNCONFIGURED
} USBKBD_USBState;

/* Static variables and handles */
static volatile USBKBD_USBState state;
static volatile UChar kbLEDs;
static GateMutex_Handle gateKeyboard;
static GateMutex_Handle gateUSBWait;
static Semaphore_Handle semKeyboard;
static Semaphore_Handle semUSBConnected;

/* Function prototypes */
static USBKBDEventType cbKeyboardHandler(Void *cbData, USBKBDEventType event,
		                                 USBKBDEventType eventMsg,
                                         Void *eventMsgPtr);
static Void USBKBD_hwiHandler(UArg arg0);
static Int sendChar(Int ch, UInt timeout);
static Bool waitUntilSent(UInt timeout);
Void USBKBD_getState(USBKBD_State *keyboardState);
Void USBKBD_init(Void);
Int USBKBD_putChar(Int ch, UInt timeout);
Int USBKBD_putString(String chArray, UInt length, UInt timeout);
Bool USBKBD_waitForConnect(UInt timeout);

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
    (16 + 1) * 2,
    USB_DTYPE_STRING,
    'K', 0, 'e', 0, 'y', 0, 'b', 0, 'o', 0, 'a', 0, 'r', 0, 'd', 0, ' ', 0,
    'E', 0, 'x', 0, 'a', 0, 'm', 0, 'p', 0, 'l', 0, 'e', 0
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
    (22 + 1) * 2,
    USB_DTYPE_STRING,
    'H', 0, 'I', 0, 'D', 0, ' ', 0, 'K', 0, 'e', 0, 'y', 0, 'b', 0, 'o', 0,
    'a', 0, 'r', 0, 'd', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0,
    'f', 0, 'a', 0, 'c', 0, 'e', 0
};

/* The configuration description string. */
const UChar configString[] =
{
    (26 + 1) * 2,
    USB_DTYPE_STRING,
    'H', 0, 'I', 0, 'D', 0, ' ', 0, 'K', 0, 'e', 0, 'y', 0, 'b', 0, 'o', 0,
    'a', 0, 'r', 0, 'd', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 'f', 0, 'i', 0,
    'g', 0, 'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0, 'o', 0, 'n', 0
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
static tUSBDHIDKeyboardDevice keyboardDevice =
{
	USB_VID_TI_1CBE,
    USB_PID_KEYBOARD,
    500,
    USB_CONF_ATTR_SELF_PWR | USB_CONF_ATTR_RWAKE,
    cbKeyboardHandler,
    NULL,
    stringDescriptors,
    STRINGDESCRIPTORSCOUNT
};
#else  /* MWARE */
static tHIDKeyboardInstance keyboardInstance;
const tUSBDHIDKeyboardDevice keyboardDevice =
{
    USB_VID_TI,
    USB_PID_KEYBOARD,
    500,
    USB_CONF_ATTR_SELF_PWR | USB_CONF_ATTR_RWAKE,
    cbKeyboardHandler,
    NULL,
    stringDescriptors,
    STRINGDESCRIPTORSCOUNT,
    &keyboardInstance /* Old usblib stores a pointer */
};
#endif

/* Keyboard character lookup table */
const UChar keyUsageCodes[][2] =
{
    { 0,                      HID_KEYB_USAGE_SPACE      },    //   0x20
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_1          },    // ! 0x21
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_FQUOTE     },    // " 0x22
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_3          },    // # 0x23
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_4          },    // $ 0x24
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_5          },    // % 0x25
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_7          },    // & 0x26
    { 0,                      HID_KEYB_USAGE_FQUOTE     },    // ' 0x27
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_9          },    // ( 0x28
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_0          },    // ) 0x29
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_8          },    // * 0x2a
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_EQUAL      },    // + 0x2b
    { 0,                      HID_KEYB_USAGE_COMMA      },    // , 0x2c
    { 0,                      HID_KEYB_USAGE_MINUS      },    // - 0x2d
    { 0,                      HID_KEYB_USAGE_PERIOD     },    // . 0x2e
    { 0,                      HID_KEYB_USAGE_FSLASH     },    // / 0x2f
    { 0,                      HID_KEYB_USAGE_0          },    // 0 0x30
    { 0,                      HID_KEYB_USAGE_1          },    // 1 0x31
    { 0,                      HID_KEYB_USAGE_2          },    // 2 0x32
    { 0,                      HID_KEYB_USAGE_3          },    // 3 0x33
    { 0,                      HID_KEYB_USAGE_4          },    // 4 0x34
    { 0,                      HID_KEYB_USAGE_5          },    // 5 0x35
    { 0,                      HID_KEYB_USAGE_6          },    // 6 0x36
    { 0,                      HID_KEYB_USAGE_7          },    // 7 0x37
    { 0,                      HID_KEYB_USAGE_8          },    // 8 0x38
    { 0,                      HID_KEYB_USAGE_9          },    // 9 0x39
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_SEMICOLON  },    // : 0x3a
    { 0,                      HID_KEYB_USAGE_SEMICOLON  },    // ; 0x3b
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_COMMA      },    // < 0x3c
    { 0,                      HID_KEYB_USAGE_EQUAL      },    // = 0x3d
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_PERIOD     },    // > 0x3e
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_FSLASH     },    // ? 0x3f
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_2          },    // @ 0x40
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_A          },    // A 0x41
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_B          },    // B 0x42
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_C          },    // C 0x43
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_D          },    // D 0x44
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_E          },    // E 0x45
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_F          },    // F 0x46
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_G          },    // G 0x47
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_H          },    // H 0x48
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_I          },    // I 0x49
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_J          },    // J 0x4a
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_K          },    // K 0x4b
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_L          },    // L 0x4c
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_M          },    // M 0x4d
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_N          },    // N 0x4e
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_O          },    // O 0x4f
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_P          },    // P 0x50
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_Q          },    // Q 0x51
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_R          },    // R 0x52
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_S          },    // S 0x53
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_T          },    // T 0x54
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_U          },    // U 0x55
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_V          },    // V 0x56
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_W          },    // W 0x57
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_X          },    // X 0x58
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_Y          },    // Y 0x59
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_Z          },    // Z 0x5a
    { 0,                      HID_KEYB_USAGE_LBRACKET   },    // [ 0x5b
    { 0,                      HID_KEYB_USAGE_BSLASH     },    // \ 0x5c
    { 0,                      HID_KEYB_USAGE_RBRACKET   },    // ] 0x5d
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_6          },    // ^ 0x5e
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_MINUS      },    // _ 0x5f
    { 0,                      HID_KEYB_USAGE_BQUOTE     },    // ` 0x60
    { 0,                      HID_KEYB_USAGE_A          },    // a 0x61
    { 0,                      HID_KEYB_USAGE_B          },    // b 0x62
    { 0,                      HID_KEYB_USAGE_C          },    // c 0x63
    { 0,                      HID_KEYB_USAGE_D          },    // d 0x64
    { 0,                      HID_KEYB_USAGE_E          },    // e 0x65
    { 0,                      HID_KEYB_USAGE_F          },    // f 0x66
    { 0,                      HID_KEYB_USAGE_G          },    // g 0x67
    { 0,                      HID_KEYB_USAGE_H          },    // h 0x68
    { 0,                      HID_KEYB_USAGE_I          },    // i 0x69
    { 0,                      HID_KEYB_USAGE_J          },    // j 0x6a
    { 0,                      HID_KEYB_USAGE_K          },    // k 0x6b
    { 0,                      HID_KEYB_USAGE_L          },    // l 0x6c
    { 0,                      HID_KEYB_USAGE_M          },    // m 0x6d
    { 0,                      HID_KEYB_USAGE_N          },    // n 0x6e
    { 0,                      HID_KEYB_USAGE_O          },    // o 0x6f
    { 0,                      HID_KEYB_USAGE_P          },    // p 0x70
    { 0,                      HID_KEYB_USAGE_Q          },    // q 0x71
    { 0,                      HID_KEYB_USAGE_R          },    // r 0x72
    { 0,                      HID_KEYB_USAGE_S          },    // s 0x73
    { 0,                      HID_KEYB_USAGE_T          },    // t 0x74
    { 0,                      HID_KEYB_USAGE_U          },    // u 0x75
    { 0,                      HID_KEYB_USAGE_V          },    // v 0x76
    { 0,                      HID_KEYB_USAGE_W          },    // w 0x77
    { 0,                      HID_KEYB_USAGE_X          },    // x 0x78
    { 0,                      HID_KEYB_USAGE_Y          },    // y 0x79
    { 0,                      HID_KEYB_USAGE_Z          },    // z 0x7a
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_LBRACKET   },    // { 0x7b
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_BSLASH     },    // | 0x7c
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_RBRACKET   },    // } 0x7d
    { HID_KEYB_LEFT_SHIFT,    HID_KEYB_USAGE_BQUOTE     },    // ~ 0x7e
};

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
static USBKBDEventType cbKeyboardHandler(Void *cbData, USBKBDEventType event,
		                                 USBKBDEventType eventMsg, Void *eventMsgPtr)
{
    /* Determine what event has happened */
    switch (event) {
        case USB_EVENT_CONNECTED:
            state = USBKBD_STATE_IDLE;
            Semaphore_post(semUSBConnected);
            break;

        case USB_EVENT_DISCONNECTED:
            state = USBKBD_STATE_UNCONFIGURED;
            break;

        case USB_EVENT_TX_COMPLETE:
            state = USBKBD_STATE_IDLE;
            Semaphore_post(semKeyboard);
            break;

        case USB_EVENT_SUSPEND:
            state = USBKBD_STATE_SUSPENDED;
            break;

        case USB_EVENT_RESUME:
            state = USBKBD_STATE_IDLE;
            Semaphore_post(semKeyboard);
            break;

        case USBD_HID_KEYB_EVENT_SET_LEDS:
            kbLEDs = eventMsg;
            break;

        default:
            break;
    }

    return (0);
}

/*
 *  ======== USBKBD_hwiHandler ========
 *  This function calls the USB library's device interrupt handler.
 */
static Void USBKBD_hwiHandler(UArg arg0)
{
    USB0DeviceIntHandler();
}

/*
 *  ======== sendChar ========
 *  Function simulates a keyboard key press and key release.
 *
 *  It handles printable characters and the return character
 *
 *  @param(ch)      Character to be sent
 *
 *  @return         Returns the passed in character (ch) if it was sent.
 *                  Return 0 if it wasn't sent.
 *
 */
static Int sendChar(Int ch, UInt timeout)
{
    if ((ch >= ' ') && (ch <= '~')) {
        /* Character manipulation for lookup table */
        ch -= ' ';

        state = USBKBD_STATE_SENDING;
        USBDHIDKeyboardKeyStateChange((Void *)&keyboardDevice,
                                      keyUsageCodes[ch][0],
                                      keyUsageCodes[ch][1],
                                      TRUE);

        /* Wait until the key press was sent */
        if (!waitUntilSent(timeout)) {
            /* It wasn't sent */
            return (0);
        }

        state = USBKBD_STATE_SENDING;
        USBDHIDKeyboardKeyStateChange((Void *)&keyboardDevice,
                                      0,
                                      keyUsageCodes[ch][1],
                                      FALSE);

        /* Wait until the key release was sent */
        if (!waitUntilSent(timeout)) {
            /* It wasn't sent */
            return (0);
        }

        return (ch + ' ');
    }
    else if ((ch == '\n') || (ch == '\r')) {
        state = USBKBD_STATE_SENDING;
        USBDHIDKeyboardKeyStateChange((Void *)&keyboardDevice,
                                      0,
                                      HID_KEYB_USAGE_ENTER,
                                      TRUE);

        /* Wait until the key press was sent */
        if (!waitUntilSent(timeout)) {
            /* It wasn't sent */
            return (0);
        }

        state = USBKBD_STATE_SENDING;
        USBDHIDKeyboardKeyStateChange((Void *)&keyboardDevice,
                                      0,
                                      HID_KEYB_USAGE_ENTER,
                                      FALSE);

        /* Wait until the key release was sent */
        if (!waitUntilSent(timeout)) {
            /* It wasn't sent */
            return (0);
        }

        return (ch);

    }
    else {
        /* Non-printable character, but return a good value anyways */
        return (ch);
    }

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
    /* Wait until the key press was sent */
    if (!Semaphore_pend(semKeyboard, timeout) ||
        (state != USBKBD_STATE_IDLE) ) {

        /*
         * There was a timeout waiting for the USB to successfully notify
         * the host of the new state
         */
        state = USBKBD_STATE_UNCONFIGURED;
        return (FALSE);
    }

    return (TRUE);
}

/*
 *  ======== USBKBD_getState ========
 */
Void USBKBD_getState(USBKBD_State *keyboardState)
{
    UInt key;

    key = Hwi_disable();
    keyboardState->numLED = (kbLEDs & HID_KEYB_NUM_LOCK) ? TRUE : FALSE;
    keyboardState->capsLED = (kbLEDs & HID_KEYB_CAPS_LOCK) ? TRUE : FALSE;
    keyboardState->scrollLED = (kbLEDs & HID_KEYB_SCROLL_LOCK) ? TRUE : FALSE;
    Hwi_restore(key);
}

/*
 *  ======== USBKBD_init ========
 */
 Void USBKBD_init(Void)
 {
    Hwi_Handle hwi;
    Error_Block eb;
    Semaphore_Params semParams;

    Error_init(&eb);

    /* Install interrupt handler */
    hwi = Hwi_create(INT_USB0, USBKBD_hwiHandler, NULL, &eb);
    if (hwi == NULL) {
        System_abort("Can't create USB Hwi");
    }

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

    gateKeyboard = GateMutex_create(NULL, &eb);
    if (gateKeyboard == NULL) {
        System_abort("Can't create keyboard gate");
    }

    gateUSBWait = GateMutex_create(NULL, &eb);
    if (gateUSBWait == NULL) {
        System_abort("Could not create USB Wait gate");
    }

    /* State specific variables */
    state = USBKBD_STATE_UNCONFIGURED;
    kbLEDs = 0x00;

    /* Set the USB stack mode to Device mode with VBUS monitoring */
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    /*
     * Pass our device information to the USB HID device class driver,
     * initialize the USB controller and connect the device to the bus.
     */
    if (!USBDHIDKeyboardInit(0, &keyboardDevice)) {
        System_abort("Error initializing the keyboard");
    }

}

/*
 *  ======== USBKBD_putChar ========
 */
Int USBKBD_putChar(Int ch, UInt timeout)
{
    /* Indicate failure */
    Int retValue = 0;
    UInt key;

    switch (state) {
        case USBKBD_STATE_UNCONFIGURED:
            USBKBD_waitForConnect(timeout);
            break;

        case USBKBD_STATE_SUSPENDED:
            /* Acquire lock */
            key = GateMutex_enter(gateKeyboard);

            state = USBKBD_STATE_SENDING;
            USBDHIDKeyboardRemoteWakeupRequest((Void *)&keyboardDevice);

            if (waitUntilSent(timeout)) {
                retValue = sendChar(ch, timeout);
            }

            /* Release lock */
            GateMutex_leave(gateKeyboard, key);
            break;

        case USBKBD_STATE_SENDING:
        case USBKBD_STATE_IDLE:
            /* Acquire lock */
            key = GateMutex_enter(gateKeyboard);

            retValue = sendChar(ch, timeout);

            /* Release lock */
            GateMutex_leave(gateKeyboard, key);
            break;

        default:
            break;
    }

    return (retValue);
}

/*
 *  ======== USBKBD_putString ========
 */
Int USBKBD_putString(String chArray, UInt length, UInt timeout)
{
    Int ch;
    Int count = 0;

    while ((*chArray) && (count < length)){

        /* Save current character */
        ch = (Int)(*chArray++);

        if (ch != USBKBD_putChar(ch, timeout)) {
            break;
        }

        /* Increment a counter */
        count++;
    }

    return (count);
}

/*
 *  ======== USBKBD_waitForConnect ========
 */
Bool USBKBD_waitForConnect(UInt timeout)
{
    Bool ret = TRUE;
    UInt key;

    /* Need exclusive access to prevent a race condition */
    key = GateMutex_enter(gateUSBWait);

    if (state == USBKBD_STATE_UNCONFIGURED) {
        if (!Semaphore_pend(semUSBConnected, timeout)) {
            ret = FALSE;
        }
    }

    GateMutex_leave(gateUSBWait, key);

    return (ret);
}

