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
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/gates/GateMutex.h>
#include <ti/sysbios/knl/Semaphore.h>

/* Example/Board Header files */
#include "USBKBD.h"

/* driverlib header files */
#include "ucs.h"

/* usblib430 header files */
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"
#include "USB_API/USB_Common/defMSP430USB.h"
#include "USB_API/USB_Common/usb.h"
#include "USB_API/USB_HID_API/UsbHid.h"

/* Code taken from the MSP430 usblib430 keyboard application */
typedef struct {
    UChar       modifiers;
    UChar       reserved;
    UChar       keys[6];
} KeyReport;

typedef union {
    UChar       keyArray[8];
    KeyReport   keyReport;
}KeyUnion;

static KeyUnion keyUnion;

/* Static variables and handles */
static GateMutex_Handle gateKeyboard;
static GateMutex_Handle gateUSBWait;
static Semaphore_Handle semKeyboard;
static Semaphore_Handle semLED;
static Semaphore_Handle semUSBConnected;

/* Function prototypes */
static Int sendChar(Int ch, UInt timeout);
Void USBKBD_getState(USBKBD_State *keyboardState);
Void USBKBD_init(Void);
Int USBKBD_putChar(Int ch, UInt timeout);
Int USBKBD_putString(String chArray, UInt length, UInt timeout);
Bool USBKBD_waitForConnect(UInt timeout);

/* Start of callback functions needed by USB_Common/Usb.[ch] */
BYTE USB_handleClockEvent()   {return FALSE;}
BYTE USB_handleResetEvent()   {return FALSE;}
BYTE USB_handleResumeEvent()  {return FALSE;}
BYTE USB_handleSuspendEvent() {return FALSE;}

BYTE USB_handleVbusOffEvent()
{
    UCS_XT2Off(__MSP430_BASEADDRESS_UCS__);
    Log_print0(Diags_USER1, "USB: Disconnect complete");
    return (TRUE);
}

BYTE USB_handleVbusOnEvent()
{
    if (USB_enable() == kUSB_succeed) {
        USB_reset();
        USB_connect();
        Log_print0(Diags_USER1, "USB: VBus detected");
    }
    return (TRUE);
}

BYTE USB_handleEnumCompleteEvent()
{
    Semaphore_post(semUSBConnected);
    Log_print0(Diags_USER1, "USB: Enumeration complete");
    return (TRUE);
}

/* Start of callback functions needed by USB_HID_API/UsbHid.[ch] */
/*
 * This event indicates that data has been received for port port, but no data receive operation is underway.
 * returns TRUE to keep CPU awake
 */
BYTE USBHID_handleDataReceived(BYTE intfNum)
{
    Semaphore_post(semLED);
    Log_print0(Diags_USER1, "USB: USBHID_handleDataReceived");
    return (FALSE);
}

/*
 * This event indicates that a send operation on port port has just been completed.
 * returns TRUE to keep CPU awake
 */
BYTE USBHID_handleSendCompleted(BYTE intfNum)
{
    Semaphore_post(semKeyboard);
    Log_print0(Diags_USER1, "USB: USBHID_handleSendCompleted");
    return (TRUE);
}

/*
 * This event indicates that a receive operation on port port has just been completed.
 * returns TRUE to keep CPU awake
 */
BYTE USBHID_handleReceiveCompleted(BYTE intfNum)
{
    Log_print0(Diags_USER1, "USB: USBHID_handleReceiveCompleted");
    return (TRUE);
}

/*
 * This event indicates that a Set_Protocol request was received from the host
 * The application may maintain separate reports for boot and report protocols.
 * The protocol field is either HID_BOOT_PROTOCOL or
 * HID_REPORT_PROTOCOL
 */
BYTE USBHID_handleBootProtocol(BYTE protocol, BYTE intfnum)
{
    Log_print0(Diags_USER1, "USB: USBHID_handleBootProtocol");
    return (TRUE);
}

/*
 * This event indicates that a Set_Report request was received from the host
 * The application needs to supply a buffer to retrieve the report data that will be sent
 * as part of this request. This handler is passed the reportType, reportId, the length of data
 * phase as well as the interface number.
 */
BYTE *USBHID_handleEP0SetReport(BYTE reportType, BYTE reportId,
                                WORD requestedLength,
                                BYTE intfnum)
{
    switch (reportType) {
        case USB_REQ_HID_INPUT:
        /* Return pointer to input Report Buffer */
        Log_print0(Diags_USER1, "USB: USBHID_handleEP0SetReport: USB_REQ_HID_INPUT");
        return 0;

    case USB_REQ_HID_OUTPUT:
        /* Return pointer to output Report Buffer */
        Log_print0(Diags_USER1, "USB: USBHID_handleEP0SetReport: USB_REQ_HID_OUTPUT");
        return 0;

    case USB_REQ_HID_FEATURE:
        /* Return pointer to feature Report Buffer */
        Log_print0(Diags_USER1, "USB: USBHID_handleEP0SetReport: USB_REQ_HID_FEATURE");
        return 0;

    default:
        Log_print0(Diags_USER1, "USB: USBHID_handleEP0SetReport: default");
        return 0;
    }
}

/*
 * This event indicates that data as part of Set_Report request was received from the host
 * The application can return TRUE to wake up the CPU. If the application supplied a buffer
 * as part of USBHID_handleSetReport, then this buffer will contain the Set Report data.
 */
BYTE USBHID_handleEP0SetReportDataAvailable(BYTE intfnum)
{
    Log_print0(Diags_USER1, "USB: USBHID_handleEP0SetReportDataAvailable");
    return (TRUE);
}

/*
 * This event indicates that a Get_Report request was received from the host
 * The application can supply a buffer of data that will be sent to the host.
 * This handler is passed the reportType, reportId, the requested length as
 * well as the interface number.
 */
BYTE *USBHID_handleEP0GetReport(BYTE reportType, BYTE reportId,
                                WORD requestedLength,
                                BYTE intfnum)
{
    switch (reportType) {
    case USB_REQ_HID_INPUT:
        /* Return pointer to input Report Buffer */
        Log_print0(Diags_USER1, "USB: USBHID_handleEP0GetReport: USB_REQ_HID_INPUT");
        return 0;
    case USB_REQ_HID_OUTPUT:
        /* Return pointer to OUTput Report Buffer */
        Log_print0(Diags_USER1, "USB: USBHID_handleEP0GetReport: USB_REQ_HID_OUTPUT");
        return 0;
    case USB_REQ_HID_FEATURE:
        /* Return pointer to FEATURE Report Buffer */
        Log_print0(Diags_USER1, "USB: USBHID_handleEP0GetReport: USB_REQ_HID_FEATURE");
        return 0;
    default:
        Log_print0(Diags_USER1, "USB: USBHID_handleEP0GetReport: default");
        return 0;
    }
}

#define SHIFT 0x80
const UChar _asciimap[128] =
{
    0x00,           // NUL
    0x00,           // SOH
    0x00,           // STX
    0x00,           // ETX
    0x00,           // EOT
    0x00,           // ENQ
    0x00,           // ACK
    0x00,           // BEL
    0x2a,           // BS    Backspace
    0x2b,           // TAB    Tab
    0x28,           // LF    Enter
    0x00,           // VT
    0x00,           // FF
    0x00,           // CR
    0x00,           // SO
    0x00,           // SI
    0x00,           // DEL
    0x00,           // DC1
    0x00,           // DC2
    0x00,           // DC3
    0x00,           // DC4
    0x00,           // NAK
    0x00,           // SYN
    0x00,           // ETB
    0x00,           // CAN
    0x00,           // EM
    0x00,           // SUB
    0x00,           // ESC
    0x00,           // FS
    0x00,           // GS
    0x00,           // RS
    0x00,           // US

    0x2c,           //  ' '
    0x1e | SHIFT,   // !
    0x34 | SHIFT,   // "
    0x20 | SHIFT,   // #
    0x21 | SHIFT,   // $
    0x22 | SHIFT,   // %
    0x24 | SHIFT,   // &
    0x34,           // '
    0x26 | SHIFT,   // (
    0x27 | SHIFT,   // )
    0x25 | SHIFT,   // *
    0x2e | SHIFT,   // +
    0x36,           // ,
    0x2d,           // -
    0x37,           // .
    0x38,           // /
    0x27,           // 0
    0x1e,           // 1
    0x1f,           // 2
    0x20,           // 3
    0x21,           // 4
    0x22,           // 5
    0x23,           // 6
    0x24,           // 7
    0x25,           // 8
    0x26,           // 9
    0x33 | SHIFT,   // :
    0x33,           // ;
    0x36 | SHIFT,   // <
    0x2e,           // =
    0x37 | SHIFT,   // >
    0x38 | SHIFT,   // ?
    0x1f | SHIFT,   // @
    0x04 | SHIFT,   // A
    0x05 | SHIFT,   // B
    0x06 | SHIFT,   // C
    0x07 | SHIFT,   // D
    0x08 | SHIFT,   // E
    0x09 | SHIFT,   // F
    0x0a | SHIFT,   // G
    0x0b | SHIFT,   // H
    0x0c | SHIFT,   // I
    0x0d | SHIFT,   // J
    0x0e | SHIFT,   // K
    0x0f | SHIFT,   // L
    0x10 | SHIFT,   // M
    0x11 | SHIFT,   // N
    0x12 | SHIFT,   // O
    0x13 | SHIFT,   // P
    0x14 | SHIFT,   // Q
    0x15 | SHIFT,   // R
    0x16 | SHIFT,   // S
    0x17 | SHIFT,   // T
    0x18 | SHIFT,   // U
    0x19 | SHIFT,   // V
    0x1a | SHIFT,   // W
    0x1b | SHIFT,   // X
    0x1c | SHIFT,   // Y
    0x1d | SHIFT,   // Z
    0x2f,           // [
    0x31,           // bslash
    0x30,           // ]
    0x23 | SHIFT,   // ^
    0x2d | SHIFT,   // _
    0x35,           // `
    0x04,           // a
    0x05,           // b
    0x06,           // c
    0x07,           // d
    0x08,           // e
    0x09,           // f
    0x0a,           // g
    0x0b,           // h
    0x0c,           // i
    0x0d,           // j
    0x0e,           // k
    0x0f,           // l
    0x10,           // m
    0x11,           // n
    0x12,           // o
    0x13,           // p
    0x14,           // q
    0x15,           // r
    0x16,           // s
    0x17,           // t
    0x18,           // u
    0x19,           // v
    0x1a,           // w
    0x1b,           // x
    0x1c,           // y
    0x1d,           // z
    0x2f | SHIFT,   //
    0x31 | SHIFT,   // |
    0x30 | SHIFT,   // }
    0x35 | SHIFT,   // ~
    0               // DEL
};

static Bool sendReport(UInt timeout)
{
    USBHID_sendReport(keyUnion.keyArray, KB_INTFNUM);
    if(Semaphore_pend(semKeyboard, timeout)) {
        return (TRUE);
    }
    else {
        return (FALSE);
    }
}

// press() adds the specified key (printing, non-printing, or modifier)
// to the persistent key report and sends the report.  Because of the way
// USB HID works, the host acts like the key remains pressed until we
// call release(), releaseAll(), or otherwise clear the report and resend.
static UInt keyPress(UChar k)
{
    UChar i;

    if (k >= 136)           // it's a non-printing key (not a modifier)
            k = k - 136;
    else if (k >= 128) {    // it's a modifier key
            keyUnion.keyReport.modifiers |= (1 << (k - 128));
            k = 0;
    } else {
            k = _asciimap[k];                               // it's a printing key
            if (k & 0x80) {                                 // capital letter or other character reached with shift
                    keyUnion.keyReport.modifiers |= 0x02;   // the left shift modifier
                    k &= 0x7F;
            }
    }

    // Add k to the key report only if it's not already present
    // and if there is an empty slot.
    if (keyUnion.keyReport.keys[0] != k && keyUnion.keyReport.keys[1] != k &&
        keyUnion.keyReport.keys[2] != k && keyUnion.keyReport.keys[3] != k &&
        keyUnion.keyReport.keys[4] != k && keyUnion.keyReport.keys[5] != k) {

            for (i = 0; i < 6; i++) {
                    if (keyUnion.keyReport.keys[i] == 0x00) {
                            keyUnion.keyReport.keys[i] = k;
                            break;
                    }
            }
            if (i == 6)
                    return 0;
    }
    return 1;
}

// release() takes the specified key out of the persistent key report and
// sends the report.  This tells the OS the key is no longer pressed and that
// it shouldn't be repeated any more.
static UInt keyRelease(UChar k)
{
    UChar i;

    if (k >= 136)           // it's a non-printing key (not a modifier)
            k = k - 136;
    else if (k >= 128) {    // it's a modifier key
            keyUnion.keyReport.modifiers &= ~(1 << (k - 128));
            k = 0;
    } else {                                                        // it's a printing key
            k = _asciimap[k];
            if (k & 0x80) {                                         // capital letter or other character reached with shift
                    keyUnion.keyReport.modifiers &= ~(0x02);        // the left shift modifier
                    k &= 0x7F;
            }
    }

    // Test the key report to see if k is present.  Clear it if it exists.
    // Check all positions in case the key is present more than once
    // (which it shouldn't be)
    for (i = 0; i < 6; i++)
            if (0 != k && keyUnion.keyReport.keys[i] == k)
                    keyUnion.keyReport.keys[i] = 0x00;

    return 1;
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
    keyPress(ch);
    if (!sendReport(timeout)) {
        return 0;
    }

    keyRelease(ch);
    if (!sendReport(timeout)) {
        return 0;
    }

    return (ch);
}

/*
 *  ======== USBKBD_getState ========
 */
Void USBKBD_getState(USBKBD_State *keyboardState)
{
    static UChar hidData[8] = {0};

    /* If data is available */
    if(Semaphore_pend(semLED, BIOS_NO_WAIT)) {
        USBHID_receiveReport(hidData, KB_INTFNUM);
        keyboardState->numLED =    hidData[0] & 0x01;
        keyboardState->capsLED =   hidData[0] & 0x02;
        keyboardState->scrollLED = hidData[0] & 0x04;
    }
}

/*
 *  ======== USBKBD_init ========
 */
 Void USBKBD_init(Void)
{
    Error_Block eb;
    Semaphore_Params semParams;
    GateMutex_Params gateParams;

    Error_init(&eb);

    /* RTOS primitives */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    semParams.__iprms.name = "semUSBConnected";
    semUSBConnected = Semaphore_create(0, &semParams, &eb);
    if (semUSBConnected == NULL) {
        System_abort("Can't create USB semaphore");
    }

    semParams.__iprms.name = "semKeyboard";
    semKeyboard = Semaphore_create(0, &semParams, &eb);
    if (semKeyboard == NULL) {
        System_abort("Can't create USB semKeyboard");
    }

    semParams.__iprms.name = "semLED";
    semLED = Semaphore_create(0, &semParams, &eb);
    if (semLED == NULL) {
        System_abort("Can't create USB semLED");
    }

    GateMutex_Params_init(&gateParams);
    gateParams.__iprms.name = "gateKeyboard";
    gateKeyboard = GateMutex_create(NULL, &eb);
    if (gateKeyboard == NULL) {
        System_abort("Can't create gate");
    }

    gateParams.__iprms.name = "gateUSBWait";
    gateUSBWait = GateMutex_create(NULL, &eb);
    if (gateUSBWait == NULL) {
        System_abort("Could not create USB Wait gate");
    }

    /* Initialize the USB module, enable events and connect if UBUS is present */
    USB_setup(TRUE, TRUE);

    keyUnion.keyReport.keys[0] = 0;
    keyUnion.keyReport.keys[1] = 0;
    keyUnion.keyReport.keys[2] = 0;
    keyUnion.keyReport.keys[3] = 0;
    keyUnion.keyReport.keys[4] = 0;
    keyUnion.keyReport.keys[5] = 0;
    keyUnion.keyReport.modifiers = 0;
}

/*
 *  ======== USBKBD_putChar ========
 */
Int USBKBD_putChar(Int ch, UInt timeout)
{
    UInt retValue = 0;
    UInt key;

    if (USB_connectionInfo() & kUSB_Enumerated) {
        key = GateMutex_enter(gateKeyboard);
        retValue = sendChar(ch, timeout);
        GateMutex_leave(gateKeyboard, key);
    }

    return (retValue);
}

/*
 *  ======== USBKBD_putString ========
 */
Int USBKBD_putString(String chArray, UInt length, UInt timeout)
{
    Int i = 0;

    while(length--) {
        if (!USBKBD_putChar(chArray[i++], timeout)) {
            break;
        }
    }

    return (i);
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

    if (!(USB_connectionInfo() & kUSB_Enumerated)) {
        if (!Semaphore_pend(semUSBConnected, timeout)) {
            ret = FALSE;
        }
    }

    GateMutex_leave(gateUSBWait, key);

    return (ret);
}
