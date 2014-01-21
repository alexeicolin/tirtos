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
 *    ======== USBCDCMOUSE.c ========
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
#include "USBCDCMOUSE.h"

/* driverlib header files */
#include "ucs.h"

/* usblib430 header files */
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"
#include "USB_API/USB_Common/defMSP430USB.h"
#include "USB_API/USB_Common/usb.h"
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_API/USB_HID_API/UsbHid.h"

/* Code taken from the MSP430 usblib430 mouse application */
typedef struct {
    UChar   buttons;
    UChar   dX;
    UChar   dY;
    UChar   dZ;
} MouseReport;

/* Static variables and handles */
static GateMutex_Handle         gateTxSerial;
static GateMutex_Handle         gateRxSerial;
static GateMutex_Handle         gateMouse;
static GateMutex_Handle         gateUSBWait;
static Semaphore_Handle         semTxSerial;
static Semaphore_Handle         semRxSerial;
static Semaphore_Handle         semMouse;
static Semaphore_Handle         semUSBConnected;

/* Function prototypes */
static UInt  rxData(UChar *pStr, UInt length, UInt timeout);
static Int   sendState(USBCDCMOUSE_State *mouseState, UInt timeout);
static UInt  txData(const UChar *pStr, Int length, UInt timeout);
Void         USBCDCMOUSE_init(Void);
UInt         USBCDCMOUSE_receiveData(UChar *pStr, UInt length, UInt timeout);
UInt         USBCDCMOUSE_sendData(const UChar *pStr, UInt length, UInt timeout);
UInt         USBCDCMOUSE_setState(USBCDCMOUSE_State *mouseState, UInt timeout);
Bool         USBCDCMOUSE_waitForConnect(UInt timeout);

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

/* Start of callback functions needed by USB_CDC_API/UsbCdc.[ch] */
BYTE USBCDC_handleDataReceived(BYTE intfNum) {return FALSE;}

BYTE USBCDC_handleSetLineCoding(BYTE intfNum, ULONG lBaudrate)
{
    Log_print1(Diags_USER1, "SetLineCoding baudrate: %d\n", lBaudrate);
    return (TRUE);
}

BYTE USBCDC_handleSetControlLineState(BYTE intfNum, BYTE lineState)
{
    Log_print1(Diags_USER1, "SetControlLineState: %x\n", lineState);
    return (TRUE);
}

BYTE USBCDC_handleSendCompleted(BYTE intfNum)
{
    if (intfNum == USBCDCD_INTFNUM) {
        Semaphore_post(semTxSerial);
        Log_print0(Diags_USER1, "USB: USBCDC_handleSendCompleted");
    }
    return (TRUE);
}
BYTE USBCDC_handleReceiveCompleted(BYTE intfNum)
{
    if (intfNum == USBCDCD_INTFNUM) {
        Semaphore_post(semRxSerial);
        Log_print0(Diags_USER1, "USB: USBCDC_handleReceiveCompleted");
    }
    return (TRUE);
}

/* Start of callback functions needed by USB_HID_API/UsbHid.[ch] */
/*
 * This event indicates that data has been received for port port, but no data receive operation is underway.
 * returns TRUE to keep CPU awake
 */
BYTE USBHID_handleDataReceived(BYTE intfNum)
{
    Log_print0(Diags_USER1, "USB: USBHID_handleDataReceived");
    return (FALSE);
}

/*
 * This event indicates that a send operation on port port has just been completed.
 * returns TRUE to keep CPU awake
 */
BYTE USBHID_handleSendCompleted(BYTE intfNum)
{
    Semaphore_post(semMouse);
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

/* Start of static helper functions */
/*
 *  ======== rxData ========
 */
static UInt rxData(UChar *pStr, UInt length, UInt timeout)
{
    UChar    recvResult;
    UInt    ret = 0;

    recvResult = USBCDC_receiveData(pStr, length, USBCDCD_INTFNUM);
    /*
     * We will pend on a Semaphore_pend() if we received a
     * kUSBCDC_receiveStarted and block until we received all the data. If it
     * returned a kUSBCDC_receiveCompleted; a callback has already occurred
     * so we call Semaphore_post() to just consume it but it won't cause a
     * context switch.
     */
    if ((recvResult == kUSBCDC_receiveStarted) || (recvResult == kUSBCDC_receiveCompleted)) {
        if (!Semaphore_pend(semRxSerial, timeout)) {
            USBCDC_abortReceive(&length, USBCDCD_INTFNUM);
        }
        ret = length;
    }

    return (ret);
}

/*
 *  ======== sendState ========
 */
static Int sendState(USBCDCMOUSE_State *mouseState, UInt timeout)
{
    UInt key;
    Int retValue;
    MouseReport mouseReport = {0};

    /* Set the bit packed button values */
    mouseReport.buttons |= (mouseState->button1) ? 0x01 : 0;
    mouseReport.buttons |= (mouseState->button2) ? 0x02 : 0;
    mouseReport.buttons |= (mouseState->button3) ? 0x04 : 0;
    mouseReport.dX = mouseState->deltaX;
    mouseReport.dY = mouseState->deltaY;

    /* Acquire lock */
    key = GateMutex_enter(gateMouse);
    USBHID_sendReport((void*)&mouseReport, M_INTFNUM);
    retValue = Semaphore_pend(semMouse, timeout);

    /* Release lock */
    GateMutex_leave(gateMouse, key);

    return (retValue);
}

/*
 *  ======== txData ========
 */
static UInt txData(const UChar *pStr, Int length, UInt timeout)
{
    UChar    sendResult;
    UInt    ret = 0;

    sendResult = USBCDC_sendData(pStr, length, USBCDCD_INTFNUM);
    if (sendResult == kUSBCDC_sendStarted) {
        if (!Semaphore_pend(semTxSerial, timeout)) {
            USBCDC_abortSend((UInt *)&length, USBCDCD_INTFNUM);
        }
        ret = length;
    }
    return (ret);
}

/*
 *  ======== USBCDCMOUSE_init ========
 */
Void USBCDCMOUSE_init(Void)
{
    Error_Block eb;
    Semaphore_Params semParams;
    GateMutex_Params gateParams;

    Error_init(&eb);

    /* RTOS primitives */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    semParams.__iprms.name = "semTxSerial";
    semTxSerial = Semaphore_create(0, &semParams, &eb);
    if (semTxSerial == NULL) {
        System_abort("Can't create TX semaphore");
    }

    semParams.__iprms.name = "semRxSerial";
    semRxSerial = Semaphore_create(0, &semParams, &eb);
    if (semRxSerial == NULL) {
        System_abort("Can't create RX semaphore");
    }

    semParams.__iprms.name = "semMouse";
    semMouse = Semaphore_create(0, &semParams, &eb);
    if (semMouse == NULL) {
        System_abort("Can't create USB semaphore");
    }

    semParams.__iprms.name = "semUSBConnected";
    semUSBConnected = Semaphore_create(0, &semParams, &eb);
    if (semUSBConnected == NULL) {
        System_abort("Can't create USB semaphore");
    }

    GateMutex_Params_init(&gateParams);
    gateParams.__iprms.name = "gateTxSerial";
    gateTxSerial = GateMutex_create(NULL, &eb);
    if (gateTxSerial == NULL) {
        System_abort("Can't create gate");
    }

    gateParams.__iprms.name = "gateRxSerial";
    gateRxSerial = GateMutex_create(NULL, &eb);
    if (gateRxSerial == NULL) {
        System_abort("Can't create gate");
    }

    gateParams.__iprms.name = "gateMouse";
    gateMouse = GateMutex_create(NULL, &eb);
    if (gateMouse == NULL) {
        System_abort("Can't create mouse gate");
    }

    gateParams.__iprms.name = "gateUSBWait";
    gateUSBWait = GateMutex_create(NULL, &eb);
    if (gateUSBWait == NULL) {
        System_abort("Could not create USB Wait gate");
    }

    /* Initialize the USB module, enable events and connect if UBUS is present */
    USB_setup(TRUE, TRUE);
}

/*
 *  ======== USBCDCMOUSE_receiveData ========
 */
UInt USBCDCMOUSE_receiveData(UChar *pStr, UInt length, UInt timeout)
{
    UInt retValue = 0;
    UInt key;

    if (USB_connectionInfo() & kUSB_Enumerated) {
        key = GateMutex_enter(gateRxSerial);
        retValue = rxData(pStr, length, timeout);
        GateMutex_leave(gateRxSerial, key);
    }

    return (retValue);
}

/*
 *  ======== USBCDCMOUSE_sendData ========
 */
UInt USBCDCMOUSE_sendData(const UChar *pStr, UInt length, UInt timeout)
{
    UInt retValue = 0;
    UInt key;

    if (USB_connectionInfo() & kUSB_Enumerated) {
        key = GateMutex_enter(gateTxSerial);
        retValue = txData(pStr, length, timeout);
        GateMutex_leave(gateTxSerial, key);
    }

    return (retValue);
}

/*
 *  ======== USBCDCMOUSE_setState ========
 */
UInt USBCDCMOUSE_setState(USBCDCMOUSE_State *mouseState, UInt timeout)
{
    UInt retValue = 0;

    if (USB_connectionInfo() & kUSB_Enumerated) {
        retValue = sendState(mouseState, timeout);
    }

    return (retValue);
}

/*
 *  ======== USBCDCMOUSE_waitForConnect ========
 */
Bool USBCDCMOUSE_waitForConnect(UInt timeout)
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
