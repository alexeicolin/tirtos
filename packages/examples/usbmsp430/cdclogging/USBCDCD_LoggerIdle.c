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
 *	======== USBCDCD_LoggerIdle.c ========
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
#include "USBCDCD_LoggerIdle.h"

/* driverlib header files */
#include "ucs.h"

/* usblib430 header files */
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/types.h"
#include "USB_API/USB_Common/defMSP430USB.h"
#include "USB_API/USB_Common/usb.h"
#include "USB_API/USB_CDC_API/UsbCdc.h"

/* Function prototypes */
static UInt txData(const UChar *pStr, UInt length);
Void USBCDCD_init(Void);
Int  USBCDCD_LoggerIdle_sendData(UChar *pStr, Int length);

/* Start of callback functions needed by USB_Common/Usb.[ch] */
BYTE USB_handleClockEvent()   {return FALSE;}
BYTE USB_handleResetEvent()   {return FALSE;}
BYTE USB_handleResumeEvent()  {return FALSE;}
BYTE USB_handleSuspendEvent() {return FALSE;}

BYTE USB_handleVbusOffEvent()
{
    UCS_XT2Off(__MSP430_BASEADDRESS_UCS__);
    return (TRUE);
}

BYTE USB_handleVbusOnEvent()
{
    if (USB_enable() == kUSB_succeed) {
        USB_reset();
        USB_connect();
    }
    return (TRUE);
}

BYTE USB_handleEnumCompleteEvent(Void)
{
    return (TRUE);
}

/* Start of callback functions needed by USB_CDC_API/UsbCdc.[ch] */
BYTE USBCDC_handleDataReceived(BYTE intfNum) {return FALSE;}

BYTE USBCDC_handleSetLineCoding(BYTE intfNum, ULONG lBaudrate)
{
    return (TRUE);
}

BYTE USBCDC_handleSetControlLineState(BYTE intfNum, BYTE lineState)
{
    return (TRUE);
}

BYTE USBCDC_handleSendCompleted(BYTE intfNum)
{
    return (TRUE);
}

BYTE USBCDC_handleReceiveCompleted(BYTE intfNum)
{
    return (FALSE);
}

/* Start of static helper functions */
/*
 *  ======== txData ========
 */
static UInt txData(const UChar *pStr, UInt length)
{
    UChar    sendResult;
    UInt    ret = 0;

    sendResult = USBCDC_sendData(pStr, length, USBCDCD_INTFNUM);
    if (sendResult == kUSBCDC_sendStarted) {
        ret = length;
    }
    return (ret);
}

/*
 *  ======== USBCDCD_init ========
 */
Void USBCDCD_init(Void)
{
    /* Initialize the USB module, enable events and connect if UBUS is present */
    USB_setup(TRUE, TRUE);
}


/*
 *  ======== USBCDCD_LoggerIdle_sendData ========
 */
Int USBCDCD_LoggerIdle_sendData(UChar *pStr, Int length)
{
    UInt retValue = 0;

    if (USB_connectionInfo() & kUSB_Enumerated) {
        retValue = txData(pStr, length);
    }

    return (retValue);
}
