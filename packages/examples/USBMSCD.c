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
 *  ======== USBMSCD.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/fatfs/diskio.h>
#include <ti/sysbios/gates/GateMutexPri.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

/* TI-RTOS driver header files */
#include <ti/drivers/SDSPI.h>

#include <stdint.h>

/* driverlib Header files */
#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <driverlib/udma.h>

/* usblib Header files */
#include <usblib/usb-ids.h>
#include <usblib/usblib.h>
#include <usblib/usbhid.h>
#include <usblib/device/usbdevice.h>
#include <usblib/device/usbdmsc.h>

/* Example/Board Header files */
#include "USBMSCD.h"

/* Static variables and handles */
static volatile struct {
    ULong drive;
} driveInformation;

#pragma DATA_SECTION(DMAControlTable, ".dma");
#pragma DATA_ALIGN(DMAControlTable, 1024)
static tDMAControlTable DMAControlTable[64];
static tMSCInstance MSCInstance;

/* Function prototypes */
/*
 * There is no need to have a callback function at the moment as all the
 * callbacks to the required functions are already called in a Hwi context.
 * The callback handler is kept commented out so that it can be easily enabled
 * for future developement.
 */
static Void close(Void *drv);
static Void USBMSCD_hwiHandler(UArg arg0);
static ULong numBlocks(Void *drv);
static Void *open(ULong drv);
static ULong read(Void *drv, UChar *data, ULong sector, ULong blockCount);
static ULong write(Void *drv, UChar *data, ULong sector, ULong blockCount);
Void USBMSCD_init(Void);

/* Extern'd functions from the SD Driver */
extern DSTATUS SDSPI_diskInitialize(UChar drv);
extern DRESULT SDSPI_diskRead(UChar drv, UChar *buff,
                              ULong sector, UChar count);
extern DRESULT SDSPI_diskWrite(UChar drv, const UChar *buff,
                               ULong sector, UChar count);
extern DRESULT SDSPI_diskIOctrl(UChar drv, UChar ctrl, Void *buff);

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
    (19 + 1) * 2,
    USB_DTYPE_STRING,
    'M', 0, 'a', 0, 's', 0, 's', 0, ' ', 0, 'S', 0, 't', 0, 'o', 0, 'r', 0,
    'a', 0, 'g', 0, 'e', 0, ' ', 0, 'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0,
    'e', 0
};

/* The serial number string. */
const UChar serialNumberString[] =
{
    (8 + 1) * 2,
    USB_DTYPE_STRING,
    '1', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0
};

/* The interface description string. */
const UChar dataInterfaceString[] =
{
    (19 + 1) * 2,
    USB_DTYPE_STRING,
    'B', 0, 'u', 0, 'l', 0, 'k', 0, ' ', 0, 'D', 0, 'a', 0, 't', 0,
    'a', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0, 'r', 0, 'f', 0,
    'a', 0, 'c', 0, 'e', 0
};

/* The configuration description string. */
const UChar configString[] =
{
    (23 + 1) * 2,
    USB_DTYPE_STRING,
    'B', 0, 'u', 0, 'l', 0, 'k', 0, ' ', 0, 'D', 0, 'a', 0, 't', 0,
    'a', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 'f', 0, 'i', 0, 'g', 0,
    'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0, 'o', 0, 'n', 0
};

/* The descriptor string table. */
const UChar * const stringDescriptors[] =
{
    langDescriptor,
    manufacturerString,
    productString,
    serialNumberString,
    dataInterfaceString,
    configString
};

#define STRINGDESCRIPTORSCOUNT (sizeof(stringDescriptors) / \
                                sizeof(UChar *))

const tUSBDMSCDevice MSCDevice =
{
    USB_VID_TI,
    USB_PID_MSC,
    "TI      ",
    "Mass Storage    ",
    "1.00",
    500,
    USB_CONF_ATTR_SELF_PWR,
    stringDescriptors,
    STRINGDESCRIPTORSCOUNT,
    /* 
     * Set of callback functions that are called directly by the interrupt
     * service routine
     */
    {
        open,
        close,
        read,
        write,
        numBlocks
    },
    NULL, //cbMSCHandler,
    &MSCInstance
};

/*
 *  ======== cbMSCHandler ========
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
//static ULong cbMSCHandler(Void *cbData, ULong event,
//                          ULong eventMsg, Void *eventMsgPtr)
//{
//    /* Determine what event has happened */
//    switch (event) {
//        case USB_EVENT_CONNECTED:
//            break;
//
//        case USB_EVENT_DISCONNECTED:
//            break;
//
//        case USBD_MSC_EVENT_WRITING:
//            break;
//
//        case USBD_MSC_EVENT_READING:
//            break;
//
//        case USBD_MSC_EVENT_IDLE:
//        default:
//            break;
//    }
//
//    return (0);
//}

/*
 *  ======== close ========
 */
static Void close(Void *drv)
{
    /* Nothing needs to be done here */
    return;
}

/*
 *  ======== USBMSCD_hwiHandler ========
 *  This function calls the USB library's device interrupt handler.
 */
static Void USBMSCD_hwiHandler(UArg arg0)
{
    USB0DeviceIntHandler();
}

/*
 *  ======== numBlocks ========
 */
static ULong numBlocks(Void *drv)
{
    ULong sectorCount;

    SDSPI_diskIOctrl(driveInformation.drive, GET_SECTOR_COUNT,
                    &sectorCount);
    return (sectorCount);
}

/*
 *  ======== open ========
 */
static Void *open(ULong drv)
{
    DSTATUS status;

    /* SD Card needs to get initialized */
    status = SDSPI_diskInitialize(drv);
    if (status) {
        return (NULL);
    }
   
    /* Drive is not initialized, save status flags */
    driveInformation.drive = drv; 

    return ((Void *)&driveInformation);
}

/*
 *  ======== read ========
 */
static ULong read(Void *drv, UChar *data, ULong sector, ULong blockCount)
{

    if (SDSPI_diskRead(driveInformation.drive, data,
                       sector, blockCount) == RES_OK) {
        return (blockCount * 512);
    }

    return (0);
}

/*
 *  ======== write ========
 */
static ULong write(Void *drv, UChar *data, ULong sector, ULong blockCount)
{
    if (SDSPI_diskWrite(driveInformation.drive, data,
                        sector, blockCount) == RES_OK) {
        return (blockCount * 512); /* Fixed to 512 by the USB CDC driver */
    }

    return (0);
}

/*
 *  ======== USBMSCD_init ========
 */
Void USBMSCD_init(Void)
{
    Hwi_Handle hwi;
    Error_Block eb;
    Clock_Params clockParams;

    uDMAControlBaseSet(&DMAControlTable[0]);

    Error_init(&eb);

    /* Install interrupt handler */
    hwi = Hwi_create(INT_USB0, USBMSCD_hwiHandler, NULL, &eb);
    if (hwi == NULL) {
        System_abort("Can't create USB Hwi");
    }

    /* Set the USB stack mode to Device mode with VBUS monitoring */
    USBStackModeSet(0, USB_MODE_DEVICE, 0);

    /*
     * Pass our device information to the USB HID device class driver,
     * initialize the USB controller and connect the device to the bus.
     */
    if (!USBDMSCInit(0, &MSCDevice)) {
        System_abort("Error initializing the MSC device");
    }

}
