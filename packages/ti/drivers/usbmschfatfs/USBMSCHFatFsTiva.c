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

#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/family/arm/m3/Hwi.h>

#include <ti/drivers/usbmschfatfs/USBMSCHFatFsTiva.h>

/* driverlib header files */
#include <inc/hw_ints.h>
#include <inc/hw_types.h>

/* usblib Header files */
#include <usblib/usb-ids.h>
#include <usblib/usblib.h>
#include <usblib/usbmsc.h>
#include <usblib/host/usbhost.h>
#include <usblib/host/usbhmsc.h>

#if defined(TIVAWARE)
/* c99 types needed by TivaWare */
#include <stdint.h>
typedef uint32_t                    USBMSCEventType;
#else /* MWare */
#define g_sUSBHostMSCClassDriver    g_USBHostMSCClassDriver
#define eUSBModeHost                USB_MODE_HOST
#define ui32Event                   ulEvent
typedef ULong                       USBMSCEventType;
#endif

#define DRIVE_NOT_MOUNTED           ~0

/*
 * Array of USBMSCHFatFs_Handles to determine the association of the FatFs drive
 * number with a USBMSCHFatFs_Handle
 * _VOLUMES is defined in <ti/sysbios/fatfs/ffconf.h>
 * As only one USB MSC Host class can be defined only the 1st element is used.
 */
extern USBMSCHFatFs_Config USBMSCHFatFs_config[];

/* Function prototypes */
static Void USBMSCHFatFsTiva_cbMSCHandler(USBMSCType instance,
                                          USBMSCEventType event,
                                          Void *eventMsgPtr);
static Void USBMSCHFatFsTiva_hwiHandler(UArg arg0);
static Void USBMSCHFatFsTiva_serviceUSBHost(UArg arg0, UArg arg1);
static Void USBMSCHFatFsTiva_usbHCDEvents(Void *cbData);

/* FatFs disk I/O functions */
static DSTATUS  USBMSCHFatFsTiva_diskInitialize(UChar bValue);
static DRESULT  USBMSCHFatFsTiva_diskIOctl(UChar drv, UChar ctrl, Void *buf);
static DRESULT  USBMSCHFatFsTiva_diskRead(UChar drv, UChar *buf, ULong sector,
                                          UChar count);
static DSTATUS  USBMSCHFatFsTiva_diskStatus(UChar drv);
static DRESULT  USBMSCHFatFsTiva_diskWrite(UChar drv, const UChar *buf,
                                           ULong sector, UChar count);

/* USBMSCHFatFs functions */
Void USBMSCHFatFsTiva_init(USBMSCHFatFs_Handle handle);
USBMSCHFatFs_Handle USBMSCHFatFsTiva_open(USBMSCHFatFs_Handle handle,
                                          UInt drv,
                                          USBMSCHFatFs_Params *params);
Void USBMSCHFatFsTiva_close(USBMSCHFatFs_Handle handle);
Bool USBMSCHFatFsTiva_waitForConnect(USBMSCHFatFs_Handle handle, UInt timeout);

/* USBMSCHFatFs function table for USBMSCHFatFsTiva implementation */
const USBMSCHFatFs_FxnTable USBMSCHFatFsTiva_fxnTable = {
    USBMSCHFatFsTiva_init,
    USBMSCHFatFsTiva_open,
    USBMSCHFatFsTiva_close,
    USBMSCHFatFsTiva_waitForConnect
};

/* Default USBMSCHFatFs params */
extern const USBMSCHFatFs_Params USBMSCHFatFs_defaultParams;

/* MACRO to create a generic USB event host driver */
DECLARE_EVENT_DRIVER(USBMSCHFatFs_eventDriver, 0, 0,
                     USBMSCHFatFsTiva_usbHCDEvents);

/* A list of available Host Class Drivers */
static tUSBHostClassDriver const * const usbHCDDriverList[] = {
    &g_sUSBHostMSCClassDriver,  /* MSC Host class driver */
    &USBMSCHFatFs_eventDriver   /* Generic event notification handler */
};

/* Variable containing the number of HCDs in usbHCDDriverList */
static const ULong numHostClassDrivers =
    sizeof(usbHCDDriverList) / sizeof(tUSBHostClassDriver *);

/*
 *  ======== USBMSCHFatFsTiva_cbMSCHandler ========
 *  Callback handler for the USB stack.
 *
 *  Callback handler call by the USB stack to notify us on what has
 *  happened in regards to the mouse. This is done in a context of the priority
 *  of the USBMSCHFatFsTiva_serviceUSBHost task.
 *
 *  @param  instance    A driver instance of MSC.
 *
 *  @param  event       Identifies the event that occurred in regards to this
 *                      device.
 *
 *  @param  eventMsgPtr A data pointer associated with a particular event.
 */
static Void USBMSCHFatFsTiva_cbMSCHandler(USBMSCType instance,
                                          USBMSCEventType event,
                                          Void *eventMsgPtr)
{
    USBMSCHFatFsTiva_Object    *object = USBMSCHFatFs_config->object;

    /* Determine what event has happened */
    switch (event) {
        case MSC_EVENT_OPEN:
            object->state = USBMSCHFatFsTiva_CONNECTED;
            Log_print0(Diags_USER2, "USBMSCHFatFs: MSC DEVICE CONNECTED; "
                                    "Posting semUSBConnected");
            Semaphore_post(Semaphore_handle(&object->semUSBConnected));
            break;

        case MSC_EVENT_CLOSE:
            object->state = USBMSCHFatFsTiva_NO_DEVICE;
            Log_print0(Diags_USER2, "USBMSCHFatFs: MSC DEVICE DISCONNECTED");
            break;

        default:
            break;
    }
}

/*
 *  ======== USBMSCHFatFsTiva_diskInitialize ========
 *  This function checks the USB Drive to see if it's ready to be accessed.
 *
 *  This function attempts to read the USBHMSCDriveReady() function up to 10
 *  times to get a good return code. For some reason the first attempt to read
 *  this function doesn't return a good response value. Generally, you should
 *  get a good response on the second attempt.
 *  A gateMutex lock is required to prevent concurrent access to the USB
 *  Library's state machine variables within MSCInstance.
 *
 *  @param  drv Drive Number
 *
 *  @return Returns the disk status to the FatFs module
 */
static DSTATUS USBMSCHFatFsTiva_diskInitialize(UChar drv)
{
    UInt                        i;
    UInt                        key;
    UInt                        driveReady;
    USBMSCHFatFsTiva_Object    *object = USBMSCHFatFs_config->object;

    /* Determine if the USB Drive is ready up to 10 times */
    for (i = 0; i < 10; i++ ) {

        key = GateMutex_enter(GateMutex_handle(&(object->gateUSBLibAccess)));
        driveReady = USBHMSCDriveReady(object->MSCInstance);
        GateMutex_leave(GateMutex_handle(&(object->gateUSBLibAccess)), key);

        if (driveReady == 0) {
            Log_print1(Diags_USER1, "USBMSCHFatFs: disk initialization: "
                                    "ready after %d tries", i);

            return (0x00); /* Disk OK */
        }
    }

    Log_print0(Diags_USER1, "USBMSCHFatFs: disk initialization: not ready");
    return (STA_NOINIT);
}

/*
 *  ======== USBMSCHFatFsTiva_diskIOctl ========
 *  This function performs misc. control functions documented by FatFs
 *
 *  NOTE: Formatting the USB MSC f_mkfs() is NOT supported!
 *
 *  @param  drv     Drive Number
 *
 *  @param  ctrl    FatFs control function to be executed
 *
 *  @param  buf     Pointer to a buffer to which data is written
 */
static DRESULT USBMSCHFatFsTiva_diskIOctl(UChar drv, UChar ctrl, Void *buf)
{
    USBMSCHFatFsTiva_Object    *object = USBMSCHFatFs_config->object;

    if (object->state != USBMSCHFatFsTiva_CONNECTED) {
        Log_print0(Diags_USER1, "USBMSCHFatFs: disk IO control: not "
                                "initialized");

        return (RES_NOTRDY);
    }

    switch (ctrl) {
        case CTRL_SYNC:
            Log_print0(Diags_USER1, "USBMSCHFatFs: disk IO control: OK");
            return (RES_OK);

        default:
            Log_print0(Diags_USER1, "USBMSCHFatFs: disk IO control: "
                                    "parameter error");
            return (RES_PARERR);
    }
}

/*
 *  ======== USBMSCHFatFsTiva_diskRead ========
 *  This function reads sector(s) from the disk drive
 *
 *  @param  drv     Drive Number
 *
 *  @param  buf     Pointer to a buffer to which data is written
 *
 *  @param  sector  Sector number to read from
 *
 *  @param  count   Number of sectors to be read
 */
static DRESULT USBMSCHFatFsTiva_diskRead(UChar drv, UChar *buf,
                                              ULong sector, UChar count)
{
    UInt                        key;
    Long                        driveRead;
    USBMSCHFatFsTiva_Object    *object = USBMSCHFatFs_config->object;

    Log_print2(Diags_USER1, "USBMSCHFatFs: diskRead: Sector %d, Count %d",
                             sector, count);

    if (object->state != USBMSCHFatFsTiva_CONNECTED) {
        Log_print0(Diags_USER1, "USBMSCHFatFs: diskRead: not initialized");
        return (RES_NOTRDY);
    }

    /* READ BLOCK */
    key = GateMutex_enter(GateMutex_handle(&(object->gateUSBLibAccess)));
    driveRead = USBHMSCBlockRead(object->MSCInstance, sector, buf, count);
    GateMutex_leave(GateMutex_handle(&(object->gateUSBLibAccess)), key);

    if (driveRead == 0) {
        Log_print0(Diags_USER2, "USBMSCHFatFs: diskRead: OK");
        return (RES_OK);
    }
    else {
        Log_print0(Diags_USER2, "USBMSCHFatFs: diskRead: ERROR");
        return (RES_ERROR);
    }
}

/*
 *  ======== USBMSCHFatFsTiva_diskStatus ========
 *  Returns the current status of a drive
 *
 *  @param  drv         Drive Number
 */
static DSTATUS USBMSCHFatFsTiva_diskStatus(UChar drv)
{
    USBMSCHFatFsTiva_Object    *object = USBMSCHFatFs_config->object;

    if (object->state != USBMSCHFatFsTiva_CONNECTED) {
        Log_print0(Diags_USER1, "USBMSCHFatFs: diskStatus: not initialized");
        return (STA_NOINIT);
    }

    return (0x00); /* Disk OK */
}

#if _READONLY == 0
/*
 *  ======== USBMSCHFatFsTiva_diskWrite ========
 *  This function writes sector(s) to the disk drive
 *
 *  @param  drv     Drive Number
 *
 *  @param  buf     Pointer to a buffer from which data is read
 *
 *  @param  sector  Sector number to write to
 *
 *  @param  count   Number of sectors to be written
 */
static DRESULT USBMSCHFatFsTiva_diskWrite(UChar drv, const UChar *buf,
                                          ULong sector, UChar count)
{
    UInt                        key;
    Long                        driveWrite;
    USBMSCHFatFsTiva_Object    *object = USBMSCHFatFs_config->object;

    Log_print2(Diags_USER1, "USBMSCHFatFs: diskWrite: Sector %d, Count %d",
                             sector, count);

    if (!count) {
        Log_print0(Diags_USER1, "USBMSCHFatFs: diskWrite: ERROR");
        return (RES_PARERR);
    }

    if (object->state != USBMSCHFatFsTiva_CONNECTED) {
        Log_print0(Diags_USER1, "USBMSCHFatFs: diskWrite: not initialized");
        return (RES_NOTRDY);
    }

    key = GateMutex_enter(GateMutex_handle(&(object->gateUSBLibAccess)));
    driveWrite = USBHMSCBlockWrite(object->MSCInstance, sector, (UChar *)buf, count);
    GateMutex_leave(GateMutex_handle(&(object->gateUSBLibAccess)), key);

    if (driveWrite == 0) {
        Log_print0(Diags_USER2, "USBMSCHFatFs: diskWrite: OK");
        return (RES_OK);
    }
    else {
        Log_print0(Diags_USER2, "USBMSCHFatFs: diskWrite: ERROR");
        return (RES_ERROR);
    }
}
#endif /* _READONLY */

/*
 *  ======== USBMSCHFatFsTiva_hwiHandler ========
 *  This function calls the USB library's interrupt handler.
 */
static Void USBMSCHFatFsTiva_hwiHandler(UArg arg0)
{
    /*
     * This function call generates a VBUS error interrupts; therefore we call
     * the OTG equivalent instead based on working TivaWare examples
     */
    //USB0HostIntHandler();
    USB0OTGModeIntHandler();
}

/*
 *  ======== USBMSCHFatFsTiva_serviceUSBHost ========
 *  Task to periodically service the USB Stack
 *
 *  USBHCDMain handles the USB Stack's statemachine. For example it handles the
 *  enumeration process when a device connects.
 *  Future USB library improvement goal is to remove this polling requirement..
 */
static Void USBMSCHFatFsTiva_serviceUSBHost(UArg arg0, UArg arg1)
{
    UInt                        key;
    USBMSCHFatFsTiva_Object    *object = USBMSCHFatFs_config->object;

    while (TRUE) {
        key = GateMutex_enter(GateMutex_handle(&(object->gateUSBLibAccess)));
        USBHCDMain();
        GateMutex_leave(GateMutex_handle(&(object->gateUSBLibAccess)), key);

        /* Future enhancement to remove the Task_sleep */
        Task_sleep(10);
    }
}

/*
 *  ======== USBMSCHFatFsTiva_usbHCDEvents ========
 *  Generic USB Host Class Driver event callback.
 *
 *  This callback is called to notify the application that an unknown
 *  device was connected.
 */
static Void USBMSCHFatFsTiva_usbHCDEvents(Void *cbData)
{
    tEventInfo                 *pEventInfo = (tEventInfo *)cbData;
    USBMSCHFatFsTiva_Object    *object = USBMSCHFatFs_config->object;

    switch (pEventInfo->ui32Event) {
        case USB_EVENT_UNKNOWN_CONNECTED:
            /* An unknown device was detected. */
            object->state = USBMSCHFatFsTiva_UNKNOWN;
            Log_print0(Diags_USER2, "USBMSCHFatFs: usbHCDEvent Callback: "
                                    "UNKNOWN DEVICE CONNECTED");
            break;

        case USB_EVENT_DISCONNECTED:
            /* Unknown device has been removed. */
            object->state = USBMSCHFatFsTiva_NO_DEVICE;
            Log_print0(Diags_USER2, "USBMSCHFatFs: usbHCDEvent Callback: "
                                    "UNKNOWN DEVICE DISCONNECTED");
            break;

        case USB_EVENT_POWER_FAULT:
            /* No power means no device is present. */
            object->state = USBMSCHFatFsTiva_POWER_FAULT;
            Log_print0(Diags_USER2, "USBMSCHFatFs: usbHCDEvent Callback: "
                                    "POWER FAULT");
            break;

        default:
            break;
    }
}

/*
 *  ======== USBMSCHFatFsTiva_init ========
 */
Void USBMSCHFatFsTiva_init(USBMSCHFatFs_Handle handle)
{
    USBMSCHFatFsTiva_Object    *object = handle->object;

    object->driveNumber = DRIVE_NOT_MOUNTED;
    object->state = USBMSCHFatFsTiva_NO_DEVICE;
}

/*
 *  ======== USBMSCHFatFsTiva_open ========
 */
USBMSCHFatFs_Handle USBMSCHFatFsTiva_open(USBMSCHFatFs_Handle handle,
                                          UInt drv,
                                          USBMSCHFatFs_Params *params)
{
    UInt                            key;
    DRESULT                         dresult;
    FRESULT                         fresult;
    USBMSCHFatFsTiva_Object        *object = handle->object;
    USBMSCHFatFsTiva_HWAttrs const *hwAttrs = handle->hwAttrs;
    union {
        Task_Params                 taskParams;
        Semaphore_Params            semParams;
        GateMutex_Params            gateParams;
    } paramsUnion;

    key = Hwi_disable();
    if (object->driveNumber != DRIVE_NOT_MOUNTED) {
        Hwi_restore(key);
        return (NULL);
    }
    object->driveNumber = drv;
    Hwi_restore(key);

    /* Store the USBMSCHFatFs parameters */
    if (params == NULL) {
        /* No params passed in, so use the defaults */
        params = (USBMSCHFatFs_Params *) &USBMSCHFatFs_defaultParams;
    }

    /* Initialize the USB stack for host mode. */
    USBStackModeSet(0, eUSBModeHost, NULL);

    /* Register host class drivers */
    USBHCDRegisterDrivers(0, usbHCDDriverList, numHostClassDrivers);

    /* Open an instance of the MSC host driver */
    object->MSCInstance = USBHMSCDriveOpen(0, USBMSCHFatFsTiva_cbMSCHandler);
    if (!(object->MSCInstance)) {
        Log_print0(Diags_USER1,"USBMSCHFatFs: Error initializing the MSC Host");
        USBMSCHFatFsTiva_close(handle);
        return (NULL);
    }

    /* Create the Hwi object to service interrupts */
   Hwi_construct(&(object->hwi), hwAttrs->intNum, USBMSCHFatFsTiva_hwiHandler,
                 NULL, NULL);

    /* Initialize USB power configuration */
    USBHCDPowerConfigInit(0, USBHCD_VBUS_AUTO_HIGH | USBHCD_VBUS_FILTER);

    /* Enable the USB stack */
    USBHCDInit(0, object->memPoolHCD, HCDMEMORYPOOLSIZE);

    /* RTOS primitives */
    Semaphore_Params_init(&(paramsUnion.semParams));
    paramsUnion.semParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&(object->semUSBConnected), 0, &(paramsUnion.semParams));

    GateMutex_Params_init(&(paramsUnion.gateParams));
    paramsUnion.gateParams.instance->name = "USB Library Access";
    GateMutex_construct(&(object->gateUSBLibAccess), &(paramsUnion.gateParams));

    paramsUnion.gateParams.instance->name = "USB Wait";
    GateMutex_construct(&(object->gateUSBWait), &(paramsUnion.gateParams));

    /*
     * Note that serviceUSBHost() should not be run until the USB Stack has been
     * initialized!!
     */
    Task_Params_init(&(paramsUnion.taskParams));
    paramsUnion.taskParams.priority = params->servicePriority;
    Task_construct(&(object->taskHCDMain),USBMSCHFatFsTiva_serviceUSBHost,
                   &(paramsUnion.taskParams), NULL);

    /* Register the new disk_*() functions */
    dresult = disk_register(drv,
                            USBMSCHFatFsTiva_diskInitialize,
                            USBMSCHFatFsTiva_diskStatus,
                            USBMSCHFatFsTiva_diskRead,
                            USBMSCHFatFsTiva_diskWrite,
                            USBMSCHFatFsTiva_diskIOctl);

    /* Check for drive errors */
    if (dresult != RES_OK) {
        Log_error0("USBMSCHFatFs: disk functions not registered");
        USBMSCHFatFsTiva_close(handle);
        return (NULL);
    }

    /* Mount the FatFs (this function does not access the SDCard yet...) */
    fresult = f_mount(drv, &(object->filesystem));
    if (fresult != FR_OK) {
        Log_error1("USBMSCHFatFs: drive %d not mounted", drv);
        USBMSCHFatFsTiva_close(handle);
        return (NULL);
    }

    Log_print1(Diags_USER1, "USBMSCHFatFs: drive %d opened", drv);

    return (handle);
}

/*
 *  ======== USBMSCHFatFsTiva_close ========
 */
Void USBMSCHFatFsTiva_close(USBMSCHFatFs_Handle handle)
{
    UInt                        key;
    DRESULT                     dresult;
    FRESULT                     fresult;
    USBMSCHFatFsTiva_Object    *object = handle->object;

    /* Unmount the FatFs drive */
    fresult = f_mount(object->driveNumber, NULL);
    if (fresult != FR_OK) {
        Log_print0(Diags_USER1, "USBMSCHFatFs: could not unmount FatFs volume");
    }

    /* Close USB Drive */
    USBHMSCDriveClose(object->MSCInstance);

    /* Unregister the disk_*() functions */
    dresult = disk_unregister(object->driveNumber);
    if (dresult != RES_OK) {
        Log_print0(Diags_USER1, "USBMSCHFatFs: error unregistering disk "
                                "functions");
    }

    /* Delete the HCDMain service task*/
    Task_destruct(&(object->taskHCDMain));

    /* Delete the semaphore */
    Semaphore_destruct(&(object->semUSBConnected));

    /* Delete the gate */
    GateMutex_destruct(&(object->gateUSBLibAccess));

    /* Delete the gate */
    GateMutex_destruct(&(object->gateUSBWait));

    /* Delete the hwi */
    Hwi_destruct(&(object->hwi));

    Log_print1(Diags_USER1, "USBMSCHFatFs: drive %d closed",
                             object->driveNumber);

    key = Hwi_disable();
    object->driveNumber = DRIVE_NOT_MOUNTED;
    Hwi_restore(key);
}

/*
 *  ======== USBMSCHFatFsTiva_waitForConnect ========
 */
Bool USBMSCHFatFsTiva_waitForConnect(USBMSCHFatFs_Handle handle, UInt timeout)
{
    UInt                        key;
    Bool                        ret = TRUE;
    USBMSCHFatFsTiva_Object    *object = handle->object;

    /* Need exclusive access to prevent a race condition */
    key = GateMutex_enter(GateMutex_handle(&(object->gateUSBWait)));

    if (object->state == USBMSCHFatFsTiva_NO_DEVICE) {
        if (!Semaphore_pend(Semaphore_handle(&(object->semUSBConnected)),
                                             timeout)) {
            ret = FALSE;
        }
    }

    GateMutex_leave(GateMutex_handle(&(object->gateUSBWait)), key);

    return (ret);
}
