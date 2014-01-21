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
/** ============================================================================
 *  @file       USBMSCHFatFs.h
 *
 *  @brief      USBMSCHFatFs driver interface.
 *
 *  The USBMSCHFatFs header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/USBMSCHFatFs.h>
 *  @endcode
 *
 *  # Operation #
 *
 *  The USBMSCHFatFs driver is a driver designed to hook into FatFs. It
 *  implements a set of functions that FatFs needs to call to perform basic
 *  block data transfers.
 *
 *  This driver is designed to work with the USB Library. Because it is running
 *  in host mode, we need to add protection when accessing the USB Library. A
 *  gate was added to prevent the task servicing the USB library to preempt any
 *  other task accessing the USB Library.
 *
 *  Once the driver has been opened, the application may used the FatFs APIs or
 *  the standard C runtime file I/O calls (fopen, fclose, etc...). Once the
 *  driver has been closed, ensure the application does NOT make any file I/O
 *  calls.
 *
 *  ## Opening the driver #
 *
 *  @code
 *  USBMSCHFatFs_Handle      handle;
 *  USBMSCHFatFs_Params      params;
 *
 *  USBMSCHFatFs_Params_init(&params);
 *  params.servicePriority  = somePriority;
 *  handle = USBMSCHFatFs_open(someUSBMSCHFatFs_configIndexValue, &params);
 *  if (!handle) {
 *      System_printf("USBMSCHFatFs did not open");
 *  }
 *  @endcode
 *
 *  # Implementation #
 *
 *  This module serves as the main interface for TI-RTOS
 *  applications. Its purpose is to redirect the module's APIs to specific
 *  peripheral implementations which are specified using a pointer to a
 *  USBMSCHFatFs_FxnTable.
 *
 *  The USBMSCHFatFs driver interface module is joined (at link
 *  time) to a NULL-terminated array of USBMSCHFatFs_Config data structures
 *  named *USBMSCHFatFs_config*. *USBMSCHFatFs_config* is implemented in the
 *  application with each entry being an instance of a USBMSCHFatFs peripheral.
 *  Each entry in *USBMSCHFatFs_config* contains a:
 *  - (USBMSCHFatFs_FxnTable *) to a set of functions that implement a
 *    USBMSCHFatFs peripheral
 *  - (Void *) data object that is associated with the USBMSCHFatFs_FxnTable
 *  - (Void *) hardware attributes that are associated to the
 *    USBMSCHFatFs_FxnTable
 *
 *  Currently the following USBMSCHFatFs peripheral implementations are
 *  supported:
 *  - @ref USBMSCHFatFsTiva.h
 *
 *  # Instrumentation #
 *
 *  The USBMSCHFatFs driver interface produces log statements if
 *  instrumentation is enabled.
 *
 *  Diagnostics Mask | Log details |
 *  ---------------- | ----------- |
 *  Diags_USER1      | basic USBMSCHFatFs operations performed |
 *  Diags_USER2      | detailed USBMSCHFatFs operations performed |
 *
 *  ============================================================================
 */

#ifndef ti_drivers_USBMSCHFATFS__include
#define ti_drivers_USBMSCHFATFS__include

#ifdef __cplusplus
extern "C" {
#endif

#include <xdc/std.h>
#include <ti/drivers/ENV.h>

/*!
 *  @brief      USBMSCHFatFs Handler
 */
typedef struct USBMSCHFatFs_Config *USBMSCHFatFs_Handle;

/*!
 *  @brief      USBMSCHFatFs Parameters
 */
typedef struct USBMSCHFatFs_Params {
    Int servicePriority; /*!< USB service Task priority. Default is highest Task priority */
} USBMSCHFatFs_Params;

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              USBMSCHFatFs_init().
 */
typedef Void (*USBMSCHFatFs_InitFxn) (USBMSCHFatFs_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              USBMSCHFatFs_open().
 */
typedef USBMSCHFatFs_Handle (*USBMSCHFatFs_OpenFxn) (USBMSCHFatFs_Handle handle,
                                                     UInt drv,
                                                   USBMSCHFatFs_Params *params);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              USBMSCHFatFs_close().
 */
typedef Void (*USBMSCHFatFs_CloseFxn) (USBMSCHFatFs_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              USBMSCHFatFs_waitForConnect().
 */
typedef Bool (*USBMSCHFatFs_waitForconnectFxn) (USBMSCHFatFs_Handle handle,
                                                UInt timeout);

/*!
 *  @brief      The definition of a USBMSCHFatFs function table that contains
 *              the required set of functions to control a specific USBMSCHFatFs
 *              driver implementation.
 */
typedef struct USBMSCHFatFs_FxnTable {
    /*! Function to initialized the given data object */
    USBMSCHFatFs_InitFxn         initFxn;

    /*! Function to open the specified peripheral */
    USBMSCHFatFs_OpenFxn         openFxn;

    /*! Function to close the specified peripheral */
    USBMSCHFatFs_CloseFxn        closeFxn;

    /*! Function to call waitForConnect */
    USBMSCHFatFs_waitForconnectFxn waitForConnectFxn;
} USBMSCHFatFs_FxnTable;

/*!
 *  @brief
 *
 *  The USBMSCHFatFs_Config structure contains a set of pointers used to
 *  characterize the USBMSCHFatFs driver implementation.
 *
 *  This structure needs to be defined before calling USBMSCHFatFs_init() and
 *  it must not be changed thereafter.
 */
typedef struct USBMSCHFatFs_Config {
    /*!
     * Pointer to a function table of a driver specific implementation of
     * USBMSCHFatFs functions.
     */
    USBMSCHFatFs_FxnTable const *fxnTablePtr;

    /*! Pointer to a driver specific data object */
    Void        *object;

    /*! Pointer to a driver specific hardware attributes structure */
    Void const  *hwAttrs;
} USBMSCHFatFs_Config;

/*!
 *  @brief  Function to closes a given USBMSCHFatFs peripheral specified by the
 *          USBMSCHFatFs handle. This function unmounts the file system mounted
 *          by USBMSCHFatFs_open and unregisters the USBMSCHFatFs driver from
 *          BIOS' FatFs module.
 *
 *  @pre    USBMSCHFatFs_open() had to be called first.
 *
 *  @post   After calling this function, it is safe to remove the USB drive
 *
 *  @param  handle  A USBMSCHFatFs handle returned from USBMSCHFatFs_open
 *
 *  @sa     USBMSCHFatFs_open
 */
extern Void USBMSCHFatFs_close(USBMSCHFatFs_Handle handle);

/*!
 *  @brief  Function to perform USB initializations
 *
 *  @pre    The USB controller needs to be powered up and clocked. The
 *          USBMSCHFatFs_config structure must exist and be persistent before
 *          this function can be called. This function must also be called
 *          before any other USBMSCHFatFs_ driver APIs.
 */
extern Void USBMSCHFatFs_init(Void);

/*!
 *  @brief  This function registers the USBMSCHFatFs driver with BIOS' FatFs
 *          module and mounts the FatFs file system.
 *
 *  @pre    USB controller has been initialized
 *
 *  @param  index         Logical peripheral number indexed into the HWAttrs
 *                        table
 *
 *  @param  drv           Drive number to be associated with the USBMSCHFatFs
 *                        FatFs driver
 *
 *  @param  params        Pointer to an parameter block, if NULL it will use
 *                        default values
 *
 *  @return A pointer to a USBMSCHFatFs_Handle on success or a NULL it was
 *          already opened
 *
 *  @sa     USBMSCHFatFs_close
 */
extern USBMSCHFatFs_Handle USBMSCHFatFs_open(UInt index, UInt drv,
                                             USBMSCHFatFs_Params *params);

/*!
 *  @brief  Function to initialize the USBMSCHFatFs_Params structure to its
 *          defaults
 *
 *  Defaults values are:
 *  servicePriority = Task_numPriorities - 1;
 *
 *  @param  params  Parameter structure to initialize
 */
extern Void USBMSCHFatFs_Params_init(USBMSCHFatFs_Params *params);

/*!
 *  @brief  Function blocks task execution while no USB drive is enumerated.
 *          After the USBMSCHFatFs driver has been opened this functino is used
 *          to determine when a USB drive is has been enumerated.
 *
 *  @param  handle  A USBMSCHFatFs handle
 *
 *  @param  timeout Timeout period in system ticks for the task to block
 *
 *  @return status:
 *          - TRUE: Successful
 *          - FALSE: timed out
 */
extern Bool USBMSCHFatFs_waitForConnect(USBMSCHFatFs_Handle handle,
                                        UInt timeout);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_USBMSCHFATFS__include */
