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

#include <ti/drivers/USBMSCHFatFs.h>

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <ti/sysbios/knl/Task.h>

/* FatFs */
#include <ti/sysbios/fatfs/ffconf.h>

/* Externs */
extern const USBMSCHFatFs_Config USBMSCHFatFs_config[];

/* Used to check status and initialization */
static Int USBMSCHFatFs_count = -1;

/* Default USBMSCHFatFs parameters structure */
const USBMSCHFatFs_Params USBMSCHFatFs_defaultParams = {
    15 /* servicePriority */
};

/*
 *  ======== USBMSCHFatFs_close ========
 */
Void USBMSCHFatFs_close(USBMSCHFatFs_Handle handle)
{
    Assert_isTrue((handle != NULL) && (USBMSCHFatFs_count != -1), NULL);

    handle->fxnTablePtr->closeFxn(handle);
}

/*
 *  ======== USBMSCHFatFs_init ========
 */
Void USBMSCHFatFs_init(Void)
{
    Assert_isTrue(USBMSCHFatFs_count == -1, NULL);

    if (USBMSCHFatFs_count == -1) {
        /* Call each driver's init function */
        for (USBMSCHFatFs_count = 0; USBMSCHFatFs_config[USBMSCHFatFs_count].fxnTablePtr != NULL; USBMSCHFatFs_count++) {
            USBMSCHFatFs_config[USBMSCHFatFs_count].fxnTablePtr->
                initFxn((USBMSCHFatFs_Handle)&(USBMSCHFatFs_config[USBMSCHFatFs_count]));
        }
    }
}

/*
 *  ======== USBMSCHFatFs_open ========
 */
USBMSCHFatFs_Handle USBMSCHFatFs_open(UInt index, UInt drv,
                                      USBMSCHFatFs_Params *params)
{
    USBMSCHFatFs_Handle         handle;

    /* _VOLUMES is defined in <ti/sysbios/fatfs/ffconf.h> */
    Assert_isTrue((index < USBMSCHFatFs_count) && (drv <= _VOLUMES), NULL);

    /* Get handle for this driver instance */
    handle = (USBMSCHFatFs_Handle)&(USBMSCHFatFs_config[index]);

    return (handle->fxnTablePtr->openFxn(handle, drv, params));
}

/*
 *  ======== USBMSCHFatFs_Params_init ========
 *
 *  Defaults values are:
 *  @code
 *  servicePriority  = Task_numPriorities - 1;
 *  @endcode
 *
 *  @param  params  Parameter structure to initialize
 */
Void USBMSCHFatFs_Params_init(USBMSCHFatFs_Params *params)
{
    *params = USBMSCHFatFs_defaultParams;
}

/*
 *  ======== USBMSCHFatFs_waitForConnect ========
 */
Bool USBMSCHFatFs_waitForConnect(USBMSCHFatFs_Handle handle, UInt timeout)
{
    Assert_isTrue(handle != NULL, NULL);

    return (handle->fxnTablePtr->waitForConnectFxn(handle, timeout));
}

