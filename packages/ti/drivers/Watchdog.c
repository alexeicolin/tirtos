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
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>

#include <ti/drivers/Watchdog.h>

#include <ti/sysbios/hal/Hwi.h>

/* Externs */
extern const Watchdog_Config Watchdog_config[];

/* Also used to check status for initialization */
static Int Watchdog_count = -1;

/* Default Watchdog parameters structure */
const Watchdog_Params Watchdog_defaultParams = {
    NULL,                   /* callbackFxn */
    Watchdog_RESET_ON,      /* resetMode */
    Watchdog_DEBUG_STALL_ON /* debugStallMode */
};

/*
 *  ======== Watchdog_clear ========
 */
Void Watchdog_clear(Watchdog_Handle handle)
{
    Assert_isTrue(Watchdog_count >= 0 && handle != NULL, NULL);

    handle->fxnTablePtr->watchdogClear(handle);
}

/*
 *  ======== Watchdog_close ========
 */
Void Watchdog_close(Watchdog_Handle handle)
{
    Assert_isTrue(Watchdog_count >= 0 && handle != NULL, NULL);

    handle->fxnTablePtr->watchdogClose(handle);
}

/*
 *  ======== Watchdog_init ========
 */
Void Watchdog_init(Void)
{
    /* Allow multiple calls for Watchdog_init */
    if (Watchdog_count >= 0) {
        return;
    }

    /* Call each driver's init function */
    for (Watchdog_count = 0;
            Watchdog_config[Watchdog_count].fxnTablePtr != NULL;
            Watchdog_count++) {
        Watchdog_config[Watchdog_count].fxnTablePtr->watchdogInit(
                (Watchdog_Handle)&(Watchdog_config[Watchdog_count]));
    }
}

/*
 *  ======== Watchdog_open ========
 */
Watchdog_Handle Watchdog_open(UInt index, Watchdog_Params *params)
{
    Watchdog_Handle handle;

    Assert_isTrue(Watchdog_count >= 0 && (Int)index < Watchdog_count , NULL);

    handle = (Watchdog_Handle)&(Watchdog_config[index]);
    return (handle->fxnTablePtr->watchdogOpen(handle, params));
}

/*
 *  ======== Watchdog_Params_init ========
 */
Void Watchdog_Params_init(Watchdog_Params *params)
{
    Assert_isTrue(params != NULL, NULL);

    *params = Watchdog_defaultParams;
}


/*
 *  ======== Watchdog_setReload ========
 */
Void Watchdog_setReload(Watchdog_Handle handle, ULong value)
{
    Assert_isTrue(Watchdog_count >= 0 && handle != NULL, NULL);

    handle->fxnTablePtr->watchdogSetReload(handle, value);
}
