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

#include <ti/drivers/watchdog/WatchdogTiva.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

/* Function prototypes */
Void WatchdogTiva_clear(Watchdog_Handle handle);
Void WatchdogTiva_close(Watchdog_Handle handle);
Void WatchdogTiva_init(Watchdog_Handle handle);
Watchdog_Handle WatchdogTiva_open(Watchdog_Handle handle, Watchdog_Params *params);
Void WatchdogTiva_setReload(Watchdog_Handle handle, ULong value);

/* Watchdog function table for Tiva implementation */
const Watchdog_FxnTable WatchdogTiva_fxnTable = {
        WatchdogTiva_clear,
        WatchdogTiva_close,
        WatchdogTiva_init,
        WatchdogTiva_open,
        WatchdogTiva_setReload
};

/* Default Watchdog params */
extern const Watchdog_Params Watchdog_defaultParams;

/*
 *  ======== WatchdogTiva_clear ========
 */
Void WatchdogTiva_clear(Watchdog_Handle handle)
{
    WatchdogTiva_HWAttrs const *hwAttrs = handle->hwAttrs;

    WatchdogIntClear(hwAttrs->baseAddr);
}

/*
 *  ======== WatchdogTiva_close ========
 */
Void WatchdogTiva_close(Watchdog_Handle handle)
{
    /*Not supported for Tiva */
    Assert_isTrue(FALSE, NULL);
}

/*
 *  ======== Watchdog_init ========
 */
Void WatchdogTiva_init(Watchdog_Handle handle)
{
    WatchdogTiva_Object *object = handle->object;

    object->isOpen = FALSE;
}

/*
 *  ======== WatchdogTiva_open ========
 */
Watchdog_Handle WatchdogTiva_open(Watchdog_Handle handle, Watchdog_Params *params)
{
    UInt key;
    Hwi_Params hwiParams;
    WatchdogTiva_HWAttrs const *hwAttrs = handle->hwAttrs;
    WatchdogTiva_Object        *object  = handle->object;

    /* If params are NULL use defaults. */
    if (params == NULL) {
        params = (Watchdog_Params *) &Watchdog_defaultParams;
    }

    /* Don't allow preemption */
    key = Hwi_disable();

    /* Check if the Watchdog is open already with the HWAttrs */
    if (object->isOpen == TRUE) {
        Hwi_restore(key);
        Log_warning1("Watchdog: Handle %x already in use.", (UArg)handle);
        return (NULL);
    }

    object->isOpen = TRUE;
    Hwi_restore(key);

    /* Construct Hwi object for watchdog */
    Hwi_Params_init(&hwiParams);
    hwiParams.arg = (UArg)handle;

    if (params->callbackFxn != NULL) {
        Hwi_construct(&(object->hwi), hwAttrs->intNum, params->callbackFxn,
                      &hwiParams, NULL);
    }

    WatchdogUnlock(hwAttrs->baseAddr);
    WatchdogReloadSet(hwAttrs->baseAddr, hwAttrs->reloadValue);
    WatchdogIntClear(hwAttrs->baseAddr);

#ifndef CCWARE
    /* Set reset mode */
    if (params->resetMode == Watchdog_RESET_ON) {
        WatchdogResetEnable(hwAttrs->baseAddr);
    }
    else {
        WatchdogResetDisable(hwAttrs->baseAddr);
    }
#endif

    /* Set debug stall mode */
    if (params->debugStallMode == Watchdog_DEBUG_STALL_ON) {
        WatchdogStallEnable(hwAttrs->baseAddr);
    }
    else {
        WatchdogStallDisable(hwAttrs->baseAddr);
    }

    WatchdogEnable(hwAttrs->baseAddr);

    WatchdogLock(hwAttrs->baseAddr);

    Log_print1(Diags_USER1, "Watchdog: handle %x opened" ,(UArg)handle);

    /* Return handle of the Watchdog object */
    return (handle);
}

/*
 *  ======== WatchdogTiva_setReload ========
 */
Void WatchdogTiva_setReload(Watchdog_Handle handle, ULong value)
{
    WatchdogTiva_HWAttrs const *hwAttrs = handle->hwAttrs;

    /* Set value */
    WatchdogUnlock(hwAttrs->baseAddr);
    WatchdogReloadSet(hwAttrs->baseAddr, value);
    WatchdogLock(hwAttrs->baseAddr);

    Log_print2(Diags_USER1, "Watchdog: WDT with handle 0x%x has been set to "
               "reload to 0x%x", (UArg)handle, value);
}
