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

#include <ti/drivers/watchdog/WatchdogMSP430.h>

#include <ti/sysbios/hal/Hwi.h>

extern Watchdog_Config Watchdog_config[];

Void WatchdogMSP430_clear(Watchdog_Handle handle);
Void WatchdogMSP430_close(Watchdog_Handle handle);
Void WatchdogMSP430_init(Watchdog_Handle handle);
Watchdog_Handle WatchdogMSP430_open(Watchdog_Handle handle, Watchdog_Params *params);
Void WatchdogMSP430_setReload(Watchdog_Handle handle, ULong value);

/* Watchdog function table for MSP430 implementation */
const Watchdog_FxnTable WatchdogMSP430_fxnTable = {
        WatchdogMSP430_clear,
        WatchdogMSP430_close,
        WatchdogMSP430_init,
        WatchdogMSP430_open,
        WatchdogMSP430_setReload
};

/* Default Watchdog params */
extern const Watchdog_Params Watchdog_defaultParams;

/*
 *  ======== WatchdogMSP430_clear ========
 */
Void WatchdogMSP430_clear(Watchdog_Handle handle)
{
    WatchdogMSP430_HWAttrs const *hwAttrs = handle->hwAttrs;

    WDT_A_resetTimer(hwAttrs->baseAddr);
}

/*
 *  ======== WatchdogMSP430_close ========
 */
Void WatchdogMSP430_close(Watchdog_Handle handle)
{
    WatchdogMSP430_HWAttrs const *hwAttrs = handle->hwAttrs;
    WatchdogMSP430_Object        *object  = handle->object;

    SFR_clearInterrupt(hwAttrs->sfrAddr, SFR_WATCHDOG_INTERVAL_TIMER_INTERRUPT);
    SFR_disableInterrupt(hwAttrs->sfrAddr, SFR_WATCHDOG_INTERVAL_TIMER_INTERRUPT);
    WDT_A_hold(hwAttrs->baseAddr);

    object->isOpen = FALSE;
}

/*
 *  ======== WatchdogMSP430_init ========
 */
Void WatchdogMSP430_init(Watchdog_Handle handle)
{
    WatchdogMSP430_Object *object = handle->object;

    object->isOpen = FALSE;
}

/*
 *  ======== WatchdogMSP430_open ========
 */
Watchdog_Handle WatchdogMSP430_open(Watchdog_Handle handle, Watchdog_Params *params)
{
    UInt key;
    WatchdogMSP430_HWAttrs const *hwAttrs = handle->hwAttrs;
    WatchdogMSP430_Object        *object = handle->object;

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

    /* If params are NULL use defaults. */
    if (params == NULL) {
        params = (Watchdog_Params *) &Watchdog_defaultParams;
    }

    SFR_clearInterrupt(hwAttrs->sfrAddr, SFR_WATCHDOG_INTERVAL_TIMER_INTERRUPT);

    /* Set reset mode */
    if (params->resetMode == Watchdog_RESET_ON) {
        WDT_A_watchdogTimerInit(hwAttrs->baseAddr, hwAttrs->clockSource, hwAttrs->clockDivider);
    }
    else {
        WDT_A_intervalTimerInit(hwAttrs->baseAddr, hwAttrs->clockSource, hwAttrs->clockDivider);
        SFR_enableInterrupt(hwAttrs->sfrAddr, SFR_WATCHDOG_INTERVAL_TIMER_INTERRUPT);
    }

    WDT_A_start(hwAttrs->baseAddr);

    Log_print1(Diags_USER1, "Watchdog: opened and enabled with handle "
            "0x%x.", (UArg)handle);

    /* Return handle of the Watchdog object */
    return (handle);
}

/*
 *  ======== WatchdogMSP430_setReload ========
 */
Void WatchdogMSP430_setReload(Watchdog_Handle handle, ULong value)
{
    /*Not supported for MSP430 */
    Assert_isTrue(FALSE, NULL);
}
