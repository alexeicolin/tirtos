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
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>

#include <ti/drivers/GPIO.h>

#if defined(MWARE) || defined(TIVAWARE) || defined(CCWARE)
#include <ti/sysbios/family/arm/m3/Hwi.h>
#elif defined(MSP430WARE)
#include <ti/sysbios/hal/Hwi.h>
#elif defined(STARTERWARE)
#include <ti/sysbios/family/arm/a8/intcps/Hwi.h>
#else
#error This platform is not supported by TI-RTOS currently.
#endif

/* Defines to resolve differences between GPIO APIs */
#if defined(MSP430WARE)
#define GPIOIntClear        GPIO_clearInterruptFlag
#define GPIOIntDisable      GPIO_disableInterrupt
#define GPIOIntEnable       GPIO_enableInterrupt
#define GPIOIntStatus       GPIO_getInterruptStatus
#define GPIOIntTypeSet      GPIO_interruptEdgeSelect
#define GPIOPinRead         GPIO_getInputPinValue

#elif defined(MWARE)
#define GPIOIntClear        GPIOPinIntClear
#define GPIOIntStatus       GPIOPinIntStatus
#define GPIOIntDisable      GPIOPinIntDisable
#define GPIOIntEnable       GPIOPinIntEnable
#endif

extern const GPIO_Config GPIO_config[];
static Int GPIO_count = -1; /* Also used to check status for initialization */

/*
 *  ======== GPIO_clearInt ========
 */
Void GPIO_clearInt(UInt index)
{
    GPIO_HWAttrs const *attrs;

    Assert_isTrue(GPIO_count >= 0 && (Int)index < GPIO_count, NULL);

    attrs = GPIO_config[index].hwAttrs;

    /* Clear GPIO interrupt flag */
#if defined(STARTERWARE)
    GPIOPinIntClear(attrs->port, GPIO_INT_LINE_1, attrs->pin);
#else
    GPIOIntClear(attrs->port, attrs->pin);
#endif

    Log_print2(Diags_USER1, "GPIO: port 0x%x, pin 0x%x interrupt flag cleared",
               attrs->port, attrs->pin);
}

/*
 *  ======== GPIO_disableInt ========
 */
Void GPIO_disableInt(UInt index)
{
    UInt key;
    GPIO_HWAttrs const *attrs;

    Assert_isTrue(GPIO_count >= 0 && (Int)index < GPIO_count, NULL);

    attrs = GPIO_config[index].hwAttrs;

    /* Make atomic update */
    key = Hwi_disable();

#if defined(STARTERWARE)
    GPIOPinIntDisable(attrs->port, GPIO_INT_LINE_1, attrs->pin);
#else
    GPIOIntDisable(attrs->port, attrs->pin);
#endif

    Hwi_restore(key);

    Log_print2(Diags_USER1, "GPIO: port 0x%x, pin 0x%x interrupts disabled",
               attrs->port, attrs->pin);
}

/*
 *  ======== GPIO_enableInt ========
 */
Void GPIO_enableInt(UInt index, GPIO_IntType intType)
{
    UInt key;
    GPIO_HWAttrs const *attrs;

    Assert_isTrue(GPIO_count >= 0 && (Int)index < GPIO_count, NULL);

    attrs = GPIO_config[index].hwAttrs;

    /* Make atomic update */
    key = Hwi_disable();

    /* Set type of interrupt and then enable it */
	GPIOIntTypeSet(attrs->port, attrs->pin, intType);
#if defined(STARTERWARE)
    GPIOPinIntClear(attrs->port, GPIO_INT_LINE_1, attrs->pin);
	GPIOPinIntEnable(attrs->port, GPIO_INT_LINE_1, attrs->pin);
#else
    GPIOIntClear(attrs->port, attrs->pin);
	GPIOIntEnable(attrs->port, attrs->pin);
#endif

    Hwi_restore(key);

    Log_print2(Diags_USER1, "GPIO: port 0x%x, pin 0x%x interrupts enabled",
               attrs->port, attrs->pin);
}

/*
 *  ======== GPIO_hwiIntFxn ========
 *  Hwi function that processes GPIO interrupts.
 */
 #ifndef MSP430WARE
Void GPIO_hwiIntFxn(UArg callbacks)
{
    Bits32 pins;
    UInt i;
    GPIO_Callbacks* portCallback;

    portCallback = (GPIO_Callbacks*)callbacks;

    /* Find out which pins have their interrupt flags set */
#if defined(STARTERWARE)
    pins = GPIORawIntStatus(portCallback->port, GPIO_INT_LINE_1, 0xFFFFFFFF) & 0xFFFFFFFF;
#else
    pins = GPIOIntStatus(portCallback->port, 0xFF) & 0xFF;
#endif

    /* Match the interrupt to its corresponding callback function */
    for (i = 0; pins; i++) {
        if (pins & 0x1) {
            Assert_isTrue(portCallback->callbackFxn[i] != NULL, NULL);
            portCallback->callbackFxn[i]();
        }
        pins = pins >> 1;
    }
}
#endif

/*
 *  ======== GPIO_init ========
 */
Void GPIO_init()
{
    /* Allow multiple calls for GPIO_init */
    if (GPIO_count >= 0) {
        return;
    }

    /* Determine the value for GPIO_count */
    for (GPIO_count = 0; GPIO_config[GPIO_count].hwAttrs != NULL; GPIO_count++) {
    }
}

/*
 *  ======== GPIO_read ========
 */
Bits32 GPIO_read(UInt index)
{
    UInt key;
    Bits32 value;
    GPIO_HWAttrs const *attrs;

    Assert_isTrue(GPIO_count >= 0 && (Int)index < GPIO_count, NULL);

    attrs = GPIO_config[index].hwAttrs;

    /* Make atomic update */
    key = Hwi_disable();
	value = GPIOPinRead(attrs->port, attrs->pin);

    Hwi_restore(key);

    Log_print3(Diags_USER1, "GPIO: port 0x%x, pin 0x%x read 0x%x",
               attrs->port, attrs->pin, value);

    return (value);
}

/*
 *  ======== GPIO_setupCallbacks ========
 *  This function is not thread-safe.
 */
Void GPIO_setupCallbacks(GPIO_Callbacks const *callbacks)
{
#if defined(MSP430WARE)
    Assert_isTrue(FALSE, NULL);
#else
    Error_Block eb;
    Hwi_Params hwiParams;
    static Int index = 0;

    Assert_isTrue(GPIO_count > 0 && callbacks != NULL, NULL);
    Assert_isTrue(callbacks->hwiStruct != NULL && callbacks != NULL, NULL);

    /* Create Hwi object for GPIO pin */
    Hwi_Params_init(&hwiParams);
    hwiParams.arg = (UArg)callbacks;
    Error_init(&eb);

    /* Construct hardware interrupt */
    Hwi_construct((Hwi_Struct *)(callbacks->hwiStruct), callbacks->intNum,
                  GPIO_hwiIntFxn, &hwiParams, &eb);
    if (Error_check(&eb)) {
        /* Error creating Hwi */
        Log_error1("GPIO: Error constructing Hwi for GPIO Port %d",
                   callbacks->port);
    }
    else {
        index++;
    }
#endif
}

/*
 *  ======== GPIO_toggle ========
 */
Void GPIO_toggle(UInt index)
{
    UInt                  key;
    GPIO_HWAttrs const    *attrs;
#if  !defined(MSP430WARE)
    Bits32                value;
#endif

    Assert_isTrue(GPIO_count >= 0 && (Int)index < GPIO_count, NULL);
    Assert_isTrue(GPIO_config[index].hwAttrs->direction == GPIO_OUTPUT, NULL);

    attrs = GPIO_config[index].hwAttrs;

    /* Make atomic update */
    key = Hwi_disable();

#if   defined(MSP430WARE)
    GPIO_toggleOutputOnPin(attrs->port, attrs->pin);
#elif defined(STARTERWARE)
    value = GPIOPinRead(attrs->port, attrs->pin);
    value = (value > 0) ? 0x0 : 0x1;
    GPIOPinWrite(attrs->port, attrs->pin, value);
#else /* Tiva and MWare */
    value = GPIOPinRead(attrs->port, attrs->pin);
    value ^= attrs->pin;
    GPIOPinWrite(attrs->port, attrs->pin, value);
#endif

    Hwi_restore(key);

#if  !defined(MSP430WARE)
    Log_print3(Diags_USER1, "GPIO: port 0x%x, pin 0x%x toggled to 0x%x",
               attrs->port, attrs->pin, value);
#else
    Log_print2(Diags_USER1, "GPIO: port 0x%x, pin 0x%x toggled",
               attrs->port, attrs->pin);
#endif
}

/*
 *  ======== GPIO_write ========
 */
Void GPIO_write(UInt index, Bits32 value)
{
    UInt key;
    GPIO_HWAttrs const *attrs;

    Assert_isTrue(GPIO_count >= 0 && (Int)index < GPIO_count, NULL);
    Assert_isTrue(GPIO_config[index].hwAttrs->direction == GPIO_OUTPUT, NULL);

    attrs = GPIO_config[index].hwAttrs;

    key = Hwi_disable();

#if defined(MSP430WARE)
    if (value != 0) {
        GPIO_setOutputHighOnPin(attrs->port, attrs->pin);
    }
    else {
        GPIO_setOutputLowOnPin(attrs->port, attrs->pin);
    }
#else
    GPIOPinWrite(attrs->port, attrs->pin, value);
#endif

    Hwi_restore(key);

    Log_print3(Diags_USER1, "GPIO: port 0x%x, pin 0x%x wrote 0x%x",
               attrs->port, attrs->pin, value);
}
