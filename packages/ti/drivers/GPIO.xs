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
 *  ======== GPIO.xs ========
 */

/*
 *  ======== viewInitBasic ========
 */
function viewInitBasic(view)
{
    var Program = xdc.useModule('xdc.rov.Program');
    var GPIO = xdc.useModule('ti.drivers.GPIO');
    var ScalarStructs = xdc.useModule('xdc.rov.support.ScalarStructs');

    var eventViews = new Array();

    /* Fetch GPIO config struct */
    var GPIOConfigSymbol = Program.getSymbolValue("GPIO_config");

    /* Loop through all GPIOs--count to 64 in case not NULL-terminated */
    for (var i = 0; i <= 64; i++) {
        /* Create a new element for ROV */
        var viewElem = Program.newViewStruct('ti.drivers.GPIO', 'Basic');

        /* Get the address of the next GPIO Handle */
        var gpioHandleAddr = Number(GPIOConfigSymbol) +
                            (xdc.om[GPIO.GPIO_Config$fetchDesc.type].$sizeof() * i);

        /* Fetch the structure from memory */
        var gpioHandle = Program.fetchStruct(GPIO.GPIO_Config$fetchDesc, gpioHandleAddr.toString());

        /* Check to see if the array has terminated with a NULL */
        if (gpioHandle.hwAttrs == 0) {
            break;
        }

        var hwattrs = Program.fetchStruct(GPIO.GPIO_HWAttrs$fetchDesc, gpioHandle.hwAttrs);

        viewElem.Index = i.toString(10);
        viewElem.Port = "0x" + hwattrs.port.toString(16);
        viewElem.Pins = "0x" + hwattrs.pin.toString(16);
        switch (hwattrs.direction) {
            case GPIO.GPIO_INPUT:
                viewElem.Direction = "INPUT";
                break;
            case GPIO.GPIO_OUTPUT:
                viewElem.Direction = "OUTPUT";
                break;
        }
        var GPIOval = Program.fetchStruct(
                 {type: 'xdc.rov.support.ScalarStructs.S_UChar', isScalar: true},
                                    (hwattrs.port + (hwattrs.pin << 2)), false);
        viewElem.Value = "0x" + GPIOval.elem.toString(16);

        /* Add the this element into the array of elements */
        eventViews[eventViews.length] = viewElem;
    }

    view.elements = eventViews;
}
