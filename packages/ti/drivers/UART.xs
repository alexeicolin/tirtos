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
 *  ======== UART.xs ========
 */

var ClockFreqs;

/*
 *  ======== module$use ========
 *  This function will automatically include ClockFreqs modules if we're
 *  dealing with a MSP430 device
 */
function module$use()
{
    if (Program.build.target.$name.search("msp430") != -1) {
        ClockFreqs = xdc.useModule('ti.sysbios.family.msp430.ClockFreqs');
    }
}

/*
 *  ======== viewInitBasic ========
 */
function viewInitBasic(view)
{
    var Program = xdc.useModule('xdc.rov.Program');
    var UART = xdc.useModule('ti.drivers.UART');

    var tree = viewDetailed();

    /* List if entries for the ROV module */
    var eventViews = new Array();

    for (var elem in tree) {

        /* Create a new element for ROV */
        var viewElem = Program.newViewStruct('ti.drivers.UART', 'Basic');
        viewElem.baseAddr      = "0x" + (tree[elem].hwAttrs.baseAddr).toString(16);
        viewElem.functionTable = tree[elem].fxnTablePtr;
        viewElem.uartHandle     = elem.toString(16);

        /* Add this element to the array of elements */
        eventViews[eventViews.length] = viewElem;
    }

    view.elements = eventViews;
}

/*
 *  ======== viewInitConfig ========
 */
function viewInitConfig(view)
{
    var Program = xdc.useModule('xdc.rov.Program');
    var UART = xdc.useModule('ti.drivers.UART');

    var tree = viewDetailed();

    if (Program.build.target.stdTypes.t_Int.size == 4) {
        var timeoutWaitForever = 0xFFFFFFFF;
    }
    else {
        var timeoutWaitForever = 0xFFFF;
    }

    /* List if entries for the ROV module */
    var eventViews = new Array();

    for (var elem in tree) {

        /* Create a new element for ROV */
        var viewElem = Program.newViewStruct('ti.drivers.UART', 'Config');

        viewElem.baseAddr      = "0x" + (tree[elem].hwAttrs.baseAddr).toString(16);

        if (tree[elem].object.writeTimeout == timeoutWaitForever ) {
            viewElem.writeTimeout = "BIOS_WAIT_FOREVER";
        }
        else {
            viewElem.writeTimeout = Number(tree[elem].object.writeTimeout).toString(10);
        }

        if (tree[elem].object.readTimeout == timeoutWaitForever ) {
            viewElem.readTimeout = "BIOS_WAIT_FOREVER";
        }
        else {
            viewElem.readTimeout = Number(tree[elem].object.readTimeout).toString(10);
        }

        switch (tree[elem].object.writeMode) {
            case UART.UART_MODE_BLOCKING:
                viewElem.writeMode = "Blocking";
                break;
            case UART.UART_MODE_CALLBACK:
                viewElem.writeMode = "Callback";
                break;
        }

        switch (tree[elem].object.readMode) {
            case UART.UART_MODE_BLOCKING:
                viewElem.readMode = "Blocking";
                break;
            case UART.UART_MODE_CALLBACK:
                viewElem.readMode = "Callback";
                break;
        }

        switch (tree[elem].object.readReturnMode) {
            case UART.UART_RETURN_FULL:
                viewElem.readReturnMode = "Full";
                break;
            case UART.UART_RETURN_NEWLINE:
                viewElem.readReturnMode = "Newline";
                break;
        }

        switch (tree[elem].object.readDataMode) {
            case UART.UART_DATA_BINARY:
                viewElem.readDataMode = "Binary";
                break;
            case UART.UART_DATA_TEXT:
                viewElem.readDataMode = "Text";
                break;
        }

        switch (tree[elem].object.writeDataMode) {
            case UART.UART_DATA_BINARY:
                viewElem.writeDataMode = "Binary";
                break;
            case UART.UART_DATA_TEXT:
                viewElem.writeDataMode = "Text";
                break;
        }

        switch (tree[elem].object.readEcho) {
            case UART.UART_ECHO_OFF:
                viewElem.readEcho = "Off";
                break;
            case UART.UART_ECHO_ON:
                viewElem.readEcho = "On";
                break;
        }

        /* Add this element to the array of elements */
        eventViews[eventViews.length] = viewElem;
    }

    view.elements = eventViews;
}

/*
 *  ======== viewInitWriteBuffer ========
 */
function viewInitWriteBuffer(view)
{
    var Program = xdc.useModule('xdc.rov.Program');
    var UART = xdc.useModule('ti.drivers.UART');

    var tree = viewDetailed();

    /* List if entries for the ROV module */
    var eventViews = new Array();

    for (var elem in tree) {

        if (Number(tree[elem].object.writeSize) != 0) {

            var numChars = Number(tree[elem].object.writeSize);
            var writeBuffer = Program.fetchArray(
                {type: 'xdc.rov.support.ScalarStructs.S_UChar', isScalar: true},
                Number(tree[elem].object.writeBuf), numChars);

            var hexBuffer = new String();
            var stringBuffer = new String();
            for (var j = 0; j < numChars; j++) {
                hexBuffer += writeBuffer[j].toString(16) + " ";
                stringBuffer += String.fromCharCode(writeBuffer[j]);
            }

            /* Create a new element for ROV */
            var viewElem = Program.newViewStruct('ti.drivers.UART', 'Write Buffer');
            viewElem.Base = "0x" + Number(tree[elem].hwAttrs.baseAddr).toString(16);
            viewElem.Format = "String";
            viewElem.Contents = stringBuffer;
            eventViews[eventViews.length] = viewElem;

            var viewElem2 = Program.newViewStruct('ti.drivers.UART', 'Write Buffer');
            viewElem2.Base = "";
            viewElem2.Format = "Hex";
            viewElem2.Contents = hexBuffer;
            eventViews[eventViews.length] = viewElem2;
        }
    }

    view.elements = eventViews;
}

/*
 *  ======== viewInitReadBuffer ========
 */
function viewInitReadBuffer(view)
{
var Program = xdc.useModule('xdc.rov.Program');
    var UART = xdc.useModule('ti.drivers.UART');

    var tree = viewDetailed();

    /* List if entries for the ROV module */
    var eventViews = new Array();

    for (var elem in tree) {

        if (Number(tree[elem].object.readCount) != 0) {

            var numChars = Number(tree[elem].object.readCount);
            var addr = Number(tree[elem].object.readBuf) - (numChars *
                Program.build.target.stdTypes.t_Char.size);
            var readBuffer = Program.fetchArray(
                {type: 'xdc.rov.support.ScalarStructs.S_UChar', isScalar: true},
                addr, numChars);

            var hexBuffer = new String();
            var stringBuffer = new String();
            for (var j = 0; j < numChars; j++) {
                hexBuffer += readBuffer[j].toString(16) + " ";
                stringBuffer += String.fromCharCode(readBuffer[j]);
            }

            /* Create a new element for ROV */
            var viewElem = Program.newViewStruct('ti.drivers.UART', 'Read Buffer');
            viewElem.Base = "0x" + Number(tree[elem].hwAttrs.baseAddr).toString(16);
            viewElem.Format = "String";
            viewElem.Contents = stringBuffer;
            eventViews[eventViews.length] = viewElem;

            var viewElem2 = Program.newViewStruct('ti.drivers.UART', 'Read Buffer');
            viewElem2.Base = "";
            viewElem2.Format = "Hex";
            viewElem2.Contents = hexBuffer;
            eventViews[eventViews.length] = viewElem2;
        }
    }

    view.elements = eventViews;
}

/*
 *  ======== viewDetailed ========
 */
function viewDetailed()
{
    var file = xdc.findFile("ti/drivers/rovdriver.xs");
    if (file) {
        /* Include rovdriver.xs' functions */
        xdc.includeFile(file);

        var uartDrivers = new Array();
        uartDrivers.push({
                        name: 'UARTTiva_fxnTable',
                        object: 'UARTTiva_Object',
                        hwAttrs: 'UARTTiva_HWAttrs'
                        });
        uartDrivers.push({
                        name: 'UARTUSCIA_fxnTable',
                        object: 'UARTUSCIA_Object',
                        hwAttrs: 'UARTUSCIA_HWAttrs'
                        });

        var tree = getDriverStructs("UART", uartDrivers);
        return (tree);
    }
}
