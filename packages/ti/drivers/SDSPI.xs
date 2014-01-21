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
 *  ======== SDSPI.xs ========
 */

var FatFS;
var ClockFreqs;
var Timestamp;

/*
 *  ======== module$use ========
 *  This function will automatically include the required FatFS and ClocksFreqs
 *  module from SYS/BIOS.
 */
function module$use()
{
    FatFS = xdc.useModule('ti.sysbios.fatfs.FatFS');
    Timestamp = xdc.useModule('xdc.runtime.Timestamp');
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
    var SDSPI = xdc.useModule('ti.drivers.SDSPI');

    var tree = viewDetailed();

    /* List if entries for the ROV module */
    var eventViews = new Array();

    for (var elem in tree) {

        /* Create a new element for ROV */
        var viewElem = Program.newViewStruct('ti.drivers.SDSPI', 'Basic');
        viewElem.baseAddr      = "0x" + (tree[elem].hwAttrs.baseAddr).toString(16);
        viewElem.functionTable = tree[elem].fxnTablePtr;
        viewElem.sdspiHandle   = elem.toString(16);

        /* Add this element to the array of elements */
        eventViews[eventViews.length] = viewElem;
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

        var sdspiDrivers = new Array();
        sdspiDrivers.push({
                        name: 'SDSPITiva_fxnTable',
                        object: 'SDSPITiva_Object',
                        hwAttrs: 'SDSPITiva_HWAttrs'
                        });
        sdspiDrivers.push({
                        name: 'SDSPIUSCIA_fxnTable',
                        object: 'SDSPIUSCIA_Object',
                        hwAttrs: 'SDSPIUSCIA_HWAttrs'
                        });
        sdspiDrivers.push({
                        name: 'SDSPIUSCIB_fxnTable',
                        object: 'SDSPIUSCIB_Object',
                        hwAttrs: 'SDSPIUSCIB_HWAttrs'
                        });

        var tree = getDriverStructs("SDSPI", sdspiDrivers);
        return (tree);
    }
}
