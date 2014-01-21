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
 *  ======== EMAC.xs ========
 */

function viewInitBasic(view)
{
    var Program = xdc.useModule('xdc.rov.Program');
    var EMAC = xdc.useModule('ti.drivers.EMAC');

    var tree = viewDetailed();

    /* List if entries for the ROV module */
    var eventViews = new Array();

    for (var elem in tree) {

        /* Create a new element for ROV */
        var viewElem = Program.newViewStruct('ti.drivers.EMAC', 'Basic');
        viewElem.functionTable = tree[elem].fxnTablePtr;
        viewElem.emacHandle     = elem.toString(16);

        /* Add this element to the array of elements */
        eventViews[eventViews.length] = viewElem;
    }

    var modCfg = Program.getModuleConfig('ti.drivers.EMAC');
    viewElem.libType = modCfg.libType;

    var macAddress = Program.fetchArray(
                {type: 'xdc.rov.support.ScalarStructs.S_UChar', isScalar: true},
                Number(tree[elem].hwAttrs.macAddress), 6);

    viewElem.macAddress = macAddress[0].toString(16) + "-" +
                          macAddress[1].toString(16) + "-" +
                          macAddress[2].toString(16) + "-" +
                          macAddress[3].toString(16) + "-" +
                          macAddress[4].toString(16) + "-" +
                          macAddress[5].toString(16);


    var emacTivaSymbol = Program.getSymbolValue("EMACTiva_fxnTable");
    var emacSnowDataSymbol = Program.getSymbolValue("EMACSnow_private");
    if (emacSnowDataSymbol != -1) {
        var data = Program.fetchStruct(
                       EMAC.EMACSnow_Data$fetchDesc, emacSnowDataSymbol);

        /* linkUp field is either zero (down) or non-zero (UP) */
        if (data.linkUp) {
            viewElem.linkUp = true;
        }
        else {
           viewElem.linkUp = false;
        }
    }
    else if (emacTivaSymbol != -1) {
        var emacConfigSymbol = Program.getSymbolValue("EMAC_config");
        var emacHandle = Program.fetchStruct(EMAC.EMAC_Config$fetchDesc,
                                             emacConfigSymbol);
        var object = Program.fetchStruct(
                 EMAC.EMACTiva_Object$fetchDesc, emacHandle.object);

        /* linkUp field is either zero (down) or non-zero (UP) */
        if (object.linkUp) {
            viewElem.linkUp = true;
        }
        else {
           viewElem.linkUp = false;
        }
    }
    view.elements = eventViews;
}

/*
 *  ======== viewInitStats ========
 */
function viewInitStats(view)
{
    var Program = xdc.useModule('xdc.rov.Program');
    var EMAC = xdc.useModule('ti.drivers.EMAC');

    var tree = viewDetailed();

    /* List if entries for the ROV module */
    var eventViews = new Array();

    for (var elem in tree) {
        /* Create a new element for ROV */
        var viewElem = Program.newViewStruct('ti.drivers.EMAC', 'Statistics');
        eventViews[eventViews.length] = viewElem;
    }

    var emacTivaSymbol = Program.getSymbolValue("EMACTiva_fxnTable");
    var emacSnowDataSymbol = Program.getSymbolValue("EMACSnow_private");

    if (emacSnowDataSymbol != -1) {
        var data = Program.fetchStruct(
                       EMAC.EMACSnow_Data$fetchDesc, emacSnowDataSymbol);
        /* Fill in the values */
        viewElem.rxCount      = data.rxCount;
        viewElem.rxDropped    = data.rxDropped;
        viewElem.txSent       = data.txSent;
        viewElem.txDropped    = data.txDropped;
    }
    else if (emacTivaSymbol != -1) {
        var emacConfigSymbol = Program.getSymbolValue("EMAC_config");
        var emacHandle = Program.fetchStruct(EMAC.EMAC_Config$fetchDesc,
                                             emacConfigSymbol);
        var object = Program.fetchStruct(
                 EMAC.EMACTiva_Object$fetchDesc, emacHandle.object);

        viewElem.rxCount      = object.rxCount;
        viewElem.rxDropped    = object.rxDropped;
        viewElem.txSent       = object.txSent;
        viewElem.txDropped    = object.txDropped;
    }

    view.elements = eventViews;
}

/*
 *  ======== viewDetailed ========
 */
function viewDetailed()
{
    var xdc_cfg_Program = xdc.useModule('xdc.cfg.Program');

    var file = xdc.findFile("ti/drivers/rovdriver.xs");
    if (file) {
        /* Include rovdriver.xs' functions */
        xdc.includeFile(file);
        var emacDrivers = new Array();
        emacDrivers.push({
                        name: 'EMACTiva_fxnTable',
                        object: 'EMACTiva_Object',
                        hwAttrs: 'EMACTiva_HWAttrs'
                        });
        emacDrivers.push({
                        name: 'EMACSnow_fxnTable',
                        object: 'EMACSnow_Object',
                        hwAttrs: 'EMACSnow_HWAttrs'
                        });
        var tree = getDriverStructs("EMAC", emacDrivers);

        return (tree);
    }
}
