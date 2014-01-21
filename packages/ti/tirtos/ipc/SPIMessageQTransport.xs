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
 *  ======== SPIMessageQTransport.xs ========
 */

var SPIMessageQTransport = null;
var Queue = null;
var Clock = null;
var MessageQ = null;
var SPI = null;
var Swi = null;

/*
 *  ======== module$use ========
 */
function module$use()
{
    SPIMessageQTransport = this;
    Queue     = xdc.useModule('ti.sysbios.knl.Queue');
    Clock     = xdc.useModule("ti.sysbios.knl.Clock");
    Swi       = xdc.useModule("ti.sysbios.knl.Swi");
    MessageQ  = xdc.useModule("ti.sdo.ipc.MessageQ");
    SPI       = xdc.useModule('ti.drivers.SPI');
    IMessageQTransport =
        xdc.useModule("ti.sdo.ipc.interfaces.IMessageQTransport");
}

/*
 *  ======== module$static$init ========
 */
function module$static$init(mod, params)
{
    if (params.errFxn != null) {
        mod.errFxn = params.errFxn;
    }
    else {
        mod.errFxn = SPIMessageQTransport.defaultErrFxn;
    }
}

/*
 *  ======== instance$static$init ========
 */
function instance$static$init(obj, procId, params)
{
    SPIMessageQTransport.$logError(
        "Static creation of SPIMessageQTransport instances is not supported.",
        SPIMessageQTransport);
}

/*
 *  ======== viewInitModule ========
 *  Initializes the Module view in ROV.
 */
function viewInitModule(view, mod)
{
    var SPIMessageQTransportCfg  = Program.getModuleConfig('ti.tirtos.ipc.SPIMessageQTransport');

    view.libType = SPIMessageQTransportCfg.libType;

    if (SPIMessageQTransportCfg.checksum == true) {
        view.checksum = "Enabled";
    }
    else {
        view.checksum = "Disabled";
    }
}

/*
 *  ======== viewInitProxy ========
 *  Initializes the Proxy view in ROV.
 */
function viewInitInstances(view, obj)
{
    var SPIMessageQTransport = xdc.useModule('ti.tirtos.ipc.SPIMessageQTransport');

    /* Retrieve the SPIMessageQTransport instance's name */
    view.ready = obj.ready;
    view.maxMsgSize = obj.maxMsgSize;
    if (obj.master == true) {
        view.mode = "Master";
    }
    else {
        view.mode = "Slave";
    }

    view.rxMsgDropped = obj.rxMsgDropped;
    view.txMsgDropped = obj.txMsgDropped;

    if (obj.priority == SPIMessageQTransport.Priority_NORMAL) {
        view.priority = "Normal";
    }
    else if (obj.priority == SPIMessageQTransport.Priority_HIGH) {
        view.priority = "High";
    }
    else {
        view.priority = "INVALID";
    }

    if (obj.handshake == SPIMessageQTransport.LinkStatus_DOWN) {
        view.handshakeCompleted = false;
    }
    else if (obj.handshake == SPIMessageQTransport.LinkStatus_UP) {
        view.handshakeCompleted = true;
    }
}
