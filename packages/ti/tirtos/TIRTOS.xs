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
 *  ======== TIRTOS.xs ========
 */

var EMAC;
var GPIO;
var I2C;
var SDSPI;
var SPI;
var UART;
var USBMSCHFatFs;
var Watchdog;
var WiFi;

/*
 *  ======== module$use ========
 */
function module$use()
{
    EMAC         = xdc.module('ti.drivers.EMAC');
    GPIO         = xdc.module('ti.drivers.GPIO');
    I2C          = xdc.module('ti.drivers.I2C');
    SDSPI        = xdc.module('ti.drivers.SDSPI');
    SPI          = xdc.module('ti.drivers.SPI');
    UART         = xdc.module('ti.drivers.UART');
    USBMSCHFatFs = xdc.module('ti.drivers.USBMSCHFatFs');
    Watchdog     = xdc.module('ti.drivers.Watchdog');
    WiFi         = xdc.module('ti.drivers.WiFi');
}

function module$meta$init()
{
    var Program = xdc.useModule("xdc.cfg.Program");
    if (xdc.om.$name != "cfg") {
        return;
    }

    /* Enable/disable drivers according to the device family */
    var deviceName = "none";
    try {
        deviceName = Program.cpu.deviceName;
    } catch (e) {
        print("Device name not found.");
    }
    enableDrivers(this, deviceName);
}

function enableDrivers(instance, deviceName) {
    /* Disable drivers not supported for a device family */
    if(deviceName.match(/F28M3.*/)) {
        instance.wifiDriverCompatible = false;
    }
   else if(deviceName.match(/TM4C.*/)) {
       /* Compatible with all drivers. */
   }
   else if(deviceName.match(/LM4F.*/)) {
        instance.ndkCompatible = false;
        instance.emacDriverCompatible = false;
   }
    else if (deviceName.match(/MSP430.*/)) {
        instance.ndkCompatible = false;
        instance.emacDriverCompatible = false;
        instance.usbmschfatfsDriverCompatible = false;
    }
    else {
        /* Temporarily used for Tiva/Stellaris. */
    }
}

function module$validate()
{
    Program.global.TI_DRIVERS_EMAC_INCLUDED         = EMAC.$used;
    Program.global.TI_DRIVERS_GPIO_INCLUDED         = GPIO.$used;
    Program.global.TI_DRIVERS_I2C_INCLUDED          = I2C.$used;
    Program.global.TI_DRIVERS_SDSPI_INCLUDED        = SDSPI.$used;
    Program.global.TI_DRIVERS_SPI_INCLUDED          = SPI.$used;
    Program.global.TI_DRIVERS_UART_INCLUDED         = UART.$used;
    Program.global.TI_DRIVERS_USBMSCHFATFS_INCLUDED = USBMSCHFatFs.$used;
    Program.global.TI_DRIVERS_WATCHDOG_INCLUDED     = Watchdog.$used;
    Program.global.TI_DRIVERS_WIFI_INCLUDED         = WiFi.$used;
}