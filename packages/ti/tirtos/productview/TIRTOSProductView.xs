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
 *  ======== TIRTOSProductView.xs ========
 *
 */
function getDriverList(deviceName) {
    var drivers = [];

    if(deviceName.match(/F28M3.*/)) {
         drivers.push("EMAC", "GPIO", "I2C", "SDSPI", "SPI", "UART", "USBMSCHFatFs", "Watchdog");
    }
    else if(deviceName.match(/LM4F.*/)) {
        drivers.push("GPIO", "I2C", "SDSPI", "SPI", "UART", "USBMSCHFatFs", "Watchdog", "WiFi");
    }
    else if(deviceName.match(/TM4C.*/)) {
        drivers.push("EMAC", "GPIO", "I2C", "SDSPI", "SPI", "UART", "USBMSCHFatFs", "Watchdog", "WiFi");
    }
    else if (deviceName.match(/MSP430.*/)) {
        drivers.push("GPIO", "I2C", "SDSPI", "SPI", "UART", "Watchdog", "WiFi");
    }
    else {
        /* Temporarily used for Tiva/Stellaris. */
        drivers.push("EMAC", "GPIO", "I2C", "SDSPI", "SPI", "UART", "USBMSCHFatFs", "Watchdog", "WiFi");
    }

    /* Return a list of drivers */
    return drivers;
}

function getDrivers(instance, drivers) {
    var driverArray = [];

    /* Add drivers to Available Products */
    for(var i = 0; i < drivers.length; ++i) {
        if(drivers[i] == "EMAC") {
            instance.ti_drivers_EMAC.elemName = "EMAC Driver";
            instance.ti_drivers_EMAC.moduleName = "ti.drivers.EMAC";
            instance.ti_drivers_EMAC.iconFileName = "icons/dev.ico";
            driverArray.push(instance.ti_drivers_EMAC);
        }
        else if(drivers[i] == "GPIO") {
            instance.ti_drivers_GPIO.elemName = "GPIO Driver";
            instance.ti_drivers_GPIO.moduleName = "ti.drivers.GPIO";
            instance.ti_drivers_GPIO.iconFileName = "icons/dev.ico";
            driverArray.push(instance.ti_drivers_GPIO);
        }
        else if(drivers[i] == "I2C") {
            instance.ti_drivers_I2C.elemName = "I2C Driver";
            instance.ti_drivers_I2C.moduleName = "ti.drivers.I2C";
            instance.ti_drivers_I2C.iconFileName = "icons/dev.ico";
            driverArray.push(instance.ti_drivers_I2C);
        }
        else if(drivers[i] == "SDSPI") {
            instance.ti_drivers_SDSPI.elemName = "SDSPI Driver";
            instance.ti_drivers_SDSPI.moduleName = "ti.drivers.SDSPI";
            instance.ti_drivers_SDSPI.iconFileName = "icons/dev.ico";
            driverArray.push(instance.ti_drivers_SDSPI);
        }
        else if(drivers[i] == "SPI") {
            instance.ti_drivers_SPI.elemName = "SPI Driver";
            instance.ti_drivers_SPI.moduleName = "ti.drivers.SPI";
            instance.ti_drivers_SPI.iconFileName = "icons/dev.ico";
            driverArray.push(instance.ti_drivers_SPI);
        }
        else if(drivers[i] == "UART") {
            instance.ti_drivers_UART.elemName = "UART Driver";
            instance.ti_drivers_UART.moduleName = "ti.drivers.UART";
            instance.ti_drivers_UART.iconFileName = "icons/dev.ico";
            driverArray.push(instance.ti_drivers_UART);
        }
        else if(drivers[i] == "USBMSCHFatFs") {
            instance.ti_drivers_USBMSCHFatFs.elemName = "USBMSCHFatFs Driver";
            instance.ti_drivers_USBMSCHFatFs.moduleName = "ti.drivers.USBMSCHFatFs";
            instance.ti_drivers_USBMSCHFatFs.iconFileName = "icons/dev.ico";
            driverArray.push(instance.ti_drivers_USBMSCHFatFs);
        }
        else if(drivers[i] == "Watchdog") {
            instance.ti_drivers_Watchdog.elemName = "Watchdog Driver";
            instance.ti_drivers_Watchdog.moduleName = "ti.drivers.Watchdog";
            instance.ti_drivers_Watchdog.iconFileName = "icons/dev.ico";
            driverArray.push(instance.ti_drivers_Watchdog);
        }
        else if(drivers[i] == "WiFi") {
            instance.ti_drivers_WiFi.elemName = "WiFi Driver";
            instance.ti_drivers_WiFi.moduleName = "ti.drivers.WiFi";
            instance.ti_drivers_WiFi.iconFileName = "icons/dev.ico";
            driverArray.push(instance.ti_drivers_WiFi);
        }
        else {
            print("DRIVER IS NOT SUPPORTED");
        }
    }
    /* Add the Drivers Sub-Group to the Available Products */
    instance.driverGroup.elemName = "Drivers";
    instance.driverGroup.moduleName = null;
    instance.driverGroup.iconFileName = "icons/dev.ico";
    instance.driverGroup.elemArray = driverArray;
}

function getMonitors(instance) {
    /* Add UARTMon */
    instance.ti_tirtos_utils_UARTMon.elemName = "UART Monitor";
    instance.ti_tirtos_utils_UARTMon.moduleName = "ti.tirtos.utils.UARTMon";
    instance.ti_tirtos_utils_UARTMon.iconFileName = "icons/uart.ico";

    /* Add Monitors */
    instance.monitors.elemName = "Monitors";
    instance.monitors.moduleName = null;
    instance.monitors.iconFileName = "icons/sys.ico";
    instance.monitors.elemArray = [
        instance.ti_tirtos_utils_UARTMon
    ];
}

function getProducts(instance, drivers) {
    /* SYS/BIOS */
    instance.ti_bios.iconFileName = "icons/bios.gif";
    instance.ti_bios.elemName = "SYSBIOS";
    instance.ti_bios.moduleName = null;

    /* NDK */
    instance.ti_ndk.iconFileName = "icons/bios.gif";
    instance.ti_ndk.elemName = "NDK";
    instance.ti_ndk.moduleName = null;

    /* UIA */
    instance.ti_uia.iconFileName = "icons/bios.gif";
    instance.ti_uia.elemName = "UIA";
    instance.ti_uia.moduleName = null;
}

function getSystemProviders(instance){
    /* Add SysFlex */
    instance.ti_tirtos_utils_SysFlex.elemName = "SysFlex";
    instance.ti_tirtos_utils_SysFlex.moduleName = "ti.tirtos.utils.SysFlex";
    instance.ti_tirtos_utils_SysFlex.iconFileName = "icons/sys.ico";

    /* Add System Providers */
    instance.systemProvidersGroup.elemName = "System Providers";
    instance.systemProvidersGroup.moduleName = null;
    instance.systemProvidersGroup.iconFileName = "icons/hst.ico";
    instance.systemProvidersGroup.elemArray = [
        instance.ti_tirtos_utils_SysFlex
    ];
}

function module$meta$init()
{
    var Program = xdc.useModule("xdc.cfg.Program");
    var drivers = [];

    this.ti_tirtos_TIRTOS.elemName = "TI-RTOS";
    this.ti_tirtos_TIRTOS.moduleName = "ti.tirtos.TIRTOS";
    this.ti_tirtos_TIRTOS.iconFileName = "icons/gbl.ico";

    /* Determine device */
    var deviceName = "none";
    try {
        deviceName = Program.cpu.deviceName;
    } catch (e) {
        print("Device name not found.");
    }
    drivers = getDriverList(deviceName);

    /* Add Sub-Groups to Available Products */
    getDrivers(this, drivers);
    getMonitors(this);
    getProducts(this, drivers);
    getSystemProviders(this);
}

/*
 *  ======== getProductDescriptor ========
 */
function getProductDescriptor()
{
    /* Products group
     *     first get each sub-product's ProductView tree,
     *     then create the array of sub-products
     *
     * Note: we load the sub-product modules here (rather than in
     * module$meta$init()) to ensure that this module can be loaded in
     * contexts other than just during ProductView computations; e.g., in
     * plugingen and build tooling which try to validate the
     * existance/loadability of this module.
     */
    var ProductView = xdc.module("ti.sysbios.productview.BiosProductView");
    this.ti_bios.elemArray = ProductView.getProductDescriptor().elemArray;

    ProductView = xdc.module("ti.ndk.productview.NdkProductView");
    this.ti_ndk.elemArray = ProductView.getProductDescriptor().elemArray;

    ProductView = xdc.module("ti.uia.productview.UiaProductView");
    this.ti_uia.elemArray = ProductView.getProductDescriptor().elemArray;

    /* initialize the Product group's list of products */
    this.productGroup.elemName = "Products";
    this.productGroup.moduleName = null;
    this.productGroup.iconFileName = "icons/gbl.ico";
    this.productGroup.elemArray = [
        this.ti_bios,
        this.ti_ndk,
        this.ti_uia
    ];

    /* Top-level TI-RTOS product tree
     *
     * Note: we do the assignment below _after_ each of the individual
     * groups have been computed.  This is because assignment to maps
     * is a "deep copy"; so, modification of productGoup _after_ the
     * assignment to tirtosProduct.elemArray does _not_ affect any
     * values in tirtosProduct.elemArray, for example.
     */
    this.tirtosProduct.elemName = "TI-RTOS";
    this.tirtosProduct.moduleName = null;
    this.tirtosProduct.iconFileName = "icons/bios.gif";
    this.tirtosProduct.elemArray = [
        this.ti_tirtos_TIRTOS,
        this.driverGroup,
        this.systemProvidersGroup,
        this.monitors,
        this.productGroup
    ];

    return this.tirtosProduct;
}
