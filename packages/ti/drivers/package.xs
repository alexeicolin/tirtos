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
 *  ======== package.xs ========
 */

/*
 *  ======== getLibs ========
 *  This package does not build any libraries.
 */
function getLibs() {
    var EMAC = xdc.module("ti.drivers.EMAC");
    var GPIO = xdc.module("ti.drivers.GPIO");
    var I2C = xdc.module("ti.drivers.I2C");
    var SDSPI = xdc.module("ti.drivers.SDSPI");
    var UART = xdc.module("ti.drivers.UART");
    var USBMSCHFatFs = xdc.module("ti.drivers.USBMSCHFatFs");
    var SPI = xdc.module("ti.drivers.SPI");
    var Watchdog = xdc.module("ti.drivers.Watchdog");
    var WiFi = xdc.module("ti.drivers.WiFi");

    var retString = "";
    var driverGroup = "";
    var wiFiHostDriverType = "";

    /* Determine with driver group */
    if (Program.cpu.deviceName.match(/F28M3.*/)) {
        driverGroup = "_mware.aem3;";
    }
    else if (Program.cpu.deviceName.match(/LM4F.*/)) {
        if(Program.build.target.suffix == "em4f")
            driverGroup = "_tivaware.aem4f;";
        else if (Program.build.target.suffix == "rm4f")
            driverGroup = "_tivaware.arm4f;";
        else if (Program.build.target.suffix == "m4fg")
            driverGroup = "_tivaware.am4fg;";
    }
    else if (Program.cpu.deviceName.match(/TM4E.*/)) {
        if(Program.build.target.suffix == "em4f")
            driverGroup = "_tivaware.aem4f;";
        else if (Program.build.target.suffix == "rm4f")
            driverGroup = "_tivaware.arm4f;";
        else if (Program.build.target.suffix == "m4fg")
            driverGroup = "_tivaware.am4fg;";
    }
    else if (Program.cpu.deviceName.match(/TM4C.*/)) {
        if(Program.build.target.suffix == "em4f")
            driverGroup = "_tivaware.aem4f;";
        else if (Program.build.target.suffix == "rm4f")
            driverGroup = "_tivaware.arm4f;";
	else if (Program.build.target.suffix == "m4fg")
            driverGroup = "_tivaware.am4fg;";
    }
    else if (Program.build.target.suffix == "e430X") {
        driverGroup = "_" + Program.cpu.deviceName.match(/MSP430.*/) + ".ae430X;";
    }
    else if (Program.build.target.suffix == "r430XS") {
        driverGroup = "_" + Program.cpu.deviceName.match(/MSP430.*/) + ".ar430XS;";
    }
    else {
        throw ("Driver not found for this device " + Program.cpu.deviceName +
               " and target " + Program.build.target.suffix);
    }

    /* Add all the applicable drivers to retString */

    if (EMAC.$used == true) {
        retString += "lib/";
        switch (EMAC.libType) {
            case EMAC.LibType_Instrumented:
                retString += "instrumented/"
                break;
            case EMAC.LibType_NonInstrumented:
                retString += "nonInstrumented/"
                break;
        }

        retString += "emac" + driverGroup;
    }

    if (GPIO.$used == true) {
        retString += "lib/";
        switch (GPIO.libType) {
            case GPIO.LibType_Instrumented:
                retString += "instrumented/"
                break;
            case GPIO.LibType_NonInstrumented:
                retString += "nonInstrumented/"
                break;
        }
        retString += "gpio" + driverGroup;
    }

    if (I2C.$used == true) {
        retString += "lib/";
        switch (I2C.libType) {
            case I2C.LibType_Instrumented:
                retString += "instrumented/"
                break;
            case I2C.LibType_NonInstrumented:
                retString += "nonInstrumented/"
                break;
        }
        retString += "i2c" + driverGroup;
    }

    if (SDSPI.$used == true) {
        retString += "lib/";
        switch (SDSPI.libType) {
            case SDSPI.LibType_Instrumented:
                retString += "instrumented/"
                break;
            case SDSPI.LibType_NonInstrumented:
                retString += "nonInstrumented/"
                break;
        }
        retString += "sdspi" + driverGroup;
    }

    if (UART.$used == true) {
        retString += "lib/";
        switch (UART.libType) {
            case UART.LibType_Instrumented:
                retString += "instrumented/"
                break;
            case UART.LibType_NonInstrumented:
                retString += "nonInstrumented/"
                break;
        }
        retString += "uart" + driverGroup;
    }

    if (Watchdog.$used == true) {
        retString += "lib/";
        switch (Watchdog.libType) {
            case Watchdog.LibType_Instrumented:
                retString += "instrumented/"
                break;
            case Watchdog.LibType_NonInstrumented:
                retString += "nonInstrumented/"
                break;
        }
        retString += "watchdog" + driverGroup;
    }

    if (USBMSCHFatFs.$used == true) {
        retString += "lib/";
        switch (USBMSCHFatFs.libType) {
            case USBMSCHFatFs.LibType_Instrumented:
                retString += "instrumented/"
                break;
            case USBMSCHFatFs.LibType_NonInstrumented:
                retString += "nonInstrumented/"
                break;
        }
        retString += "usbmschfatfs" + driverGroup;
    }

    if (SPI.$used == true) {
        retString += "lib/";
        switch (SPI.libType) {
            case SPI.LibType_Instrumented:
                retString += "instrumented/"
                break;
            case SPI.LibType_NonInstrumented:
                retString += "nonInstrumented/"
                break;
        }
        retString += "spi" + driverGroup;
    }

    if (WiFi.$used == true) {
        retString += "lib/";
        switch (WiFi.libType) {
            case WiFi.LibType_Instrumented:
                retString += "instrumented/"
                break;
            case WiFi.LibType_NonInstrumented:
                retString += "nonInstrumented/"
                break;
        }
        switch (WiFi.hdType) {
            case WiFi.HDType_SingleThread:
                wiFiHostDriverType += "_singlethread"
                break;
            case WiFi.HDType_MultiThread:
                wiFiHostDriverType += "_multithread"
                break;
        }
        retString += "wifi" + wiFiHostDriverType + driverGroup;
    }

    return (retString);
};
