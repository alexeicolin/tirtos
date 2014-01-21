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
 *    from this software without specific prior written permispion.
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
 *  ======== Driver.rov.xs ========
 */

/*
 *  ======== getDriverStructs ========
 *  This function should be moved to a generic ti.drivers.Driver module
 *
 *  This function generated a hash table of a given driver's _config structure
 *  and a list of specific driver implmementations (symbol name)
 *
 *  topLevelModuleName:     specific driver module to generate a hash table for
 *  driverImplementations:  array of driver implmementation specific function
 *                          table, object and hwAttrs pointers
 *
 *  Usage (for detailed view):
 *      var spiDrivers = new Array();
 *      spiDrivers.push({
 *                  name:    'SPITivaDMA_FxnTable',
 *                  object:  'SPITivaDMA_object',
 *                  hwAttrs: 'SPITivaDMA_hwAttrs'
 *                  });
 *      // Add additional driver implementations here
 *      spiDrivers.push({
 *                  name:    'SPITypeB_FxnTable',
 *                  object:  'SPITypeB_object',
 *                  hwAttrs: 'SPITypeB_hwAttrs'
 *                  });
 *
 *      var tree = getDriverStructs("ti.drivers.SPI", "SPI", spiDrivers);
 *      return (tree);
 *
 *  The generated hash table will look something like the following:
 *
 *  tree[address of driver's handle]
 *      |
 *      |- ["hwAttrs"]
 *      |----driver implmentation specific variables (e.g. baseAddr, intNum,...)
 *      |
 *      |- ["object"]
 *      |----driver implmentation specific variables (e.g. hwi, txCount, etc...)
 *      |
 *      |- ["fxnTablePtr"]: same as driverImplementations["name"]
 */
function getDriverStructs(topLevelModuleName, driverImplementations)
{
    /* Use the indicated module */
    var driver = xdc.useModule("ti.drivers." + topLevelModuleName);

    /* Hold the entire tree in this array */
    var tree = new Array();

    /*
     * Get the address of the top level Driver_config structure indicated by the
     * .map file. We will use this to determine which Driver implementations we
     * have in our current application and open up the associated drivers.
     */
    var driverConfigSymbol = Program.getSymbolValue(topLevelModuleName + "_config");
    if (driverConfigSymbol == -1) {
        /*
         * Driver_config symbol was not found in the map file. This probably
         * means that you aren't using the SPI driver and the linker optimized y
         * you application by removing it
         */
        return tree;
    }

    /* Create a new hash table based on the driver handle's address */
    var fxnTablePtrs = new Array();
    for (var drv in driverImplementations) {
        /* Try to find and get the address of the implementation specific driver */
        var index = (Program.getSymbolValue(driverImplementations[drv].name)).toString(16);

        /*
         * If found, create a new entry at the indicated key with the associated
         * implementation specific symbols.
         */
        if (index != "-1") {
            fxnTablePtrs[index] = driverImplementations[drv];
        }
    }

    /*
     * Extract every Handle (an entry in the config struct) from Driver_config
     * Hard-coded a limit of 32 peripherals, just in case the user didn't
     * NULL-terminate the _config[] array.
     */
    for (var i = 0; i <= 32; i++) {
        /* Get the address of the next Handle */
        var desc = topLevelModuleName + "_Config$fetchDesc";
        var driverHandleAddr = Number(driverConfigSymbol) + (xdc.om[driver[desc].type].$sizeof() * i);

        /* Fetch the structure from memory */
        var driverHandle = Program.fetchStruct(driver[desc], driverHandleAddr.toString());

        /* Check to see if the array has terminated with a NULL */
        if (driverHandle.fxnTablePtr == 0) {
            break;
        }

        /* Format the function pointer to be used with the hash table */
        var functionPtrString = Number((driverHandle.fxnTablePtr)).toString(16);

        /* Get the hwAttrs from the target */
        var hwattrsString = fxnTablePtrs[functionPtrString].hwAttrs + "$fetchDesc";
        var hwattrs    = Program.fetchStruct(driver[hwattrsString], driverHandle.hwAttrs);

        /* Get the object from the target */
        var objectString  = fxnTablePtrs[functionPtrString].object  + "$fetchDesc";
        var object     = Program.fetchStruct(driver[objectString],  driverHandle.object);

        /* Remove metadata from the extracted target object */
        delete object.$addr;
        delete object.$type;
        delete object.transaction$fetchDesc;

        /* Remove metadata from the extracted target hwAttrs */
        delete hwattrs.$addr;
        delete hwattrs.$type;

        /* Create a new entry for this peripheral */
        var peripheral = new Array();
        peripheral["hwAttrs"]      = hwattrs;
        peripheral["object"]       = object;
        peripheral["fxnTablePtr"]  = fxnTablePtrs[functionPtrString].name;

        /* Add this entry to the hash table */
        tree["0x" + driverHandleAddr.toString(16)] = peripheral;
    }

    return(tree);
}
