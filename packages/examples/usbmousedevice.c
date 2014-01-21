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
 *  ======== usbmousedevice.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>

/* Example/Board Header files */
#include "Board.h"

/* USB Reference Module Header file */
#include "USBMD.h"

typedef signed char offsetCoordinates[2];

/* Constants */
const offsetCoordinates mouseLookupTable[] = {
    /* X,  Y */
    {  2,  0 },
    {  0,  2 },
    {  2,  0 },
    {  0, -2 },
    { -2,  0 },
    {  0,  2 },
    { -2,  0 },
    {  0, -2 }, /* These offset values move the cursor in a figure 8 pattern */

    /* Termination entry (X == 0) && (Y == 0) */
    {  0,  0 }
};

/*
 *  ======== taskFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void taskFxn(UArg arg0, UArg arg1)
{
    USBMD_State state;

    Int i;
    Int j;

    while (TRUE) {

        /* Block while the device is NOT connected to the USB */
        USBMD_waitForConnect(BIOS_WAIT_FOREVER);

        /* For each entry in the lookup table */
        for (j = 0; mouseLookupTable[j][0] || mouseLookupTable[j][1]; j++) {

            /* Perform the action i times to make it look gradual */
            for (i = 0; i < 20; i++) {
                state.deltaX = mouseLookupTable[j][0];
                state.deltaY = mouseLookupTable[j][1];
                state.button1 = GPIO_read(Board_BUTTON0) ? FALSE : TRUE;
                state.button2 = FALSE;
                state.button3 = FALSE;
                USBMD_setState(&state, BIOS_WAIT_FOREVER);
            }
        }
    }
}

/*
 *  ======== main ========
 */
Int main(Void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initUSB(Board_USBDEVICE);

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the USB Mouse Device example\nSystem provider is "
                  "set to SysMin. Halt the target and use ROV to view output.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    USBMD_init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
