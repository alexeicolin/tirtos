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
 *    ======== usbcdcmousedevice.c ========
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
#include "USBCDCMOUSE.h"

#include <string.h>

typedef signed char offsetCoordinates[2];

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

const UChar text[] = "TI-RTOS controls USB.\r\n";

/*
 *  ======== mouseCursorFxn ========
 */
Void mouseCursorFxn(UArg arg0, UArg arg1)
{
    USBCDCMOUSE_State state;

    Int i;
    Int j;

    while (TRUE) {

        /* Block while the device is NOT connected to the USB */
        USBCDCMOUSE_waitForConnect(BIOS_WAIT_FOREVER);

        /* For each entry in the lookup table */
        for (j = 0; mouseLookupTable[j][0] || mouseLookupTable[j][1]; j++) {

            /* Perform the action i times to make it look gradual */
            for (i = 0; i < 20; i++) {
                state.deltaX = mouseLookupTable[j][0];
                state.deltaY = mouseLookupTable[j][1];
                state.button1 = GPIO_read(Board_BUTTON0) ? FALSE : TRUE;
                state.button2 = FALSE;
                state.button3 = FALSE;
                USBCDCMOUSE_setState(&state, BIOS_WAIT_FOREVER);
            }
        }
    }
}

/*
 *  ======== cdcPeriodicFxn ========
 *  Task to transmit serial data.
 *
 *  This task periodically sends data to the USB host once it's connected.
 */
Void cdcPeriodicFxn(UArg arg0, UArg arg1)
{
    while (TRUE) {

        /* Block while the device is NOT connected to the USB */
        USBCDCMOUSE_waitForConnect(BIOS_WAIT_FOREVER);

        USBCDCMOUSE_sendData(text, sizeof(text), BIOS_WAIT_FOREVER);
        GPIO_toggle(Board_LED0);

        /* Send data periodically */
        Task_sleep(2000);
    }
}

/*
 *  ======== cdcRecvFxn ========
 *  Task to receive serial data.
 *
 *  This task will receive data when data is available and block while the
 *  device is not connected to the USB host or if no data was received.
 */
Void cdcRecvFxn(UArg arg0, UArg arg1)
{
    UInt received;
    UChar data[32];

    while (TRUE) {

        /* Block while the device is NOT connected to the USB */
        USBCDCMOUSE_waitForConnect(BIOS_WAIT_FOREVER);

        received = USBCDCMOUSE_receiveData(data, 31, BIOS_WAIT_FOREVER);
        data[received] = '\0';
        GPIO_toggle(Board_LED1);
        if (received) {
            System_printf("Rcvd \"%s\" (%d bytes)\r\n", data, received);
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

    System_printf("Starting the USB CDC Mouse Device example\nSystem provider "
                  "is set to SysMin. Halt the target and use ROV to view "
                  "output.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    USBCDCMOUSE_init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
