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
 *  ======== watchdog.c ========
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
#include <ti/drivers/Watchdog.h>

/* Example/Board Header files */
#include "Board.h"

/*
 *  ======== taskFxn ========
 *  Sets a flag if a button has been pressed. Task for this function is created
 *  statically. See the project's .cfg file.
 */
Void taskFxn(UArg arg0, UArg arg1)
{
    ULong           currButton;
    ULong           prevButton;
    Watchdog_Params params;
    Watchdog_Handle watchdog;
    Bool            flag = FALSE;  /* False if button has not been pressed */

    /* Create and enable a Watchdog with resets enabled */
    Watchdog_Params_init(&params);
    params.resetMode = Watchdog_RESET_ON;

    watchdog = Watchdog_open(Board_WATCHDOG0, &params);
    if (watchdog == NULL) {
        System_abort("Error opening Watchdog!\n");
    }

    while (TRUE) {
        /*
         *  This could be done with GPIO interrupts, but for this simple
         *  example polling is used to check the button.
         */

        /* Read GPIO pin */
        currButton = GPIO_read(Board_BUTTON0);

        /* Set a flag if the button was pressed */
        if ((currButton == 0) && (prevButton != 0)) {
            flag = 1;
        }
        prevButton = currButton;

        /* Clear the Watchdog and toggle the LED until a button press. */
        if (!flag) {
            Watchdog_clear(watchdog);
            GPIO_toggle(Board_LED0);
        }

        Task_sleep(500);
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
    Board_initWatchdog();

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the Watchdog example\nSystem provider is set to "
                  "SysMin. Halt the target and use ROV to view output.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
