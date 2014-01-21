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
 *    ======== uartconsole.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/utils/Load.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>

/* Example/Board Header files */
#include "Board.h"
#include "UARTUtils.h"
#include "USBCDCD_LoggerIdle.h"

#include <file.h>
#include <stdio.h>
#include <string.h>

/*
 *  ======== consoleFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void consoleFxn(UArg arg0, UArg arg1)
{
    Int sleepDur, count;
    UInt32 cpuLoad;
    Char input[128];

    count = 1;

    /* printf goes to the UART com port */
    printf("\f======== Welcome to the Console ========\n");
    printf("Enter a command followed by return.\n"
           "Type help for a list of commands.\n\n");

    printf("%d %% ", count++);
    fflush(stdout);

    /* Loop forever receiving commands */
    while(TRUE) {
        /* Get the user's input */
        scanf("%s", input);
        /* Flush the remaining characters from stdin since they are not used. */
        fflush(stdin);

        if (!strcmp(input, "load")) {
            /* Print the CPU load */
            cpuLoad = Load_getCPULoad();
            printf("CPU Load: %d\n", cpuLoad);
        }
        else if (!strcmp(input, "sleep")) {
            /* Put the task to sleep for X ms. */
            printf("Enter a duration (ms): ");
            fflush(stdout);
            scanf("%d", &sleepDur);
            fflush(stdin);
            Task_sleep(sleepDur);
        }
        else if (!strcmp(input, "exit")) {
            /* Exit the console task */
            printf("Are you sure you want to exit the console? Y/N: ");
            fflush(stdout);
            scanf("%s", input);
            fflush(stdin);
            if ((input[0] == 'y' || input[0] == 'Y') && input[1] == 0x00) {
                printf("Exiting console, goodbye.\n");
                Task_exit();
            }
        }
        else {
            /* Print a list of valid commands. */
            printf("Valid commands:\n"
                   "- load: Get the CPU and task load.\n"
                   "- sleep: Put the console task to sleep.\n"
                   "- exit: Exit the console task.\n");
        }

        printf("%d %% ", count++);
        fflush(stdout);
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
    Board_initUART();
    Board_initUSB(Board_USBDEVICE);

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the UART Console example\n");

    /*
     *  Add the UART device to the system.
     *  All UART peripherals must be setup and the module must be initialized
     *  before opening.  This is done by Board_initUART().  The functions used
     *  are implemented in UARTUtils.c.
     */
    add_device("UART", _MSA, UARTUtils_deviceopen,
               UARTUtils_deviceclose, UARTUtils_deviceread,
               UARTUtils_devicewrite, UARTUtils_devicelseek,
               UARTUtils_deviceunlink, UARTUtils_devicerename);

    /* Open UART0 for writing to stdout and set buffer */
    freopen("UART:0", "w", stdout);
    setvbuf(stdout, NULL, _IOLBF, 128);

    /* Open UART0 for reading from stdin and set buffer */
    freopen("UART:0", "r", stdin);
    setvbuf(stdin, NULL, _IOLBF, 128);

    /*
     *  Initialize UART port 0 used by SysCallback.  This and other SysCallback
     *  UART functions are implemented in UARTUtils.c. Calls to System_printf
     *  will go to UART0, the same as printf.
     */
    UARTUtils_systemInit(0);

    /* Initialize the USB CDC device for logging transport */
    USBCDCD_init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
