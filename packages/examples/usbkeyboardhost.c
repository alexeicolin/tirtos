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
 *  ======== usbkeyboardhost.c ========
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
#include "USBKBH.h"

/* A macro to decide what type of function is to be used to receive characters */
#define USEGETCHAR  1
#define BUFFLENGTH  0x20

/*
 *  ======== taskFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void taskFxn(UArg arg0, UArg arg1)
{
#if USEGETCHAR
    Int character;
#else
    Char lineBuff[BUFFLENGTH];
    Int rxCount;
#endif

    USBKBH_State state;

    while (TRUE) {

    	/* Block while the device is NOT connected to the USB */
    	USBKBH_waitForConnect(BIOS_WAIT_FOREVER);

        /* Determine the status of the keyboard */
        USBKBH_getState(&state);

        /* Update LED outputs */
        GPIO_write(Board_LED0, state.capsLED ? Board_LED_ON : Board_LED_OFF);
        GPIO_write(Board_LED1, state.scrollLED ? Board_LED_ON : Board_LED_OFF);

        /* Updated the keyboard's LED status */
        USBKBH_setState(&state);

#if USEGETCHAR /* Use the getChar method */
        /* Wait for a character for 100 tick; this allows for LEDs to update */
        character = USBKBH_getChar(100);

        if (character) {
            /* Print character if it received one */
            System_printf("%c", character);
        }

#else /* Pend task until it gets an entire line (<LF>) */
        rxCount = USBKBH_getString(lineBuff, BUFFLENGTH, BIOS_WAIT_FOREVER);
        System_printf("\"%s\" size %d\n", lineBuff, rxCount);
#endif
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
    Board_initUSB(Board_USBHOST);

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the USB Keyboard Host example\nSystem provider is "
                  "set to SysMin. Halt the target and use ROV to view output.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    USBKBH_init();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
