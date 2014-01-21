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
 *  ======== i2ctpl0401evm.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/knl/Task.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

/* Example/Board Header files */
#include "Board.h"

typedef struct RGBCMD {
    /*
     * Starting location for the LED drivers I2C register pointer.
     * It identifies the LED cluster (RGB LED) on the EVM.
     * The LED driver increments this pointer as the next 3 bytes are written.
     */
    UChar LED;
    UChar B; /* Blue LED PWM value */
    UChar G; /* Green LED PWM value */
    UChar R; /* Red LED PWM value */
} RGBCMD;

/* Table of LED patterns */
const RGBCMD rgbcmd[] = {
    /* Pattern for RGB D6*/
    {0x82, 0x00, 0x00, 0x00},
    {0x82, 0xFF, 0x00, 0x00},
    {0x82, 0x00, 0xFF, 0x00},
    {0x82, 0x00, 0x00, 0xFF},
    {0x82, 0x80, 0x00, 0x80},
    {0x82, 0x00, 0x80, 0x00},
    {0x82, 0x00, 0x00, 0x80},
    {0x82, 0x00, 0x00, 0x00},

    /* Pattern for RGB D7 */
    {0x87, 0x00, 0x00, 0x00},
    {0x87, 0x20, 0x00, 0x00},
    {0x87, 0x40, 0x20, 0x00},
    {0x87, 0xC0, 0x40, 0x20},
    {0x87, 0xFF, 0xC0, 0x40},
    {0x87, 0xFF, 0xFF, 0xC0},
    {0x87, 0xFF, 0xFF, 0xFF},
    {0x87, 0xFF, 0xFF, 0xFF},
    {0x87, 0xC0, 0xFF, 0xFF},
    {0x87, 0x40, 0xC0, 0xFF},
    {0x87, 0x20, 0x40, 0xC0},
    {0x87, 0x00, 0x20, 0x40},
    {0x87, 0x00, 0x00, 0x20},
    {0x87, 0x00, 0x00, 0x00},

    /* End of Table */
    {0x00, 0x00, 0x00, 0x00}
};

Void cycleLED(UArg arg0, UArg arg1)
{
    UChar            i = 0;
    UChar           writeBuffer[4];
    I2C_Handle      handle;
    I2C_Params      i2cparams;
    I2C_Transaction i2c;

    I2C_Params_init(&i2cparams);
    i2cparams.bitRate = I2C_400kHz;
    handle = I2C_open(Board_I2C_TPL0401, &i2cparams);
    if (handle == NULL) {
        System_abort("I2C was not opened");
    }

    i2c.slaveAddress = 0x40;
    i2c.readCount = 0;
    i2c.readBuf = NULL;
    i2c.writeBuf = writeBuffer;

    /* Enable the PWM oscillator */
    writeBuffer[0] = 0x00;
    writeBuffer[1] = 0x81;
    i2c.writeCount = 2;
    if (!I2C_transfer(handle, &i2c)) {
        GPIO_write(Board_LED1, Board_LED_ON);
        System_abort("Bad I2C transfer!");
    }

    /* Bring the LEDs into PWM mode */
    writeBuffer[0] = 0x8C;
    writeBuffer[1] = 0xAA;
    writeBuffer[2] = 0xAA;
    i2c.writeCount = 3;
    if (!I2C_transfer(handle, &i2c)) {
        GPIO_write(Board_LED1, Board_LED_ON);
        System_abort("Bad I2C transfer!");
    }

    i2c.writeCount = 4;
    while (TRUE) {
        /* Point to the new LED pattern */
        i2c.writeBuf = (UChar *) &(rgbcmd[i]);

        if (!I2C_transfer(handle, &i2c)) {
            GPIO_write(Board_LED1, Board_LED_ON);
            System_abort("Bad I2C transfer!");
        }

        /* Reached the end of the RGB patterns; reset index */
        if (rgbcmd[++i].LED == 0x00) {
            i = 0;
        }
        Task_sleep(100);
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
    Board_initI2C();

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target and use ROV to view output.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
