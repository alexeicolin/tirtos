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
 *    ======== i2crf430cl330_load.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/utils/Load.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>

/* Example/Board Header files */
#include "Board.h"

#include <stdio.h>
#include <stdlib.h>

#define CONTROL_REG         0xFFFE
#define STATUS_REG          0xFFFC
#define INT_ENABLE_REG      0xFFFA
#define INT_FLAG_REG        0xFFF8
#define CRC_RESULT_REG      0xFFF6
#define CRC_LENGTH_REG      0xFFF4
#define CRC_START_ADDR_REG  0xFFF2
#define COMM_WD_CTRL_REG    0xFFF0
#define VERSION_REG         0xFFEE //contains the software version of the ROM
#define TEST_FUNCTION_REG   0xFFE2
#define TEST_MODE_REG       0xFFE0

//define the different virtual register bits
//CONTROL_REG bits
#define SW_RESET            (1<<0)
#define RF_ENABLE           (1<<1)
#define INT_ENABLE          (1<<2)
#define INTO_HIGH           (1<<3)
#define INTO_DRIVE          (1<<4)
#define BIP8_ENABLE         (1<<5)
#define STANDBY_ENABLE      (1<<6)
#define TEST430_ENABLE      (1<<7)

//STATUS_REG bits
#define READY               (1<<0)
#define CRC_ACTIVE          (1<<1)
#define RF_BUSY             (1<<2)

//INT_ENABLE_REG bits
#define EOR_INT_ENABLE      (1<<1)
#define EOW_INT_ENABLE      (1<<2)
#define CRC_INT_ENABLE      (1<<3)
#define BIP8_ERROR_INT_ENABLE (1<<4)
#define NDEF_ERROR_INT_ENABLE (1<<5)
#define GENERIC_ERROR_INT_ENABLE (1<<7)

//INT_FLAG_REG bits
#define EOR_INT_FLAG        (1<<1)
#define EOW_INT_FLAG        (1<<2)
#define CRC_INT_FLAG        (1<<3)
#define BIP8_ERROR_INT_FLAG (1<<4)
#define NDEF_ERROR_INT_FLAG (1<<5)
#define GENERIC_ERROR_INT_FLAG (1<<7)

//COMM_WD_CTRL_REG bits
#define WD_ENABLE           (1<<0)
#define TIMEOUT_PERIOD_2_SEC 0
#define TIMEOUT_PERIOD_32_SEC (1<<1)
#define TIMEOUT_PERIOD_8_5_MIN (1<<2)
#define TIMEOUT_PERIOD_MASK 0x0E

#define RF430_DEFAULT_DATA        {                                            \
/*NDEF Tag Application Name*/                                                  \
0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01,                                      \
                                                                               \
/*Capability Container ID*/                                                    \
0xE1, 0x03,                                                                    \
0x00, 0x0F, /* CCLEN */                                                        \
0x20,       /* Mapping version 2.0 */                                          \
0x00, 0x3B, /* MLe (49 bytes); Maximum R-APDU data size */                     \
0x00, 0x34, /* MLc (52 bytes); Maximum C-APDU data size */                     \
0x04,       /* Tag, File Control TLV (4 = NDEF file) */                        \
0x06,       /* Length, File Control TLV (6 = 6 bytes of data for this tag) */  \
0xE1, 0x04, /* File Identifier */                                              \
0x0B, 0xDF, /* Max NDEF size (3037 bytes of useable memory) */                 \
0x00,       /* NDEF file read access condition, read access without any security */ \
0x00,       /* NDEF file write access condition; write access without any security */ \
                                                                               \
/* NDEF File ID */                                                             \
0xE1, 0x04,                                                                    \
                                                                               \
/* NDEF File for Hello World  (48 bytes total length) */                       \
0x00, 0x14, /* NLEN; NDEF length (3 byte long message) */                      \
0xD1, 0x01, 0x10,                                                              \
0x54, /* T = text */                                                           \
0x02,                                                                          \
0x65, 0x6E, /* 'e', 'n', */                                                    \
                                                                               \
'C','P','U',' ','L','o','a','d',':',' ',' ',' ',' '                            \
}

typedef struct MESSAGE_DATA {
    UShort dataAddr;
    UChar  message[0x80];
} MESSAGE_DATA;

static MESSAGE_DATA messageNDEF = {
    0x0000,
    RF430_DEFAULT_DATA
};

Void writeRegister(I2C_Handle handle, UShort regAddr, UShort value)
{
    UChar               txBuffer[4];
    I2C_Transaction     i2cTransaction;

    i2cTransaction.slaveAddress = 0x28;

    /* Write to a 16-bit status register */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 4;
    i2cTransaction.readCount = 0;

    txBuffer[0] = regAddr >> 8;   //HB Addr
    txBuffer[1] = regAddr & 0xFF; //LB Addr
    txBuffer[2] = value   & 0xFF;
    txBuffer[3] = value   >> 8;

    if (!I2C_transfer(handle, &i2cTransaction)) {
        GPIO_write(Board_LED1, Board_LED_ON);
        System_abort("Bad I2C transfer!");
    }
}

Void readRegister(I2C_Handle handle, UShort regAddr, UChar *data, UInt length)
{
    UChar               txBuffer[2];
    I2C_Transaction     i2cTransaction;

    i2cTransaction.slaveAddress = 0x28;

    /* Write to a 16-bit status register */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.readBuf = data;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readCount = length;

    txBuffer[0] = regAddr >> 8;   //HB Addr
    txBuffer[1] = regAddr & 0xFF; //LB Addr

    if (!I2C_transfer(handle, &i2cTransaction)) {
        GPIO_write(Board_LED1, Board_LED_ON);
        System_abort("Bad I2C transfer!");
    }
}

Void writedata(I2C_Handle handle, MESSAGE_DATA *data)
{
    I2C_Transaction i2cTransaction;
    i2cTransaction.slaveAddress = 0x28;

    /* Write to NDEF memory */
    i2cTransaction.writeBuf = (UChar *)data;
    i2cTransaction.writeCount = sizeof(MESSAGE_DATA);
    i2cTransaction.readCount = 0;

    if (!I2C_transfer(handle, &i2cTransaction)) {
        GPIO_write(Board_LED1, Board_LED_ON);
        System_abort("Bad I2C transfer!");
    }
}

/*
 *  ======== taskFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void nfcLoadTask(UArg arg0, UArg arg1)
{
    I2C_Handle          i2c;
    I2C_Params          i2cParams;

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_NFC, &i2cParams);
    if (i2c == NULL) {
        System_abort("Error Initializing I2C\n");
    }
    else {
        System_printf("I2C Initialized!\n");
    }

    /* Reset the device and wait */
    writeRegister(i2c, CONTROL_REG, SW_RESET);
    Task_sleep(1000);

    GPIO_write(Board_LED0, Board_LED_OFF);

    while (1) {
        writeRegister(i2c, CONTROL_REG, 0x00);
        /* Need to wait at least 1-2ms to let any RF comm. finish */
        Task_sleep(3);

        /* Write the latest status information */
        System_sprintf((Char *)&(messageNDEF.message[45]), "%3d",
                       Load_getCPULoad());
        writedata(i2c, &messageNDEF);

        writeRegister(i2c, CONTROL_REG, RF_ENABLE);
        Task_sleep(1000);
    }
}

typedef struct DUMMYTASKCFG {
    UInt    ticks;
    UInt    countInc;
    UInt    gpioIndex;
} DUMMYTASKCFG;

const DUMMYTASKCFG dummyCfg[] = {
    {100, 3, Board_LED0},
    {50, 10, Board_LED1},
    {2,  35, Board_LED2}
};

Void dummyTask(UArg arg0, UArg arg1)
{
    UInt    count = 0;
    UInt    index = (UInt) arg0;

    while(1) {
        Task_sleep(rand() % (dummyCfg[index].ticks + 1));
        count += (dummyCfg[index].countInc);

        GPIO_toggle(dummyCfg[index].gpioIndex);

        System_printf("Task sleep tick: %d; count %d\n",
                       dummyCfg[index].ticks, count);
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

    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the I2C example\nSystem provider is set to "
                  "SysMin. Halt the target and use ROV to view output.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    srand(0xBEA5);

    /* Start BIOS */
    BIOS_start();

    return (0);
}
