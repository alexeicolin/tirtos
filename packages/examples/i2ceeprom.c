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
 *  ======== i2ceeprom.c ========
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
#include <ti/drivers/I2C.h>

/* Example/Board Header files */
#include "Board.h"

#include <string.h>

I2C_Handle i2c;
UChar txBuffer[Board_EEPROMADDRESSIZE + Board_EEPROMPAGESIZE];
UChar rxBuffer[Board_EEPROMADDRESSIZE + Board_EEPROMPAGESIZE];

/*
 *  ======== writeEEPROM ========
 *  Function performs a write operation onto the EERPOM
 *
 *  IMPORTANT:
 *  The transmitBuffer must include the EEPROMs internal address pointer of
 *  size txAddressLength at the beginning of the transmitBuffer array.
 *  To send a 64 bytes of data, the transmitBuffer and writeCount  must be 64
 *  bytes + txAddressLength bytes in size.
 */
UInt writeEEPROM(UChar slaveAddress, UChar *transmitBuffer,
                 UInt txAddressLength, UInt txDataLength)
{
    UInt transferOK;
    I2C_Transaction i2cTransaction;

    i2cTransaction.slaveAddress = slaveAddress; /* 7-bit slave address */
    i2cTransaction.writeBuf = transmitBuffer; /* txBuffer to be TX'd */
    /* Number of bytes to be TX'd */
    i2cTransaction.writeCount = txAddressLength + txDataLength;
    i2cTransaction.readBuf = NULL;           /* rxBuffer to be RX'd */
    i2cTransaction.readCount = 0;             /* Number of bytes to be RX'd */
    transferOK = I2C_transfer(i2c, &i2cTransaction); /* Perform I2C transfer */

    return (transferOK);
}

/*
 *  ======== readEEPROM ========
 *  Function performs a read operation from the EEPROM
 *
 *  IMPORTANT:
 *  The receiveBuffer must include the EEPROMs internal address pointer of
 *  size rxAddressLength at the beginning of the receiveBuffer array.
 *  To receive 64 bytes of data, the receiveBuffer must be 64 bytes +
 *  rxAddressLength bytes in size.
 */
UInt readEEPROM(UChar slaveAddress, UChar *receiveBuffer,
                UInt rxAddressLength, UInt rxDataLength)
{
    UInt transferOK;
    I2C_Transaction i2cTransaction;

    /* Read the Erased Page in the EEPROM */
    i2cTransaction.slaveAddress = slaveAddress; /* 7-bit slave address */
    i2cTransaction.writeBuf = receiveBuffer; /* txBuffer to be TX'd */
    i2cTransaction.writeCount = rxAddressLength; /* Number of bytes to TX */
    /* rxBuffer to be RX'd */
    i2cTransaction.readBuf = receiveBuffer + rxAddressLength;
    i2cTransaction.readCount = rxDataLength; /* Number of bytes to RX */
    transferOK = I2C_transfer(i2c, &i2cTransaction); /* Perform I2C transfer */

    return (transferOK);
}

/*
 *  ======== waitEEPROM ========
 *  The waitEEPROM function polls the EEPROM to determine if it's ready receive
 *  another I2C command immediately after a write function has been called.
 *
 *  EEPROMs require some significant amount of time to complete a write
 *  operation, therefore we need to check when its ready for the next command.
 */
Void waitEEPROM(UChar slaveAddress)
{
    UChar txBuffer[1];
    I2C_Transaction i2cTransaction;

    /* Wait for the EEPROM to be ready for the next read/write operation */
    txBuffer[0] = 0x00;                           /* Some dummy value */
    i2cTransaction.slaveAddress = slaveAddress; /* 7-bit slave address */
    i2cTransaction.writeBuf = txBuffer;      /* txBuffer to be TX'd */
    i2cTransaction.writeCount = 1;            /* Number of bytes to be TX'd */
    i2cTransaction.readBuf = NULL;          /* rxBuffer to be RX'd */
    i2cTransaction.readCount = 0;            /* Number of bytes to be RX'd */
    /*
     * Perform I2C transfer until a good I2C has been ACK'd by the slave
     * indicating that the I2C slave is now ready for the next command
     */
    while (!I2C_transfer(i2c, &i2cTransaction)) {
    }
}

/*
 *  ======== taskFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void taskFxn(UArg arg0, UArg arg1)
{
    UInt        i;
    UInt        transferOK;
    I2C_Params  i2cParams;

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_EEPROM, &i2cParams);
    if (i2c == NULL) {
        System_abort("Error Initializing I2C\n");
    }
    else {
        System_printf("I2C Initialized!\n");
    }

    /* Set txBuffer and rxBuffer EEPROM byte address */
    for (i = 0; i < Board_EEPROMADDRESSIZE; i++) {
        txBuffer[i] = Board_EEPROMSTARTADDR >> (i * 8); /* EEPROM TX Address */
        rxBuffer[i] = Board_EEPROMSTARTADDR >> (i * 8); /* EEPROM RX Address */
    }

    /* Write a page with all 0xFF's into the EEPROM */
    for (i = Board_EEPROMADDRESSIZE;
                      i < Board_EEPROMADDRESSIZE + Board_EEPROMPAGESIZE; i++) {
        /* Fill with "Erased" content */
        txBuffer[i] = 0xFF;
    }
    transferOK = writeEEPROM(Board_EEPROMADDRESS, txBuffer,
                             Board_EEPROMADDRESSIZE, Board_EEPROMPAGESIZE);
    if (!transferOK) {
        System_abort("Error in I2C Transfer\n");
    }

    waitEEPROM(Board_EEPROMADDRESS);

    /* Read a page from the EEPROM */
    transferOK = readEEPROM(Board_EEPROMADDRESS, rxBuffer,
                            Board_EEPROMADDRESSIZE, Board_EEPROMPAGESIZE);
    if (!transferOK) {
        System_abort("Error in I2C Transfer\n");
    }

    /* Compare memory contents */
    if (memcmp(txBuffer, rxBuffer,
               Board_EEPROMADDRESSIZE + Board_EEPROMPAGESIZE) == 0) {
        System_printf("Page has been erased\n");
    }
    else {
        System_printf("Page was NOT erased\n");
    }


    /* Write a page with incremented values into the EEPROM */
    for (i = Board_EEPROMADDRESSIZE;
                      i < Board_EEPROMADDRESSIZE + Board_EEPROMPAGESIZE; i++) {
        txBuffer[i] = i - Board_EEPROMADDRESSIZE;
    }
    transferOK = writeEEPROM(Board_EEPROMADDRESS, txBuffer,
                             Board_EEPROMADDRESSIZE, Board_EEPROMPAGESIZE);
    if (!transferOK) {
        System_abort("Error in I2C Transfer\n");
    }

    waitEEPROM(Board_EEPROMADDRESS);


    /* Read a page from the EEPROM */
    transferOK = readEEPROM(Board_EEPROMADDRESS, rxBuffer,
                            Board_EEPROMADDRESSIZE, Board_EEPROMPAGESIZE);
    if (!transferOK) {
        System_abort("Error in I2C Transfer\n");
    }

    /* Compare memory contents */
    if (memcmp(txBuffer, rxBuffer,
               Board_EEPROMADDRESSIZE + Board_EEPROMPAGESIZE) == 0) {
        System_printf("Page was successfully written with data\n");
    }
    else {
        System_printf("Page was NOT written with data\n");
    }

    /* Deinitialized I2C */
    I2C_close(i2c);
    System_printf("I2C Deinitialized!\n");

    System_printf("Done\n");

    System_flush();

    /*
     * Spin to avoid SDOCM00105002 "TI-RTOS examples with no heap get an error
     *                              if they call BIOS_exit"
     */
    while(true);
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

    System_printf("Starting the I2C example\nSystem provider is set to "
                  "SysMin. Halt the target and use ROV to view output.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
