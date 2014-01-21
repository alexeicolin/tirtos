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
 *  ======== demo.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>

/* IPC Header files */
#include <ti/ipc/MessageQ.h>
#include <ti/ipc/MultiProc.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>

/* Example/Board Header file */
#include "TMDXDOCKH52C1.h"

#include "demo.h"
#include "USBCDCD_LoggerIdle.h"

#include <stdio.h>
#include <string.h>
#include <file.h>

/*
 * EXTERNALTEMP = 1: Use an external TMP102 I2C Temperature sensor
 * EXTERNALTEMP = 0: Use a software counter as a temperature sensor proxy
 */
#ifndef EXTERNALTEMP
#define EXTERNALTEMP 0
#endif

extern Void SerialInit();
extern Void SerialMain();

Float temperatureF = 32;
Float temperatureC = 0;
Int recordingEnabled = RECORDING_CLOSED;

/*
 *  ======== temperature_func ========
 *  Allocates a message and ping-pongs the message around the processors.
 *  A local message queue is created and a remote message queue is opened.
 *  Messages are sent to the remote message queue and retrieved from the
 *  local MessageQ.
 */
Void temperature_func(UArg arg0, UArg arg1)
{
    MessageQ_Msg     msg;
    MessageQ_Handle  messageQ;
    MessageQ_QueueId remoteQueueId;
    Int              status;
    Ptr              buf;
    HeapBuf_Handle   heapHandle;
    HeapBuf_Params   hbparams;
    SizeT            blockSize;
    UInt             numBlocks;
    UInt             i = 0;
    FILE            *dst;
    Char             logBuffer[40];
    SDSPI_Handle     sdspiHandle;
    Error_Block      eb;
#if EXTERNALTEMP
    Short            temperature;
    I2C_Handle       i2c;
    UChar            rxBuffer[2];

    /* Keep it in flash to save SRAM */
    I2C_Transaction i2cTransaction = {
                NULL,       /* TxBuffer */
                0U,         /* TxCount */

                rxBuffer,   /* RxBuffer */
                2U,         /* RxCount */

                (UChar)0x48 /* slaveAddress */
    };

    /* Init I2C */
    i2c = I2C_open(TMDXDOCKH52C1_I2C1, NULL);

    if (i2c == NULL) {
        System_abort("Error Initializing I2C\n");
    }

#else
    Int              change = 1;
#endif /* EXTERNALTEMP */

    /* Compute the blockSize & numBlocks for the HeapBuf */
    numBlocks = 2;
    blockSize = sizeof(TempMsg);

    /* Alloc a buffer from the default heap */
    buf = Memory_alloc(0, numBlocks * blockSize, 0, NULL);

    /*
     *  Create the heap that is used for allocating MessageQ messages.
     */
    Error_init(&eb);
    HeapBuf_Params_init(&hbparams);
    hbparams.align          = 0;
    hbparams.numBlocks      = numBlocks;
    hbparams.blockSize      = blockSize;
    hbparams.bufSize        = numBlocks * blockSize;
    hbparams.buf            = buf;
    heapHandle = HeapBuf_create(&hbparams, &eb);
    if (heapHandle == NULL) {
        System_abort("HeapBuf_create failed\n" );
    }

    /* Register default system heap with MessageQ */
    MessageQ_registerHeap((IHeap_Handle)(heapHandle), HEAPID);

    /* Create the local message queue */
    messageQ = MessageQ_create(M3QUEUENAME, NULL);
    if (messageQ == NULL) {
        System_abort("MessageQ_create failed\n" );
    }

    /* Open the remote message queue. Spin until it is ready. */
    do {
        status = MessageQ_open(C28QUEUENAME, &remoteQueueId);
        /*
         *  Sleep for 1 clock tick to avoid inundating remote processor
         *  with interrupts if open failed
         */
        if (status < 0) {
            Task_sleep(1);
        }
    } while (status < 0);

    /* Allocate a message to be ping-ponged around the processors */
    msg = MessageQ_alloc(HEAPID, sizeof(TempMsg));
    if (msg == NULL) {
       System_abort("MessageQ_alloc failed\n" );
    }

    MessageQ_setMsgId(msg, TEMPERATURE_CONVERSION);

    /*
     *  Send the message to the remote processor and wait for a message
     *  from the previous processor.
     */
    System_printf("Start the main loop\n");
    while (TRUE) {

        if (recordingEnabled == RECORDING_OPEN) {
            /* Start SD Card */
            sdspiHandle = SDSPI_open(TMDXDOCKH52C1_SDSPI0, 0, NULL);
            if (sdspiHandle == NULL) {
                System_abort("Error registering sd disk_* functions\n");
            }

            /* Open File on SD Card */
            dst = fopen("fat:0:CRec.csv", "w");
            if (!dst) {
                System_printf("Error opening file. Is the SD card inserted?\n");
                recordingEnabled = RECORDING_CLOSED;
            }
            else {
                /* Store the .csv header */
                System_sprintf(logBuffer, "i,temperature\n");
                fwrite(logBuffer, 1, strlen(logBuffer), dst);
                fflush(dst);
                recordingEnabled = RECORDING_WRITING;
                GPIO_write(TMDXDOCKH52C1_LD3, TMDXDOCKH52C1_LED_ON);
            }
        }
        else if (recordingEnabled == RECORDING_CLOSE) {
                /* Close file */
                fclose(dst);
                SDSPI_close(sdspiHandle);
                recordingEnabled = RECORDING_CLOSED;
                GPIO_write(TMDXDOCKH52C1_LD3, TMDXDOCKH52C1_LED_OFF);
        }
#if EXTERNALTEMP
        if (I2C_transfer(i2c, &i2cTransaction)) {
            temperature = rxBuffer[0] << 4 | rxBuffer[1] >> 4;
            if (rxBuffer[0] & 0x80) {
                temperature |= 0xF000;
            }
            temperatureC = temperature/16;
        }
        else {
            System_printf("Failed to read I2C\n");
        }
        Log_print1(Diags_USER1, "temperatureC = %d", temperatureC);
#else
        temperatureC = temperatureC + change;
        if (temperatureC >= 35) {
            change = -1;
        }
        else if (temperatureC <= 0) {
            change = 1;
        }
#endif
        /* Send the temperature to the C28 to convert to Celsius */
        ((TempMsg *)msg)->temperatureC = temperatureC;

        Log_print0(Diags_USER1, "sending msg");
        /* send the message to the remote processor */
        status = MessageQ_put(remoteQueueId, msg);
        if (status < 0) {
           System_abort("MessageQ_put had a failure/error\n");
        }
        Log_print0(Diags_USER1, "getting msg");

        /* Get a message */
        status = MessageQ_get(messageQ, &msg, MessageQ_FOREVER);
        if (status < 0) {
           System_abort("This should not happen since timeout is forever\n");
        }
        Log_print0(Diags_USER1, "got msg");

        /* Copy into the global variable so the webpage can display it */
        temperatureF = ((TempMsg *)msg)->temperatureF;

        if (recordingEnabled == RECORDING_WRITING) {
        Log_print0(Diags_USER1, "writing SD card");
            /* Format the logBuffer to a writeable char string */
            System_sprintf(logBuffer, "%u,%d\n", i++, (Int)temperatureC);

            /* Write buffer to the SD Card */
            fwrite(logBuffer, 1, strlen(logBuffer), dst);
            fflush(dst);
            Log_print0(Diags_USER1, "done with SD card");
        }

        Log_print0(Diags_USER1, "sleep");
        Task_sleep(200);
        Log_print0(Diags_USER1, "awake");
    }
}

/*
 *  ======== main ========
 */
Int main(Void)
{
    Task_Handle taskHandle;
    Task_Params taskParams;
    Error_Block eb;

    /* Set up the board specific items */
    TMDXDOCKH52C1_initGeneral();
    TMDXDOCKH52C1_initUART();
    TMDXDOCKH52C1_initGPIO();
    TMDXDOCKH52C1_initSDSPI();
    TMDXDOCKH52C1_initUSB(TMDXDOCKH52C1_USBDEVICE);
    TMDXDOCKH52C1_initEMAC();
#if EXTERNALTEMP
    TMDXDOCKH52C1_initI2C();
#endif /* EXTERNALTEMP */

    System_printf("Demo with HTTP, I2C, and SD\nSystem provider is set to SysMin, halt the target and use ROV to view output.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Turn on user LED 2 */
    GPIO_write(TMDXDOCKH52C1_LD2, TMDXDOCKH52C1_LED_ON);

    USBCDCD_init();
    SerialInit();

    /* Create the Tasks that communicates to the UART. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 640;
    taskParams.priority = 2;
    taskParams.instance->name = "Serial";
    Error_init(&eb);
    taskHandle = Task_create(SerialMain, &taskParams, &eb);
    if (taskHandle == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }

    /* Create the Task that communicates to I2C temperature sensor. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 1280;
    taskParams.priority = 10;
    taskParams.instance->name = "Temperature";
    taskHandle = Task_create(temperature_func, &taskParams, &eb);
    if (taskHandle == NULL) {
        System_printf("Task_create() failed!\n");
        BIOS_exit(0);
    }

    /* Start BIOS. Will not return from this call. */
    BIOS_start();

    return (0);
}
