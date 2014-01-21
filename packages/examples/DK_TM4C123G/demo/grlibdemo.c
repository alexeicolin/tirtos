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
 *  ======== grlibdemo.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/fatfs/ff.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDSPI.h>

/* Graphiclib Header file */
#include <grlib/grlib.h>
#include "cfal96x64x16.h"

/* Example/Board Header file */
#include "DK_TM4C123G.h"
#include "USBCDCD.h"

#include <file.h>
#include <stdio.h>
#include <string.h>

/* Screen size */
#define SCREEN_WIDTH 96
#define SCREEN_HEIGHT 64

/* File-list size */
#define FILELIST_LENGTH 7

/* Characters count each line */
#define CHARS_ONE_LINE 17

/* Buffer length for the receive buffer of USBCDC */
#define RECEIVE_LENGTH 3

/* Drive number used for FatFs */
#define DRIVE_NUM 0

/*!
 *  ======== DrawCommand ========
 *  Enum defines several drawing commands
 */
typedef enum DrawCommand{
    IMAGE,
    COMMAND,
    STRING
} DrawCommand;

/*!
 *  ======== DrawMessage ========
 *  Drawing message contains information needed for drawing
 *
 *  @field(drawCommand)     drawing command for current message
 *  @field(drawImageIndex)  image index in the image list to draw
 *  @field(drawString)      string needed to draw
 *  @field(drawDirCount)    count of directories displayed on the screen
 *  @field(drawDir)         array holding strings of directories
 *                          (7 strings with width 16)
 */
typedef struct DrawMessage{
    DrawCommand drawCommand;
    UInt drawImageIndex;
    Char drawString[CHARS_ONE_LINE + 1];
    UInt drawDirCount;
    Char drawDir[FILELIST_LENGTH][CHARS_ONE_LINE + 1];
} DrawMessage;

/* Holding the last command */
UChar lastCommand[CHARS_ONE_LINE + 1] = {'\0'};

/* Images */
extern const UChar image_TI_Black[];
extern const UChar image_TI_Ad[];
const UChar *image_Gallery[2] = {image_TI_Black, image_TI_Ad};

/* Up/Down arrow keys (ASCII code converted from USB keyboard) */
const UChar downArrow[4] = {0x1b, 0x5b, 0x42, 0x00};
const UChar upArrow[4] = {0x1b, 0x5b, 0x41, 0x00};

/* Handles created dynamically in the c */
Mailbox_Handle mailboxHandle = NULL;

/* Global context for drawing */
tContext context;

/*
 *  ======== clearDisplay ========
 *  Clear the screen with black color
 */
Void clearDisplay()
{
    tRectangle rect = {0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1};
    GrContextForegroundSetTranslated(&context, 0);
    GrRectFill(&context, &rect);
}

/*
 *  ======== readDir ========
 *  Read directories from SDSPI and put into the message
 */
Void readDir(const char *path, DrawMessage *drawMsg, UInt startIdx,
             UInt endIdx)
{
    FRESULT res;
    DIR dir;
    FILINFO finfo;
    Int i;

    drawMsg->drawDirCount = 0;

    /* Open the directory */
    res = f_opendir(&dir, path);
    if (res == FR_OK) {
        for (i=0; i<=endIdx; i++) {
            /* Read files/folders from the directory */
            res = f_readdir(&dir, &finfo);
            if (res != FR_OK || finfo.fname[0] == 0) {
                /* Break on error or end of directory */
                break;
            }
            if (finfo.fname[0] == '.') {
                i--;
                continue;
            }

            if (i >= startIdx) {
                strncpy(drawMsg->drawDir[drawMsg->drawDirCount], finfo.fname,
                        CHARS_ONE_LINE);
                drawMsg->drawDir[drawMsg->drawDirCount][CHARS_ONE_LINE] = '\0';
                drawMsg->drawDirCount++;
            }
        }
    }
}

/*
 *  ======== grlibTaskFxn ========
 *  Drawing task
 *
 *  It is pending for the message either from console task or from button ISR.
 *  Once the messages received, it draws to the screen based on information
 *  contained in the message.
 */
Void grlibTaskFxn(UArg arg0, UArg arg1)
{
    DrawMessage curMsg;
    const UChar *pucCurImage;
    const Char *pcCurStr;
    UInt i = 0;
    UInt fontHeight = GrStringHeightGet(&context);

    while (TRUE) {
        Mailbox_pend(mailboxHandle, &curMsg, BIOS_WAIT_FOREVER);

        /* Clear screen before drawing */
        clearDisplay();

        /* Parse the message and draw */
        switch (curMsg.drawCommand) {
        case IMAGE:
            pucCurImage = image_Gallery[curMsg.drawImageIndex];

            /* Draw image at (0,0) */
            GrImageDraw(&context, pucCurImage, 0, 0);
            break;

        case COMMAND:
            pcCurStr = curMsg.drawString;

            /* Set foreground color white (0xFF) */
            GrContextForegroundSetTranslated(&context, 0xFF);

            /* Draw string at (0,0) */
            GrStringDraw(&context, pcCurStr, -1, 0, 0, 0);

            for (i = 0; i < curMsg.drawDirCount; i++) {
                pcCurStr = curMsg.drawDir[i];

                /* Draw string for each line */
                GrStringDraw(&context, pcCurStr, -1, 0,
                            (i + 1) * fontHeight, 0);
            }
            break;

        case STRING:
            pcCurStr = curMsg.drawString;
            GrContextForegroundSetTranslated(&context, 0xFF);
            GrStringDraw(&context, pcCurStr, -1, 0, 0, 0);
            break;

        default:
            break;
        }
    }
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the left button
 *
 *  It posts a message with image index 0 to display.
 */
Void gpioButtonFxn0(Void)
{
    UInt key;
    DrawMessage drawMsg;

    drawMsg.drawCommand = IMAGE;
    drawMsg.drawImageIndex = 0;

    key = Gate_enterSystem();
    /* Clear the last command */
    lastCommand[0] = 0x0;
    Gate_leaveSystem(key);

    /* Do not wait if there is no room for the new mail */
    Mailbox_post(mailboxHandle, &drawMsg, BIOS_NO_WAIT);

    GPIO_clearInt(DK_TM4C123G_SW3);
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the right button
 *
 *  It posts a message with image index 1 to display.
 */
Void gpioButtonFxn1(Void)
{
    UInt key;
    DrawMessage drawMsg;

    drawMsg.drawCommand = IMAGE;
    drawMsg.drawImageIndex = 1;

    key = Gate_enterSystem();
    /* Clear the last command */
    lastCommand[0] = 0x0;
    Gate_leaveSystem(key);

    /* Do not wait if there is no room for the new mail */
    Mailbox_post(mailboxHandle, &drawMsg, BIOS_NO_WAIT);

    GPIO_clearInt(DK_TM4C123G_SW4);
}

/*
 *  ======== consoleTaskFxn ========
 *  Console task
 *
 *  This task listens to the key pressed in the keyboard through USBCDC.
 *  The string ended with return character '\n' will trigger the task
 *  to send this string to the mailbox.
 *  For example, when the user enter "ls\n", this task will scan all the
 *  files in the root of SD card and send the file list to the mailbox to
 *  inform the drawing task to display on the screen.
 *  The up/down arrow can be used to scroll up/down to display more files
 *  in the SD card.
 */
Void consoleTaskFxn (UArg arg0, UArg arg1)
{
    UInt received;
    UInt remLength = CHARS_ONE_LINE;
    UInt copyLength = 0;
    UChar data[RECEIVE_LENGTH + 1];
    UChar totalData[CHARS_ONE_LINE + 1] = {'\0'};
    UInt pageIdx = 0;
    Bool reachTop = TRUE;
    Bool reachEnd = FALSE;
    UInt key;
    DrawMessage drawMsg;

    drawMsg.drawCommand = COMMAND;

    while (TRUE) {

        /* Block while the device is NOT connected to the USB */
        USBCDCD_waitForConnect(BIOS_WAIT_FOREVER);
        received = USBCDCD_receiveData(data,
                    RECEIVE_LENGTH, BIOS_WAIT_FOREVER);
        data[received] = '\0';

        if (received) {
            System_printf("Received \"%s\" (%d bytes)\r\n", data, received);

            /* Echo back to USBCDC */
            USBCDCD_sendData(data, received, BIOS_WAIT_FOREVER);

            /* If it is arrow up/down */
            if (!strcmp((const char*)data, (const char*)downArrow)) {
                if (!strcmp((const char*)lastCommand, "ls")
                        && reachEnd == FALSE) {
                    /* Put command string to the message */
                    strcpy(drawMsg.drawString, ">ls");

                    /* Read directories from SD card and put to the message */
                    pageIdx++;
                    readDir("", &drawMsg, pageIdx,
                            pageIdx + FILELIST_LENGTH - 1);
                    if (drawMsg.drawDirCount < FILELIST_LENGTH) {
                        reachEnd = TRUE;
                    }
                    reachTop = FALSE;

                    /* Send the file list (scroll down once) to the mailbox */
                    Mailbox_post(mailboxHandle, &drawMsg, BIOS_WAIT_FOREVER);
                }
            }
            else if (!strcmp((const char*)data, (const char*)upArrow)) {
                if (!strcmp((const char*)lastCommand, "ls")
                        && reachTop == FALSE) {
                    /* Put command string to the message */
                    strcpy(drawMsg.drawString, ">ls");

                    /* Read directories from SD card and put to the message */
                    pageIdx--;
                    readDir("", &drawMsg, pageIdx,
                            pageIdx + FILELIST_LENGTH - 1);
                    if (pageIdx == 0) {
                        reachTop = TRUE;
                    }
                    reachEnd = FALSE;

                    /* Send the file list (scroll up once) to the mailbox */
                    Mailbox_post(mailboxHandle, &drawMsg, BIOS_WAIT_FOREVER);
                }
            }
            else {
                /* Concatenate all characters to check if it is a command */
                copyLength = received < remLength ? received : remLength;
                strncat((char*)totalData, (const char*)data, copyLength);
                remLength -= copyLength;

                /* If it is the command */
                if (data[received - 1] == '\r') {
                    USBCDCD_sendData("\n", 1, BIOS_WAIT_FOREVER);

                    /* Replace '\n' with '\0' character */
                    totalData[CHARS_ONE_LINE - remLength - 1] = '\0';

                    /* It is the ls command */
                    if (!strcmp((const char*)totalData, "ls")) {
                        /* Put command string to the message */
                        strcpy(drawMsg.drawString, ">ls");

                        /*
                         * Read directories from SD card
                         * and put into the message
                         */
                        readDir("", &drawMsg, 0, FILELIST_LENGTH - 1);
                        reachEnd = (drawMsg.drawDirCount < FILELIST_LENGTH)
                                    ? TRUE : FALSE;

                        /* Send the file list (no scrolling) to the mailbox */
                        Mailbox_post(mailboxHandle, &drawMsg,
                                     BIOS_WAIT_FOREVER);
                    }
                    /* Other unrecognized commands */
                    else {
                        drawMsg.drawString[0] = '>';
                        drawMsg.drawString[1] = '\0';
                        strncat(drawMsg.drawString, (const char*)totalData,
                                CHARS_ONE_LINE - 1);
                        drawMsg.drawString[CHARS_ONE_LINE] = '\0';
                        drawMsg.drawDirCount = 0;

                        /* Send the command to the mailbox */
                        Mailbox_post(mailboxHandle, &drawMsg,
                                     BIOS_WAIT_FOREVER);

                    }

                    /* Copy to the global command buffer */
                    key = Gate_enterSystem();
                    strcpy((char*)lastCommand, (const char*)totalData);
                    Gate_leaveSystem(key);

                    /* Reset page index and buffer */
                    pageIdx = 0;
                    reachTop = TRUE;
                    reachEnd = FALSE;
                    remLength = CHARS_ONE_LINE;
                    totalData[0] = '\0';
                }
            }
        }
    }
}


/*
 *  ======== LCD_init ========
 */
Void LCD_init()
{
    /* LCD driver init */
    CFAL96x64x16Init();
    GrContextInit(&context, &g_sCFAL96x64x16);

    /* Setup font */
    GrContextFontSet(&context, g_psFontFixed6x8);
}


/*
 *  ======== main ========
 */
Int main(Void)
{
    SDSPI_Params sdspiParams;
    Mailbox_Params mboxParams;
    Task_Params grlibTaskParams;
    Task_Params consoleTaskParams;
    Task_Handle grlibTaskHandle;
    Task_Handle consoleTaskHandle;
    SDSPI_Handle sdspiHandle;
    Error_Block eb;

    /* Init board-specific functions. */
    DK_TM4C123G_initGeneral();
    DK_TM4C123G_initGPIO();
    DK_TM4C123G_initSDSPI();
    DK_TM4C123G_initUSB(DK_TM4C123G_USBDEVICE);

    /* Turn on user LED */
    GPIO_write(DK_TM4C123G_LED, DK_TM4C123G_LED_ON);

    /* Init LCD and USBCDC */
    LCD_init();
    USBCDCD_init();

    /* Init and enable interrupts */
    GPIO_setupCallbacks(&DK_TM4C123G_gpioPortMCallbacks);
    GPIO_enableInt(DK_TM4C123G_SW3, GPIO_INT_RISING);
    GPIO_enableInt(DK_TM4C123G_SW4, GPIO_INT_RISING);

    /* Mount and register the SD Card */
    SDSPI_Params_init(&sdspiParams);
    sdspiHandle = SDSPI_open(DK_TM4C123G_SDSPI0, DRIVE_NUM, &sdspiParams);
    if (sdspiHandle == NULL) {
        System_abort("Error starting the SD card\nAborting...");
    }
    else {
        System_printf("Drive %u is mounted\n", DRIVE_NUM);
    }

    /* SYS/BIOS Mailbox create */
    Error_init(&eb);
    Mailbox_Params_init(&mboxParams);
    mailboxHandle = Mailbox_create(sizeof(DrawMessage), 2, &mboxParams, &eb);
    if (mailboxHandle == NULL) {
        System_abort("Mailbox create failed\nAborting...");
    }

    /* Console task create */
    Error_init(&eb);
    Task_Params_init(&consoleTaskParams);
    consoleTaskParams.instance->name = "consoleTask";
    consoleTaskParams.stackSize = 880;
    consoleTaskParams.priority = 2;
    consoleTaskHandle = Task_create(consoleTaskFxn, &consoleTaskParams, &eb);
    if (consoleTaskHandle == NULL) {
        System_abort("Console task was not created\nAborting...");
    }

    /* Grlib task create */
    Error_init(&eb);
    Task_Params_init(&grlibTaskParams);
    grlibTaskParams.instance->name = "grlibTask";
    grlibTaskParams.stackSize = 800;
    grlibTaskParams.priority = 1;
    grlibTaskHandle = Task_create(grlibTaskFxn, &grlibTaskParams, &eb);
    if (grlibTaskHandle == NULL) {
        System_abort("Grlib task was not created\nAborting...");
    }

    System_printf("Starting the example\n%s, %s",
            "System provider is set to SysMin",
            "halt the target and use ROV to view output.\n");

    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS. Will not return from this call. */
    BIOS_start();

    return (0);
}
