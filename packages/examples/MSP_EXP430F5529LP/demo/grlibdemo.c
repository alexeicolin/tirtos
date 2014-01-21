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
#include <ti/sysbios/knl/Event.h>

/* TIRTOS Header files */
#include <msp430.h>
#ifdef DIR
#undef DIR
#endif
#include <ti/sysbios/fatfs/ff.h>


/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/USCIUART.h>
#include <ti/drivers/USCISDSPI.h>

/* Board Header file */
#include "Board.h"
#include "grlib/drivers/grlib.h"
#include "Dogs102x64_UC1701.h"

#include <file.h>
#include <stdio.h>
#include <string.h>

/* File-list size */
#define FILELIST_LENGTH 7

/* Characters count each line */
#define CHARS_ONE_LINE 17

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

/* Image reference */
extern const tImage image_TILogoBig_1BPP_UNCOMP;
extern const tImage image_Launchpad_1BPP_UNCOMP;
const tImage* image_Gallery[2] = {&image_TILogoBig_1BPP_UNCOMP, &image_Launchpad_1BPP_UNCOMP};

/* Up/Down arrow keys (ASCII code converted from keyboard) */
const UChar downArrow[4] = {0x1b, 0x5b, 0x42, 0x00};
const UChar upArrow[4] = {0x1b, 0x5b, 0x41, 0x00};

/* Global context for drawing */
tContext context;

/* Handles created dynamically in the c */
Mailbox_Handle mailboxHandle = NULL;


/*
 *  ======== readDir ========
 *  Read directories from SDSPI and put into the message
 */
Void readDir(const char *path, DrawMessage *drawMsg, UInt startIdx, UInt endIdx)
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
                strncpy(drawMsg->drawDir[drawMsg->drawDirCount],
                        finfo.fname, CHARS_ONE_LINE);
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
Void grlibTaskFxn(UArg a0, UArg a1)
{
    DrawMessage curMsg;
    const tImage *curImage;
    const char *curStr;
    UInt i = 0;
    UInt fontHeight = GrStringHeightGet(&context);

    while (TRUE) {
        Mailbox_pend(mailboxHandle, &curMsg, BIOS_WAIT_FOREVER);

        /* Clear screen before drawing */
        GrClearDisplay(&context);

        /* Parse the message and draw */
        switch (curMsg.drawCommand) {
        case IMAGE:
            curImage = image_Gallery[curMsg.drawImageIndex];
            /* Draw image at (0,0) */
            GrImageDraw(&context, curImage, 0, 0);
            break;

        case COMMAND:
            curStr = curMsg.drawString;
            /* Draw string at (0,0) */
            GrStringDraw(&context, curStr, -1, 0, 0, 0);

            for (i = 0; i < curMsg.drawDirCount; i++) {
                curStr = curMsg.drawDir[i];
                /* Draw string for each line */
                GrStringDraw(&context, curStr, -1, 0, (i + 1)*fontHeight, 0);
            }
            break;

        case STRING:
            curStr = curMsg.drawString;
            GrStringDraw(&context, curStr, -1, 0, 0, 0);
            break;

        default:
            break;
        }
    }
}

/*
 *  ======== consoleTaskFxn ========
 *  Console task
 *
 *  This task listens to the key pressed in the keyboard through UART.
 *  The string ended with return character '\n' will trigger the task
 *  to send this string to the mailbox.
 *  For example, when the user enter "ls\n", this task will scan all the
 *  files in the root of SD card and send the file list to the mailbox to
 *  inform the drawing task to display on the screen.
 *  The up/down arrow can be used to scroll up/down to display more files
 *  in the SD card.
 */
Void consoleTaskFxn (UArg a0, UArg a1)
{
    Char input;
    UInt received;
    UInt remLength = CHARS_ONE_LINE;
    UInt copyLength = 0;
    Char totalData[CHARS_ONE_LINE+1] = {'\0'};
    UInt pageIdx = 0;
    Bool reachTop = TRUE;
    Bool reachEnd = FALSE;
    UInt key;
    DrawMessage drawMsg;
    USCIUART_Handle uartHandle = (USCIUART_Handle)a0;

    drawMsg.drawCommand = COMMAND;

    /* Loop forever echoing */
    while (TRUE) {
        received = USCIUART_read(uartHandle, &input, 1);
        //data[received] = '\0';

        if (received > 0) {
            /* Echo back */
            USCIUART_write(uartHandle, &input, 1);

            /* Concatenate all the characters to check if it is a command */
            copyLength = received < remLength ? received : remLength;
            strncat(totalData, &input, copyLength);
            remLength -= copyLength;

            /* If it is arrow down */
            if (!strcmp((const char*)totalData, (const char*)downArrow)) {
                if (!strcmp((const char*)lastCommand, "ls") && reachEnd == FALSE) {
                    /* Put command string to the message */
                    strcpy(drawMsg.drawString, ">ls");

                    /* Read directories from SD card and put to the message */
                    pageIdx++;
                    readDir("", &drawMsg, pageIdx, pageIdx + FILELIST_LENGTH - 1);
                    if (drawMsg.drawDirCount < FILELIST_LENGTH) {
                        reachEnd = TRUE;
                    }
                    reachTop = FALSE;

                    /* Send to the mailbox */
                    Mailbox_post(mailboxHandle, &drawMsg, BIOS_WAIT_FOREVER);
                }

                remLength = CHARS_ONE_LINE;
                totalData[0] = '\0';
            }
            /* If it is arrow up */
            else if (!strcmp((const char*)totalData, (const char*)upArrow)) {
                if (!strcmp((const char*)lastCommand, "ls") && reachTop == FALSE) {
                    /* Put command string to the message */
                    strcpy(drawMsg.drawString, ">ls");

                    /* Read directories from SD card and put to the message */
                    pageIdx--;
                    readDir("", &drawMsg, pageIdx, pageIdx + FILELIST_LENGTH - 1);
                    if (pageIdx == 0) {
                        reachTop = TRUE;
                    }
                    reachEnd = FALSE;

                    /* Send to the mailbox */
                    Mailbox_post(mailboxHandle, &drawMsg, BIOS_WAIT_FOREVER);
                }

                remLength = CHARS_ONE_LINE;
                totalData[0] = '\0';
            }
            /* If it is the command */
            else if (input == '\r') {
                USCIUART_write(uartHandle, "\n", 1);

                /* Replace '\r' with '\0' character */
                totalData[CHARS_ONE_LINE - remLength - 1] = '\0';

                /* It is the ls command */
                if (!strcmp((const char*)totalData, "ls")) {
                    /* Put command string to the message */
                    strcpy(drawMsg.drawString, ">ls");

                    /* Read directories from SD card and put to the message */
                    readDir("", &drawMsg, 0, FILELIST_LENGTH - 1);
                    reachEnd = drawMsg.drawDirCount < FILELIST_LENGTH ? TRUE : FALSE;

                    /* Send to the mailbox */
                    Mailbox_post(mailboxHandle, &drawMsg, BIOS_WAIT_FOREVER);
                }
                /* Other unrecognized commands */
                else {
                    drawMsg.drawString[0] = '>';
                    drawMsg.drawString[1] = '\0';
                    strncat(drawMsg.drawString, (const char*)totalData, CHARS_ONE_LINE - 1);
                    drawMsg.drawString[CHARS_ONE_LINE] = '\0';
                    drawMsg.drawDirCount = 0;

                    /* Send to the mailbox */
                    Mailbox_post(mailboxHandle, &drawMsg, BIOS_WAIT_FOREVER);
                }

                /* Copy to the global command buffer */
                key = Gate_enterSystem();
                strcpy((char*)lastCommand, (const char*)totalData);
                Gate_leaveSystem(key);

                /* reset page index */
                pageIdx = 0;
                reachTop = TRUE;
                reachEnd = FALSE;
                remLength = CHARS_ONE_LINE;
                totalData[0] = '\0';
            }

        }
    }
}

void Buttons_startWDT()
{
    /* WDT as 250ms interval counter */
    SFRIFG1 &= ~WDTIFG;
    WDTCTL = WDTPW + WDTSSEL_1 + WDTTMSEL + WDTCNTCL + WDTIS_5;
    SFRIE1 |= WDTIE;

//  Switch to driver API in future
//    Watchdog_Params watchdogParams;
//    Watchdog_Params_init(&watchdogParams);
//    watchdogParams.resetMode = Watchdog_RESET_OFF;
//    Watchdog_open(MSP_EXP430F5529LP_WATCHDOG1, &watchdogParams);
}

/*
 *  ======== WDT_ISR ========
 */
#define BUTTON_S2       0x0400
#define BUTTON_S1       0x0080
#define BUTTON_ALL      0x0480
Void WDT_ISR(UArg index)
{
    /* change to Watchdog API when Watchdog driver is done */
    SFRIFG1 &= ~WDTIFG;
    SFRIE1 &= ~WDTIE;
    WDTCTL = WDTPW + WDTHOLD;

    /* change to GPIO API when GPIO driver is done */
    PAIES &= ~BUTTON_ALL; //select falling edge trigger
    PAIFG &= ~BUTTON_ALL; //clear flags
    PAIE |= BUTTON_ALL;   //enable interrupts

//Switch to driver API in future
//    Watchdog_close(watchdogHandle);
//
//    GPIO_enableInt(MSP_EXP430F5529LP_S1, GPIO_INT_RISING);
//    GPIO_enableInt(MSP_EXP430F5529LP_S2, GPIO_INT_FALLING);

}

/*
 *  ======== Port2_ISR ========
 */
Void Port2_ISR(UArg index)
{
    DrawMessage drawMsg;
    drawMsg.drawCommand = IMAGE;
    UInt key;

    switch (__even_in_range(P2IV, P2IV_P2IFG7))
    {
        case  P2IV_P2IFG2:
                PAIE &= ~BUTTON_S2;
                Buttons_startWDT();

                drawMsg.drawImageIndex = 1;

                key = Gate_enterSystem();
                lastCommand[0] = 0x0;
                Gate_leaveSystem(key);

                Mailbox_post(mailboxHandle, &drawMsg, BIOS_NO_WAIT);
            break;

        default:
            break;
    }
}

/*
 *  ======== Port1_ISR ========
 */
Void Port1_ISR(UArg index)
{
    DrawMessage drawMsg;
    drawMsg.drawCommand = IMAGE;
    UInt key;

    switch (__even_in_range(P1IV, P1IV_P1IFG7))
    {
        case  P1IV_P1IFG7:
            PAIE &= ~BUTTON_S1;
            Buttons_startWDT();

            drawMsg.drawImageIndex = 0;

            key = Gate_enterSystem();
            lastCommand[0] = 0x0;
            Gate_leaveSystem(key);

            Mailbox_post(mailboxHandle, &drawMsg, BIOS_NO_WAIT);
            break;

        default:
            break;
    }
}


/*
 *  ======== LCD_init ========
 */
void LCD_init()
{
    /* Set up LCD */
    Dogs102x64_UC1701Init();
    GrContextInit(&context, &g_sDogs102x64_UC1701);

    GrContextForegroundSet(&context, ClrBlack);
    GrContextBackgroundSet(&context, ClrWhite);

    GrClearDisplay(&context);

    /* Set up font */
    GrContextFontSet(&context, &g_sFontFixed6x8);
}

/*
 *  ======== main ========
 */
Int main(Void)
{
    USCISDSPI_Params sdspiParams;
    Mailbox_Params mboxParams;
    USCIUART_Params uartParams;
    Task_Params grlibTaskParams;
    Task_Params consoleTaskParams;
    DrawMessage tempDrawMsg;
    SDSPI_Handle sdspiHandle;
    Task_Handle grlibTaskHandle;
    Task_Handle consoleTaskHandle;
    USCIUART_Handle uartHandle;
    Error_Block eb;

    /* Call board init functions. */
    MSP_EXP430F5529LP_initGeneral();
    MSP_EXP430F5529LP_initUSCIUART();
    MSP_EXP430F5529LP_initGPIO();

    /* Mount and register the SD Card */
    MSP_EXP430F5529LP_initUSCISDSPI();
    USCISDSPI_Params_init(&sdspiParams);
    sdspiHandle = USCISDSPI_open(MSP_EXP430F5529LP_SDSPI1, DRIVE_NUM, &sdspiParams);
    if (sdspiHandle == NULL) {
        System_abort("Error starting the SD card\n");
    }
    else {
        System_printf("Drive %u is mounted\n", DRIVE_NUM);
    }
    /* Force to initialize disk I/O to cache I/O handle */
    readDir("", &tempDrawMsg, 0, 0);

    /* Init LCD */
    LCD_init();
    GrStringDraw(&context, "Demo start...", -1, 2, 3, 0);

    /* SYS/BIOS Mailbox init */
    Error_init(&eb);
    Mailbox_Params_init(&mboxParams);
    mailboxHandle = Mailbox_create(sizeof(DrawMessage), 2, &mboxParams, &eb);
    if (mailboxHandle == NULL) {
        System_abort("Mailbox create failed");
    }

    /* Create a UART with data processing off. */
    USCIUART_Params_init(&uartParams);
    uartParams.writeDataMode = USCIUART_DATA_BINARY;
    uartParams.readDataMode = USCIUART_DATA_BINARY;
    uartParams.readReturnMode = USCIUART_RETURN_FULL;
    uartParams.readEcho = USCIUART_ECHO_OFF;
    uartHandle = USCIUART_open(MSP_EXP430F5529LP_USCIUART1, &uartParams);
    if (uartHandle == NULL) {
        System_abort("Error opening the UART");
    }

    /* Console task create */
    Error_init(&eb);
    Task_Params_init(&consoleTaskParams);
    consoleTaskParams.instance->name = "consoleTask";
    consoleTaskParams.stackSize = 768;
    consoleTaskParams.priority = 2;
    consoleTaskParams.arg0 = (UArg)uartHandle;
    consoleTaskHandle = Task_create(consoleTaskFxn, &consoleTaskParams, &eb);
    if (consoleTaskHandle == NULL) {
        System_abort("Console task was not created\nAborting...\n");
    }

    /* Grlib task create */
    Error_init(&eb);
    Task_Params_init(&grlibTaskParams);
    grlibTaskParams.instance->name = "grlibTask";
    grlibTaskParams.stackSize = 512;
    grlibTaskParams.priority = 1;
    grlibTaskHandle = Task_create(grlibTaskFxn, &grlibTaskParams, &eb);
    if (grlibTaskHandle == NULL) {
        System_abort("Grlib task was not created\nAborting...\n");
    }

    System_printf("Starting the example\nSystem provider is set to SysMin, halt the target and use ROV to view output.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS. Will not return from this call. */
    BIOS_start();
}
