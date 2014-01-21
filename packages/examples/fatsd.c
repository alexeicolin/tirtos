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
 *  ======== fatsd.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDSPI.h>

/* Example/Board Header files */
#include "Board.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Buffer size used for the file copy process */
#ifndef CPY_BUFF_SIZE
#define CPY_BUFF_SIZE       2048
#endif

/* String conversion macro */
#define STR_(n)             #n
#define STR(n)              STR_(n)

/* Drive number used for FatFs */
#define DRIVE_NUM           0

const Char  inputfile[] = "fat:"STR(DRIVE_NUM)":input.txt";
const Char outputfile[] = "fat:"STR(DRIVE_NUM)":output.txt";

const Char textarray[] = \
"***********************************************************************\n"
"0         1         2         3         4         5         6         7\n"
"01234567890123456789012345678901234567890123456789012345678901234567890\n"
"This is some text to be inserted into the inputfile if there isn't     \n"
"already an existing file located on the SDCard.                        \n"
"If an inputfile already exists, or if the file was already once        \n"
"generated, then the inputfile will NOT be modified.                    \n"
"***********************************************************************\n";

UChar cpy_buff[CPY_BUFF_SIZE + 1];

/*
 *  ======== taskFxn ========
 *  Task to perform a file copy
 *
 *  Task tries to open an existing file inputfile[]. If the file doesn't
 *  exist, create one and write some known content into it.
 *  The contents of the inputfile[] are then copied to an output file
 *  outputfile[]. Once completed, the contents of the output file are
 *  printed onto the system console (stdout).
 *
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void taskFxn(UArg arg0, UArg arg1)
{
    SDSPI_Handle sdspiHandle;
    SDSPI_Params sdspiParams;

    /* Variables for the CIO functions */
    FILE *src, *dst;

    /* Variables to keep track of the file copy progress */
    UInt bytesRead = 0;
    UInt bytesWritten = 0;
    UInt filesize;
    UInt totalBytesCopied = 0;

    /* Mount and register the SD Card */
    SDSPI_Params_init(&sdspiParams);
    sdspiHandle = SDSPI_open(Board_SDSPI0, DRIVE_NUM, &sdspiParams);
    if (sdspiHandle == NULL) {
        System_abort("Error starting the SD card\n");
    }
    else {
        System_printf("Drive %u is mounted\n", DRIVE_NUM);
    }

    /* Try to open the source file */
    src = fopen(inputfile, "r");
    if (!src) {
        System_printf("Creating a new file \"%s\"...", inputfile);

        /* Open file for both reading and writing */
        src = fopen(inputfile, "w+");
        if (!src) {
            System_printf("Error: \"%s\" could not be created\n",
                    inputfile);
            System_abort("Aborting...\n");
        }

        fwrite(textarray, 1, strlen(textarray), src);
        fflush(src);

        /* Reset the internal file pointer */
        rewind(src);

        System_printf("done\n");
    }
    else {
        System_printf("Using existing copy of \"%s\"\n", inputfile);
    }

    /* Create a new file object for the file copy */
    dst = fopen(outputfile, "w");
    if (!dst) {
        System_printf("Error opening \"%s\"\n", outputfile);
        System_abort("Aborting...\n");
    }
    else {
        System_printf("Starting file copy\n");
    }

    /*  Copy the contents from the src to the dst */
    while (TRUE) {
        /*  Read from source file */
        bytesRead = fread(cpy_buff, 1, CPY_BUFF_SIZE, src);
        if (bytesRead == 0) {
            break; /* Error or EOF */
        }

        /*  Write to dst file */
        bytesWritten = fwrite(cpy_buff, 1, bytesRead, dst);
        if (bytesWritten < bytesRead) {
            System_printf("Disk Full\n");
            break; /* Error or Disk Full */
        }

        /*  Update the total number of bytes copied */
        totalBytesCopied += bytesWritten;
    }

    fflush(dst);

    /* Get the filesize of the source file */
    fseek(src, 0, SEEK_END);
    filesize = ftell(src);
    rewind(src);

    /* Close both inputfile[] and outputfile[] */
    fclose(src);
    fclose(dst);

    System_printf("File \"%s\" (%u B) copied to \"%s\" (Wrote %u B)\n",
                  inputfile, filesize, outputfile, totalBytesCopied);

    /* Now output the outputfile[] contents onto the console */
    dst = fopen(outputfile, "r");
    if (!dst) {
        System_printf("Error opening \"%s\"\n", outputfile);
        System_abort("Aborting...\n");
    }

    /* Print file contents */
    while (TRUE) {
        /* Read from output file */
        bytesRead = fread(cpy_buff, 1, CPY_BUFF_SIZE, dst);
        if (bytesRead == 0) {
            break; /* Error or EOF */
        }
        cpy_buff[bytesRead] = '\0';
        /* Write output */
        System_printf("%s", cpy_buff);
        System_flush();
    }

    /* Close the file */
    fclose(dst);

    /* Stopping the SDCard */
    SDSPI_close(sdspiHandle);
    System_printf("Drive %u unmounted\n", DRIVE_NUM);

    BIOS_exit(0);
}

/*
 *  ======== main ========
 */
Int main(Void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initSDSPI();

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the FatSD example\n");

    /* Start BIOS */
    BIOS_start();

    return (0);
}
