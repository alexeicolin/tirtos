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
 *  ======== cmdline.c ========
 */

/* TI-RTOS Header files */
#include <ti/drivers/UART.h>
#include <ti/drivers/GPIO.h>

/* driverlib Header files */
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <utils/cmdline.h>

/* Example/Board Header files */
#include "TMDXDOCK28M36.h"

#include <string.h>

//*****************************************************************************
//
// Defines the maximum number of arguments that can be parsed.
//
//*****************************************************************************
#ifndef CMDLINE_MAX_ARGS
#define CMDLINE_MAX_ARGS        8
#endif

//****************************************************************************
// Defines the size of the buffer that holds the command line.
//****************************************************************************
#define CMD_BUF_SIZE            64

//****************************************************************************
// Globals used by both classes.
//****************************************************************************
volatile unsigned long g_ulFlags;

//****************************************************************************
// The flags used by this application for the g_ulFlags value.
//****************************************************************************
#define FLAG_MOVE_UPDATE       0
#define FLAG_CONNECTED         1
#define FLAG_LED_ACTIVITY      2
#define FLAG_COMMAND_RECEIVED  3

//****************************************************************************
// The buffer that holds the command line.
//****************************************************************************
static char g_pcCmdBuf[CMD_BUF_SIZE] = "help";

//****************************************************************************
// UART handle for console I/O.
//****************************************************************************
static UART_Handle uart0;

//****************************************************************************
// This function will print out to the console UART.
//****************************************************************************
void CommandPrint(const char *pcStr)
{
    UART_write(uart0, pcStr, strlen(pcStr));

    // Toggle the LED if in activity mode.
    if(HWREGBITW(&g_ulFlags, FLAG_LED_ACTIVITY))
    {
        // Toggle the LED.
        GPIO_toggle(TMDXDOCK28M36_D1);
    }
}

//****************************************************************************
// This function will read from the console UART and echo.
//****************************************************************************
Void CommandScan(char *pcStr)
{
    memset(pcStr, 0, CMD_BUF_SIZE);
    UART_read(uart0, pcStr, CMD_BUF_SIZE);
}

//****************************************************************************
// This command allows setting, clearing or toggling the Status LED.
// The first argument should be one of the following:
// on     - Turn on the LED.
// off    - Turn off the LEd.
// toggle - Toggle the current LED status.
// activity - Set the LED mode to monitor serial activity.
//****************************************************************************
int
Cmd_led(int argc, char *argv[])
{
    // These values only check the second character since all parameters are
    // different in that character.
    if(argv[1][1] == 'n')
    {
        // Turn on the LED.
        GPIO_write(TMDXDOCK28M36_D1, TMDXDOCK28M36_LED_ON);

        // Switch off activity mode.
        HWREGBITW(&g_ulFlags, FLAG_LED_ACTIVITY) = 0;
    }
    else if(argv[1][1] == 'f')
    {
        // Turn off the LED.
        GPIO_write(TMDXDOCK28M36_D1, TMDXDOCK28M36_LED_OFF);

        // Switch off activity mode.
        HWREGBITW(&g_ulFlags, FLAG_LED_ACTIVITY) = 0;
    }
    else if(argv[1][1] == 'o')
    {
        // Toggle the LED.
        GPIO_toggle(TMDXDOCK28M36_D1);

        // Switch off activity mode.
        HWREGBITW(&g_ulFlags, FLAG_LED_ACTIVITY) = 0;
    }
    else if(argv[1][1] == 'c')
    {
        // If this is the "activity" value then set the activity mode.
        HWREGBITW(&g_ulFlags, FLAG_LED_ACTIVITY) = 1;
    }
    else
    {
        // The command format was not correct so print out some help.
        CommandPrint("\nled <on|off|toggle|activity>\n");
        CommandPrint("  on       - Turn on the LED.\n");
        CommandPrint("  off      - Turn off the LED.\n");
        CommandPrint("  toggle   - Toggle the LED state.\n");
        CommandPrint("  activity - LED state will toggle on UART activity.\n");
    }

    return(0);
}

//****************************************************************************
// This function implements the "help" command.  It prints a simple list of
// the available commands with a brief description.
//****************************************************************************
int
Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    // Print some header text.
    CommandPrint("\nAvailable commands\n");
    CommandPrint("------------------\n");

    // Point at the beginning of the command table.
    pEntry = &g_sCmdTable[0];

    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    while(pEntry->pcCmd)
    {
        // Print the command name and the brief description.
        CommandPrint(pEntry->pcCmd);
        CommandPrint(pEntry->pcHelp);
        CommandPrint("\n");

        // Advance to the next entry in the table.
        pEntry++;
    }

    // Return success.
    return(0);
}

//****************************************************************************
// This is the table that holds the command names, implementing functions, and
// brief description.
//****************************************************************************
tCmdLineEntry g_sCmdTable[] =
{
    { "help",  Cmd_help,     "  : Display list of commands" },
    { "h",     Cmd_help,  "     : alias for help" },
    { "?",     Cmd_help,  "     : alias for help" },
    { "led",   Cmd_led,     "   : Set LED mode (on|off|toggle|activity)" },
    { 0, 0, 0 }
};

//****************************************************************************
// This is the serial initialization routine.
//****************************************************************************
Void
SerialInit(Void)
{
    UART_Params uartParams;

    /* Create a UART with the parameters below. */
    UART_Params_init(&uartParams);
    uartParams.writeMode = UART_MODE_BLOCKING;
    uartParams.readMode = UART_MODE_BLOCKING;
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readReturnMode = UART_RETURN_NEWLINE;
    uartParams.readEcho = UART_ECHO_ON;
    uart0 = UART_open(0, &uartParams);
}

//*****************************************************************************
//
//! Process a command line string into arguments and execute the command.
//!
//! \param pcCmdLine points to a string that contains a command line that was
//! obtained by an application by some means.
//!
//! This function will take the supplied command line string and break it up
//! into individual arguments.  The first argument is treated as a command and
//! is searched for in the command table.  If the command is found, then the
//! command function is called and all of the command line arguments are passed
//! in the normal argc, argv form.
//!
//! The command table is contained in an array named <tt>g_sCmdTable</tt> which
//! must be provided by the application.
//!
//! \return Returns \b CMDLINE_BAD_CMD if the command is not found,
//! \b CMDLINE_TOO_MANY_ARGS if there are more arguments than can be parsed.
//! Otherwise it returns the code that was returned by the command function.
//
//*****************************************************************************
int
CmdLineProcess(char *pcCmdLine)
{
    static char *argv[CMDLINE_MAX_ARGS + 1];
    char *pcChar;
    int argc;
    int bFindArg = 1;
    tCmdLineEntry *pCmdEntry;

    //
    // Initialize the argument counter, and point to the beginning of the
    // command line string.
    //
    argc = 0;
    pcChar = pcCmdLine;

    //
    // Advance through the command line until a zero character is found.
    //
    while(*pcChar)
    {
        //
        // If there is a space, then replace it with a zero, and set the flag
        // to search for the next argument.
        //
        if(*pcChar == ' ' || *pcChar == '\n')
        {
            *pcChar = 0;
            bFindArg = 1;
        }

        //
        // Otherwise it is not a space, so it must be a character that is part
        // of an argument.
        //
        else
        {
            //
            // If bFindArg is set, then that means we are looking for the start
            // of the next argument.
            //
            if(bFindArg)
            {
                //
                // As long as the maximum number of arguments has not been
                // reached, then save the pointer to the start of this new arg
                // in the argv array, and increment the count of args, argc.
                //
                if(argc < CMDLINE_MAX_ARGS)
                {
                    argv[argc] = pcChar;
                    argc++;
                    bFindArg = 0;
                }

                //
                // The maximum number of arguments has been reached so return
                // the error.
                //
                else
                {
                    return(CMDLINE_TOO_MANY_ARGS);
                }
            }
        }

        //
        // Advance to the next character in the command line.
        //
        pcChar++;
    }

    //
    // If one or more arguments was found, then process the command.
    //
    if(argc)
    {
        //
        // Start at the beginning of the command table, to look for a matching
        // command.
        //
        pCmdEntry = &g_sCmdTable[0];

        //
        // Search through the command table until a null command string is
        // found, which marks the end of the table.
        //
        while(pCmdEntry->pcCmd)
        {
            //
            // If this command entry command string matches argv[0], then call
            // the function for this command, passing the command line
            // arguments.
            //
            if(!strcmp(argv[0], pCmdEntry->pcCmd))
            {
                return(pCmdEntry->pfnCmd(argc, argv));
            }

            //
            // Not found, so advance to the next entry.
            //
            pCmdEntry++;
        }
    }

    //
    // Fall through to here means that no matching command was found, so return
    // an error.
    //
    return(CMDLINE_BAD_CMD);
}

//****************************************************************************
//
// This is the main loop serial handling function.
//
//****************************************************************************
Void
SerialMain(Void)
{
    int iStatus;

    // Set first command as help to display commands and prompt
    //g_pcCmdBuf = "help";

    //
    // Main application loop.
    //
    while (TRUE)
    {
        // Process the command line.
        iStatus = CmdLineProcess(g_pcCmdBuf);

        // Handle the case of bad command.
        if(iStatus == CMDLINE_BAD_CMD)
        {
            CommandPrint(g_pcCmdBuf);
            CommandPrint(" is not a valid command!\n");
        }
        CommandPrint("\n> ");

        // Get the next command
        CommandScan(g_pcCmdBuf);
    }
}
