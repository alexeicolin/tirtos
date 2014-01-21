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
 *    ======== tcpEchoIPv6.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>

 /* NDK Header files */
#include <ti/ndk/inc/netmain.h>
#include <ti/ndk/inc/_stack.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>

/* Example/Board Header files */
#include "Board.h"

#define TCPPACKETSIZE 1024
#define TCPPORT 1000
#define NUMTCPWORKERS 3

/*
 *  ======== tcpWorker ========
 *  Task to handle TCP connection. Can be multiple Tasks running
 *  this function.
 */
Void tcpWorker(UArg arg0, UArg arg1)
{
    SOCKET clientfd = (SOCKET)arg0;
    Int nbytes;
    Bool flag = TRUE;
    Char *buffer;
    Error_Block eb;

    fdOpenSession(TaskSelf());

    System_printf("tcpWorker: start clientfd = 0x%x\n", clientfd);

    /* Make sure Error_Block is initialized */
    Error_init(&eb);

    /* Get a buffer to receive incoming packets. Use the default heap. */
    buffer = Memory_alloc(NULL, TCPPACKETSIZE, 0, &eb);
    if (buffer == NULL) {
        System_printf("tcpWorker: failed to alloc memory\n");
        Task_exit();
    }

    /* Loop while we receive data */
    while (flag) {
        nbytes = recv(clientfd, (char *)buffer, TCPPACKETSIZE, 0);
        if (nbytes > 0) {
            /* Echo the data back */
            send(clientfd, (char *)buffer, nbytes, 0 );
        }
        else {
            fdClose(clientfd);
            flag = FALSE;
        }
    }
    System_printf("tcpWorker stop clientfd = 0x%x\n", clientfd);

    /* Free the buffer back to the heap */
    Memory_free(NULL, buffer, TCPPACKETSIZE);

    fdCloseSession(TaskSelf());
    /*
     *  Since deleteTerminatedTasks is set in the cfg file,
     *  the Task will be deleted when the idle task runs.
     */
    Task_exit();
}

/*
 *  ======== tcpHandler ========
 *  Creates new Task to handle new TCP connections.
 */
Void tcpHandler(UArg arg0, UArg arg1)
{
    SOCKET lSocket;
    struct sockaddr_in6 sLocalAddr;
    SOCKET clientfd;
    struct sockaddr_in6 client_addr;
    Int addrlen=sizeof(client_addr);
    Int optval;
    Int optlen = sizeof(optval);
    Int status;
    Task_Handle taskHandle;
    Task_Params taskParams;
    Error_Block eb;

    fdOpenSession(TaskSelf());

    lSocket = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);
    if (lSocket < 0) {
        System_printf("tcpHandler: socket failed\n");
        Task_exit();
        return;
    }

    memset((char *)&sLocalAddr, 0, sizeof(sLocalAddr));
    sLocalAddr.sin6_family = AF_INET6;
    sLocalAddr.sin6_len = sizeof(sLocalAddr);
    /* accept any IPv6 address (IPv6 equivalent of INADDR_ANY) */
    memcpy((void *)&sLocalAddr.sin6_addr, (void *)(&IPV6_UNSPECIFIED_ADDRESS), sizeof(struct in6_addr));
    sLocalAddr.sin6_port = htons(arg0);

    status = bind(lSocket, (struct sockaddr *)&sLocalAddr, sizeof(sLocalAddr));
    if (status < 0) {
        System_printf("tcpHandler: bind failed\n");
        fdClose(lSocket);
        Task_exit();
        return;
    }

    if (listen(lSocket, NUMTCPWORKERS) != 0){
        System_printf("tcpHandler: listen failed\n");
        fdClose(lSocket);
        Task_exit();
        return;
    }

    if (setsockopt(lSocket, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0) {
        System_printf("tcpHandler: setsockopt failed\n");
        fdClose(lSocket);
        Task_exit();
        return;
    }

    while (TRUE) {
        /* Wait for incoming request */
        clientfd = accept(lSocket, (struct sockaddr*)&client_addr, &addrlen);
        System_printf("tcpHandler: Creating thread clientfd = %d\n", clientfd);

        /* Init the Error_Block */
        Error_init(&eb);

        /* Initialize the defaults and set the parameters. */
        Task_Params_init(&taskParams);
        taskParams.arg0 = (UArg)clientfd;
        taskParams.stackSize = 1024;
        taskHandle = Task_create((Task_FuncPtr)tcpWorker, &taskParams, &eb);
        if (taskHandle == NULL) {
            System_printf("tcpHandler: Failed to create new Task\n");
        }
    }
}

/*
 *  ======== IPv6DADStatus ========
 *  IPv6 initialization callback function
 */
static void IPv6DADStatus(IP6N Address, unsigned short dev_index, unsigned char Status)
{
    Char strIPAddress[40];

    /* Convert the IP Address to String Format. */
    IPv6IPAddressToString (Address, strIPAddress);

    /* Print the status of the address. */
    System_printf("Address: %s on device %d is %s\n", strIPAddress, dev_index,
            (Status == 1) ? "UNIQUE" : "DUPLICATE");

    System_flush();

    return;
}

/*
 *  ======== netOpenHook ========
 *  NDK network open hook used to initialize IPv6
 */
Void netOpenHook()
{
    Task_Handle taskHandle;
    Task_Params taskParams;
    Error_Block eb;
    Int status;
    Int dev_index = 1;

    /* Make sure Error_Block is initialized */
    Error_init(&eb);

    status = IPv6InterfaceInit(dev_index, IPv6DADStatus);
    if (status < 0) {
        System_printf("Error %d: failed to add IPv6 interface\n", status);
    }

    /*
     *  Create the Task that farms out incoming TCP connections.
     *  arg0 will be the port that this task listens to.
     */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 1024;
    taskParams.priority = 1;
    taskParams.arg0 = TCPPORT;
    taskHandle = Task_create((Task_FuncPtr)tcpHandler, &taskParams, &eb);
    if (taskHandle == NULL) {
        System_printf("main: Failed to create tcpHandler Task\n");
    }

    System_flush();
}

/*
 *  ======== netCloseHook ========
 *  NDK network close hook used to de-initialize IPv6
 */
void netCloseHook()
{
    Int status = 0;
    Int dev_index = 1;

    /* Enter the kernel Mode. */
    llEnter ();
    status = IPv6InterfaceDeInit(dev_index);
    llExit ();

    /* Were we able to deinitialize the stack? */
    if (status < 0) {
        System_printf("Error: Unable to de-initialize the IPv6 stack on device %d\n", dev_index);
    }
    else {
        System_printf("IPv6 stack has been deinitialized on %d\n", dev_index);
    }

    System_flush();
}

/*
 *  ======== main ========
 */
Int main(Void)
{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initEMAC();

    System_printf("Starting the TCP Echo IPv6 example\nSystem provider is set "
                  "to SysMin. Halt the target and use ROV to view output.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

