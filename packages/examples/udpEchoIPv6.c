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
 *    ======== udpEcho.c ========
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

#define UDPPACKETSIZE 1024
#define UDPPORT 1000

/*
 *  ======== udpHandler ========
 *  Echo back all UDP data received.
 *
 *  Since we are using UDP, the same socket is used for all incoming requests.
 */
Void udpHandler(UArg arg0, UArg arg1)
{
    SOCKET lSocket;
    struct sockaddr_in6 sLocalAddr;
    struct sockaddr_in6 client_addr;
    struct timeval to;
    fd_set readfds;
    Int addrlen = sizeof(client_addr);
    Int status;
    Error_Block eb;

    Int nbytes;
    Bool flag = TRUE;
    Char *buffer;

    fdOpenSession(TaskSelf());

    /* Make sure Error_Block is initialized */
    Error_init(&eb);

    /* Get a buffer to receive incoming packets. Use the default heap. */
    buffer = Memory_alloc(NULL, UDPPACKETSIZE, 0, &eb);
    if (buffer == NULL) {
        System_printf("udpWorker: failed to alloc memory\n");
        Task_exit();
    }

    lSocket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (lSocket < 0) {
        System_printf("udpHandler: socket failed\n");
        Task_exit();
        return;
    }

    memset((char *)&sLocalAddr, 0, sizeof(sLocalAddr));
    sLocalAddr.sin6_family = AF_INET6;
    sLocalAddr.sin6_len = sizeof(sLocalAddr);
    /* accept any IPv6 address (IPv6 equivalent of INADDR_ANY) */
    memcpy((void *)&sLocalAddr.sin6_addr, (void *)(&IPV6_UNSPECIFIED_ADDRESS),
            sizeof(struct in6_addr));
    sLocalAddr.sin6_port = htons(arg0);

    status = bind(lSocket, (struct sockaddr *)&sLocalAddr, sizeof(sLocalAddr));
    if (status < 0) {
        System_printf("udpHandler: bind failed: returned: %d, error: %d\n",
                status, fdError());
        fdClose(lSocket);
        Task_exit();
        return;
    }

    /* Give user time to connect with udpSendReceive client app */
    to.tv_sec  = 30;
    to.tv_usec = 0;
    if (setsockopt( lSocket, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof( to ) ) < 0) {
        System_printf("udpHandler: setsockopt SO_SNDTIMEO failed\n");
        fdClose(lSocket);
        Task_exit();
        return;
    }

    if (setsockopt( lSocket, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof( to ) ) < 0) {
        System_printf("udpHandler: setsockopt SO_RCVTIMEO failed\n");
        fdClose(lSocket);
        Task_exit();
        return;
    }

    /* Loop while we receive data */
    while (flag) {
        /*
         * Wait for the reply. Timeout after TIMEOUT seconds (assume UDP
         * packet dropped)
         */
        FD_ZERO(&readfds);
        FD_SET(lSocket, &readfds);

        if (fdSelect(0, &readfds, NULL, NULL, NULL) != 1) {
        	status = fdError();
            System_printf("timed out waiting for client\n", status);
            continue;
        }

        /* use recvfrom for IPv6 (non copy APIs not supported for IPv6) */
        nbytes = recvfrom(lSocket, buffer, UDPPACKETSIZE, MSG_WAITALL,
                (struct sockaddr *)&client_addr, &addrlen);

        if (nbytes >= 0) {
            /* Echo the data back */
            sendto(lSocket, (char *)buffer, nbytes, 0,
                    (struct sockaddr *)&client_addr, sizeof(client_addr));
        }
        else {
        	status = fdError();
        	if (status == EWOULDBLOCK) {
        		System_printf("udpHandler: Waiting for client to send UDP data\n");
        		continue;
        	}
        	else {
            System_printf(
                    "udpHandler: recvfrom failed: returned: %d, error: %d\n",
                    nbytes, status);
            fdClose(lSocket);
            flag = FALSE;
            }
        }
    }
    System_printf("udpHandler stop, lSocket = 0x%x\n", lSocket);

    /* Free the buffer back to the heap */
    Memory_free(NULL, buffer, UDPPACKETSIZE);

    if (lSocket) {
        fdClose(lSocket);
    }

    fdCloseSession(TaskSelf());

    /*
     *  Since deleteTerminatedTasks is set in the cfg file,
     *  the Task will be deleted when the idle task runs.
     */
    Task_exit();
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
     *  Create the Task that handles UDP "connections."
     *  arg0 will be the port that this task listens to.
     */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 1024;
    taskParams.priority = 1;
    taskParams.arg0 = UDPPORT;
    taskHandle = Task_create((Task_FuncPtr)udpHandler, &taskParams, &eb);
    if (taskHandle == NULL) {
        System_printf("main: Failed to create udpHandler Task\n");
    }
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

    System_printf("Starting the UDP Echo example\nSystem provider is set to "
                  "SysMin. Halt the target and use ROV to view output.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
