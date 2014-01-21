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
 *    ======== udpEchoCC3000.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>
#include <string.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/WiFi.h>

/* SimpleLink Wi-Fi Host Driver Header files */
#include <cc3000_host_driver/include/common/cc3000_common.h>
#include <cc3000_host_driver/include/netapp.h>
#include <cc3000_host_driver/include/nvmem.h>
#include <cc3000_host_driver/include/socket.h>
#include <cc3000_host_driver/include/wlan.h>

/* Example/Board Header files */
#include "Board.h"

/* Use smaller packet size for MSP430 */
#ifdef MSP430WARE
#define UDPPACKETSIZE   256
#else
#define UDPPACKETSIZE   1024
#endif

/* Port number to which the connection is bound */
#define UDPPORT         1000

/* CC3000 firmware version compatible with this example */
#define PACKAGEID       1
#define PACKAGEBLDNUM   24

/* Flags set in callback */
Bool smartConfigFinished = FALSE;
Bool deviceConnected = FALSE;
Bool dhcpComplete = FALSE;

/* Buffer to hold data received */
Char buffer[UDPPACKETSIZE];
Char ipRecvd[4];

/*
 *  ======== asynchCallback ========
 *  Callback for handling unsolicited events. A function pointer is passed in
 *  through the WiFi_Params to register the callback with the Host Driver.
 */
Void asynchCallback(long eventType, char *data, unsigned char length)
{
    switch(eventType){
        case HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE:
            /* Smart Config process has completed */
            smartConfigFinished = TRUE;
            break;

        case HCI_EVNT_WLAN_UNSOL_CONNECT:
            /* CC3000 connected to an AP */
            deviceConnected = TRUE;
            GPIO_write(Board_LED0, Board_LED_ON);
            break;

        case HCI_EVNT_WLAN_UNSOL_DISCONNECT:
            /* CC3000 disconnected from an AP */
            deviceConnected = FALSE;
            GPIO_write(Board_LED0, Board_LED_OFF);
            break;

        case HCI_EVNT_WLAN_UNSOL_DHCP:
            /* DHCP report */
            dhcpComplete = TRUE;
            ipRecvd[0] = data[3];
            ipRecvd[1] = data[2];
            ipRecvd[2] = data[1];
            ipRecvd[3] = data[0];
            break;
    }
}

/*
 *  ======== smartConfigFxn ========
 *  Starts the Smart Config process which allows the user to tell the CC3000
 *  which AP to connect to, using a smart phone app. Downloads available here:
 *  http://www.ti.com/tool/smartconfig
 */
Void smartConfigFxn(Void)
{
    UChar subnetMask[] = {0, 0, 0, 0};
    UChar ipAddr[] = {0, 0, 0, 0};
    UChar defaultGateway[] = {0, 0, 0, 0};
    UChar dnsServer[] = {0, 0, 0, 0};

    /* Acquire an IP address */
    netapp_dhcp((unsigned long *)ipAddr, (unsigned long *)subnetMask,
                (unsigned long *)defaultGateway, (unsigned long *)dnsServer);

    /* Delete current policy */
    wlan_ioctl_set_connection_policy(0, 0, 0);

    /* Restart the CC3000 */
    wlan_stop();
    Task_sleep(50);
    wlan_start(0);

    /* Set Smart Config prefix and start Smart Config */
    wlan_smart_config_set_prefix("TTT");
    wlan_smart_config_start(0);

    /* Wait for Smart Config to finish. LED will blink until complete. */
    while (smartConfigFinished == 0) {
        Task_sleep(400);
        GPIO_toggle(Board_LED0);
    }

    /* Configure to connect automatically to the AP retrieved */
    wlan_ioctl_set_connection_policy(0, 0, 1);

    GPIO_write(Board_LED0, Board_LED_OFF);

    /* Restart the CC3000 */
    wlan_stop();
    Task_sleep(50);
    wlan_start(0);

    /* Set connection flag to 'disconnected' */
    deviceConnected = 0;
    dhcpComplete = 0;

    /* Mask out all non-required events */
    wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE | HCI_EVNT_WLAN_UNSOL_INIT |
                        HCI_EVNT_WLAN_ASYNC_PING_REPORT);
}

/*
 *  ======== echoFxn ========
 *  Prints IP address and echos messages through UDP.
 *
 *  Task for this function is created statically. See the project's .cfg file.
 */
Void echoFxn(UArg arg0, UArg arg1)
{
    Int         nbytes;
    Int         status;
    Int         selectRes;
    Bool        flag = TRUE;
    UChar       spNum[2] = {0};
    ULong       currButton;
    ULong       prevButton = 0;
    Long        lSocket;
    fd_set      readSet;
    timeval     timeout;
    WiFi_Params params;
    WiFi_Handle handle;
    sockaddr_in sLocalAddr;
    sockaddr_in client_addr;
    socklen_t   addrlen = sizeof(client_addr);

    /* Turn LED off. It will be used as a connection indicator */
    GPIO_write(Board_LED0, Board_LED_OFF);

    /* Open WiFi */
    WiFi_Params_init(&params);
    params.bitRate = 4000000;
    handle = WiFi_open(Board_WIFI, Board_SPI_CC3000, asynchCallback, &params);

    /* Mask out all non-required events */
    wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE | HCI_EVNT_WLAN_UNSOL_INIT |
                        HCI_EVNT_WLAN_ASYNC_PING_REPORT);

    /* Check service pack version */
    nvmem_read_sp_version(spNum);
    if ((spNum[0] != PACKAGEID) || (spNum[1] != PACKAGEBLDNUM)) {
        System_printf("You are using service pack version %d.%d! This example "
                      "is recommended for use\nwith %d.%d. Run the TI-RTOS "
                      "CC3000 Patcher example to get this version.\n\n",
                      spNum[0], spNum[1], PACKAGEID, PACKAGEBLDNUM);
        System_flush();
    }

    /*
     * Wait for the WiFi device to connect to an AP. If a profile for the AP in
     * use has not been stored yet, press Board_BUTTON0 to put the CC3000 in
     * Smart Config mode.
     */
    while ((deviceConnected != TRUE) || (dhcpComplete != TRUE)) {
        /*
         *  Start Smart Config if a button is pressed. This could be done with
         *  GPIO interrupts, but for simplicity polling is used to check the
         *  button.
         */
        currButton = GPIO_read(Board_BUTTON0);
        if((currButton == 0) && (prevButton != 0))
        {
            smartConfigFxn();
        }
        prevButton = currButton;
        Task_sleep(50);
    }

    System_printf("CC3000 has connected to AP and acquired an IP address.\n");

    /* Print IP address */
    System_printf("IP Address: %d.%d.%d.%d\n", ipRecvd[0], ipRecvd[1],
                                               ipRecvd[2], ipRecvd[3]);
    System_flush();

    /* Echo data using UDP */
    lSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (lSocket == -1) {
        System_printf("socket failed\n");
        Task_exit();
    }

    memset((char *)&sLocalAddr, 0, sizeof(sLocalAddr));
    sLocalAddr.sin_family = AF_INET;
    sLocalAddr.sin_addr.s_addr = htonl(0);
    sLocalAddr.sin_port = htons(UDPPORT);

    status = bind(lSocket, (const sockaddr*)&sLocalAddr, sizeof(sLocalAddr));
    if (status < 0) {
        System_printf("bind failed\n");
        closesocket(lSocket);
        Task_exit();
    }

    /* Set up half-second timeout for select() */
    memset(&timeout, 0, sizeof(timeval));
    timeout.tv_sec = 0;
    timeout.tv_usec = 500000;

    /* Loop while we receive data */
    while (flag) {
        FD_ZERO(&readSet);
        FD_SET(lSocket, &readSet);

        /* Wait for the reply. If timeout, assume UDP packet dropped */
        selectRes = select(lSocket + 1, &readSet, NULL, NULL, &timeout);

        if ((selectRes > 0) && (selectRes != -1)) {
            if(FD_ISSET(lSocket, &readSet)) {
                nbytes = recvfrom(lSocket, buffer, UDPPACKETSIZE, 0,
                                 (sockaddr*)&client_addr, &addrlen);
                if (nbytes > 0) {
                    /* Echo the data back */
                    sendto(lSocket, buffer, nbytes, 0, (sockaddr*)&client_addr,
                           sizeof(client_addr));
                }
                else {
                    flag = FALSE;
                }
            }
        }

        if (selectRes == -1) {
            flag = FALSE;
        }
    }

    System_printf("Closing socket 0x%x.\n", lSocket);
    closesocket(lSocket);

    WiFi_close(handle);

    Task_exit();
}

/*
 *  ======== main ========
 */
Int main(Void)
{
    /* Call board init functions. */
    Board_initGeneral();
    Board_initGPIO();
    Board_initWiFi();

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the UDP Echo example for the CC3000 \nSystem"
                  " provider is set to SysMin. Halt the target and use ROV to "
                  "view output.\n\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
