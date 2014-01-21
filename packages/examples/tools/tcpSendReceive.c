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
 * --/COPYRIGHT--*/
 
/*
 *  ======== tcpSendReceive.c ========
 *  Tool to exercise the TCP portion of the sockets example.
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include "sockets.h"

#define MAXBUF    1024

/*
 *  ======== main ========
 */
int main(int argc, char *argv[])
{
    int i;
    int sockfd = 0;
    int bytes_read, bytes_sent;
    struct addrinfo hints;
    struct addrinfo *results = NULL;
    int status = EXIT_SUCCESS;
    int count = 0;
    int id;
    int value;
    unsigned int sleepTime = 1000;
    unsigned int buffSize  = MAXBUF;
    char *buffer = NULL;
    time_t start;
    start = time(NULL);

    /* parameter check */
    if (argc < 4 || argc > 6) {
        printf("usage: %s <IPv4 or IPv6 addr> <port> <id> -l[length] -s[sleep in uS]\n", argv[0]);
        status = EXIT_FAILURE;
        goto QUIT;
    }

    id = atoi(argv[3]);

    /* Parse options */
    i = argc - 1;
    while ((i > 3) && (argv[i][0] == '-')) {
        switch (argv[i][1]) {
            case 'l':
                buffSize = atoi(&argv[i][2]);
                break;
            case 's':
                sleepTime = atoi(&argv[i][2]);
                break;
            default:
                printf("Valid options are -l[length] and -s[sleep in uS]\n");
                status = EXIT_FAILURE;
                goto QUIT;
        }
        i--;
    }

    buffer = malloc(buffSize);
    if (buffer == NULL) {
        printf("malloc failed\n");
        status = EXIT_FAILURE;
    }
    memset(buffer, 0, buffSize);

    /* initialize sockets environment */
    socketsStartup();

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    /*
     * getaddrinfo() fills in the results struct for us appropriately
     * depending on whether the IP address is v4 or v6
     *
     * argv[1] = IPv4 or IPv6 address passed in from command line
     * argv[2] = port number passed in from command line
     */
    value = getaddrinfo(argv[1], argv[2], &hints, &results);

    if (value != 0) {
        fprintf(stderr, "getaddrinfo failed: %d\n", value);
        if (value == -2 || value == 11004) {
            fprintf(stderr, "unrecognized IP address\n");
        }
        status = EXIT_FAILURE;
        goto QUIT;
    }

    /* create socket. ai_family determined for us via getaddrinfo() call */
    if ((sockfd = socket(results->ai_family, SOCK_STREAM, 0)) < 0) {
        fprintf(stderr, "socket failed: %d\n", errno);
        status = errno;
        goto QUIT;
    }
    
    /* connect. ai_addr set to AF_INET or AF_INET6 by getaddrinfo() call */
    if (connect(sockfd, results->ai_addr, results->ai_addrlen) < 0) {
        printf("connect failed: %d\n", errno);
        status = errno;
        goto QUIT;
    }
    
    printf("Starting test with a %d uSec delay between transmits\n", sleepTime);

    /* loop */
    i = 0;
    while (1) {
        buffer[0] = (char)(++i);
        buffer[buffSize - 1] = (char)~i;
        bytes_sent = send(sockfd, buffer, buffSize, 0);
        if (bytes_sent != buffSize) {
            printf("[id %d] stopping test. send returned %d\n", id, bytes_sent);
            status = EXIT_FAILURE;
            goto QUIT;
        }

        bytes_read = recv(sockfd, buffer, buffSize, 0);
        if (bytes_read != buffSize) {
            printf("[id %d] stopping test. recv returned %d\n", id, bytes_read);
            status = EXIT_FAILURE;
            goto QUIT;
        }

        count++;

        if (count % 1000 == 0) {
            printf("[id %d] count = %d, time = %ld\n", id, count,
                   time(NULL) - start);
        }

        /* Sleep specified time */
#if defined(__GNUC__) && defined(linux)
        usleep(sleepTime);
#else
        Sleep(sleepTime / 1000);
#endif

    if ((buffer[0] != (char)i) || (buffer[buffSize - 1] != (char)~i)) {
            printf("mismatch buffer[0] = %d, (char)i = %d\n", buffer[0],
                   (char)i);
            printf("mismatch buffer[buffSize - 1] = %d, (char)~i = %d\n",
                   buffer[buffSize - 1], (char)~i);
        }
    }

QUIT:
    /* clean up */
    if (sockfd) {
        closesocket(sockfd);
    }

    if (results) {
        freeaddrinfo(results);
    }

    socketsShutdown();

    if (buffer) {
        free(buffer);
    }
    return (status);
}
