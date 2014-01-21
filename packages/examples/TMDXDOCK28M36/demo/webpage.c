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
 *  ======== webpage.c ========
 */
#include <ti/ndk/inc/netmain.h>

#include <xdc/std.h>
#include <string.h>

/* XDCtools Header files */
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/IHeap.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/utils/Load.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/ndk/inc/netmain.h>
#include <ti/ndk/inc/_stack.h>
#include <ti/ndk/inc/tools/cgiparse.h>
#include <ti/ndk/inc/tools/cgiparsem.h>

#include "default.h"
#include "logobar.h"
#include "dspchip.h"
#include "jquery.min.h"
#include "jquery.flot.min.h"
#include "layout.css.h"
#include "demo.h"

int cgiInform( SOCKET htmlSock, int ContentLength, char *pArgs );
int listLoad( SOCKET htmlSock, int ContentLength );
int listTasks( SOCKET htmlSock, int ContentLength );
int listHeapMems( SOCKET htmlSock, int ContentLength );
int listDataT( SOCKET htmlSock, int ContentLength );
int listDataA( SOCKET htmlSock, int ContentLength );
int enableRecording( SOCKET htmlSock, int ContentLength );
int disableRecording( SOCKET htmlSock, int ContentLength );

extern Float temperatureF;
extern Float temperatureC;
extern Int recordingEnabled;

//
// Page Creation Commonly Used Data
//
static const char *pstr_HTML_START =
       "<html><body text=#000000 bgcolor=#ffffff>\r\n";
static const char *pstr_HTML_END =
       "</body></html>\r\n";
static const char *pstr_TI_START =
       "<center><p><image src=\"logobar.gif\"></p>\r\n";
static const char *pstr_TI_END =
       "</center></body></html>\r\n";
static const char *pstr_ROW_START =
       "<tr>";
static const char *pstr_ROW_END =
       "</tr>\r\n";
static const char *pstr_DIVIDER =
       "<hr WIDTH=\"100%\"><br>\r\n";
static const char *pstr_TABLE_START =
       "<table border cellspacing=0 cellpadding=5>\r\n";
static const char *pstr_TABLE_END =
       "</table><br>\r\n";
static const char *pstr_LINK_MAIN =
       "<a href=index.html>Return to Main Page</a><br><br>\r\n";

//
// Page Creation Macro
//
#define html(str) httpSendClientStr(htmlSock, (char *)str)

static Void CreateIPUse(SOCKET htmlSock);
static Void CreatehtmlSockets(SOCKET htmlSock);
static Void CreateRoute(SOCKET htmlSock);

/*
 *  ======== AddWebFiles ========
 *  Add the webpages into NDK's embedded filesystem
 */
Void AddWebFiles(Void)
{
    Void *pFxn;

    efs_createfile("index.html", DEFAULT_SIZE, (UINT8 *)DEFAULT);
    efs_createfile("logobar.gif", LOGOBAR_SIZE, (UINT8 *)LOGOBAR);
    efs_createfile("dspchip.gif", DSPCHIP_SIZE, (UINT8 *)DSPCHIP);
    efs_createfile("jquery.flot.min.js", JQUERY_FLOT_SIZE, (UINT8 *)JQUERY_FLOT);
    efs_createfile("jquery.min.js", JQUERY_SIZE, (UINT8 *)JQUERY);
    pFxn = (Void*) &cgiInform;
    efs_createfile("inform.cgi", 0, (UINT8 *) pFxn);
    pFxn = (Void*) &listTasks;
    efs_createfile("listTasks.cgi", 0, (UINT8 *) pFxn);
    pFxn = (Void*) &listHeapMems;
    efs_createfile("listHeapMems.cgi", 0, (UINT8 *) pFxn);
    pFxn = (Void*) &listLoad;
    efs_createfile("listLoad.cgi", 0, (UINT8 *) pFxn);
    pFxn = (Void*) &listDataT;
    efs_createfile("listDataT.cgi", 0, (UINT8 *) pFxn);
    pFxn = (Void*) &listDataA;
    efs_createfile("listDataA.cgi", 0, (UINT8 *) pFxn);
    pFxn = (Void*) &enableRecording;
    efs_createfile("enableRecording.cgi", 0, (UINT8 *) pFxn);
    pFxn = (Void*) &disableRecording;
    efs_createfile("disableRecording.cgi", 0, (UINT8 *) pFxn);
}

/*
 *  ======== RemoveWebFiles ========
 *  Remove the webpages into NDK's embedded filesystem
 */
Void RemoveWebFiles(Void)
{
    efs_destroyfile("index.html");
    efs_destroyfile("logobar.gif");
    efs_destroyfile("dspchip.gif");
    efs_destroyfile("jquery.flot.min.js");
    efs_destroyfile("jquery.min.js");
    efs_destroyfile("inform.cgi");
    efs_destroyfile("listTasks.cgi");
    efs_destroyfile("listHeapMems.cgi");
    efs_destroyfile("listLoad.cgi");
    efs_destroyfile("listDataT.cgi");
    efs_destroyfile("listDataA.cgi");
    efs_destroyfile("enableRecording.cgi");
    efs_destroyfile("disableRecording.cgi");
}

/*
 *  ======== enableRecording ========
 *  Enable Temperature recording
 *
 */
int enableRecording(SOCKET s, int length)
{
    httpSendStatusLine(s, HTTP_OK, "text/plain");
    httpSendClientStr(s, CRLF);

    if (recordingEnabled == RECORDING_CLOSED) {
        recordingEnabled = RECORDING_OPEN;
    }
    return 1;
}

/*
 *  ======== disableRecording ========
 *
 *  Disable Temperature recording
 *
 */
int disableRecording(SOCKET s, int length)
{
    httpSendStatusLine(s, HTTP_OK, "text/plain");
    httpSendClientStr(s, CRLF);

    if ((recordingEnabled == RECORDING_OPEN) ||
        (recordingEnabled == RECORDING_WRITING)) {
        recordingEnabled = RECORDING_CLOSE;
    }
    return 1;
}

/*
 *  ======== listDataT ========
 *
 *  Display the current temperature
 *
 */
int listDataT(SOCKET s, int length)
{
    Char buf[40];

    httpSendStatusLine(s, HTTP_OK, "text/plain");
    httpSendClientStr(s, CRLF);

    System_sprintf(buf, "%d %d\n", (int)temperatureF, (int)temperatureC);
    httpSendClientStr(s, buf);

    return 1;
}

/*
 *  ======== listDataA ========
 *
 *  Display the current Accel.
 *
 */
int listDataA(SOCKET s, int length)
{
    Char buf[20];
    Char value;
    static Int temp = 0;

    httpSendStatusLine(s, HTTP_OK, "text/plain");
    httpSendClientStr(s, CRLF);

    value = temp++;
    System_sprintf(buf, "%d\n", (int)value);
    httpSendClientStr(s, buf);

    return 1;
}

/*
 *  ======== dumpTask ========
 *
 *  Generate HTML table row for a task and send to the supplied socket.
 *
 */
static Void dumpTask(Task_Handle task, SOCKET s)
{
    Task_Stat status;
    Char buf[300];

    Task_stat(task, &status);
    System_sprintf(buf,
        "<tr><td>0x%x %s</td><td>%d</td><td>%d</td><td>%d</td><td>%d</td></tr>\n",
        task, Task_Handle_name(task),
        status.priority, status.mode, status.stackSize, status.used);

    httpSendClientStr(s, buf);
}

/*
 *  ======== listTasks ========
 *
 *  Iterate over both static and dynamics task lists and dump HTML
 *  to a socket. Structured for use with CGI functions in NDK HTTP.
 *
 */
int listTasks(SOCKET s, int length)
{
    Task_Object * task;
    Char buf[200];
    Int i;
    static UInt scalar = 0;

    if (scalar == 0) {
        scalar = 1000000u / Clock_tickPeriod;
    }

    httpSendStatusLine(s, HTTP_OK, CONTENT_TYPE_HTML);
    httpSendClientStr(s, CRLF);
    httpSendClientStr(s,
        "<html><head><title>SYS/BIOS Tasks</title></head><body><h1>Tasks</h1>\n");

    System_sprintf(buf, "<p>Up for %d seconds</p>\n",
        ((unsigned long)Clock_getTicks() / scalar));
    httpSendClientStr(s, buf);

    httpSendClientStr(s, "<table border=\"1\">\n");
    httpSendClientStr(s, "<tr><th>task</th><th>priority</th><th>state</th><th>stack size</th><th>stack used</th></tr>");


    for (i = 0; i < Task_Object_count(); i++) {
        task = Task_Object_get(NULL, i);
        dumpTask(task, s);
    }

    task = Task_Object_first();
    while (task) {
        dumpTask(task, s);
        task = Task_Object_next(task);
    }

    httpSendClientStr(s, "</table></body></html>");

    return 1;
}

/*
 *  ======== dumpHeapMem ========
 *
 *  Generate HTML table row for a heap and send to the supplied socket.
 *
 */
static Void dumpHeapMem(HeapMem_Handle heap, SOCKET s)
{
    Memory_Stats status;
    Char buf[300];

    Memory_getStats((IHeap_Handle)heap, &status);
    System_sprintf(buf,
        "<tr><td>0x%x %s</td><td>%d</td><td>%d</td><td>%d</td></tr>\n",
        heap, HeapMem_Handle_name(heap),
        status.totalSize, status.totalFreeSize, status.largestFreeSize);

    httpSendClientStr(s, buf);
}

/*
 *  ======== listHeapMems ========
 *
 *  Iterate over both static and dynamics heap lists and dump HTML
 *  to a socket. Structured for use with CGI functions in NDK HTTP.
 *
 */
int listHeapMems(SOCKET s, int length)
{
    HeapMem_Object * heap;
    Char buf[200];
    Int i;
    static UInt scalar = 0;

    if (scalar == 0) {
        scalar = 1000000u / Clock_tickPeriod;
    }

    httpSendStatusLine(s, HTTP_OK, CONTENT_TYPE_HTML);
    httpSendClientStr(s, CRLF);
    httpSendClientStr(s,
        "<html><head><title>SYS/BIOS HeapMems</title></head><body><h1>HeapMems</h1>\n");

    System_sprintf(buf, "<p>Up for %d seconds</p>\n",
        ((unsigned long)Clock_getTicks() / scalar));
    httpSendClientStr(s, buf);

    httpSendClientStr(s, "<table border=\"1\">\n");
    httpSendClientStr(s, "<tr><th>heap</th><th>totalSize</th><th>totalFreeSize</th><th>largestFreeSize</th></tr>");


    for (i = 0; i < HeapMem_Object_count(); i++) {
        heap = HeapMem_Object_get(NULL, i);
        dumpHeapMem(heap, s);
    }

    heap = HeapMem_Object_first();
    while (heap) {
        dumpHeapMem(heap, s);
        heap = HeapMem_Object_next(heap);
    }

    httpSendClientStr(s, "</table></body></html>");

    return 1;
}
/*
 *  ======== listLoad ========
 *  Return the currect CPU load
 *
 *
 */
int listLoad(SOCKET s, int length)
{
    static UInt scalar = 0;
    Char buf[50];

    if (scalar == 0) {
        scalar = 1000000u / Clock_tickPeriod;
    }

    httpSendStatusLine(s, HTTP_OK, CONTENT_TYPE_HTML);
    httpSendClientStr(s, CRLF);
    httpSendClientStr(s,
        "<html><head><title>CPU Load</title></head><body><h1>CPU Load</h1>\n");

    System_sprintf(buf, "<p>Up for %d seconds</p>\n",
        ((unsigned long)Clock_getTicks() / scalar));
    httpSendClientStr(s, buf);

    System_sprintf(buf, "<p>CPU Load = %d </p>\n", Load_getCPULoad());
    httpSendClientStr(s, buf);

    httpSendClientStr(s, "</html>");

    return 1;
}

//
// Main CGI Function
//
int cgiInform(SOCKET htmlSock, int ContentLength, char *pArgs)
{
    char *postdata = 0;
    char *name, *value;
    int  bytes;
    int  parseIndex;

    //
    // Here, lets support either a "get" or a "post". In a
    // "get", there is no ContentLength
    //
    if( !ContentLength )
    {
        // Since we know we are in "get" mode, the "get" arguments
        // must be valid
        if( pArgs )
        {
            value = pArgs;
            goto CHECKARGS;
        }

        // We don't support a get with no arguments
        http405(htmlSock);
        goto FATAL;
    }

    // Allocate space for the CGI post data
    postdata = (char *)mmBulkAlloc(ContentLength+1);
    if( !postdata )
        goto FATAL;

    // Read in the post data from the socket
    bytes = recv(htmlSock, postdata, ContentLength, MSG_WAITALL);
    if( bytes < 1 )
        goto FATAL;

    // Setup to parse the post data
    parseIndex = 0;
    postdata[ContentLength] = '\0';

    // Read until we've read the item "ip"
    do {
        name  = cgiParseVars( postdata, &parseIndex );
        value = cgiParseVars( postdata, &parseIndex );
    } while( strcmp("ip", name ) && parseIndex != -1 );

CHECKARGS:
    // Now check the value of "ip"
    if (strcmp("ipinfo", value) == 0)
        CreateIPUse(htmlSock);
    else if (strcmp("sockets", value) == 0)
        CreatehtmlSockets(htmlSock);
    else if (strcmp("route", value) == 0)
        CreateRoute(htmlSock);
    else
        http405(htmlSock);

FATAL:
    // Free the data we've allocated
    if( postdata )
        mmBulkFree(postdata);

    // We always leave the socket open
    return( 1 );
}

Void CreateIPUse(SOCKET htmlSock)
{
    IPN     myIP;
    IPN     yourIP;
    char    pszmyIP[32];
    char    pszyourIP[32];
    struct  sockaddr_in Info;
    int     InfoLength;
    char    tmpBuf[128];
    HOSTENT *dnsInfo;
    char    htmlbuf[MAX_RESPONSE_SIZE];
    int     rc;

    InfoLength = sizeof(Info);
    getsockname( htmlSock, (PSA)&Info, &InfoLength );
    myIP = Info.sin_addr.s_addr;
    NtIPN2Str( myIP, pszmyIP );

    InfoLength = sizeof(Info);
    getpeername( htmlSock, (PSA)&Info, &InfoLength );
    yourIP = Info.sin_addr.s_addr;
    NtIPN2Str( yourIP, pszyourIP );

    httpSendStatusLine(htmlSock, HTTP_OK, CONTENT_TYPE_HTML);
    // CRLF before entity
    html( CRLF );

    html(pstr_HTML_START);
    html(pstr_TI_START);
    html("<h1>IP Address Information</h1>\r\n");
    html(pstr_DIVIDER);

    html(pstr_TABLE_START);

    html(pstr_ROW_START);
    System_sprintf(htmlbuf,"<td>HTTP Server IP Address</td><td>%s</td>", pszmyIP);
    html(htmlbuf);
    html(pstr_ROW_END);

    html(pstr_ROW_START);
    html("<td>HTTP Server Hostname</td>");
    rc = DNSGetHostByAddr( myIP, tmpBuf, sizeof(tmpBuf) );
    if( rc )
        System_sprintf(htmlbuf, "<td>%s</td>", DNSErrorStr(rc) );
    else
    {
        dnsInfo = (HOSTENT*) tmpBuf;
        System_sprintf(htmlbuf, "<td>%s</td>", dnsInfo->h_name);
    }
    html(htmlbuf);
    html(pstr_ROW_END);

    html(pstr_ROW_START);
    System_sprintf(htmlbuf, "<td>Your IP Address</td><td>%s</td>", pszyourIP);
    html(htmlbuf);
    html(pstr_ROW_END);

    html(pstr_ROW_START);
    html("<td>Your Hostname</td>");
    rc = DNSGetHostByAddr( yourIP, tmpBuf, sizeof(tmpBuf) );
    if( rc )
        System_sprintf(htmlbuf, "<td>%s</td>", DNSErrorStr(rc) );
    else
    {
        dnsInfo = (HOSTENT*) tmpBuf;
        System_sprintf(htmlbuf, "<td>%s</td>", dnsInfo->h_name);
    }
    html(htmlbuf);
    html(pstr_ROW_END);

    html(pstr_TABLE_END);
    html(pstr_DIVIDER);
    html(pstr_LINK_MAIN);
    html(pstr_TI_END);
    html(pstr_HTML_END);
}

static Void DumphtmlSockets( SOCKET htmlSock, uint htmlSockProt );

Void CreatehtmlSockets(SOCKET htmlSock)
{
    httpSendStatusLine(htmlSock, HTTP_OK, CONTENT_TYPE_HTML);
    // CRLF before entity
    html( CRLF );

    html(pstr_HTML_START);
    html(pstr_TI_START);
    html("<h1>TCP/IP Socket State Information</h1>\r\n");

    html(pstr_DIVIDER);
    html("<h2>TCP Sockets</h2>\r\n");
    DumphtmlSockets( htmlSock, SOCKPROT_TCP );

    html(pstr_DIVIDER);
    html("<h2>UDP Sockets</h2>\r\n");
    DumphtmlSockets( htmlSock, SOCKPROT_UDP );

    html(pstr_DIVIDER);
    html(pstr_LINK_MAIN);
    html(pstr_TI_END);
    html(pstr_HTML_END);
}

static const char *States[] = { "CLOSED","LISTEN","SYNSENT","SYNRCVD",
                          "ESTABLISHED","CLOSEWAIT","FINWAIT1","CLOSING",
                          "LASTACK","FINWAIT2","TIMEWAIT" };

static Void DumphtmlSockets( SOCKET htmlSock, uint htmlSockProt )
{
    UINT8   *pBuf;
    int     Entries,i;
    SOCKPCB *ppcb;
    char    str[32];
    char    htmlbuf[MAX_RESPONSE_SIZE];


    pBuf = mmBulkAlloc(2048);
    if( !pBuf )
        return;

    // Use llEnter / llExit since we're calling into the stack
    llEnter();
    Entries = SockGetPcb( htmlSockProt, 2048, pBuf );
    llExit();

    html(pstr_TABLE_START);

    html(pstr_ROW_START);
    html("<td>Local IP</td><td>LPort</td>");
    html("<td>Foreign IP</td><td>FPort</td>\r\n");
    if( htmlSockProt == SOCKPROT_TCP )
        html("<td>State</td>\r\n");
    html(pstr_ROW_END);

    for(i=0; i<Entries; i++)
    {
        ppcb = (SOCKPCB *)(pBuf+(i*sizeof(SOCKPCB)));

        html(pstr_ROW_START);
        NtIPN2Str( ppcb->IPAddrLocal, str );
        System_sprintf(htmlbuf, "<td>%-15s</td><td>%-5u</td>", str, htons(ppcb->PortLocal) );
        html(htmlbuf);
        NtIPN2Str( ppcb->IPAddrForeign, str );
        System_sprintf(htmlbuf, "<td>%-15s</td><td>%-5u</td>\r\n", str, htons(ppcb->PortForeign) );
        html(htmlbuf);
        if( htmlSockProt == SOCKPROT_TCP )
        {
            System_sprintf(htmlbuf,"<td>%s</td>\r\n",States[ppcb->State]);
            html(htmlbuf);
        }
        html(pstr_ROW_END);
    }

    html(pstr_TABLE_END);

    mmBulkFree( pBuf );
}

Void CreateRoute(SOCKET htmlSock)
{
    HANDLE  hRt,hIF,hLLI;
    uint    wFlags,IFType,IFIdx;
    UINT32  IPAddr,IPMask;
    char    str[32];
    UINT8   MacAddr[6];
    char    htmlbuf[MAX_RESPONSE_SIZE];

    httpSendStatusLine(htmlSock, HTTP_OK, CONTENT_TYPE_HTML);
    // CRLF before entity
    html( CRLF );

    html(pstr_HTML_START);
    html(pstr_TI_START);
    html("<h1>TCP/IP Current Route Table</h1>\r\n");
    html(pstr_DIVIDER);

    // Start walking the tree
    llEnter();
    hRt = RtWalkBegin();
    llExit();

    html(pstr_TABLE_START);

    html(pstr_ROW_START);
    html("<td>Address</td><td>Subnet Mask</td>");
    html("<td>Flags</td><td>Gateway</td>\r\n");
    html(pstr_ROW_END);

    // While there are routes, print the route information
    while( hRt )
    {
        html(pstr_ROW_START);

        // Get the IP addess and IP mask and flags of the route
        llEnter();
        IPAddr = RtGetIPAddr( hRt );
        IPMask = RtGetIPMask( hRt );
        wFlags = RtGetFlags( hRt );
        hIF    = RtGetIF( hRt );
        if( hIF )
        {
            IFType = IFGetType(hIF);
            IFIdx  = IFGetIndex(hIF);
        }
        else
            IFType = IFIdx = 0;
        llExit();

        // Print address and mask
        NtIPN2Str( IPAddr, str );
        System_sprintf(htmlbuf, "<td>%-15s</td>", str );
        html(htmlbuf);
        NtIPN2Str( IPMask, str );
        System_sprintf(htmlbuf, "<td>%-15s</td>", str );
        html(htmlbuf);

        // Decode flags
        if( wFlags & FLG_RTE_UP )
            strcpy(str,"U");
        else
            strcpy(str," ");
        if( wFlags & FLG_RTE_GATEWAY )
            strcat(str,"G");
        else
            strcat(str," ");
        if( wFlags & FLG_RTE_HOST )
            strcat(str,"H");
        else
            strcat(str," ");
        if( wFlags & FLG_RTE_STATIC )
            strcat(str,"S");
        else
            strcat(str," ");
        if( wFlags & FLG_RTE_CLONING )
            strcat(str,"C");
        else
            strcat(str," ");
        if( wFlags & FLG_RTE_IFLOCAL )
            strcat(str,"L");
        else
            strcat(str," ");

        System_sprintf(htmlbuf, "<td>%s</td>", str );
        html(htmlbuf);

        // If the route is a gateway, print the gateway IP address as well
        if( wFlags & FLG_RTE_GATEWAY )
        {
            llEnter();
            IPAddr = RtGetGateIP( hRt );
            llExit();
            NtIPN2Str( IPAddr, str );
            System_sprintf(htmlbuf, "<td>%-15s</td>", str );
            html(htmlbuf);
        }
        // Else if non-local host route on Ethernet, print ARP entry
        else if( IFType == HTYPE_ETH &&
                 (wFlags&FLG_RTE_HOST) && !(wFlags&FLG_RTE_IFLOCAL) )
        {
            // The stack has a MAC address if it has an LLI (link-layer info)
            // object, and LLIGetMacAddr returns 1.
            llEnter();
            if( !(hLLI = RtGetLLI( hRt )) || !LLIGetMacAddr( hLLI, MacAddr, 6 ) )
                llExit();
            else
            {
                llExit();
                System_sprintf( htmlbuf,"<td>%02X:%02X:%02X:%02X:%02X:%02X</td>",
                           MacAddr[0], MacAddr[1], MacAddr[2],
                           MacAddr[3], MacAddr[4], MacAddr[5] );
                html(htmlbuf);
            }
        }
        // Else just print out the interface
        else if( IFIdx )
        {
            if( wFlags & FLG_RTE_IFLOCAL )
            {
                System_sprintf( htmlbuf,"<td>local (if-%d)</td>", IFIdx );
                html(htmlbuf);
            }
            else
            {
                System_sprintf( htmlbuf,"<td>if-%d</td>", IFIdx );
                html(htmlbuf);
            }
        }

        html(pstr_ROW_END);

        llEnter();
        hRt = RtWalkNext( hRt );
        llExit();
    }
    llEnter();
    RtWalkEnd( 0 );
    llExit();

    html(pstr_TABLE_END);
    html(pstr_DIVIDER);
    html(pstr_LINK_MAIN);
    html(pstr_TI_END);
    html(pstr_HTML_END);
}
