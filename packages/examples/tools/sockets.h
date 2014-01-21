/*
 *  ======== sockets.h ========
 *
 *  Simple (incomplete) compatibility header file to make it easier to
 *  write portable sockets-based C programs for Linux and Windows.
 *  This is meant to be just good enough for the simple NDK examples.
 */
#ifndef _SOCKETS_H_
#define _SOCKETS_H_

#if defined(__GNUC__) && defined(linux)

#include <errno.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/types.h>
#include <netdb.h>

typedef int SOCKET;

#define closesocket(s) close(s)

/* these are not needed for Linux */
#define socketsShutdown()
#define socketsStartup()

#define kbhit() 0

#define getError() (errno)

#elif defined(__MINGW32__) || defined(_MSC_VER)

#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <conio.h>

#define getError() WSAGetLastError()

static void socketsStartup()
{
    WORD         wVersionRequested;
    WSADATA      wsaData;
    
    wVersionRequested = MAKEWORD(1, 1); 
    if (WSAStartup(wVersionRequested, &wsaData)) {
        printf("\r\nUnable to initialize WinSock for host info");
        exit(EXIT_FAILURE);
    }
}

#define socketsShutdown() WSACleanup()

static __inline int inet_pton(int af, const char * src, void * dst)
{
    int status = 0;
    unsigned long addr;

    if (af == AF_INET) {
	addr = inet_addr(src);
	if (addr != INADDR_NONE) {
	    *(unsigned long *)dst = addr;
	    status = 1;
	}
    }
    
    return status;
}

#else

#error "Unrecognized compiler"

#endif

#endif /* _SOCKETS_H_ */
