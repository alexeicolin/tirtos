/*****************************************************************************
*
*  socket.c  - CC3000 Host Driver Implementation.
*  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/

//*****************************************************************************
//
//! \addtogroup socket_api
//! @{
//
//*****************************************************************************
#include <cc3000_host_driver/include/socket.h>
#include <cc3000_host_driver/include/wlan.h>

#include <cc3000_host_driver/core_driver/inc/socket.h>

#include <osal/inc/osal.h>
#include <string.h>

extern wlan_socket_t           g_sockets[MAX_NUM_OF_SOCKETS];

/* Handles for making the APIs asychronous and thread-safe */
extern SemaphoreHandle         g_accept_semaphore;
extern SemaphoreHandle         g_select_sleep_semaphore;
extern MutexHandle             g_main_mutex;
extern MutexKey                mtx_key;

extern sockaddr                g_accept_sock_addr;

extern int                     g_wlan_stopped;
extern int                     g_accept_new_sd;
extern int                     g_accept_addrlen;
extern int                     g_should_poll_accept;

//Enable this flag if and only if you must comply with BSD socket close() function
#ifdef _API_USE_BSD_CLOSE
    #define close(sd) closesocket(sd)
#endif

//Enable this flag if and only if you must comply with BSD socket read() and write() functions
#ifdef _API_USE_BSD_READ_WRITE
    #define read(sd, buf, len, flags) recv(sd, buf, len, flags)
    #define write(sd, buf, len, flags) send(sd, buf, len, flags)
#endif


static void find_add_next_free_socket(int sd)
{
    int index = 0;
    for (index = 0; index < MAX_NUM_OF_SOCKETS; index++){
        if (g_sockets[index].status == SOC_NOT_INITED){
            g_sockets[index].status = SOCK_ON;
            g_sockets[index].sd = sd;
            return;
        }
    }
}

static void find_clear_socket(int sd)
{
    int index = 0;
    for (index = 0; index < MAX_NUM_OF_SOCKETS; index++){
        if (g_sockets[index].sd == sd){
            g_sockets[index].status = SOC_NOT_INITED;
            g_sockets[index].sd = -1;
        }
    }
}

//*****************************************************************************
//
//! socket
//!
//!  @param  domain    selects the protocol family which will be used for
//!                    communication. On this version only AF_INET is supported
//!  @param  type      specifies the communication semantics. On this version
//!                    only SOCK_STREAM, SOCK_DGRAM, SOCK_RAW are supported
//!  @param  protocol  specifies a particular protocol to be used with the
//!                    socket IPPROTO_TCP, IPPROTO_UDP or IPPROTO_RAW are
//!                    supported.
//!
//!  @return  On success, socket handle that is used for consequent socket
//!           operations. On error, -1 is returned.
//!
//!  @brief  create an endpoint for communication
//!          The socket function creates a socket that is bound to a specific
//!          transport service provider. This function is called by the
//!          application layer to obtain a socket handle.
//
//*****************************************************************************

int
socket(long domain, long type, long protocol)
{
    int ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_socket(domain, type, protocol);
    /*if the value is not error then add to our array for later reference */
    if (-1 != ret)
        find_add_next_free_socket(ret);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//! closesocket
//!
//!  @param  sd    socket handle.
//!
//!  @return  On success, zero is returned. On error, -1 is returned.
//!
//!  @brief  The socket function closes a created socket.
//
//*****************************************************************************

long
closesocket(long sd)
{
    int ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_closesocket(sd);
    /* remove from our array if no error */
    if (0 == ret){
        find_clear_socket(sd);
    }
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//! accept
//!
//!  @param[in]   sd      socket descriptor (handle)
//!  @param[out]  addr    the argument addr is a pointer to a sockaddr structure
//!                       This structure is filled in with the address of the
//!                       peer socket, as known to the communications layer.
//!                       determined. The exact format of the address returned
//!                       addr is by the socket's address sockaddr.
//!                       On this version only AF_INET is supported.
//!                       This argument returns in network order.
//!  @param[out] addrlen  the addrlen argument is a value-result argument:
//!                       it should initially contain the size of the structure
//!                       pointed to by addr.
//!
//!  @return  For socket in blocking mode:
//!                   On success, socket handle. on failure negative
//!               For socket in non-blocking mode:
//!                  - On connection establishment, socket handle
//!                  - On connection pending, SOC_IN_PROGRESS (-2)
//!                - On failure, SOC_ERROR  (-1)
//!
//!  @brief  accept a connection on a socket:
//!          This function is used with connection-based socket types
//!          (SOCK_STREAM). It extracts the first connection request on the
//!          queue of pending connections, creates a new connected socket, and
//!          returns a new file descriptor referring to that socket.
//!          The newly created socket is not in the listening state.
//!          The original socket sd is unaffected by this call.
//!          The argument sd is a socket that has been created with socket(),
//!          bound to a local address with bind(), and is  listening for
//!          connections after a listen(). The argument addr is a pointer
//!          to a sockaddr structure. This structure is filled in with the
//!          address of the peer socket, as known to the communications layer.
//!          The exact format of the address returned addr is determined by the
//!          socket's address family. The addrlen argument is a value-result
//!          argument: it should initially contain the size of the structure
//!          pointed to by addr, on return it will contain the actual
//!          length (in bytes) of the address returned.
//!
//! @sa     socket ; bind ; listen
//
//*****************************************************************************

long
accept(long sd, sockaddr *addr, socklen_t *addrlen)
{
    char val;
    int index = 0;
	int retSock;

    /* set socket options to non blocking for accept polling purposes */
    val = SOCK_ON;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    c_setsockopt( sd, SOL_SOCKET, SOCKOPT_NONBLOCK, &val, sizeof(val));
    OS_mutex_unlock(g_main_mutex, mtx_key);

    OS_semaphore_post(g_select_sleep_semaphore); /* wakeup select thread if needed */

    for (index = 0; index < MAX_NUM_OF_SOCKETS; index++){
        if (g_sockets[index].sd == sd) {
            g_sockets[index].status = SOC_ACCEPTING;
            OS_semaphore_pend(g_sockets[index].sd_semaphore, e_WAIT_FOREVER);
            break;
        }
    }

    /* suspend select thread until waiting for more data */
    OS_semaphore_pend(g_select_sleep_semaphore, e_WAIT_FOREVER);

    if (g_wlan_stopped){ /* if wlan_stop then return */
        return -1;
    }

    g_sockets[sd].status = SOCK_ON;

    memcpy(addr, &g_accept_sock_addr, g_accept_addrlen);
    memcpy(addrlen, &g_accept_addrlen, sizeof(socklen_t));
    retSock = g_accept_new_sd; //save new socket locally

    OS_semaphore_post(g_accept_semaphore);

    /* New socket created and accept success or SOC_ERROR
       If sock error do not take an empty place in the sockets array */
    if (retSock != SOC_ERROR) {
        find_add_next_free_socket(retSock);
    }

    return retSock;
}

//*****************************************************************************
//
//! bind
//!
//!  @param[in]   sd      socket descriptor (handle)
//!  @param[out]  addr    specifies the destination address. On this version
//!                       only AF_INET is supported.
//!  @param[out] addrlen  contains the size of the structure pointed to by addr.
//!
//!  @return    On success, zero is returned. On error, -1 is returned.
//!
//!  @brief  assign a name to a socket
//!          This function gives the socket the local address addr.
//!          addr is addrlen bytes long. Traditionally, this is called when a
//!          socket is created with socket, it exists in a name space (address
//!          family) but has no name assigned.
//!          It is necessary to assign a local address before a SOCK_STREAM
//!          socket may receive connections.
//!
//! @sa     socket ; accept ; listen
//
//*****************************************************************************

long
bind(long sd, const sockaddr *addr, long addrlen)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_bind(sd, addr, addrlen);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//! listen
//!
//!  @param[in]   sd      socket descriptor (handle)
//!  @param[in]  backlog  specifies the listen queue depth. On this version
//!                       backlog is not supported.
//!  @return    On success, zero is returned. On error, -1 is returned.
//!
//!  @brief  listen for connections on a socket
//!          The willingness to accept incoming connections and a queue
//!          limit for incoming connections are specified with listen(),
//!          and then the connections are accepted with accept.
//!          The listen() call applies only to sockets of type SOCK_STREAM
//!          The backlog parameter defines the maximum length the queue of
//!          pending connections may grow to.
//!
//! @sa     socket ; accept ; bind
//!
//! @note   On this version, backlog is not supported
//
//*****************************************************************************

long
listen(long sd, long backlog)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_listen(sd, backlog);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//! gethostbyname
//!
//!  @param[in]   hostname     host name
//!  @param[in]   usNameLen    name length
//!  @param[out]  out_ip_addr  This parameter is filled in with host IP address.
//!                            In case that host name is not resolved,
//!                            out_ip_addr is zero.
//!  @return    On success, positive is returned. On error, negative is returned
//!
//!  @brief  Get host IP by name. Obtain the IP Address of machine on network,
//!          by its name.
//!
//!  @note  On this version, only blocking mode is supported. Also note that
//!          the function requires DNS server to be configured prior to its usage.
//
//*****************************************************************************

#ifndef CC3000_TINY_DRIVER
int
gethostbyname(char * hostname, unsigned short usNameLen, unsigned long* out_ip_addr)
{
    int ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_gethostbyname(hostname, usNameLen, out_ip_addr);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}
#endif

//*****************************************************************************
//
//! connect
//!
//!  @param[in]   sd       socket descriptor (handle)
//!  @param[in]   addr     specifies the destination addr. On this version
//!                        only AF_INET is supported.
//!  @param[out]  addrlen  contains the size of the structure pointed to by addr
//!  @return    On success, zero is returned. On error, -1 is returned
//!
//!  @brief  initiate a connection on a socket
//!          Function connects the socket referred to by the socket descriptor
//!          sd, to the address specified by addr. The addrlen argument
//!          specifies the size of addr. The format of the address in addr is
//!          determined by the address space of the socket. If it is of type
//!          SOCK_DGRAM, this call specifies the peer with which the socket is
//!          to be associated; this address is that to which datagrams are to be
//!          sent, and the only address from which datagrams are to be received.
//!          If the socket is of type SOCK_STREAM, this call attempts to make a
//!          connection to another socket. The other socket is specified  by
//!          address, which is an address in the communications space of the
//!          socket. Note that the function implements only blocking behavior
//!          thus the caller will be waiting either for the connection
//!          establishment or for the connection establishment failure.
//!
//!  @sa socket
//
//*****************************************************************************

long
connect(long sd, const sockaddr *addr, long addrlen)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_connect(sd, addr, addrlen);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}


//*****************************************************************************
//
//! select
//!
//!  @param[in]   nfds       the highest-numbered file descriptor in any of the
//!                           three sets, plus 1.
//!  @param[out]   writesds   socket descriptors list for write monitoring
//!  @param[out]   readsds    socket descriptors list for read monitoring
//!  @param[out]   exceptsds  socket descriptors list for exception monitoring
//!  @param[in]   timeout     is an upper bound on the amount of time elapsed
//!                           before select() returns. Null means infinity
//!                           timeout. The minimum timeout is 5 milliseconds,
//!                          less than 5 milliseconds will be set
//!                           automatically to 5 milliseconds.
//!  @return    On success, select() returns the number of file descriptors
//!             contained in the three returned descriptor sets (that is, the
//!             total number of bits that are set in readfds, writefds,
//!             exceptfds) which may be zero if the timeout expires before
//!             anything interesting  happens.
//!             On error, -1 is returned.
//!                   *readsds - return the sockets on which Read request will
//!                              return without delay with valid data.
//!                   *writesds - return the sockets on which Write request
//!                                 will return without delay.
//!                   *exceptsds - return the sockets which closed recently.
//!
//!  @brief  Monitor socket activity
//!          Select allow a program to monitor multiple file descriptors,
//!          waiting until one or more of the file descriptors become
//!         "ready" for some class of I/O operation
//!
//!  @Note   If the timeout value set to less than 5ms it will automatically set
//!          to 5ms to prevent overload of the system
//!
//!  @sa socket
//
//*****************************************************************************

int
select(long nfds, fd_set *readsds, fd_set *writesds, fd_set *exceptsds, struct timeval *timeout)
{
    int ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_select(nfds, readsds, writesds, exceptsds, timeout);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//! setsockopt
//!
//!  @param[in]   sd          socket handle
//!  @param[in]   level       defines the protocol level for this option
//!  @param[in]   optname     defines the option name to Interrogate
//!  @param[in]   optval      specifies a value for the option
//!  @param[in]   optlen      specifies the length of the option value
//!  @return    On success, zero is returned. On error, -1 is returned
//!
//!  @brief  set socket options
//!          This function manipulate the options associated with a socket.
//!          Options may exist at multiple protocol levels; they are always
//!          present at the uppermost socket level.
//!          When manipulating socket options the level at which the option
//!          resides and the name of the option must be specified.
//!          To manipulate options at the socket level, level is specified as
//!          SOL_SOCKET. To manipulate options at any other level the protocol
//!          number of the appropriate protocol controlling the option is
//!          supplied. For example, to indicate that an option is to be
//!          interpreted by the TCP protocol, level should be set to the
//!          protocol number of TCP;
//!          The parameters optval and optlen are used to access optval -
//!          use for setsockopt(). For getsockopt() they identify a buffer
//!          in which the value for the requested option(s) are to
//!          be returned. For getsockopt(), optlen is a value-result
//!          parameter, initially containing the size of the buffer
//!          pointed to by option_value, and modified on return to
//!          indicate the actual size of the value returned. If no option
//!          value is to be supplied or returned, option_value may be NULL.
//!
//!  @Note   On this version the following two socket options are enabled:
//!              The only protocol level supported in this version
//!          is SOL_SOCKET (level).
//!            1. SOCKOPT_RECV_TIMEOUT (optname)
//!               SOCKOPT_RECV_TIMEOUT configures recv and recvfrom timeout
//!           in milliseconds.
//!             In that case optval should be pointer to unsigned long.
//!            2. SOCKOPT_NONBLOCK (optname). sets the socket non-blocking mode on
//!           or off.
//!             In that case optval should be SOCK_ON or SOCK_OFF (optval).
//!
//!  @sa getsockopt
//
//*****************************************************************************

#ifndef CC3000_TINY_DRIVER
int
setsockopt(long sd, long level, long optname, const void *optval, socklen_t optlen)
{
    int ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_setsockopt(sd, level, optname, optval, optlen);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}
#endif

//*****************************************************************************
//
//! getsockopt
//!
//!  @param[in]   sd          socket handle
//!  @param[in]   level       defines the protocol level for this option
//!  @param[in]   optname     defines the option name to Interrogate
//!  @param[out]   optval      specifies a value for the option
//!  @param[out]   optlen      specifies the length of the option value
//!  @return    On success, zero is returned. On error, -1 is returned
//!
//!  @brief  set socket options
//!          This function manipulate the options associated with a socket.
//!          Options may exist at multiple protocol levels; they are always
//!          present at the uppermost socket level.
//!          When manipulating socket options the level at which the option
//!          resides and the name of the option must be specified.
//!          To manipulate options at the socket level, level is specified as
//!          SOL_SOCKET. To manipulate options at any other level the protocol
//!          number of the appropriate protocol controlling the option is
//!          supplied. For example, to indicate that an option is to be
//!          interpreted by the TCP protocol, level should be set to the
//!          protocol number of TCP;
//!          The parameters optval and optlen are used to access optval -
//!          use for setsockopt(). For getsockopt() they identify a buffer
//!          in which the value for the requested option(s) are to
//!          be returned. For getsockopt(), optlen is a value-result
//!          parameter, initially containing the size of the buffer
//!          pointed to by option_value, and modified on return to
//!          indicate the actual size of the value returned. If no option
//!          value is to be supplied or returned, option_value may be NULL.
//!
//!  @Note   On this version the following two socket options are enabled:
//!              The only protocol level supported in this version
//!          is SOL_SOCKET (level).
//!            1. SOCKOPT_RECV_TIMEOUT (optname)
//!               SOCKOPT_RECV_TIMEOUT configures recv and recvfrom timeout
//!           in milliseconds.
//!             In that case optval should be pointer to unsigned long.
//!            2. SOCKOPT_NONBLOCK (optname). sets the socket non-blocking mode on
//!           or off.
//!             In that case optval should be SOCK_ON or SOCK_OFF (optval).
//!
//!  @sa setsockopt
//
//*****************************************************************************

int
getsockopt(long sd, long level, long optname, void *optval, socklen_t *optlen)
{
    int ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_getsockopt(sd, level, optname, optval, optlen);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//!  recv
//!
//!  @param[in]  sd     socket handle
//!  @param[out] buf    Points to the buffer where the message should be stored
//!  @param[in]  len    Specifies the length in bytes of the buffer pointed to
//!                     by the buffer argument.
//!  @param[in] flags   Specifies the type of message reception.
//!                     On this version, this parameter is not supported.
//!
//!  @return         Return the number of bytes received, or -1 if an error
//!                  occurred
//!
//!  @brief          function receives a message from a connection-mode socket
//!
//!  @sa recvfrom
//!
//!  @Note On this version, only blocking mode is supported.
//
//*****************************************************************************

int
recv(long sd, void *buf, long len, long flags)
{
    int ret;
    int index = 0;

    OS_semaphore_post(g_select_sleep_semaphore); //wakeup select thread if needed

    for (index = 0; index < MAX_NUM_OF_SOCKETS; index++){
        if (g_sockets[index].sd == sd){
            /* wait for data to become available */
            OS_semaphore_pend(g_sockets[index].sd_semaphore, e_WAIT_FOREVER);
        }
    }

    /* suspend select thread until waiting for more data */
    OS_semaphore_pend(g_select_sleep_semaphore, e_WAIT_FOREVER);

    if (g_wlan_stopped){ //if wlan_stop then return
        return -1;
    }

    /* call the original recv knowing there is available data
       and it's a non-blocking call */
    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_recv(sd, buf, len, flags);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//!  recvfrom
//!
//!  @param[in]  sd     socket handle
//!  @param[out] buf    Points to the buffer where the message should be stored
//!  @param[in]  len    Specifies the length in bytes of the buffer pointed to
//!                     by the buffer argument.
//!  @param[in] flags   Specifies the type of message reception.
//!                     On this version, this parameter is not supported.
//!  @param[in] from   pointer to an address structure indicating the source
//!                    address: sockaddr. On this version only AF_INET is
//!                    supported.
//!  @param[in] fromlen   source address tructure size
//!
//!  @return         Return the number of bytes received, or -1 if an error
//!                  occurred
//!
//!  @brief         read data from socket
//!                 function receives a message from a connection-mode or
//!                 connectionless-mode socket. Note that raw sockets are not
//!                 supported.
//!
//!  @sa recv
//!
//!  @Note On this version, only blocking mode is supported.
//
//*****************************************************************************
int
recvfrom(long sd, void *buf, long len, long flags,
         sockaddr *from, socklen_t *fromlen)
{
    int index = 0;
    int ret;

    OS_semaphore_post(g_select_sleep_semaphore); //wakeup select thread if needed
    for (index = 0; index < MAX_NUM_OF_SOCKETS; index++){
        if (g_sockets[index].sd == sd){
            /* wait for data to become available */
            OS_semaphore_pend(g_sockets[index].sd_semaphore, e_WAIT_FOREVER);
        }
    }
    OS_semaphore_pend(g_select_sleep_semaphore, e_WAIT_FOREVER); //suspend select thread until waiting for more data

    if (g_wlan_stopped){ //if wlan_stop then return
        return -1;
    }

    /* Call the original recv knowing there is available data
       and it's a non-blocking call */
    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_recvfrom(sd, buf, len, flags, from, fromlen);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//!  send
//!
//!  @param sd       socket handle
//!  @param buf      Points to a buffer containing the message to be sent
//!  @param len      message size in bytes
//!  @param flags    On this version, this parameter is not supported
//!
//!  @return         Return the number of bytes transmitted, or -1 if an
//!                  error occurred
//!
//!  @brief          Write data to TCP socket
//!                  This function is used to transmit a message to another
//!                  socket.
//!
//!  @Note           On this version, only blocking mode is supported.
//!
//!  @sa             sendto
//
//*****************************************************************************

int
send(long sd, const void *buf, long len, long flags)
{
    int ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_send(sd, buf, len, flags);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//!  sendto
//!
//!  @param sd       socket handle
//!  @param buf      Points to a buffer containing the message to be sent
//!  @param len      message size in bytes
//!  @param flags    On this version, this parameter is not supported
//!  @param to       pointer to an address structure indicating the destination
//!                  address: sockaddr. On this version only AF_INET is
//!                  supported.
//!  @param tolen    destination address structure size
//!
//!  @return         Return the number of bytes transmitted, or -1 if an
//!                  error occurred
//!
//!  @brief          Write data to TCP socket
//!                  This function is used to transmit a message to another
//!                  socket.
//!
//!  @Note           On this version, only blocking mode is supported.
//!
//!  @sa             send
//
//*****************************************************************************

int
sendto(long sd, const void *buf, long len, long flags,
       const sockaddr *to, socklen_t tolen)
{
    int ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_sendto(sd, buf, len, flags, to, tolen);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//!  mdnsAdvertiser
//!
//!  @param[in] mdnsEnabled         flag to enable/disable the mDNS feature
//!  @param[in] deviceServiceName   Service name as part of the published
//!                                 canonical domain name
//!  @param[in] deviceServiceNameLength   Length of the service name
//!
//!
//!  @return   On success, zero is returned, return SOC_ERROR if socket was not
//!            opened successfully, or if an error occurred.
//!
//!  @brief    Set CC3000 in mDNS advertiser mode in order to advertise itself.
//
//*****************************************************************************

int
mdnsAdvertiser(unsigned short mdnsEnabled, char * deviceServiceName, unsigned short deviceServiceNameLength)
{
    int ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_mdnsAdvertiser(mdnsEnabled, deviceServiceName, deviceServiceNameLength);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}
