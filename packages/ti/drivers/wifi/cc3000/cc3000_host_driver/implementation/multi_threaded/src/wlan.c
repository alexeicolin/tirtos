/*****************************************************************************
*
*  wlan.c  - CC3000 Host Driver Implementation.
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
//! \addtogroup wlan_api
//! @{
//
//*****************************************************************************
#include <string.h>
#include <xdc/runtime/System.h>

#include <cc3000_host_driver/include/wlan.h>
#include <cc3000_host_driver/include/socket.h>

#include <cc3000_host_driver/core_driver/inc/wlan.h>
#include <cc3000_host_driver/core_driver/inc/socket.h>

#include <osal/inc/osal.h>

extern const Int        SELECT_THREAD_PRI;

wlan_socket_t           g_sockets[MAX_NUM_OF_SOCKETS];

/* Handles for making the APIs asychronous and thread-safe */
SemaphoreHandle         g_accept_semaphore;
SemaphoreHandle         g_select_sleep_semaphore;
MutexHandle             g_main_mutex;
TaskHandle              g_select_thread;
MutexKey                mtx_key;

sockaddr                g_accept_sock_addr;

int                     g_wlan_stopped;
int                     g_accept_new_sd;
int                     g_accept_socket;
int                     g_accept_addrlen;
int                     g_should_poll_accept;

//char                    selectThreadStack[512];

static void SelectThread(void *);

//*****************************************************************************
//
//!  wlan_init
//!
//!  @param  sWlanCB   Asynchronous events callback.
//!                    0 no event call back.
//!                  -call back parameters:
//!                   1) event_type: HCI_EVNT_WLAN_UNSOL_CONNECT connect event,
//!                     HCI_EVNT_WLAN_UNSOL_DISCONNECT disconnect event,
//!                     HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE config done,
//!                     HCI_EVNT_WLAN_UNSOL_DHCP dhcp report,
//!                     HCI_EVNT_WLAN_ASYNC_PING_REPORT ping report OR
//!                     HCI_EVNT_WLAN_KEEPALIVE keepalive.
//!                   2) data: pointer to extra data that received by the event
//!                     (NULL no data).
//!                   3) length: data length.
//!                  -Events with extra data:
//!                     HCI_EVNT_WLAN_UNSOL_DHCP: 4 bytes IP, 4 bytes Mask,
//!                     4 bytes default gateway, 4 bytes DHCP server and 4 bytes
//!                     for DNS server.
//!                     HCI_EVNT_WLAN_ASYNC_PING_REPORT: 4 bytes Packets sent,
//!                     4 bytes Packets received, 4 bytes Min round time,
//!                     4 bytes Max round time and 4 bytes for Avg round time.
//!
//!  @param    sFWPatches  0 no patch or pointer to FW patches
//!  @param    sDriverPatches  0 no patch or pointer to driver patches
//!  @param    sBootLoaderPatches  0 no patch or pointer to bootloader patches
//!  @param    sReadWlanInterruptPin    init callback. the callback read wlan
//!            interrupt status.
//!  @param    sWlanInterruptEnable   init callback. the callback enable wlan
//!            interrupt.
//!  @param    sWlanInterruptDisable   init callback. the callback disable wlan
//!            interrupt.
//!  @param    sWriteWlanPin      init callback. the callback write value
//!            to device pin.
//!
//!  @return   none
//!
//!  @sa       wlan_set_event_mask , wlan_start , wlan_stop
//!
//!  @brief    Initialize wlan driver
//!
//!  @warning This function must be called before ANY other wlan driver function
//
//*****************************************************************************

void wlan_init(tWlanCB     sWlanCB,
               tFWPatches sFWPatches,
               tDriverPatches sDriverPatches,
               tBootLoaderPatches sBootLoaderPatches,
               tWlanReadInteruptPin  sReadWlanInterruptPin,
               tWlanInterruptEnable  sWlanInterruptEnable,
               tWlanInterruptDisable sWlanInterruptDisable,
               tWriteWlanPin         sWriteWlanPin)
{
    OS_mutex_create(&g_main_mutex, "MainMutex");

    OS_mutex_lock(g_main_mutex, &mtx_key);
    g_select_thread = NULL;
    c_wlan_init(sWlanCB, sFWPatches, sDriverPatches,\
                sBootLoaderPatches, sReadWlanInterruptPin,\
                sWlanInterruptEnable, sWlanInterruptDisable, sWriteWlanPin);
    OS_mutex_unlock(g_main_mutex, mtx_key);
}

//*****************************************************************************
//
//!  wlan_start
//!
//!  @param   usPatchesAvailableAtHost -  flag to indicate if patches available
//!                                    from host or from EEPROM. Due to the
//!                                    fact the patches are burn to the EEPROM
//!                                    using the patch programmer utility, the
//!                                    patches will be available from the EEPROM
//!                                    and not from the host.
//!
//!  @return        none
//!
//!  @brief        Start WLAN device. This function asserts the enable pin of
//!                the device (WLAN_EN), starting the HW initialization process.
//!                The function blocked until device Initialization is completed.
//!                Function also configure patches (FW, driver or bootloader)
//!                and calls appropriate device callbacks.
//!
//!  @Note          Prior calling the function wlan_init shall be called.
//!  @Warning       This function must be called after wlan_init and before any
//!                 other wlan API
//!  @sa            wlan_init , wlan_stop
//!
//
//*****************************************************************************

void wlan_start(unsigned short usPatchesAvailableAtHost)
{
    OS_mutex_lock(g_main_mutex, &mtx_key);

    s_thread_params params = {0};
    int index = 0;

    for(index = 0; index < MAX_NUM_OF_SOCKETS; index++){
        g_sockets[index].sd = -1;
        g_sockets[index].status = SOC_NOT_INITED;
        if (NULL == g_sockets[index].sd_semaphore)
            OS_semaphore_create(&g_sockets[index].sd_semaphore, "SockSem", e_MODE_BINARY, 0);
    }

    if (NULL == g_accept_semaphore)
        OS_semaphore_create(&g_accept_semaphore, "AcceptSem", e_MODE_BINARY, 0);

    if (NULL == g_select_sleep_semaphore)
        OS_semaphore_create(&g_select_sleep_semaphore, "SelectSleepSem", e_MODE_COUNTING, 0);

    /**/
    c_wlan_start(usPatchesAvailableAtHost);

    g_wlan_stopped = 0;
    g_should_poll_accept = 0;
    g_accept_socket = -1;

    if (NULL == g_select_thread){ /* Thread not created yet */
        /* Fill Thread Params */
        params.thread_name = "SelectThread";
        params.p_stack_start = NULL; /* TBD - Mandatory in Thread-X */
        params.p_entry_function = SelectThread;
        *(sInt32 *)params.p_func_params = 0;
        params.priority = SELECT_THREAD_PRI;

#ifdef MSP430WARE
        params.stack_size = 0x160;
#else
        params.stack_size = 0x300;
#endif

        if (e_SUCCESS != OS_thread_create(&g_select_thread, &params))
        {
            OS_mutex_unlock(g_main_mutex, mtx_key); /* Error */
        }
    }

    OS_mutex_unlock(g_main_mutex, mtx_key);
}


//*****************************************************************************
//
//!  wlan_stop
//!
//!  @param         none
//!
//!  @return        none
//!
//!  @brief         Stop WLAN device by putting it into reset state.
//!
//!  @sa            wlan_start
//
//*****************************************************************************

void wlan_stop(void)
{
    int index = 0;

    OS_mutex_lock(g_main_mutex, &mtx_key);

    c_wlan_stop();

    g_wlan_stopped = 1;
    OS_thread_terminate(g_select_thread);
    OS_thread_delete(&g_select_thread);

    for (index = 0; index < MAX_NUM_OF_SOCKETS; index++){
        g_sockets[index].sd = 1;
        g_sockets[index].status = SOC_NOT_INITED;
        OS_semaphore_delete(&g_sockets[index].sd_semaphore);
    }

    OS_semaphore_delete(&g_accept_semaphore);
    OS_semaphore_delete(&g_select_sleep_semaphore);
    OS_mutex_unlock(g_main_mutex, mtx_key);
}


//*****************************************************************************
//
//!  wlan_connect
//!
//!  @param    sec_type   security options:
//!               WLAN_SEC_UNSEC,
//!               WLAN_SEC_WEP (ASCII support only),
//!               WLAN_SEC_WPA or WLAN_SEC_WPA2
//!  @param    ssid       up to 32 bytes and is ASCII SSID of the AP
//!  @param    ssid_len   length of the SSID
//!  @param    bssid      6 bytes specified the AP bssid
//!  @param    key        up to 16 bytes specified the AP security key
//!  @param    key_len    key length
//!
//!  @return     On success, zero is returned. On error, negative is returned.
//!              Note that even though a zero is returned on success to trigger
//!              connection operation, it does not mean that CCC3000 is already
//!              connected. An asynchronous "Connected" event is generated when
//!              actual association process finishes and CC3000 is connected to
//!              the AP. If DHCP is set, An asynchronous "DHCP" event is
//!              generated when DHCP process is finish.
//!
//!
//!  @brief      Connect to AP
//!  @warning    Please Note that when connection to AP configured with security
//!              type WEP, please confirm that the key is set as ASCII and not
//!              as HEX.
//!  @sa         wlan_disconnect
//
//*****************************************************************************

#ifndef CC3000_TINY_DRIVER
long wlan_connect(unsigned long ulSecType, char *ssid, long ssid_len,
             unsigned char *bssid, unsigned char *key, long key_len)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_wlan_connect(ulSecType, ssid, ssid_len, bssid, key, key_len);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}
#else
long wlan_connect(char *ssid, long ssid_len)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_wlan_connect(ssid, ssid_len);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}
#endif

//*****************************************************************************
//
//!  wlan_disconnect
//!
//!  @return    0 disconnected done, other CC3000 already disconnected
//!
//!  @brief      Disconnect connection from AP.
//!
//!  @sa         wlan_connect
//
//*****************************************************************************

long wlan_disconnect()
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_wlan_disconnect();
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//!  wlan_ioctl_set_connection_policy
//!
//!  @param    should_connect_to_open_ap  enable(1), disable(0) connect to any
//!            available AP. This parameter corresponds to the configuration of
//!            item # 3 in the brief description.
//!  @param    should_use_fast_connect enable(1), disable(0). if enabled, tries
//!            to connect to the last connected AP. This parameter corresponds
//!            to the configuration of item # 1 in the brief description.
//!  @param    auto_start enable(1), disable(0) auto connect
//!            after reset and periodically reconnect if needed. This
//!            configuration configures option 2 in the above description.
//!
//!  @return     On success, zero is returned. On error, -1 is returned
//!
//!  @brief      When auto is enabled, the device tries to connect according
//!              the following policy:
//!              1) If fast connect is enabled and last connection is valid,
//!                 the device will try to connect to it without the scanning
//!                 procedure (fast). The last connection will be marked as
//!                 invalid, due to adding/removing profile.
//!              2) If profile exists, the device will try to connect it
//!                 (Up to seven profiles).
//!              3) If fast and profiles are not found, and open mode is
//!                 enabled, the device will try to connect to any AP.
//!              * Note that the policy settings are stored in the CC3000 NVMEM.
//!
//!  @sa         wlan_add_profile , wlan_ioctl_del_profile
//
//*****************************************************************************

long
wlan_ioctl_set_connection_policy(unsigned long should_connect_to_open_ap,
                                 unsigned long ulShouldUseFastConnect,
                                 unsigned long ulUseProfiles)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_wlan_ioctl_set_connection_policy(should_connect_to_open_ap,\
                                             ulShouldUseFastConnect,\
                                             ulUseProfiles);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//!  wlan_add_profile
//!
//!  @param    ulSecType  WLAN_SEC_UNSEC,WLAN_SEC_WEP,WLAN_SEC_WPA,WLAN_SEC_WPA2
//!  @param    ucSsid    ssid  SSID up to 32 bytes
//!  @param    ulSsidLen ssid length
//!  @param    ucBssid   bssid  6 bytes
//!  @param    ulPriority ulPriority profile priority. Lowest priority:0.
//!  @param    ulPairwiseCipher_Or_TxKeyLen  key length for WEP security
//!  @param    ulGroupCipher_TxKeyIndex  key index
//!  @param    ulKeyMgmt        KEY management
//!  @param    ucPf_OrKey       security key
//!  @param    ulPassPhraseLen  security key length for WPA\WPA2
//!
//!  @return    On success, zero is returned. On error, -1 is returned
//!
//!  @brief     When auto start is enabled, the device connects to
//!             station from the profiles table. Up to 7 profiles are supported.
//!             If several profiles configured the device choose the highest
//!             priority profile, within each priority group, device will choose
//!             profile based on security policy, signal strength, etc
//!             parameters. All the profiles are stored in CC3000 NVMEM.
//!
//!  @sa        wlan_ioctl_del_profile
//
//*****************************************************************************
long wlan_add_profile(unsigned long ulSecType,
                      unsigned char* ucSsid,
                      unsigned long ulSsidLen,
                      unsigned char *ucBssid,
                      unsigned long ulPriority,
                      unsigned long ulPairwiseCipher_Or_TxKeyLen,
                      unsigned long ulGroupCipher_TxKeyIndex,
                      unsigned long ulKeyMgmt,
                      unsigned char* ucPf_OrKey,
                      unsigned long ulPassPhraseLen)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_wlan_add_profile(ulSecType, ucSsid, ulSsidLen, ucBssid, ulPriority,
                             ulPairwiseCipher_Or_TxKeyLen, ulGroupCipher_TxKeyIndex,
                             ulKeyMgmt, ucPf_OrKey, ulPassPhraseLen);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}


//*****************************************************************************
//
//!  wlan_ioctl_del_profile
//!
//!  @param    index   number of profile to delete
//!
//!  @return    On success, zero is returned. On error, -1 is returned
//!
//!  @brief     Delete WLAN profile
//!
//!  @Note      In order to delete all stored profile, set index to 255.
//!
//!  @sa        wlan_add_profile
//
//*****************************************************************************
long wlan_ioctl_del_profile(unsigned long ulIndex)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_wlan_ioctl_del_profile(ulIndex);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//!  wlan_ioctl_get_scan_results
//!
//!  @param[in]    scan_timeout   parameter not supported
//!  @param[out]   ucResults  scan results (_wlan_full_scan_results_args_t)
//!
//!  @return    On success, zero is returned. On error, -1 is returned
//!
//!  @brief    Gets entry from scan result table.
//!            The scan results are returned one by one, and each entry
//!            represents a single AP found in the area. The following is a
//!            format of the scan result:
//!          - 4 Bytes: number of networks found
//!          - 4 Bytes: The status of the scan: 0 - aged results,
//!                     1 - results valid, 2 - no results
//!          - 42 bytes: Result entry, where the bytes are arranged as  follows:
//!
//!                         - 1 bit isValid - is result valid or not
//!                         - 7 bits rssi - RSSI value;
//!                 - 2 bits: securityMode - security mode of the AP:
//!                           0 - Open, 1 - WEP, 2 WPA, 3 WPA2
//!                         - 6 bits: SSID name length
//!                         - 2 bytes: the time at which the entry has entered into
//!                            scans result table
//!                         - 32 bytes: SSID name
//!                 - 6 bytes:  BSSID
//!
//!  @Note      scan_timeout, is not supported on this version.
//!
//!  @sa        wlan_ioctl_set_scan_params
//
//*****************************************************************************
#ifndef CC3000_TINY_DRIVER
long wlan_ioctl_get_scan_results(unsigned long ulScanTimeout,
                                 unsigned char *ucResults)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_wlan_ioctl_get_scan_results(ulScanTimeout, ucResults);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}
#endif

//*****************************************************************************
//
//!  wlan_ioctl_set_scan_params
//!
//!  @param    uiEnable - start/stop application scan:
//!            1 = start scan with default interval value of 10 min.
//!            in order to set a different scan interval value apply the value
//!            in milliseconds. minimum 1 second. 0=stop). Wlan reset
//!           (wlan_stop() wlan_start()) is needed when changing scan interval
//!            value. Saved: No
//!  @param   uiMinDwellTime   minimum dwell time value to be used for each
//!           channel, in milliseconds. Saved: yes
//!           Recommended Value: 100 (Default: 20)
//!  @param   uiMaxDwellTime    maximum dwell time value to be used for each
//!           channel, in milliseconds. Saved: yes
//!           Recommended Value: 100 (Default: 30)
//!  @param   uiNumOfProbeRequests  max probe request between dwell time.
//!           Saved: yes. Recommended Value: 5 (Default:2)
//!  @param   uiChannelMask  bitwise, up to 13 channels (0x1fff).
//!           Saved: yes. Default: 0x7ff
//!  @param   uiRSSIThreshold   RSSI threshold. Saved: yes (Default: -80)
//!  @param   uiSNRThreshold    NSR threshold. Saved: yes (Default: 0)
//!  @param   uiDefaultTxPower  probe Tx power. Saved: yes (Default: 205)
//!  @param   aiIntervalList    pointer to array with 16 entries (16 channels)
//!           each entry (unsigned long) holds timeout between periodic scan
//!           (connection scan) - in millisecond. Saved: yes. Default 2000ms.
//!
//!  @return    On success, zero is returned. On error, -1 is returned
//!
//!  @brief    start and stop scan procedure. Set scan parameters.
//!
//!  @Note     uiDefaultTxPower, is not supported on this version.
//!
//!  @sa        wlan_ioctl_get_scan_results
//
//*****************************************************************************

#ifndef CC3000_TINY_DRIVER
long wlan_ioctl_set_scan_params(unsigned long uiEnable, unsigned long uiMinDwellTime,
                                unsigned long uiMaxDwellTime, unsigned long uiNumOfProbeRequests,
                                unsigned long uiChannelMask, long iRSSIThreshold,
                                unsigned long uiSNRThreshold, unsigned long uiDefaultTxPower,
                                unsigned long *aiIntervalList)
{
    unsigned long  uiRes;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    uiRes = c_wlan_ioctl_set_scan_params(uiEnable, uiMinDwellTime, uiMaxDwellTime,\
                                         uiNumOfProbeRequests, uiChannelMask, iRSSIThreshold,\
                                         uiSNRThreshold, uiDefaultTxPower, aiIntervalList);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return (uiRes);
}
#endif

//*****************************************************************************
//
//!  wlan_set_event_mask
//!
//!  @param    mask   mask option:
//!       HCI_EVNT_WLAN_UNSOL_CONNECT connect event
//!       HCI_EVNT_WLAN_UNSOL_DISCONNECT disconnect event
//!       HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE  smart config done
//!       HCI_EVNT_WLAN_UNSOL_INIT init done
//!       HCI_EVNT_WLAN_UNSOL_DHCP dhcp event report
//!       HCI_EVNT_WLAN_ASYNC_PING_REPORT ping report
//!       HCI_EVNT_WLAN_KEEPALIVE keepalive
//!       HCI_EVNT_WLAN_TX_COMPLETE - disable information on end of transmission
//!       Saved: no.
//!
//!  @return    On success, zero is returned. On error, -1 is returned
//!
//!  @brief    Mask event according to bit mask. In case that event is
//!            masked (1), the device will not send the masked event to host.
//
//*****************************************************************************

long wlan_set_event_mask(unsigned long ulMask)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_wlan_set_event_mask(ulMask);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//!  wlan_ioctl_statusget
//!
//!  @param none
//!
//!  @return    WLAN_STATUS_DISCONNECTED, WLAN_STATUS_SCANING,
//!             STATUS_CONNECTING or WLAN_STATUS_CONNECTED
//!
//!  @brief    get wlan status: disconnected, scanning, connecting or connected
//
//*****************************************************************************

#ifndef CC3000_TINY_DRIVER
long wlan_ioctl_statusget(void)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_wlan_ioctl_statusget();
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}
#endif

//*****************************************************************************
//
//!  wlan_smart_config_start
//!
//!  @param    algoEncryptedFlag indicates whether the information is encrypted
//!
//!  @return   On success, zero is returned. On error, -1 is returned
//!
//!  @brief   Start to acquire device profile. The device acquire its own
//!           profile, if profile message is found. The acquired AP information
//!           is stored in CC3000 EEPROM only in case AES128 encryption is used.
//!           In case AES128 encryption is not used, a profile is created by
//!           CC3000 internally.
//!
//!  @Note    An asynchronous event - Smart Config Done will be generated as soon
//!           as the process finishes successfully.
//!
//!  @sa      wlan_smart_config_set_prefix , wlan_smart_config_stop
//
//*****************************************************************************
long wlan_smart_config_start(unsigned long algoEncryptedFlag)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_wlan_smart_config_start(algoEncryptedFlag);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//!  wlan_smart_config_stop
//!
//!  @param    algoEncryptedFlag indicates whether the information is encrypted
//!
//!  @return   On success, zero is returned. On error, -1 is returned
//!
//!  @brief   Stop the acquire profile procedure
//!
//!  @sa      wlan_smart_config_start , wlan_smart_config_set_prefix
//
//*****************************************************************************
long wlan_smart_config_stop(void)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_wlan_smart_config_stop();
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//!  wlan_smart_config_set_prefix
//!
//!  @param   newPrefix  3 bytes identify the SSID prefix for the Smart Config.
//!
//!  @return   On success, zero is returned. On error, -1 is returned
//!
//!  @brief   Configure station ssid prefix. The prefix is used internally
//!           in CC3000. It should always be TTT.
//!
//!  @Note    The prefix is stored in CC3000 NVMEM
//!
//!  @sa      wlan_smart_config_start , wlan_smart_config_stop
//
//*****************************************************************************
long wlan_smart_config_set_prefix(char* cNewPrefix)
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_wlan_smart_config_set_prefix(cNewPrefix);
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}

//*****************************************************************************
//
//!  wlan_smart_config_process
//!
//!  @param   none
//!
//!  @return   On success, zero is returned. On error, -1 is returned
//!
//!  @brief   process the acquired data and store it as a profile. The acquired
//!           AP information is stored in CC3000 EEPROM encrypted.
//!           The encrypted data is decrypted and stored as a profile.
//!           behavior is as defined by connection policy.
//
//*****************************************************************************

#ifndef CC3000_UNENCRYPTED_SMART_CONFIG
long wlan_smart_config_process()
{
    long ret;

    OS_mutex_lock(g_main_mutex, &mtx_key);
    ret = c_wlan_smart_config_process();
    OS_mutex_unlock(g_main_mutex, mtx_key);

    return(ret);
}
#endif //CC3000_UNENCRYPTED_SMART_CONFIG

static void SelectThread(void *ptr)
{
    struct timeval timeout;
    fd_set readsds;
    fd_set exceptsds;

    int ret = 0;
    int maxFD;
    int index = 0;

    memset(&timeout, 0, sizeof(struct timeval));
    timeout.tv_sec = 0;
    timeout.tv_usec = ((long)100) * ((long)1000); /* 100 msecs */

    while(1) //run until closed by wlan_stop
    {
        /* first check if recv/recvfrom/accept was called */
        OS_semaphore_pend(g_select_sleep_semaphore, e_WAIT_FOREVER);
        /* increase the count back by one to be decreased by the original caller */
        OS_semaphore_post(g_select_sleep_semaphore);

        FD_ZERO(&readsds);
        FD_ZERO(&exceptsds);

        /* ping correct socket descriptor param for select */
        for(index = 0; index < MAX_NUM_OF_SOCKETS; index++){
            if(g_sockets[index].status == SOCK_ON){
                FD_SET(g_sockets[index].sd, &readsds);
                FD_SET(g_sockets[index].sd, &exceptsds);
                if(maxFD <= g_sockets[index].sd)
                    maxFD = g_sockets[index].sd + 1;
            }
        }

        ret = select(maxFD, &readsds, NULL, &exceptsds, &timeout); /* Polling instead of blocking here\
                                                                to process "accept" below */
        //System_printf("SelectThread: Running..!! \n");
        //System_flush();

        if (g_wlan_stopped)
        {
            /* Wlan_stop will terminate the thread and by that all
               sync objects owned by it will be released */
            return;
        }

        if (ret>0){
            for (index = 0; index < MAX_NUM_OF_SOCKETS; index++){
                if (g_sockets[index].status != SOC_NOT_INITED && //check that the socket is valid
                    g_sockets[index].status != SOC_ACCEPTING &&    //verify this is not an accept socket
                    (FD_ISSET(g_sockets[index].sd, &readsds) ||    //and has pending data
                    FD_ISSET(g_sockets[index].sd, &exceptsds))){    //and has pending data
                    OS_semaphore_post(g_sockets[index].sd_semaphore); //release the semaphore
                }
            }
        }

        for (index = 0; index < MAX_NUM_OF_SOCKETS; index++){
            if (g_sockets[index].status == SOC_ACCEPTING) {
                OS_mutex_lock(g_main_mutex, &mtx_key);
                g_accept_new_sd = c_accept(g_sockets[index].sd, &g_accept_sock_addr, (socklen_t*)&g_accept_addrlen);
                OS_mutex_unlock(g_main_mutex, mtx_key);

                if (g_accept_new_sd != SOC_IN_PROGRESS) {
                    OS_semaphore_post(g_sockets[index].sd_semaphore);

                    /* This pend protects g_accept_new_sd */
                    OS_semaphore_pend(g_accept_semaphore, e_WAIT_FOREVER);
                }
            }
        }
    }
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
