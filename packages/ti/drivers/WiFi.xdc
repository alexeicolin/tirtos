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
 *  ======== WiFi.xdc ========
 */
package ti.drivers;
import xdc.rov.ViewInfo;

/*!
 *  ======== WiFi ========
 *  WiFi module configuration
 */
@NoRuntime
@HeaderName("")
@Template("./WiFi.xdt")
module WiFi {

    /*!
     *  ======== LibType ========
     *  WiFi library selection options
     *
     *  This enumeration defines all the WiFi library type
     *  provided by the product.  You can select the library type by setting
     *  the {@link #libType WiFi.libType} configuration parameter.
     *
     *  @field(LibType_Instrumented) The library supplied is prebuilt with
     *  logging and assertions enabled.
     *
     *  @field(LibType_NonInstrumented) The library supplied is prebuilt
     *  with logging and assertions disabled.
     */
    enum LibType {
        LibType_Instrumented,           /*! instrumented */
        LibType_NonInstrumented         /*! non-instrumented */
    };

    /*!
     *  ======== libType ========
     *  WiFi Library type
     *
     *  The WiFi runtime is provided in the form of a library that is
     *  linked with your application.  Several forms of this library are
     *  provided with the WiFi product. This configuration parameter
     *  allows you to select the form of WiFi to use.
     *
     *  The default value of libType is
     *  {@link #LibType_Instrumented WiFi_LibType_Instrumented}.  For a
     *  complete list of options and what they offer see {@link #LibType}.
     */
    metaonly config LibType libType = LibType_Instrumented;

    /*!
     *  ======== HDType ========
     *  WiFi Host Driver selection options
     *
     *  This enumeration defines all the WiFi library type
     *  provided by the product.  You can select the library type by setting
     *  the {@link #libType WiFi.HDType} configuration parameter.
     *
     *  @field(HDType_SingleThread) The library supplied is prebuilt with
     *  a version of the WiFi device's Host Driver that may only be called from
     *  a single task.
     *
     *  @field(HDType_MultiThread) The library supplied is prebuilt with
     *  a version of the WiFi device's Host Driver that is safe to call from
     *  multiple tasks, but consumes more resources than the single-thread
     *  version.
     */
    enum HDType {
        HDType_SingleThread,       /*! single-thread */
        HDType_MultiThread         /*! multi-thread */
    };

    /*!
     *  ======== hdType ========
     *  WiFi Host Driver type
     *
     *  The WiFi runtime is provided in the form of a library that is
     *  linked with your application. Several forms of this library are
     *  provided with the WiFi product. This configuration parameter
     *  allows you to select the form of WiFi to use.
     *
     *  The default value of libType is
     *  {@link #HDType_SingleThread WiFi_HDType_SingleThread}.  For a
     *  complete list of options and what they offer see {@link #HDType}.
     */
    metaonly config HDType hdType = HDType_SingleThread;

    /*!
     *  TX payload size
     *
     *  Used to calculate the size of wlan_tx_buff[]--the buffer used to send
     *  data and commands to the Wi-Fi device. The maximum permitted size is
     *  1460 bytes. Devices with limited memory may not be able to allocate the
     *  maximum payload size; smaller values can be used.
     */
    metaonly config Int txPayloadSize = 1460;

    /*!
     *  RX payload size
     *
     *  Used to calculate the size of wlan_rx_buff[]--the buffer used to
     *  receive data and events from the Wi-Fi device. The maximum permitted
     *  size is 1460 bytes.Devices with limited memory may not be able to
     *  allocate the maximum payload size; smaller values can be used.
     */
    metaonly config Int rxPayloadSize = 1460;

    /*!
     *  SelectThread priority
     *
     *  The SelectThread is a task created by the multi-thread version of the
     *  host driver to prevent long host driver calls such as recv() and
     *  accept() from preventing other tasks from making host driver API calls
     *  while they wait for a response.
     *
     *  You may select the Task priority as you would for any other Task
     *  instance.
     */
    metaonly config Int selectThreadPriority = 1;

    /*!
     *  @_nodoc
     *  ======== WiFiTivaCC3000_State ========
     *  Must match the WiFiTivaCC3000_State enum defined in
     *  WiFiTivaCC3000.h
     */
    enum WiFiTivaCC3000_State {
        STATE_POWERUP,
        STATE_INITIALIZED,
        STATE_IDLE,
        STATE_WRITE_FIRST_PART,
        STATE_WRITE_EOT,
        STATE_READ_IRQ,
        STATE_READ_FIRST_PART,
        STATE_READ_EOT
    };

    /*!
     *  @_nodoc
     *  ======== WiFiMSP430CC3000_State ========
     *  Must match the WiFiMSP430CC3000_State enum defined in
     *  WiFiMSP430CC3000.h
     */
    enum WiFiMSP430CC3000_State {
        WiFiMSP430CC3000_STATE_POWERUP,
        WiFiMSP430CC3000_STATE_INITIALIZED,
        WiFiMSP430CC3000_STATE_IDLE,
        WiFiMSP430CC3000_STATE_WRITE_EOT,
        WiFiMSP430CC3000_STATE_READ_IRQ,
        WiFiMSP430CC3000_STATE_READ_EOT
    };

    /*!
     *  @_nodoc
     *  ======== WiFi_Config ========
     *  Structure must match the WiFi_Config structure defined in WiFi.h
     */
    struct WiFi_Config {
        Void               *fxnTablePtr;
        Void               *object;
        Void               *hwAttrs;
    };

    /*!
     *  @_nodoc
     *  ======== WiFiTivaCC3000_HWAttrs ========
     *  Structure must match the WiFiTivaCC3000_HWAttrs structure defined
     *  in wifi/WiFiTivaCC3000.h
     */
    struct WiFiTivaCC3000_HWAttrs {
        ULong        irqPort;
        Bits32       irqPin;
        Int          irqIntNum;

        ULong        csPort;
        Bits32       csPin;

        ULong        enPort;
        Bits32       enPin;
    };

    /*!
     *  @_nodoc
     *  ======== WiFiMSP430CC3000_HWAttrs ========
     *  Structure must match the WiFiMSP430CC3000_HWAttrs structure defined
     *  in wifi/WiFiMSP430CC3000.h
     */
    struct WiFiMSP430CC3000_HWAttrs {
        ULong        irqPort;
        Bits32       irqPin;

        ULong        csPort;
        Bits32       csPin;

        ULong        enPort;
        Bits32       enPin;
    };

    /*!
     *  @_nodoc
     *  ======== WiFiTivaCC3000_Object ========
     *  Structure must match the WiFiTivaCC3000_Object structure defined
     *  in wifi/WiFiTivaCC3000.h
     */
    struct WiFiTivaCC3000_Object {
        Ptr                         writeCompleteSem;
        Ptr                         hwiIrq;
        Ptr                         spiHandle;
        Ptr                         spiRxHandler;
        WiFiTivaCC3000_State        spiState;
        /* SPI_Transaction          transaction; */
    };

    /*!
     *  @_nodoc
     *  ======== WiFiMSP430CC3000_Object ========
     *  Structure must match the WiFiMSP430CC3000_Object structure defined
     *  in wifi/WiFiMSP430CC3000.h
     */
    struct WiFiMSP430CC3000_Object {
        Ptr                         writeCompleteSem;
        Ptr                         spiHandle;
        Ptr                         spiRxHandler;
        WiFiMSP430CC3000_State      spiState;
        /* SPI_Transaction          transaction; */
    };

    /*!
     *  @_nodoc
     *  ======== BasicView ========
     */
    metaonly struct BasicView {
        String              functionTable;
        String              wifiHandle;
        String              spiHandle;
        String              spiState;
    };

    /*!
     *  @_nodoc
     *  ======== rovViewInfo ========
     */
    @Facet
    metaonly config ViewInfo.Instance rovViewInfo =
        ViewInfo.create({
            viewMap: [
                ['Basic',
                    {
                        type: ViewInfo.MODULE_DATA,
                        viewInitFxn: 'viewInitBasic',
                        structName: 'BasicView'
                    }
                ]
            ]
        });
}
