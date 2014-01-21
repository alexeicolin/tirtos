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
 *  ======== USBMSCHFatFs.xdc ========
 */
package ti.drivers;
import xdc.rov.ViewInfo;

/*!
 *  ======== USBMSCHFatFs ========
 */
@NoRuntime
@HeaderName("")
module USBMSCHFatFs {

    /*!
     *  ======== LibType ========
     *  USBMSCHFatFs library selection options
     *
     *  This enumeration defines all the USBMSCHFatFs library type
     *  provided by the product.  You can select the library type by setting
     *  the {@link #libType USBMSCHFatFs.libType} configuration parameter.
     *
     *  @field(LibType_Instrumented) The library supplied is prebuilt with
     *  logging and assertions enabled.
     *  Diags_USER1 mask logs basic information
     *  Diags_USER2 mask logs advanced information
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
     *  USBMSCHFatFs Library type
     *
     *  The USBMSCHFatFs runtime is provided in the form of a library that is
     *  linked with your application.  Several forms of this library are
     *  provided with the USBMSCHFatFs product. This configuration parameter
     *  allows you to select the form of USBMSCHFatFs to use.
     *
     *  The default value of libType is
     *  {@link #LibType_Instrumented USBMSCHFatFs_LibType_Instrumented}.  For a
     *  complete list of options and what they offer see {@link #LibType}.
     */
    metaonly config LibType libType = LibType_Instrumented;

    /*!
     *  @_nodoc
     *  ======== maxPeripheralCount ========
     */
    metaonly config UInt maxPeripheralCount = 32;

    /*!
     *  @_nodoc
     *  ======== USBMSCHFatFs_Config ========
     *  Structure must match the USBMSCHFatFs_Config structure defined in
     *  USBMSCHFatFs.h
     */
    struct USBMSCHFatFs_Config {
        Void               *fxnTablePtr; //Used to determine the implementation
        Void               *object;      //Used to access the data
        Void               *hwAttrs;     //Used to get the attributes
    };

    /*!
     *  @_nodoc
     *  ======== USBMSCHFatFsTiva_USBState ========
     */
    enum USBMSCHFatFsTiva_USBState {
        USBMSCHFatFsTiva_NO_DEVICE,
        USBMSCHFatFsTiva_CONNECTED,
        USBMSCHFatFsTiva_UNKNOWN,
        USBMSCHFatFsTiva_POWER_FAULT
    };

    /*!
     *  @_nodoc
     *  ======== USBMSCHFatFsTiva_hwAttrs ========
     *  Structure must match the USBMSCHFatFsTivaDMA_hwAttrs structure
     *  defined in usbmschfatfs/USBMSCHFatFsTivaDMA.h
     */
    struct USBMSCHFatFsTiva_HWAttrs {
        ULong baseAddr;

        ULong portSPI;
        ULong pinSCK;
        ULong pinMISO;
        ULong pinMOSI;

        ULong portCS;
        ULong pinCS;

        ULong portTX;
        ULong pinTX;
    };

    /*!
     *  @_nodoc
     *  ======== USBMSCHFatFsTiva_object ========
     *  Structure must match the USBMSCHFatFsTiva_object structure defined
     *  in usbmschfatfs/USBMSCHFatFsTivaDMA.h
     */
    struct USBMSCHFatFsTiva_Object {
        UInt                driveNumber;      /*!< Drive number used by FatFs */
        USBMSCHFatFsTiva_USBState state;        /*!< USB state */
        Ptr                 hwi;                /*!< Hwi handle */
        Ptr                 taskHCDMain;        /*!< Task handle */
        Ptr                 gateUSBWait;        /*!< Gate handle */
        Ptr                 gateUSBLibAccess;   /*!< Gate handle */
        Ptr                 semUSBConnected;    /*!< Semaphore handle */
        Ptr                 MSCInstance;        /*!< USB MSC instance handle */
    };

    /*!
     *  @_nodoc
     *  ======== BasicView ========
     */
    metaonly struct BasicView {
        String              usbmschfatfsHandle;
        String              baseAddr;
        String              functionTable;
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
                ],
            ]
        });
}
