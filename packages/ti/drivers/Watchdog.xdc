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
 *  ======== Watchdog.xdc ========
 */
package ti.drivers;
import xdc.rov.ViewInfo;

/*!
 *  ======== Watchdog ========
 */
@NoRuntime
@HeaderName("")
module Watchdog {

    /*!
     *  ======== LibType ========
     *  Watchdog library selection options
     *
     *  This enumeration defines all the Watchdog library type
     *  provided by the product.  You can select the library type by setting
     *  the {@link #libType Watchdog.libType} configuration parameter.
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
     *  Watchdog Library type
     *
     *  The Watchdog runtime is provided in the form of a library that is
     *  linked with your application.  Several forms of this library are
     *  provided with the Watchdog product. This configuration parameter
     *  allows you to select the form of Watchdog to use.
     *
     *  The default value of libType is
     *  {@link #LibType_Instrumented Watchdog_LibType_Instrumented}.  For a
     *  complete list of options and what they offer see {@link #LibType}.
     */
    metaonly config LibType libType = LibType_Instrumented;

    /*!
     *  @_nodoc
     *  ======== WatchdogTiva_HWAttrs ========
     *  Copied from WatchdogTiva.h and must match.
     */
    struct WatchdogTiva_HWAttrs {
        ULong       baseAddr;
        UInt        intNum;
        ULong       reloadValue;
    };

    /*!
     *  @_nodoc
     *  ======== WatchdogTiva_Object ========
     *  Copied from Watchdog.h and must match.
     */
    struct WatchdogTiva_Object {
        Bool        isOpen;
    };

    /*!
     *  @_nodoc
     *  ======== WatchdogMSP430_HWAttrs ========
     *  Copied from WatchdogMSP430.h and must match.
     */
    struct WatchdogMSP430_HWAttrs {
        ULong      baseAddr;     /*!< Base address of Watchdog */
        ULong      sfrAddr;      /*!< SFR base address for Watchdog enabled */
        UInt       clockSource;  /*!< Clock source for Watchdog */
        UInt       clockDivider; /*!< Clock divider for Watchdog */
    };

    /*!
     *  @_nodoc
     *  ======== WatchdogMSP430_Object ========
     *  Copied from Watchdog.h and must match.
     */
    struct WatchdogMSP430_Object {
        Bool        isOpen;
    };

    /*!
     *  @_nodoc
     *  ======== Watchdog_Config ========
     *  Copied from Watchdog.h and must match.
     */
    struct Watchdog_Config {
        Ptr         fxnTablePtr;
        Ptr         object;
        Ptr         hwAttrs;
    };

    /*!
     *  @_nodoc
     *  ======== BasicView ========
     */
    metaonly struct BasicView {
        String      watchdogHandle;
        String      baseAddr;
        String      functionTable;
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
