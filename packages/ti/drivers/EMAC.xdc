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
 *  ======== EMAC.xdc ========
 */
package ti.drivers;
import xdc.rov.ViewInfo;

/*!
 *  ======== EMAC ========
 *  Module which allows configuration of the EMAC module
 *
 */
@NoRuntime
@HeaderName("")
module EMAC {

    /*!
     *  ======== LibType ========
     *  EMAC library selection options
     *
     *  This enumeration defines all the EMAC library type
     *  provided by the product.  You can select the library type by setting
     *  the {@link #libType EMAC.libType} configuration parameter.
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
     *  @_nodoc
     *  ======== EMAC_Config ========
     *  Copied from EMAC.h and must match.
     */
    struct EMAC_Config {
        Ptr         fxnTablePtr;
        Ptr         object;
        Ptr         hwAttrs;
    };

    /*!
     *  @_nodoc
     *  ======== EMACTiva_Object ========
     *  Copied from EMACTiva.h and must match.
     */
    struct EMACTiva_Object {
        ti.sysbios.knl.Swi.Handle  swi;
        ti.sysbios.hal.Hwi.Handle  hwi;
        UInt                       rxCount;
        UInt                       rxDropped;
        UInt                       txSent;
        UInt                       txDropped;
        UInt                       linkUp;
        Ptr                        pTxDescList;
        Ptr                        pRxDescList;
    };

    /*!
     *  @_nodoc
     *  ======== EMACTiva_HWAttrs ========
     *  Copied from EMACTiva.h and must match.
     */
    struct EMACTiva_HWAttrs {
        UInt8       intNum;
        String      macAddress;
    };

    /*!
     *  @_nodoc
     *  ======== EMACSnow_HWAttrs ========
     *  Copied from EMACSnow.h and must match.
     */
    struct EMACSnow_HWAttrs {
        ULong       baseAddr;
        UInt8       intNum;
        String      macAddress;
    };

    /*!
     *  @_nodoc
     *  ======== EMACSnow_Data ========
     *  Copied from EMACSnow.h and must match.
     */
    struct EMACSnow_Data {
        Ptr         hEvent;
        UInt8       PBM_tx[12];
        UInt8       PBM_rx[12];
        UInt        rxCount;
        UInt        rxDropped;
        UInt        txSent;
        UInt        txDropped;
        UInt        abnormalInts;
        UInt        isrCount;
        UInt        linkUp;
    };

    /*!
     *  @_nodoc
     *  ======== EMACSnow_Object ========
     *  Copied from EMACTiva.h and must match.
     */
    struct EMACSnow_Object {
        ti.sysbios.knl.Swi.Handle  swi;
    };

    /*!
     *  @_nodoc
     *  ======== BasicView ========
     */
    metaonly struct BasicView {
        String      macAddress;
        String      functionTable;
        String      libType;
        String      emacHandle;
        Bool        linkUp;
    };

    /*!
     *  @_nodoc
     *  ======== StatsView ========
     */
    metaonly struct StatsView {
        UInt       rxCount;
        UInt       rxDropped;
        UInt       txSent;
        UInt       txDropped;
    }

    /*!
     *  @_nodoc
     *  ======== rovViewInfo ========
     */
    @Facet
    metaonly config ViewInfo.Instance rovViewInfo =
        ViewInfo.create({
            viewMap: [
                ['Basic', {type: ViewInfo.MODULE_DATA,
                           viewInitFxn: 'viewInitBasic',
                           structName: 'BasicView'}],
                ['Statistics', {type: ViewInfo.MODULE_DATA,
                                viewInitFxn: 'viewInitStats',
                                structName: 'StatsView'}],
            ]
        });

    /*!
     *  ======== libType ========
     *  EMAC Library type
     *
     *  The EMAC runtime is provided in the form of a library that is
     *  linked with your application.  Several forms of this library are
     *  provided with the EMAC product. This configuration parameter
     *  allows you to select the form of EMAC to use.
     *
     *  The default value of libType is
     *  {@link #LibType_Instrumented EMAC_LibType_Instrumented}.  For a
     *  complete list of options and what they offer see {@link #LibType}.
     */
    metaonly config LibType libType = LibType_Instrumented;
}
