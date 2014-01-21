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
 *  ======== I2C.xdc ========
 */
package ti.drivers;
import xdc.rov.ViewInfo;

/*!
 *  ======== I2C ========
 */
@NoRuntime
@HeaderName("")
module I2C {

    /*!
     *  ======== LibType ========
     *  I2C library selection options
     *
     *  This enumeration defines all the I2C library type
     *  provided by the product.  You can select the library type by setting
     *  the {@link #libType I2C.libType} configuration parameter.
     *
     *  @field(LibType_Instrumented) The library supplied is prebuilt with
     *  logging and assertions enabled.
     *  Diags_USER1 mask logs basic information
     *  Diags_USER2 mask logs more detailed information
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
     *  I2C Library type
     *
     *  The I2C runtime is provided in the form of a library that is
     *  linked with your application.  Several forms of this library are
     *  provided with the I2C product. This configuration parameter
     *  allows you to select the form of I2C to use.
     *
     *  The default value of libType is
     *  {@link #LibType_Instrumented I2C_LibType_Instrumented}.  For a
     *  complete list of options and what they offer see {@link #LibType}.
     */
    metaonly config LibType libType = LibType_Instrumented;

    /*!
     *  @_nodoc
     *  ======== I2C_Config ========
     *  Structure must match the I2C_Config structure defined in I2C.h
     */
    struct I2C_Config {
        Void               *fxnTablePtr; //Used to determine the implementation
        Void               *object;      //Used to access the data
        Void               *hwAttrs;     //Used to get the attributes
    };

    /*!
     *  @_nodoc
     *  ======== I2C callback ========
     *  I2C Callback function
     */
    typedef Void (*CallbackFxn) (I2C_Config *, I2C_Transaction *, Bool);

    /*!
     *  @_nodoc
     *  ======== I2C_TransferMode ========
     *  Structure must match the I2C_TransferMode structure defined in I2C.h
     */
    enum I2C_TransferMode {
        I2C_MODE_BLOCKING,
        I2C_MODE_CALLBACK
    };

    /*!
     *  @_nodoc
     *  ======== I2C_Transaction ========
     *  Structure must match the I2C_Transaction structure defined in I2C.h
     */
    struct I2C_Transaction {
        Ptr             txBuff;
        UInt            txCount;

        Ptr             rxBuff;
        UInt            rxCount;

        UChar           slaveAddress;

        UArg            arg;
        Ptr             nextPtr;
    };

    /*!
     *  @_nodoc
     *  ======== I2C_Mode ========
     *  Structure must match the I2C_Mode structure defined in I2C.h
     */
    enum I2C_Mode {
        IDLE_MODE = 0x00,
        TX_MODE,
        RX_MODE,
        ERROR = 0xFF
    };

    /*!
     *  @_nodoc
     *  ======== I2C_Params ========
     *  Structure must match the I2C_Params structure defined in I2C.h
     */
    struct I2C_Params  {
        I2C_TransferMode    transferMode;
        Ptr                 transferCallback;
    };

    /*!
     *  @_nodoc
     *  ======== I2CTiva_hwAttrs ========
     *  Structure must match the I2C_HWAttrs structure defined in
     *  i2c/I2CTiva.h
     */
    struct I2CTiva_HWAttrs {
        ULong   baseAddr;
        Int     intNum;
    };

    /*!
     *  @_nodoc
     *  ======== I2CTiva_Object ========
     *  Structure must match the I2C_Object structure defined in
     *  i2c/I2CTiva.h
     */
    struct I2CTiva_Object {
        Ptr                 mutex;
        Ptr                 transferComplete;
        I2C_Params          i2cParams;
        Ptr                 hwi;
        I2C_Mode            mode;
        I2C_Transaction    *currentTransaction;
        UChar              *writeBufIdx;
        UInt                writeCountIdx;
        UChar              *readBufIdx;
        UInt                readCountIdx;
        I2C_Transaction    *headPtr;
        I2C_Transaction    *tailPtr;
    };

    /*!
     *  @_nodoc
     *  ======== I2CUSCIB_hwAttrs ========
     *  Structure must match the I2CUSCIB_HWAttrs structure defined in
     *  i2c/I2CUSCIB.h
     */
    struct I2CUSCIB_HWAttrs {
        ULong   baseAddr;
        UChar   clockSource;
    };

    /*!
     *  @_nodoc
     *  ======== I2CUSCIB_Object ========
     *  Structure must match the I2CUSCIB_Object structure defined in
     *  i2c/I2CUSCIB.h
     */
    struct I2CUSCIB_Object {
        Bool                isOpen;
        Ptr                 mutex;
        Ptr                 transferComplete;
        I2C_Params          i2cParams;
        I2C_Mode            mode;
        I2C_Transaction    *currentTransaction;
        UChar              *writeBufIdx;
        UInt                writeCountIdx;
        UChar              *readBufIdx;
        UInt                readCountIdx;
        I2C_Transaction    *headPtr;
        I2C_Transaction    *tailPtr;
    };

    /*!
     *  @_nodoc
     *  ======== BasicView ========
     */
    metaonly struct BasicView {
        String              i2cHandle;
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
