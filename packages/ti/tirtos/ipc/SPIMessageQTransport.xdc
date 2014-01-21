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
 *  ======== SPIMessageQTransport.xdc ========
 */
import xdc.rov.ViewInfo;
import xdc.runtime.Assert;

/*
 *  ======== SPIMessageQTransport ========
 *  This MessageQ transport that allows communication over SPI
 *
 *  This transport supports point to point communication across an SPI
 *  link. There must be a master and slave. The master drives the SPI
 *  link. The slave transport must be created and running before the
 *  master attempts to communicate to the master. The delaying of the
 *  master can be accomplished via waiting to call the
 *  SPIMessageQTransport_create on the master processor or using the
 *  {@link #clockStartDelay} parameter.
 *
 *  During startup, the master and slave exchange a handshake. All
 *  MessageQ_puts to the remote processor will fail until this
 *  handshake is completed.
 *
 *  The transport currently only supports dynamic creates (e.g.
 *  SPIMessageQTransport_create). It does not support static creates
 *  (SPIMessageQTransport.create in the .cfg file). There is an open
 *  enhancement request for this: SDOCM00101176.
 *
 *  It is up to the caller to initialize the SPI peripheral (e.g. call SPI_init
 *  and perform pin-muxing). Internally the transport calls SPI_open().
 *
 *  Asynchronous errors can occur in the transport. When one of these occur
 *  the errFxn specified in the SPIMessageQTransport_setErrFxn() API will
 *  be called. The following is a summary of the errors and want is passed
 *  in the errFxn.
 *
 *  Bad Msg: If the transport receives a badly formed message, the errFxn is
 *  called arguments:
 *  @p(code)
 *  Reason: SPIMessageQTransport_Reason_PHYSICALERR
 *  Handle: Transport handle
 *  Ptr: pointer to the received msg
 *  UArg: SPIMessageQTransport_Failure_BADMSG
 *  @p
 *
 *  Failed Checksum: If the transport receives a message with a bad checksum,
 *  the errFxn is called arguments:
 *  @p(code)
 *  Reason: SPIMessageQTransport_Reason_PHYSICALERR
 *  Handle: Transport handle
 *  Ptr: pointer to the received msg
 *  UArg: SPIMessageQTransport_Failure_BADCHECKSUM
 *  @p
 *
 *  Allocation failure: The transport copies incoming messages into an
 *  allocated message. If the allocation fails, the errFxn is called arguments:
 *  @p(code)
 *  Reason: SPIMessageQTransport_Reason_FAILEDALLOC
 *  Handle: Transport handle
 *  Ptr: NULL
 *  UArg: heapId used to try to allocate the message
 *  @p
 *
 *  Failed transmit: If the transport fails to transmit a message,
 *  the errFxn is called arguments:
 *  @p(code)
 *  Reason: SPIMessageQTransport_Reason_FAILEDPUT
 *  Handle: Transport handle
 *  Ptr: pointer to the msg that was not transmitted. Note: the msg
 *       will be freed after the errFxn is called.
 *  UArg: SPIMessageQTransport_Failure_TRANSFER
 *  @p
 */

@InstanceFinalize
@InstanceInitError

module SPIMessageQTransport inherits ti.sdo.ipc.interfaces.IMessageQTransport
{
    /*!
     *  ======== A_nullObject ========
     *  Assert thrown when a NULL transport object is passed to callback or
     *  Clock function
     */
    config Assert.Id A_nullObject = {
        msg: "A_nullObject: Transport object is NULL."
    };

    /*!
     *  ======== LibType ========
     *  SPIMessageQTransport library selection options
     *
     *  This enumeration defines all the SPIMessageQTransport.xdc library type
     *  provided by the product.  You can select the library type by setting
     *  the {@link #libType SPIMessageQTransport.xdc.libType} configuration
     *  parameter.
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
     *  ======== Priority ========
     *  Priority of the transport instance
     *
     *  MessageQ allows for two different priorities of MessageQ transports.
     */
    enum Priority {
        Priority_NORMAL,      /*! normal */
        Priority_HIGH         /*! high */
    };

    /*!
     *  ======== Failure ========
     *  Asynchronuos SPIMessageQTransport error codes
     */
    enum Failure {
        Failure_TRANSFER,      /*! SPI_transfer failed */
        Failure_BADCHECKSUM,   /*! Checksum comparison failed */
        Failure_BADMSG         /*! Received invalid message */
    };

    /*!
     *  ======== LinkStatus ========
     *  @_nodoc
     */
    enum LinkStatus {
        LinkStatus_UP,   /*! Transport is connected to remote processor */
        LinkStatus_DOWN  /*! Transport is not connected to remote processor */
    };

    /*!
     *  ======== libType ========
     *  SPIMessageQTransport Library type
     *
     *  The SPIMessageQTransport runtime is provided in the form of a library
     *  that is linked with your application.  Several forms of this library
     *  are provided with the SPIMessageQTransport product. This configuration
     *  parameter allows you to select the form of SPIMessageQTransport to use.
     *
     *  The default value of libType is
     *  {@link #LibType_Instrumented SPIMessageQTransport_LibType_Instrumented}.
     *  For a complete list of options and what they offer see {@link #LibType}.
     */
    metaonly config LibType libType = LibType_Instrumented;

    /*!
     *  @_nodoc
     *  ======== ModuleView ========
     */
    metaonly struct ModuleView {
        String    libType;
        String    checksum;
    }

    /*!
     *  @_nodoc
     *  ======== InstanceView ========
     */
    metaonly struct InstanceView {
        String    mode;
        String priority;
        Bool ready;
        Bool handshakeCompleted;
        Int maxMsgSize;
        UInt rxMsgDropped;
        UInt txMsgDropped;
    }

    /*!
     *  @_nodoc
     *  ======== rovViewInfo ========
     */
    @Facet
    metaonly config ViewInfo.Instance rovViewInfo =
        ViewInfo.create({
            viewMap: [
                ['Module',
                    {
                        type: ViewInfo.MODULE,
                        viewInitFxn: 'viewInitModule',
                        structName: 'ModuleView'
                    }
                ],
                ['Instances',
                    {
                        type: ViewInfo.INSTANCE,
                        viewInitFxn: 'viewInitInstances',
                        structName: 'InstanceView'
                    }
                ],
            ]
        });

    /*!
     *  ======== checksumEnabled ========
     *  Determines whether to checksum each message transmitted and received
     *
     *  The SPIMessageQTransport has an option to checksum each message that
     *  sent across the SPI channel. If checksumEnabled is true, the checksum
     *  is embedded into the message when it is transmitted. On the receiving
     *  side, if checksumEnabled is true also, the checksum is computed again.
     *  If there is a mismatch, a error message is logged. The errFxn is called
     *  and the message is discarded.
     *
     *  If the sending tranport does not have checksum enabled, but the
     *  receiving side does, no action is taken on the receiving side.
     *
     *  The default is false to minimize performance impact.
     */
    config Bool checksumEnabled = false;

    /*!
     *  ======== defaultErrFxn ========
     *  This is the default error function.
     *
     *  This function is an empty function that does nothing.
     *
     *  @param(reason)  reason for error function
     *  @param(handle)  handle of transport that had error
     *  @param(ptr)     pointer to the message
     *  @param(arg)     argument passed to error function
     */
    Void defaultErrFxn(ti.sdo.ipc.interfaces.IMessageQTransport.Reason reason,
                       ti.sdo.ipc.interfaces.IMessageQTransport.Handle handle, Ptr ptr, UArg arg);

instance:
    /*!
     *  ======== maxMsgSize ========
     *  The maximum size message that will be sent to a remote
     *  processor
     *
     *  The SPIMessageQTransport internally allocates a buffer of size
     *  maxMsgSize. This buffer is used to receive incoming SPI packets.
     *
     *  The units are in MAUs and the default is 512.
     */
    config UInt maxMsgSize = 512;

    /*!
     *  ======== spiIndex ========
     *  SPI index that the transport uses
     *
     *  The transport opens the configured SPI index. Please note
     *  the SPI peripheral must be initialized. The transport calls
     *  SPI_open(). The application is responsible for initializing
     *  the SPI peripheral (e.g. set-up pin-mux and call SPI_init()).
     *  Internally the SPIMessageQTransport calls SPI_open with the
     *  spiIndex specified.
     *
     *  The default is 0.
     */
    config UInt spiIndex = 0;

    /*!
     *  ======== spiBitRate ========
     *  SPI bit rate in Hz
     *
     *  The transport uses this value when opening the SPI channel.
     *
     *  The default is 1000000 (1MHz).
     */
    config UInt spiBitRate = 1000000;

    /*!
     *  ======== heap ========
     *  Heap used to allocate internal data structures
     *
     *  The heap needs to adhere to any placement restrictions required
     *  by the SPI driver (e.g. must be in memory that the DMA can access).
     */
    config xdc.runtime.IHeap.Handle heap = null;

    /*!
     *  ======== heapId ========
     *  MessageQ heapId used to allocate a handshake message
     *
     *  The specified heap will be used to allocate a handshake message.
     *  This message will be freed once the handshake is successfully completed.
     *
     *  The heap needs to adhere to any placement restrictions required
     *  by the SPI driver (e.g. must be in memory that the DMA can access).
     */
    config UInt16 heapId = 0;

    /*!
     *  ======== master ========
     *  Configure the SPI channel to be a SPI master or slave.
     */
    config Bool master = true;

	/*!
     *  ======== swiPriority ========
	 *  SPIMessageQTransport's Swi priority
	 *
	 *  The SPIMessageQTransport defers work from it's Hwi to a Swi.
	 *  This parameter allows the application to set the priority of
	 *  this Swi.
	 *
	 *  The default value of this parameter is ~0, which yields a
     *  Swi with the highest priority: ({@link #numPriorities} - 1).
     */
    config UInt swiPriority = ~0;

    /*!
     *  ======== transportPriority ========
     *  Priority of the transport instance
     *
     *  MessageQ allows two transport to be assigned to a remote processor.
     *  There is a normal and high priority instance. If a high priority message
     *  is sent, the high priority transport is used. Similarly, if a normal
     *  priority message is sent, the normal priority transport is used.
     *
     *  If only one transport instance is assigned to communicate with a
     *  specific remote processor, all messages will go over that transport
     *  instance (regardless of whether the message is high or normal
     *  priority).
     *
     *  The default is normal priority.
     */
    config Priority transportPriority = Priority_NORMAL;

    /*!
     *  ======== clockRate ========
     *  Interval that the Master sends and receives packets
     *
     *  If the transport is configured as an SPI master {@link #master},
     *  it transmits (and receives) one message every clockRate ticks.
     *  The tick is supplied by the SYS/BIOS Clock module. The default
     *  Clock tickPeriod is 1000 microseconds. See
     *  {@link ti.sysbios.knl.Clock} for more details.
     *
     *  If the master has no messages to transmit, a transfer of zeros
     *  is performed. This is done in case the slave side has a message
     *  it wants to send to the master.
     *
     *  The default is 10 SYS/BIOS Clock ticks.
     */
    config Int clockRate = 10;

    /*!
     *  ======== clockStartDelay ========
     *  The number of ticks to delay the SPI master's first transmission
     *
     *  If the transport is configured as an SPI master {@link #master},
     *  it transmits (and receives) one message every clockRate ticks.
     *  The tick is supplied by the SYS/BIOS Clock module. The default
     *  Clock tickPeriod is 1000 microseconds. See
     *  {@link ti.sysbios.knl.Clock} for more details.
     *
     *  The first transmission will occur after clockStartDelay ticks.
     *  This parameter can be used to help make sure the slave is up and
     *  running before the first transmission occurs.
     *
     *  The default is 1 SYS/BIOS Clock ticks which is the smallest valid
     *  value.
     */
    config Int clockStartDelay = 1;

internal:

    struct Instance_State {
        ti.sysbios.knl.Queue.Object txQueue;
        ti.sysbios.knl.Clock.Handle clock;
        ti.sysbios.knl.Swi.Handle swi;
        Ptr spiHandle;
        Ptr transaction;
        UInt maxMsgSize;
        UInt spiIndex;
        UInt rxMsgDropped;
        UInt txMsgDropped;
        UInt16 procId;
        Char rxMsg[];
        xdc.runtime.IHeap.Handle heap;
        Priority priority;
        Bool master;
        Bool ready;
        LinkStatus handshake;
    };

    struct Module_State {
        ErrFxn errFxn;
    }
}
