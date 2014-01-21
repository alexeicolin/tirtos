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
/** ============================================================================
 *  @file       SPI.h
 *
 *  @brief      SPI driver interface
 *
 *  The SPI header file should be included in an application as follows:
 *  @code
 *  #include <ti/drivers/SPI.h>
 *  @endcode
 *
 *  # Operation #
 *  The SPI driver in TI-RTOS is designed to serve a means to move data
 *  between SPI peripherals. This driver does not interpret any of the data sent
 *  to or received from this peripheral.
 *
 *  The APIs in this driver serve as an interface to a typical TI-RTOS
 *  application. The specific peripheral implementations are responsible to
 *  create all the SYS/BIOS specific primitives to allow for thread-safe
 *  operation.
 *
 *  The SPI driver operates on some key definitions and assumptions:
 *  - The driver operates transparent from the chip select. Some SPI controllers
 *    feature a hardware chip select to assert SPI slave peripherals. See the
 *    specific peripheral implementations on chip select requirements.
 *
 *  - The SPI protocol does not account for a built-in handshaking mechanism and
 *    neither does this SPI driver. Therefore, when operating in ::SPI_SLAVE
 *    mode, the application must provide such a mechanism to ensure that the
 *    SPI slave is ready for the SPI master. The SPI slave must call
 *    SPI_transfer() *before* the SPI master starts transmitting. Some example
 *    application mechanisms could include:
 *    - Timed delays on the SPI master to guarantee the SPI slave is be ready
 *      for a SPI transaction.
 *    - A form of GPIO flow control from the slave to the SPI master to notify
 *      the master when ready.
 *
 *  ## Opening the driver #
 *
 *  @code
 *  SPI_Handle      handle;
 *  SPI_Params      params;
 *  SPI_Transaction spiTransaction;
 *
 *  SPI_Params_init(&params);
 *  params.bitRate  = someNewBitRate;
 *  handle = SPI_open(someSPI_configIndexValue, &params);
 *  if (!handle) {
 *      System_printf("SPI did not open");
 *  }
 *  @endcode
 *
 *  ## Transferring data #
 *  Data transmitted and received by the SPI peripheral is performed using
 *  SPI_transfer(). SPI_transfer() accepts a pointer to a SPI_Transaction
 *  structure that dictates what quantity of data is sent and received.
 *
 *  @code
 *  SPI_Transaction spiTransaction;
 *
 *  spiTransaction.count = someIntegerValue;
 *  spiTransaction.txBuf = transmitBufferPointer;
 *  spiTransaction.rxBuf = receiveBufferPointer;
 *
 *  ret = SPI_transfer(handle, &spiTransaction);
 *  if (!ret) {
 *      System_printf("Unsuccessful SPI transfer");
 *  }
 *  @endcode
 *
 *  # Implementation #
 *
 *  This module serves as the main interface for TI-RTOS  applications. Its
 *  purpose is to redirect the module's APIs to specific peripheral
 *  implementations which are specified using a pointer to a SPI_FxnTable.
 *
 *  The SPI driver interface module is joined (at link time) to a
 *  NULL-terminated array of SPI_Config data structures named *SPI_config*.
 *  *SPI_config* is implemented in the application with each entry being an
 *  instance of a SPI peripheral. Each entry in *SPI_config* contains a:
 *  - (SPI_FxnTable *) to a set of functions that implement a SPI peripheral
 *  - (Void *) data object that is associated with the SPI_FxnTable
 *  - (Void *) hardware attributes that are associated to the SPI_FxnTable
 *
 *  Currently the following SPI peripheral implementations are supported:
 *  - @ref SPITivaDMA.h
 *  - @ref SPIUSCIADMA.h
 *  - @ref SPIUSCIBDMA.h
 *
 *  # Instrumentation #
 *
 *  The SPI driver interface produces log statements if instrumentation is
 *  enabled.
 *
 *  Diagnostics Mask | Log details |
 *  ---------------- | ----------- |
 *  Diags_USER1      | basic SPI operations performed |
 *  Diags_USER2      | detailed SPI operations performed |
 *
 *  ============================================================================
 */

#ifndef ti_drivers_SPI__include
#define ti_drivers_SPI__include

#ifdef __cplusplus
extern "C" {
#endif

#include <xdc/std.h>
#include <ti/drivers/ENV.h>

/*!
 *  @brief      A handle that is returned from a SPI_open() call.
 */
typedef struct SPI_Config      *SPI_Handle;

/*!
 *  @brief
 *  A ::SPI_Transaction data structure is used with SPI_transfer(). It indicates
 *  how many ::SPI_FrameFormat frames are sent and received from the buffers
 *  pointed to txBuf and rxBuf.
 *  The arg variable is an user-definable argument which gets passed to the
 *  ::SPI_CallbackFxn when the SPI driver is in ::SPI_MODE_CALLBACK.
 */
typedef struct SPI_Transaction {
    UInt    count;      /*!< Number of frames for this transaction */
    Ptr     txBuf;      /*!< Ptr to a buffer with data to be transmitted */
    Ptr     rxBuf;      /*!< Ptr to a buffer to receive data */
    UArg    arg;        /*!< Argument to be passed to the callback function */
} SPI_Transaction;

/*!
 *  @brief      The definition of a callback function used by the SPI driver
 *              when used in ::SPI_MODE_CALLBACK
 *
 *  @param      SPI_Handle          SPI_Handle
 *  @param      SPI_Transaction*    SPI_Transaction*
 */
typedef Void        (*SPI_CallbackFxn) (SPI_Handle handle,
                                        SPI_Transaction * transaction);

/*!
 *  @brief
 *  Definitions for various SPI modes of operation.
 */
typedef enum SPI_Mode {
    SPI_MASTER      = ENV_SPI_MASTER,       /*!< SPI in master mode */
    SPI_SLAVE       = ENV_SPI_SLAVE         /*!< SPI in slave mode */
} SPI_Mode;

/*!
 *  @brief
 *  Definitions for various SPI data frame formats.
 */
typedef enum SPI_FrameFormat {
    SPI_POL0_PHA0   = ENV_SPI_POL0_PHA0,    /*!< SPI mode Polarity 0 Phase 0 */
    SPI_POL0_PHA1   = ENV_SPI_POL0_PHA1,    /*!< SPI mode Polarity 0 Phase 1 */
    SPI_POL1_PHA0   = ENV_SPI_POL1_PHA0,    /*!< SPI mode Polarity 1 Phase 0 */
    SPI_POL1_PHA1   = ENV_SPI_POL1_PHA1,    /*!< SPI mode Polarity 1 Phase 1 */
    SPI_TI          = ENV_SPI_TI,           /*!< TI mode */
    SPI_MW          = ENV_SPI_MW            /*!< Micro-wire mode */
} SPI_FrameFormat;

/*!
 *  @brief
 *
 *  SPI transfer mode determines the whether the SPI controller operates
 *  synchronously or asynchronously. In ::SPI_MODE_BLOCKING mode SPI_transfer()
 *  blocks code execution until the SPI transaction has completed. In
 *  ::SPI_MODE_CALLBACK SPI_transfer() does not block code execution and instead
 *  calls a ::SPI_CallbackFxn callback function when the transaction has
 *  completed.
 */
typedef enum SPI_TransferMode {
    /*!
     * SPI_transfer() blocks execution. This mode can only be used when called
     * within a Task context
     */
    SPI_MODE_BLOCKING,
    /*!
     * SPI_transfer() does not block code execution and will call a
     * ::SPI_CallbackFxn. This mode can be used in a Task, Swi, or Hwi context.
     */
    SPI_MODE_CALLBACK
} SPI_TransferMode;

/*!
 *  @brief
 *  SPI Parameters are used to with the SPI_open() call. Default values for
 *  these parameters are set using SPI_Params_init().
 *
 *  @sa     SPI_Params_init
 */
typedef struct SPI_Params {
    SPI_TransferMode    transferMode;       /*!< Blocking or Callback mode */
    SPI_CallbackFxn     transferCallbackFxn;/*!< Callback function pointer */
    SPI_Mode            mode;               /*!< Master or Slave mode */
    ULong               bitRate;            /*!< SPI bit rate in Hz */
    UInt                dataSize;           /*!< SPI data frame size in bits */
    SPI_FrameFormat     frameFormat;        /*!< SPI frame format */
} SPI_Params;

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_close().
 */
typedef Void        (*SPI_CloseFxn)    (SPI_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_init().
 */
typedef Void        (*SPI_InitFxn)     (SPI_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_open().
 */
typedef SPI_Handle  (*SPI_OpenFxn)     (SPI_Handle handle,
                                        SPI_Params *params);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_serviceISR().
 */
typedef Void        (*SPI_ServiceISRFxn)(SPI_Handle handle);

/*!
 *  @brief      A function pointer to a driver specific implementation of
 *              SPI_transfer().
 */
typedef Bool        (*SPI_TransferFxn) (SPI_Handle handle,
                                        SPI_Transaction *transaction);

/*!
 *  @brief      The definition of a SPI function table that contains the
 *              required set of functions to control a specific SPI driver
 *              implementation.
 */
typedef struct SPI_FxnTable {
    /*! Function to close the specified peripheral */
    SPI_CloseFxn        closeFxn;

    /*! Function to initialize the given data object */
    SPI_InitFxn         initFxn;

    /*! Function to open the specified peripheral */
    SPI_OpenFxn         openFxn;

    /*! Function to initiate a SPI data transfer */
    SPI_TransferFxn     transferFxn;

    /*! Function to service the SPI instance */
    SPI_ServiceISRFxn   serviceISRFxn;
} SPI_FxnTable;

/*!
 *  @brief
 *  The SPI_Config structure contains a set of pointers used to characterize
 *  the SPI driver implementation.
 */
typedef struct SPI_Config {
    /*! Pointer to a table of a driver-specific implementation of SPI functions */
    SPI_FxnTable const *fxnTablePtr;

    /*! Pointer to a driver specific data object */
    Void               *object;

    /*! Pointer to a driver specific hardware attributes structure */
    Void         const *hwAttrs;
} SPI_Config;

/*!
 *  @brief  Function to close a given SPI peripheral specified by the
 *          SPI handle.
 *
 *  @pre    SPI_open() has to be called first.
 *
 *  @param  handle A SPI handle returned from SPI_open()
 *
 *  @sa     SPI_open()
 */
extern Void SPI_close(SPI_Handle handle);

/*!
 *  @brief  This function initializes the SPI driver module.
 *
 *  @pre    The SPI needs to be powered up and clocked. The SPI_config structure
 *          must exist and be persistent before this function can be called.
 *          This function must also be called before any other SPI driver APIs.
 */
extern Void SPI_init(Void);

/*!
 *  @brief  This function opens a given SPI peripheral.
 *
 *  @pre    SPI controller has been initialized using SPI_init()
 *
 *  @param  index       Logical peripheral number indexed into the SPI_config
 *                      table
 *
 *  @param  params      Pointer to an parameter block, if NULL it will use
 *                      default values
 *
 *  @return A pointer to a SPI_Handle on success or a NULL it was already
 *          opened
 *
 *  @sa     SPI_close()
 *  @sa     SPI_init()
 */
extern SPI_Handle SPI_open(UInt index, SPI_Params *params);

/*!
 *  @brief  Function to initialize the SPI_Params struct to its defaults
 *
 *  Defaults values are:
 *  @code
 *  transferMode        = SPI_MODE_BLOCKING
 *  transferCallbackFxn = NULL
 *  mode                = SPI_MASTER
 *  bitRate             = 1000000 (Hz)
 *  dataSize            = 8 (bits)
 *  frameFormate        = SPI_POL0_PHA0
 *  @endcode
 *
 *  @param  params  Parameter structure to initialize
 */
extern Void SPI_Params_init(SPI_Params *params);

/*!
 *  @brief  Function to service the SPI module's interrupt service routine
 *
 *  Currently, this function is only supported with the drivers listed below.
 *  - @ref SPIUSCIADMA.h
 *  - @ref SPIUSCIBDMA.h
 *
 *  @param  handle      A SPI_Handle
 */
extern Void SPI_serviceISR(SPI_Handle handle);

/*!
 *  @brief  Function to perform SPI transactions
 *
 *  If the SPI is in ::SPI_MASTER mode, it will immediately start the
 *  transaction. If the SPI is in ::SPI_SLAVE mode, it prepares itself for a
 *  transaction with a SPI master.
 *
 *  In ::SPI_MODE_BLOCKING, SPI_transfer will block task execution until the
 *  transaction has completed.
 *
 *  In ::SPI_MODE_CALLBACK, SPI_transfer() does not block task execution and
 *  calls a ::SPI_CallbackFxn. This makes the SPI_tranfer() safe to be used
 *  within a Task, Swi, or Hwi context. The ::SPI_Transaction structure must
 *  stay persistent until the SPI_transfer function has completed!
 *
 *  @param  handle      A SPI_Handle
 *
 *  @param  transaction A pointer to a SPI_Transaction
 *
 *  @return TRUE if started successfully; else FALSE
 *
 *  @sa     SPI_open
 */
extern Bool SPI_transfer(SPI_Handle handle, SPI_Transaction *transaction);

#ifdef __cplusplus
}
#endif

#endif /* ti_drivers_SPI__include */
