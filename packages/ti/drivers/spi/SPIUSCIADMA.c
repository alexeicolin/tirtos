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

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Types.h>

#include <ti/drivers/spi/SPIUSCIADMA.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/family/msp430/ClockFreqs.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/hal/Hwi.h>

/* driverlib header files */
#include <usci_a_spi.h>
#include <dma.h>

/* SPIUSCIADMA functions */
Void         SPIUSCIADMA_close(SPI_Handle handle);
static Void  SPIUSCIADMA_hwiIntFxn(SPI_Handle handle);
Void         SPIUSCIADMA_init(SPI_Handle handle);
SPI_Handle   SPIUSCIADMA_open(SPI_Handle handle, SPI_Params *params);
Bool         SPIUSCIADMA_transfer(SPI_Handle handle,
                                  SPI_Transaction *transaction);
static Void         SPIUSCIADMA_transferCallback(SPI_Handle handle,
                                                 SPI_Transaction *msg);

/* SPI function table for SPIUSCIADMA implementation */
const SPI_FxnTable SPIUSCIADMA_fxnTable = {
    SPIUSCIADMA_close,
    SPIUSCIADMA_init,
    SPIUSCIADMA_open,
    SPIUSCIADMA_transfer,
    SPIUSCIADMA_hwiIntFxn
};

/* Default SPI params */
extern const SPI_Params SPI_defaultParams;

/*
 *  ======== SPIUSCIADMA_close ========
 *  @pre    Function assumes that the handle is not NULL
 */
Void SPIUSCIADMA_close(SPI_Handle handle)
{
    UInt                        key;
    SPIUSCIADMA_Object         *object = handle->object;
    SPIUSCIADMA_HWAttrs const  *hwAttrs = handle->hwAttrs;

    USCI_A_SPI_disable(hwAttrs->baseAddr);

    Semaphore_destruct(&(object->transferComplete));

    Log_print1(Diags_USER1, "SPI:(%p) closed", hwAttrs->baseAddr);

    key = Hwi_disable();
    object->isOpen = FALSE;
    Hwi_restore(key);
}

/*
 *  ======== SPIUSCIADMA_configDMA ========
 *  This functions configures the transmit and receive DMA channels for a given
 *  SPI_Handle and SPI_Transaction
 *
 *  @pre    Function assumes that the handle and transaction is not NULL
 */
static Void SPIUSCIADMA_configDMA(SPI_Handle handle,
                                  SPI_Transaction *transaction)
{
    UInt                        key;
    SPIUSCIADMA_HWAttrs const  *hwAttrs = handle->hwAttrs;

    /*
     * Hwi protection is needed because DMA_init could be accessing registers
     * that are shared
     */
    key = Hwi_disable();

    /* Rx Channel */
    DMA_init(hwAttrs->dmaBaseAddr,
             hwAttrs->rxDMAChannelIndex,
             DMA_TRANSFER_SINGLE,
             transaction->count,
             hwAttrs->rxDMASourceTrigger,
             DMA_SIZE_SRCBYTE_DSTBYTE,
             DMA_TRIGGER_HIGH);

    /* Tx Channel */
    DMA_init(hwAttrs->dmaBaseAddr,
             hwAttrs->txDMAChannelIndex,
             DMA_TRANSFER_SINGLE,
             transaction->count,
             hwAttrs->txDMASourceTrigger,
             DMA_SIZE_SRCBYTE_DSTBYTE,
             DMA_TRIGGER_HIGH);

    Hwi_restore(key);

    /*
     * The following APIs only access the channel specific DMA registers, which
     * we assume that we own exclusively.
     */

    /* Rx Channel */
    DMA_setSrcAddress(hwAttrs->dmaBaseAddr,
                      hwAttrs->rxDMAChannelIndex,
                      USCI_A_SPI_getReceiveBufferAddressForDMA(hwAttrs->baseAddr),
                      DMA_DIRECTION_UNCHANGED);
    DMA_setDstAddress(hwAttrs->dmaBaseAddr,
                      hwAttrs->rxDMAChannelIndex,
                      (ULong)transaction->rxBuf,
                      DMA_DIRECTION_INCREMENT);
    DMA_enableInterrupt(hwAttrs->dmaBaseAddr,
                        hwAttrs->rxDMAChannelIndex);

    /* Tx Channel */
    DMA_setSrcAddress(hwAttrs->dmaBaseAddr,
                      hwAttrs->txDMAChannelIndex,
                      (ULong)transaction->txBuf,
                      DMA_DIRECTION_INCREMENT);
    DMA_setDstAddress(hwAttrs->dmaBaseAddr,
                      hwAttrs->txDMAChannelIndex,
                      USCI_A_SPI_getTransmitBufferAddressForDMA(hwAttrs->baseAddr),
                      DMA_DIRECTION_UNCHANGED);
    DMA_enableInterrupt(hwAttrs->dmaBaseAddr,
                        hwAttrs->txDMAChannelIndex);

    /* Enable DMA Channels */
    DMA_enableTransfers(hwAttrs->dmaBaseAddr, hwAttrs->rxDMAChannelIndex);
    DMA_enableTransfers(hwAttrs->dmaBaseAddr, hwAttrs->txDMAChannelIndex);

    Log_print1(Diags_USER1,"SPI:(%p) DMA transfer enabled", hwAttrs->baseAddr);

    Log_print5(Diags_USER2,"SPI:(%p) DMA transaction: %p, "
                           "rxBuf: %p; txBuf: %p; Count: %d",
                            hwAttrs->baseAddr,
                            (UArg)transaction,
                            (UArg)transaction->rxBuf,
                            (UArg)transaction->txBuf,
                            (UArg)transaction->count);
}

/*
 *  ======== SPIUSCIADMA_hwiIntFxn ========
 *  Function to be called by the DMA ISR (Hwi context)
 */
Void SPIUSCIADMA_hwiIntFxn (SPI_Handle handle)
{
    SPI_Transaction            *msg;
    SPIUSCIADMA_Object         *object = ((SPI_Handle)handle)->object;
    SPIUSCIADMA_HWAttrs const  *hwAttrs = ((SPI_Handle)handle)->hwAttrs;

    Log_print1(Diags_USER2, "SPI:(%p) interrupt context start",
                             hwAttrs->baseAddr);

    if (DMA_getInterruptStatus(hwAttrs->dmaBaseAddr,
                               hwAttrs->txDMAChannelIndex) == DMA_INT_ACTIVE) {
        DMA_clearInterrupt(hwAttrs->dmaBaseAddr, hwAttrs->txDMAChannelIndex);
        DMA_disableInterrupt(hwAttrs->dmaBaseAddr, hwAttrs->txDMAChannelIndex);
    }

    /* Determine if the TX and RX DMA channels have completed */
    if ((object->transaction) &&
        (DMA_getInterruptStatus(hwAttrs->dmaBaseAddr,
                                hwAttrs->rxDMAChannelIndex) == DMA_INT_ACTIVE)){

        DMA_clearInterrupt(hwAttrs->dmaBaseAddr, hwAttrs->rxDMAChannelIndex);
        DMA_disableInterrupt(hwAttrs->dmaBaseAddr, hwAttrs->rxDMAChannelIndex);

        /*
         * Use a temporary transaction pointer in case the callback function
         * attempts to perform another SPI_transfer call
         */
        msg = object->transaction;

        /* Indicate we are done with this transfer */
        object->transaction = NULL;

        Log_print2(Diags_USER1,"SPI:(%p) DMA transaction: %p complete",
                                hwAttrs->baseAddr, (UArg)msg);

        /* Perform callback */
        object->transferCallbackFxn(handle, msg);
    }

    Log_print1(Diags_USER2, "SPI:(%p) interrupt context end",
                             hwAttrs->baseAddr);
}

/*
 *  ======== SPIUSCIADMA_init ========
 *  @pre    Function assumes that the handle is not NULL
 */
Void SPIUSCIADMA_init(SPI_Handle handle)
{
    SPIUSCIADMA_Object    *object = handle->object;

    /* Mark the object as available */
    object->isOpen = FALSE;
    object->transaction = NULL;
}

/*
 *  ======== SPIUSCIADMA_open ========
 *  @pre    Function assumes that the handle is not NULL
 */
SPI_Handle SPIUSCIADMA_open(SPI_Handle handle, SPI_Params *params)
{
    UInt                            key;
    UInt32                          clockFreq;
    Semaphore_Params                semParams;
    SPIUSCIADMA_Object              *object = handle->object;
    SPIUSCIADMA_HWAttrs const       *hwAttrs = handle->hwAttrs;

    /* Determine if the device index was already opened */
    key = Hwi_disable();
    if (object->isOpen != FALSE) {
        Hwi_restore(key);
        return (NULL);
    }
    /* Mark the handle as being used */
    object->isOpen = TRUE;
    Hwi_restore(key);

    /* Store the SPI parameters */
    if (params == NULL) {
        /* No params passed in, so use the defaults */
        params = (SPI_Params *) &SPI_defaultParams;
    }

    USCI_A_SPI_disableInterrupt(hwAttrs->baseAddr,
                                USCI_A_SPI_RECEIVE_INTERRUPT |
                                USCI_A_SPI_TRANSMIT_INTERRUPT);
    USCI_A_SPI_clearInterruptFlag(hwAttrs->baseAddr,
                                  USCI_A_SPI_RECEIVE_INTERRUPT |
                                  USCI_A_SPI_TRANSMIT_INTERRUPT);

    /* Store the current mode */
    object->transferMode = params->transferMode;

    if (object->transferMode == SPI_MODE_BLOCKING) {
        Log_print1(Diags_USER2, "SPI:(%p) in SPI_MODE_BLOCKING mode",
                                 hwAttrs->baseAddr);
        /*
         * Create a semaphore to block task execution for the duration of the
         * SPI transfer
         */
        Semaphore_Params_init(&semParams);
        semParams.mode = Semaphore_Mode_BINARY;
        Semaphore_construct(&(object->transferComplete), 0, &semParams);

        /* Store internal callback function */
        object->transferCallbackFxn = SPIUSCIADMA_transferCallback;
    }
    else {
        Log_print1(Diags_USER2, "SPI:(%p) in SPI_MODE_CALLBACK mode",
                                 hwAttrs->baseAddr);

        /* Check to see if a callback function was defined for async mode */
        Assert_isTrue(params->transferCallbackFxn != NULL, NULL);

        /* Save the callback function pointer */
        object->transferCallbackFxn = params->transferCallbackFxn;
    }

    /* Get the SPI clock input frequency */
    switch (hwAttrs->clockSource) {
        case USCI_A_SPI_CLOCKSOURCE_ACLK:
            clockFreq = ClockFreqs_getFrequency(ClockFreqs_Clock_ACLK);
            Log_print1(Diags_USER1, "ClockFreqs_getFrequency ACLK: %d", clockFreq);
            break;

        case USCI_A_SPI_CLOCKSOURCE_SMCLK:
            clockFreq = ClockFreqs_getFrequency(ClockFreqs_Clock_SMCLK);
            Log_print1(Diags_USER1, "ClockFreqs_getFrequency SMCLK: %d", clockFreq);
            break;

        default:
            Log_error0("SPI: Error determining clock source");
            SPIUSCIADMA_close(handle);
            return (NULL);
    }

    if (params->mode == SPI_MASTER) {
        USCI_A_SPI_masterInit(hwAttrs->baseAddr,
                              hwAttrs->clockSource,
                              clockFreq,
                              params->bitRate,
                              hwAttrs->bitOrder,
                              params->frameFormat & UCCKPH,
                              params->frameFormat & UCCKPL);
    }
    else { /* SPI_SLAVE */
        USCI_A_SPI_slaveInit(hwAttrs->baseAddr,
                             hwAttrs->bitOrder,
                             params->frameFormat & UCCKPH,
                             params->frameFormat & UCCKPL);
    }

    Log_print3(Diags_USER1, "SPI:(%p) CPU freq: %d; SPI freq to %d",
                             hwAttrs->baseAddr, clockFreq, params->bitRate);

    USCI_A_SPI_enable(hwAttrs->baseAddr);

    Log_print1(Diags_USER1, "SPI:(%p) opened", hwAttrs->baseAddr);

    return (handle);
}

/*
 *  ======== SPIUSCIADMA_transfer ========
 *  @pre    Function assumes that handle and transaction is not NULL
 */
Bool SPIUSCIADMA_transfer(SPI_Handle handle, SPI_Transaction *transaction)
{
    UInt                   key;
    SPIUSCIADMA_Object    *object = handle->object;

    /* This is a limitation by the DMA controller */
#ifndef MSP430WARE
    Assert_isTrue(transaction->count <= 0xFFFF, NULL);
#endif
    if (transaction->count == 0) {
        return (FALSE);
    }

    /* Check if a transfer is in progress */
    key = Hwi_disable();
    if (object->transaction) {
        Hwi_restore(key);

        Log_error1("SPI:(%p) transaction still in progress",
                   ((SPIUSCIADMA_HWAttrs const *)(handle->hwAttrs))->baseAddr);

        /* Transfer is in progress */
        return (FALSE);
    }
    else {
        /* Save the pointer to the transaction */
        object->transaction = transaction;
    }
    Hwi_restore(key);

    /* Enable DMA transfer */
    SPIUSCIADMA_configDMA(handle, transaction);

    if (object->transferMode == SPI_MODE_BLOCKING) {
        Log_print1(Diags_USER1, "SPI:(%p) transfer pending on transferComplete "
                                "semaphore",
                  ((SPIUSCIADMA_HWAttrs const *)(handle->hwAttrs))->baseAddr);

        Semaphore_pend(Semaphore_handle(&(object->transferComplete)),
                BIOS_WAIT_FOREVER);
    }

    return (TRUE);
}

/*
 *  ======== SPIUSCIADMA_transferCallback ========
 *  Callback function for when the SPI is in SPI_MODE_BLOCKING
 *
 *  @pre    Function assumes that the handle is not NULL
 */
static Void SPIUSCIADMA_transferCallback(SPI_Handle handle,
                                             SPI_Transaction *msg)
{
    SPIUSCIADMA_Object    *object = handle->object;

    Log_print1(Diags_USER1, "SPI:(%p) posting transferComplete semaphore",
                  ((SPIUSCIADMA_HWAttrs const *)(handle->hwAttrs))->baseAddr);

    Semaphore_post(Semaphore_handle(&(object->transferComplete)));
}
